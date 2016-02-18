/*
 * Arduino based front-end using serial communication, 16x2LCD, potentiometer, encoder, 2x MSGEQ7, PT2322 board
 */

#include <LiquidCrystal_I2C.h>
#include <Encoder.h>
#include <Button.h>
#include <PT2314.h>
#include <Wire.h>
#include <EEPROM.h>
#include <IRremote.h>

/*
#------------------------------------------------------------#
#  This section defines:                                     #
#  - all the pins on the arduino to be used                  #
#  - the IR codes                                            #
#  - the input source names                                  #
#  - the chip type                                           #
#  - the diplay type                                         #
#------------------------------------------------------------#
*/

#define RECV_PIN 14  // A0 pin for IR reciver
#define VU_IN_L 2    // A2 for left  channel VU meter
#define VU_IN_R 3    // A3 for right channel VU meter
#define ENC_A 2      // digital input for 1st encoder pin
#define ENC_B 3      // digital input for 2nd encoder pin
#define ENC_BTN 4    // digital input for encoder button
#define SOURCE_BTN 8 // digital input for source button

#define ROWS 2       // number of display rows
#define COLS 16      // number of display columns

String InputNames[4] = { "Li1", "Li2", "USB", "BT " }; //names of the inputs
String ToneNames[4] = { "BASS", "MIDDLE", "TRBLE", "VOLUME" }; //names of the inputs

#define MIN_VOLUME 0 // dB
#define MAX_VOLUME 100   // dB
#define MIN_TONE 0   // db
#define MAX_TONE 100    // db
#define ATT_LEVEL 100  //
#define NUM_MODES 4    // number of app modes (starts at 0 !)
#define NUM_SOURCES 3  // number of input sources (starts at 0!)

bool HAS_NO_MIDDLE  = true;   // does chip have middle adjustment or not 0 if it has 1 if not

const unsigned long irOk    = 2011282021;
const unsigned long irLeft  = 2011238501;
const unsigned long irRight = 2011291749;
const unsigned long irUp    = 2011287653;
const unsigned long irDown  = 2011279461;
const unsigned long irPlay  = 2011265637;
const unsigned long irMenu  = 2011250789;

//#----------------------------------------------------------#

#define NUM_READINGS 3 // number of analog reading to average it's value
#define SERIAL_SPEED 9600 // serial port speed
#define SERIAL_BUF_LEN 64 // serial reading buffer
#define EEPROM_ADDRESS_OFFSET 400 // address offset to start reading/wring to the EEPROM

#define DELAY_VOLUME 3000 // volume bar show decay
#define DELAY_ENCODER 100 // delay between sending encoder changes back to Pi
#define DELAY_MODE 400 // mode switch debounce delay
#define MultiClickTime 250 //max time for double click
#define DebounceTime = 20 // switch debounce time
#define DELAY_EEPROM 10000 // delay to store to the EEPROM
#define T_REFRESH    100            // msec bar refresh rate
#define T_PEAKHOLD   3*T_REFRESH    // msec peak hold time before return

byte  fill[6]={ 0x20,0x00,0x01,0x02,0x03,0xFF };      // character used to fill (0=empty  5=full)
byte  peak[7]={ 0x20,0x00,0x04,0x05,0x06,0x07,0x20 }; // character used to peak indicator
int   lmax[2];                                        // level max memory
int   dly[2];                                         // delay & speed for peak return
long  lastT=0;

PT2314 audio; // audio chip connected to i2c bus (GND, A4, A5)
LiquidCrystal_I2C lcd(0x27,COLS,ROWS);
Encoder enc(ENC_A, ENC_B); // encoder pins A and B connected to D2 and D3
Button btn(ENC_BTN, BUTTON_PULLUP); // encoder's button connected to GND and D4
Button btn_source(SOURCE_BTN, BUTTON_PULLUP); // source button connected to GND and A1 (15)
//ClickButton btn_test(8, LOW, CLICKBTN_PULLUP); // source button connected to GND and A1
bool backlight = true;

unsigned long last_enc = 0; // timestamp of last encoder changed

String t[ROWS]; // LCD buffer
String prev_t[ROWS]; // LCD previous buffer

// enum with application states
enum app_mode_e {
  app_mode_tone_volume = 0,
  app_mode_vu,
  app_mode_tone_bass,
  app_mode_tone_mid,
  app_mode_tone_treble
};

enum input_source_e {
  INPUT1 = 0,
  INPUT2,
  INPUT3,
  INPUT4
};

int input_source                 = INPUT1;
bool source_changed              = true;
unsigned long last_source_change  = 0; // timestamp of last input source changed
unsigned long last_source_pressed = 0; // timestamp of last input source changed
unsigned long last_backlight_changed = 0; // timestamp of last backlight changed


int prev_source                  = INPUT1;
bool need_store_source           = false;

int mode = app_mode_tone_volume; // mode set to default (volume display)
bool mode_changed = false;    // mode button has been pressed
unsigned long last_mode = 0; // timestamp of last mode changed

int tones[5]; // bass, mid, treble, volume
int prev_tones[5]; // bass, mid, treble, volume
bool need_store = false; // flag if we need to store something to the EEPROM
unsigned long last_tone = 0; // timestamp of last tone change

IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long last_ir_code = 0;
unsigned long ir_code = 0;
bool muted = false;
unsigned long last_mute = 0; // timestamp of last mute change

// custom LCD characters (volume bars)

byte vol_block[8][8]=
{
  { 0b10000, 0b10000, 0b10000, 0b10100, 0b10100, 0b10000, 0b10000, 0b10000 },
  { 0b10100, 0b10100, 0b10100, 0b10100, 0b10100, 0b10100, 0b10100, 0b10100 },
  { 0b10101, 0b10101, 0b10101, 0b10101, 0b10101, 0b10101, 0b10101, 0b10101 },
  { 0b00101, 0b00101, 0b00101, 0b00101, 0b00101, 0b00101, 0b00101, 0b00101 },
  { 0b00001, 0b00001, 0b00001, 0b00101, 0b00101, 0b00001, 0b00001, 0b00001 },
  { 0b00000, 0b00000, 0b00000, 0b00100, 0b00100, 0b00000, 0b00000, 0b00000 },
  { 0b00000, 0b00001, 0b00011, 0b00111, 0b00111, 0b00011, 0b00001, 0b00000 },
  { 0b00000, 0b10000, 0b11000, 0b11100, 0b11100, 0b11000, 0b10000, 0b00000 },
};

// custom LCD characters (vu bars)
byte vu_block[8][8]=
{
  { 0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10 },  // define character for fill the bar
  { 0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18 },
  { 0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C,0x1C },
  { 0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E,0x1E },
  { 0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08 },  // define character for peak level
  { 0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04 },
  { 0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02 },
  { 0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01 },
};

/**
 * Load custom LCD characters for volume control
 */
void loadVolumeCharacters() {
  for( int i=0 ; i<8 ; i++ )
    lcd.createChar( i,vu_block[i] );
  }

/**
 * Load custom LCD characters for VU meter
 */
void loadVUMeterCharacters(){
  for( int i=0 ; i<8 ; i++ )
    lcd.createChar( i,vol_block[i] );
  }

/**
 * Arduino setup routine
 */
void setup() {
  Serial.begin(9600);
  Serial.println("Starting");
  // set default tones
  for (int i=0; i<5; i++) {
    tones[i] = 0;
    prev_tones[i] = 0;
  }

  // setup lcd
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  irrecv.enableIRIn(); // Start the receiver
  loadVolumeCharacters();
  // restore saved tone values from EEPROM
  restoreTones();
  input_source = tones[4];

  Wire.begin();
  // init audio chip
  audio.init();
  delay(100);

  // send volume and tones to the audio chip
  send2chip();
  analogReference(DEFAULT);
  pinMode(VU_IN_L, INPUT);
  pinMode(VU_IN_R, INPUT);
  switchInput();
  setEncoder(tones[3]);
  mode_changed = true;
  }

/**
 * Application mode to display a graphical VU meter
 */

void AppVUMeter(unsigned long current)
  {
  if (mode_changed)
      {
      lcd.clear();
      loadVUMeterCharacters();
      mode_changed = false;
      setEncoder(tones[3]);
      }

  tones[3] = readEncoder(100);
  if ( ir_code == irRight )
    {
    if ( tones[3] < 100 )
      {
      tones[3] = tones[3] + 1;
      setEncoder(tones[3]);
      }
    }
  if ( ir_code == irLeft )
    {
    if ( tones[3] > 0 )
      {
      tones[3] = tones[3] - 1;
      setEncoder(tones[3]);
      }
    }

  if( millis()<lastT )
    return;

  lastT += T_REFRESH;
  int anL = map( sqrt( analogRead( VU_IN_L  )*128 ),0,128,0,80 );  // sqrt to have non linear scale (better was log)
  int anR = map( sqrt( analogRead( VU_IN_R  )*128),0,128,0,80 );
  bar( 0,anL );
  bar( 1,anR );
  }

void  bar  ( int row,int lev )
{
  lcd.setCursor( 0,row );
  lcd.write( row ? 'R' : 'L' );
  for( int i=1 ; i<16 ; i++ )
  {
    int f=constrain( lev      -i*5,0,5 );
    int p=constrain( lmax[row]-i*5,0,6 );
    if( f )
      lcd.write( fill[ f ] );
    else
      lcd.write( peak[ p ] );
  }
  if( lev>lmax[row] )
  {
    lmax[row] = lev;
    dly[row]  = -(T_PEAKHOLD)/T_REFRESH;                // Starting delay value. Negative=peak don't move
  }
  else
  {
    if( dly[row]>0 )
      lmax[row] -= dly[row];

    if( lmax[row]<0 )
      lmax[row]=0;
    else
      dly[row]++;
  }
}


/**
 * Application mode to control Bass / Middle/ Treble / Volume
 * @param unsigned long current - current timestamp and which tome to change
 */
void AppTone(unsigned long current, int tone_to_change) {
  if (mode_changed) {
    lcd.clear();
    loadVolumeCharacters();
    mode_changed = false;
    setEncoder(tones[tone_to_change]);
    //displayInput();
  }
  tones[tone_to_change] = readEncoder(100);
  if ( ir_code == irRight )
    {
    if ( tones[tone_to_change] < 100 )
      {
      tones[tone_to_change] = tones[tone_to_change] + 1;
      setEncoder(tones[tone_to_change]);
      }
    }
  if ( ir_code == irLeft )
    {
    if ( tones[tone_to_change] > 0 )
      {
      tones[tone_to_change] = tones[tone_to_change] - 1;
      setEncoder(tones[tone_to_change]);
      }
    }
  //lcd.setCursor(0,0);
  update_display(0, ToneNames[tone_to_change] + " " + tones[tone_to_change] + " " + InputNames[input_source]);
  //lcd.print(ToneNames[tone_to_change]);
  //lcd.print(" ");
  //lcd.print(tones[tone_to_change]);
  //lcd.print("  ");
  //lcd.setCursor(0,1);
  printBar(tones[tone_to_change]);
}


/**
 * Main application loop
 */

unsigned long click1 = 0;
unsigned long click2 = 0;
bool done = true;

void get_ir()
  {
  ir_code = 0;
  if (irrecv.decode(&results))
    {
    ir_code = results.value;
    if ( ir_code == 0xffffffff )
      {
      ir_code = last_ir_code;
      }
    last_ir_code = ir_code;
    Serial.println(ir_code);
    irrecv.resume(); // Receive the next value
    }
  }

void loop() {

  unsigned long current = millis();
  get_ir();
  if ((btn.isPressed() && current - last_mode >= DELAY_MODE ) or (ir_code == irOk && current - last_mode >= DELAY_MODE)){
    last_mode = current;
    if (mode == NUM_MODES) {
      mode = 0;
    } else {
      mode = mode + 1;
    }
    if ((mode == app_mode_tone_mid) and (HAS_NO_MIDDLE)){
      mode = app_mode_tone_treble;
    }
    mode_changed = true;
    need_store = true;
  }

  if ( btn_source.isPressed() && btn_source.stateChanged() )
    {
    click2 = click1;
    click1 = current;
    done = false;
    }

  if ( (click1 - click2 > 20 && click1 - click2 <= 250) or ir_code == irPlay
    && current - last_backlight_changed >= DELAY_MODE)
    {
    if (backlight == true)
        {
        lcd.noBacklight();
        backlight = false;
        }
      else
        {
        lcd.backlight();
        backlight = true;
        }
    last_backlight_changed = current;
    click2 = current;
    click1 = current;
    done = true;
    }

    if ( done == false && current - click1 >= DELAY_MODE )
      {
      last_source_change = current;
      if (input_source == NUM_SOURCES)
        {
        input_source = 0;
        }
      else
        {
        input_source = input_source + 1;
        }
      source_changed = true;
      switchInput();
      need_store_source = true;
      click1 = current;
      click2 = current;
      done   = true;
      }

    if ( ir_code == irMenu && current - last_source_change >= DELAY_MODE )
      {
      Serial.print("changing source: ");
      if (input_source == NUM_SOURCES)
        {
        input_source = 0;
        }
      else
        {
        input_source = input_source + 1;
        }
      Serial.println(input_source);
      source_changed = true;
      switchInput();
      need_store_source = true;
      last_source_change = current;
      }

  if (ir_code == irPlay )
    {
    if ( muted )
      {
      muted = false;
      send2chip();
      }
    else
      {
      muted = true;
      send2chip();
      }
    last_mute = current;
    }

  if (tones[0] != prev_tones[0] || tones[1] != prev_tones[1] || tones[2] != prev_tones[2] || tones[3] != prev_tones[3] || tones[4] != prev_tones[4]) {
    last_tone = current;
    prev_tones[0] = tones[0];
    prev_tones[1] = tones[1];
    prev_tones[2] = tones[2];
    prev_tones[3] = tones[3];
    prev_tones[4] = tones[4];
    need_store = true;
    send2chip();
  }

  // store settings in EEPROM with delay to reduce number of EEPROM write cycles
  if (need_store && current - last_tone >= DELAY_EEPROM) {
      storeTones();
      need_store = false;
  }

  switch (mode) {
    case app_mode_vu:
      AppVUMeter(current);
    break;
    case app_mode_tone_bass:
      AppTone(current, 0);
    break;
    case app_mode_tone_mid:
      AppTone(current, 1);
    break;
    case app_mode_tone_treble:
      AppTone(current, 2);
    break;
    case app_mode_tone_volume:
      AppTone(current, 3);
  }
}

/**
 * Load stored tone control values from the EEPROM (into the tones and prev_tones)
 */
void restoreTones() {

  byte value;
  int addr;

  // bass / mid / treble / volume
  for (int i=0; i<5; i++) {
    addr = i + EEPROM_ADDRESS_OFFSET;
    value = EEPROM.read(addr);
    // defaults
    if (value < 0 || value > 100) {
      value = 0;
    }
    tones[i] = value;
    prev_tones[i] = value;
  }
}

/**
 * Store tone value in the EEPROM
 * @param int mode
 */
void storeTone(int mode) {
  int addr = mode + EEPROM_ADDRESS_OFFSET;
  EEPROM.write(addr, tones[mode]);
}

/**
 * Store tone values in the EEPROM
 */
void storeTones() {
  // bass / treble / balance / volume / channel
  for (int i=0; i<5; i++) {
    storeTone(i);
  }
}


void send2chip() {
  Serial.println("sending to chip");
  audio.bass(map(tones[0], 0, 100, MIN_TONE, MAX_TONE));
  //audio.middle(map(tones[1], 0, 100, MIN_TONE, MAX_TONE));
  audio.treble(map(tones[2], 0, 100, MIN_TONE, MAX_TONE));
  audio.volume(map(tones[3], 0, 100, MIN_VOLUME, MAX_VOLUME));
  audio.attenuation(ATT_LEVEL, ATT_LEVEL);
  audio.muteOff();
  audio.loudnessOn();
  audio.attenuation(100,100);
  Serial.print("muted: ");
  Serial.println(muted);
  audio.channel(tones[4]);
  if ( muted ) { audio.muteOn(); }
  else { audio.muteOff(); }
  }

void switchInput() {
  tones[4] = input_source;
  send2chip();
  //displayInput();
  }

//void displayInput() {
//  lcd.setCursor(13,0);
//  lcd.print(InputNames[input_source]);
//  }

/**
 * Read encoder value with bounds from 0 to max_encoder_value
 * @param int max_encoder_value
 */
int readEncoder(int max_encoder_value) {
  int value = enc.read() / 4;
  if (value > max_encoder_value) {
    value = max_encoder_value;
    setEncoder(max_encoder_value);
  }
  if (value < 0) {
    value = 0;
    setEncoder(0);
  }
  return value;
}

/**
 * Save encoder value
 * @param int value
 */
void setEncoder(int value) {
  enc.write(value * 4);
}

 /**
  * Conver string object into signed integer value
  *
  * @param String s
  * @return int
  */
 int stringToInt(String s) {
     char this_char[s.length() + 1];
     s.toCharArray(this_char, sizeof(this_char));
     int result = atoi(this_char);
     return result;
 }

 /**
  * Pring a progress bar on the current cursor position
  *
  * @param int percent
  */
 void printBar(int percent) {

   double length = COLS + 0.0;
   double value = length/100*percent;
   int num_full = 0;
   double value_half = 0.0;
   int peace = 0;
   lcd.setCursor(0,1);
   // fill full parts of progress
   if (value>=1) {
    for (int i=1;i<value;i++) {
      lcd.write(3);
      num_full=i;
    }
    value_half = value-num_full;
  } else {
    value_half = value;
  }

  // fill partial part of progress
  peace=value_half*5;

  if (peace > 0 && peace <=5) {
    if (peace == 1 || peace == 2) lcd.write(1);
    if (peace == 3 || peace == 4) lcd.write(2);
    if (peace == 5) lcd.write(3);
  }

  // fill spaces
  int spaces = length - num_full;
  if (peace) {
    spaces = spaces - 1;
  }
  for (int i =0;i<spaces;i++) {
    lcd.write(6);
  }
 }

 void update_display(int line, String LineContent){
   lcd.setCursor(0,line);
   lcd.print(LineContent);
 }
