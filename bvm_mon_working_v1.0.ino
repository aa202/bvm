
#include <Wire.h>

// include public invention 
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// NEOPIXEL BEST PRACTICES for most reliable operation:
// - Add 1000 uF CAPACITOR between NeoPixel strip's + and - connections.
// - MINIMIZE WIRING LENGTH between microcontroller board and first pixel.
// - NeoPixel strip's DATA-IN should pass through a 300-500 OHM RESISTOR.
// - AVOID connecting NeoPixels on a LIVE CIRCUIT. If you must, ALWAYS
//   connect GROUND (-) first, then +, then data.
// - When using a 3.3V microcontroller with a 5V-powered NeoPixel strip,
//   a LOGIC-LEVEL CONVERTER on the data line is STRONGLY RECOMMENDED.
// (Skipping these may work OK on your workbench but can fail in the field)

// CRC calculation as per: https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/5_Mass_Flow_Meters/Sensirion_Mass_Flow_Meters_CRC_Calculation_V1.pdf

#define POLYNOMIAL 0x31     //P(x)=x^8+x^5+x^4+1 = 100110001
#define LED_PIN    6 // Which pin on the Arduino is connected to the NeoPixels?  On a Trinket or Gemma we suggest changing this to 1:
#define LED_COUNT 12 // How many NeoPixels are attached to the Arduino?

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

uint8_t CRC_prim (uint8_t x, uint8_t crc) {
  crc ^= x;
  for (uint8_t bit = 8; bit > 0; --bit) {
    if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
    else crc = (crc << 1);
  }
  return crc;
}

#define sfm3300i2c 0x40

// #define SMALL_BREATH = 1;
// #define MED_BREATH = 2;
// #define LARGE_BREATH = 3;
// #define NO_BREATH = 0;

void setup() {
  
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(127); // Set BRIGHTNESS to about 1/2 (max = 255)
  
  Wire.begin();
  Serial.begin(115200);
  delay(500); // let serial console settle

  
  pinMode(10, OUTPUT); // for the solenoid
  pinMode(3, OUTPUT); // for the red LED
  pinMode(5, OUTPUT); // for the green LED

  
  // soft reset
  Wire.beginTransmission(sfm3300i2c);
  Wire.write(0x20);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

#if 1
  Wire.beginTransmission(sfm3300i2c);
  Wire.write(0x31);  // read serial number
  Wire.write(0xAE);  // command 0x31AE
  Wire.endTransmission();
  if (6 == Wire.requestFrom(sfm3300i2c,6)) {
    uint32_t sn = 0;
    sn = Wire.read(); sn <<= 8;
    sn += Wire.read(); sn <<= 8;
    Wire.read(); // CRC - omitting for now
    sn += Wire.read(); sn <<= 8;
    sn += Wire.read();
    Wire.read(); // CRC - omitting for now
    Serial.println(sn);
  } else {
    Serial.println("serial number - i2c read error");
  }
#endif

  // start continuous measurement
  Wire.beginTransmission(sfm3300i2c);
  Wire.write(0x10);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(100);
  /* // discard the first chunk of data that is always 0xFF
  Wire.requestFrom(sfm3300i2c,3);
  Wire.read();
  Wire.read();
  Wire.read();
  */

  // pin defaults
  digitalWrite(10, LOW); // solenoid off
//  digitalWrite(3, LOW); //red off
//  digitalWrite(5, HIGH); //green on

  Serial.println("Flow\tVolume");
}


const unsigned mms = 5; // measurement interval in ms
const unsigned dms = 500; // display every XXX ms

unsigned long mt_prev = millis(); // last measurement time-stamp
float flow; // current flow value in slm
float vol = 0;  // current volume value in (standard) cubic centimeters
float vol_prev = 0; // volume calculated on previous iteration
bool flow_sign; // flow sign
bool flow_sp; // previous value of flow sign
bool crc_error; // flag

unsigned long rev_flow_t = 4294967295; // set to max value so when delta_t is measured it is always negative
unsigned long new_breath_t = 4294967295; // set to max value so when delta_t is measured it is always negative

int tvol_low_thresh = 300; // mL
int tvol_high_thresh = 400; // mL

int pixel; // pixel number of LED on NeoPixel

bool small_breath = false;
bool med_breath = false;
bool large_breath = false;

// int breath_type = 0; //1,2,3 #define

void display_flow_volume(bool force_d = false) {
  if (5 < abs(vol) || force_d) { // for convenience let's display only significant volumes (>5ml)
    //Serial.print(flow);
    //Serial.print("\t");
    
    Serial.print(vol);
    Serial.println(crc_error?" CRC error":"");  
   
    
  }
}


void SFM_measure(unsigned long ms_curr) {
  
  if (3 == Wire.requestFrom(sfm3300i2c, 3)) {
    vol_prev = vol;
    uint8_t crc = 0;
    uint16_t a = Wire.read();
    crc = CRC_prim (a, crc);
    uint8_t  b = Wire.read();
    crc = CRC_prim (b, crc);
    uint8_t  c = Wire.read();
    unsigned long mt = millis(); // measurement time-stamp
    if (crc_error = (crc != c)) // report CRC error
      return;
    a = (a<<8) | b;
    float new_flow = ((float)a - 32768) / 120;
    // an additional functionality for convenience of experimenting with the sensor
    flow_sign = 0 < new_flow;
    if (flow_sp != flow_sign) { // once the flow changed direction
      flow_sp = flow_sign;
      display_flow_volume();    // display last measurements
      vol = 0;                  // reset the volume
      

      if ((vol_prev < tvol_low_thresh) && (vol_prev > 5)){ // turns lights back on if volume is not enough
          rev_flow_t = millis();    // saves time of reversal of flow

          Serial.println("aaaaa");

          pixel = ceil(12*((float) vol_prev/(float) tvol_low_thresh)); // current last pixel that should be off

          Serial.println(pixel);
          Serial.println("aaaaaaaaa");
          
          for(int i=pixel; i>=0; i--) { // For each pixel in strip...
            strip.setPixelColor(i, strip.Color(0, 16,   0));         //  set color to green
            strip.show();                          //  Update strip to match
            delay(50);                           //  Pause for a moment
          }
        
      }
      else if (vol_prev < tvol_high_thresh && vol_prev >= tvol_low_thresh){
          rev_flow_t = millis();    // saves time of reversal of flow
      }
      else if (vol_prev > tvol_high_thresh){
          rev_flow_t = millis();    // saves time of reversal of flow
          digitalWrite(10, LOW); // tell solenoid to turn off
      }
      
      
    }
    flow = new_flow;
    unsigned long mt_delta = mt - mt_prev; // time interval of the current measurement
    mt_prev = mt;
    vol += flow/60*mt_delta; // flow measured in slm; volume calculated in (s)cc
    // /60 --> convert to liters per second
    // *1000 --> convert liters to cubic centimeters
    // /1000 --> we count time in milliseconds
    // *mt_delta --> current measurement time delta in milliseconds
  } else {
    // report i2c read error
  }

  
  // ---------- new section -----------

  // make a small number of routines to handle transitions
  // lets you name transitions
  

  if (vol >= 5 && vol_prev < 5) { // stores time at which breath was initiated
      new_breath_t = millis();
      small_breath = true;
//      Serial.println("1");
  }

  
  else if (vol >= 5 && vol < tvol_low_thresh && vol_prev < 5){ // resets lights to green if new breath delivered early
      new_breath_t = millis();
      colorSet(strip.Color(0, 16,   0), 0); // Green
      small_breath = true;
//      Serial.println("2");
  }
  
  else if ((vol >= 5) && (vol < tvol_low_thresh)){
      pixel = ceil(12*( (float) vol/ (float) tvol_low_thresh));
      Serial.println("pixel:");
      Serial.println(pixel);
      strip.setPixelColor(pixel, strip.Color(0, 0, 0)); // turns off lights one by one until lower threshold reached
      strip.show();
//      Serial.println("3");
  }

  else if (vol > tvol_low_thresh && vol_prev < tvol_low_thresh){
      colorSet(strip.Color(16, 0,   0), 400); // Solid Red
      small_breath = false;
      med_breath = true;
//      Serial.println("4");
  }

  else{
// no transition
//    Serial.println("I've missed a spot");
    
  }


  if (vol >= tvol_high_thresh && vol_prev < tvol_high_thresh) { // when the volume passes the higher tidal volume threshold
      digitalWrite(10, HIGH); // tell solenoid to turn on
      med_breath = false;
      large_breath = true;
//      Serial.println("5");
  }

  
  if (ms_curr - rev_flow_t > 0 && ms_curr - rev_flow_t < 4000 && (med_breath == true || large_breath == true)) { // waits four seconds until it resets green lights

//    Serial.println("6");
    
    if (ms_curr - rev_flow_t < 500) {
      colorSet(strip.Color(0, 0,   0), 0); // Off
    }
    else if (ms_curr - rev_flow_t < 1000) {
      colorSet(strip.Color(16, 0,   0), 0); // Red
    }
    else if (ms_curr - rev_flow_t < 1500) {
      colorSet(strip.Color(0, 0,   0), 0); // Off
    }
    else if (ms_curr - rev_flow_t < 2000) {
      colorSet(strip.Color(16, 0,   0), 0); // Red
    }
    else if (ms_curr - rev_flow_t < 2500) {
      colorSet(strip.Color(0, 0,   0), 0); // Off
    }
    else if (ms_curr - rev_flow_t < 3000) {
      colorSet(strip.Color(16, 0,   0), 0); // Red
    }
    else if (ms_curr - rev_flow_t < 3500) {
      colorSet(strip.Color(0, 0,   0), 0); // Off
    }
    
  }

  else if (((float) ms_curr - (float) rev_flow_t) >= 4000) { // waits four seconds until it resets green lights

      Serial.println("bbbbbbbbb");
      Serial.println(ms_curr);
      Serial.println(rev_flow_t);
      
      rev_flow_t = 4294967295;
      new_breath_t = 4294967295;
      colorSet(strip.Color(0, 16,   0), 0); // Green
      small_breath = false;
      med_breath = false;
      large_breath = false;
  }
  
}


unsigned long ms_prev = millis(); // timer for measurements "soft interrupts"
unsigned long ms_display = millis(); // timer for display "soft interrupts"


// # of lights to turn off = 12*ceil(vol/tvol_low_thresh)

void loop() {
  unsigned long ms_curr = millis();
  
  if (ms_curr - ms_prev >= mms) { // "soft interrupt" every mms milliseconds
    ms_prev = ms_curr;
    SFM_measure(ms_curr);
  }

  if (ms_curr - ms_display >= dms) { // "soft interrupt" every dms milliseconds
    ms_display = ms_curr;
    display_flow_volume(true);
  } 

//  if (vol > 5 && vol < tvol_low_thresh){
//      int pixel = 12*ceil(vol/tvol_low_thresh);
//      strip.setPixelColor(pixel, strip.Color(0, 0, 0)); // turns off lights one by one until lower threshold reached
//      strip.show(); 
//  }
}

// NeoPixel Functions


// Sets the color of all LEDs to one given color, then displays the color for
// a given number of milliseconds
void colorSet(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
  }
  strip.show();                          //  Update strip to match
  delay(wait);                           //  Pause for a moment
}


// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}
