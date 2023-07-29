/*           Z-Wave.Me                                                          |  Digital pins:     0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23
             Z-Uno 2 PinOut:                                                    |  Analog read pins: 3,4,5,6
                              White led uses in scetch <--------------------    |  PWM pins:         3,13,14,15,16
                              Green led loop activity                      !    |  UART pins:        7,8,24,25
                              Red   led radio/system activity              !    |  I2C  pins:        9,10            /Antenna
                               ___       ______                            !                             ___________/____
             [7-18V]----------|o  [IIIII]     o|---[23]---[ BTN ]--        !           [ SCL ]---[ 9]---|o          0   o|---[ 8]---[TX 1 ]--[SPI CS]
             [ 5V  ]----------|o  [=USB=]     o|---[22]           !        !           [ SDA ]---[10]---|o              o|---[ 7]---[TX 1 ]
             [ GND ]----------|o |R| WGR |B|  o|---[21]           !        !                     [11]---|o   [ Chip ]   o|---[ 6]---[ADC 3]
             [ 3V3 ]----------|o |S| LED |T|  o|---[20]           !        !                     [12]---|o   [######]   o|---[ 5]---[ADC 2]
             [ SCK ]---[ 0]---|o |T|     |N|  o|---[19]           !        !--[ LED ]--[PWM 1]---[13]---|o   [Z-Wave]   o|---[ 4]---[ADC 1]
             [MISO ]---[ 1]---|o              o|---[18]---[INT 1]--        !           [PWM 2]---[14]---|o              o|---[ 3]---[ADC 0]---[PWM 0]
             [MOSI ]---[ 2]---|o              o|---[17]---[INT 0]          !           [PWM 3]---[15]---|o              o|---[25]---[RX 0 ]
             [TX 0 ]---[24]---|o              o|---[16]---[PWM 4]          !           [PWM 4]---[16]---|o              o|---[24]---[TX 0 ]
             [RX 0 ]---[25]---|o              o|---[15]---[PWM 3]          !           [INT 0]---[17]---|o              o|---[ 2]---[MOSI ]
   [PWM 0]---[ADC 0]---[ 3]---|o              o|---[14]---[PWM 2]          !         --[INT 1]---[18]---|o              o|---[ 1]---[MISO ]
             [ADC 1]---[ 4]---|o   [Z-Wave]   o|---[13]---[PWM 1]--[ LED ]--         !           [19]---|o |B|      |R| o|---[ 0]---[ SCK ]
             [ADC 2]---[ 5]---|o   [######]   o|---[12]                              !           [20]---|o |T|  LED |S| o|----------[ 3V3 ]
             [ADC 3]---[ 6]---|o   [ Chip ]   o|---[11]                              !           [21]---|o |N|  RGW |T| o|----------[ GND ]
             [TX 1 ]---[ 7]---|o              o|---[10]---[ SDA ]                    !           [22]---|o     [=USB=]  o|----------[ 5V  ]
   [SPI CS]--[RX 1 ]---[ 8]---|o__0___________o|---[ 9]---[ SCL ]                    --[ BTN ]---[23]---|o_____[IIIII]__o|----------[7-18V]
                                 /
                                /Antenna
*/
#include "EEPROM.h"               // For save data in EEPROM
#include "ZUNO_DHT.h"             // For DHT-22 Sensor
#include "ZUNO_DS18B20.h"         // For ds18b20 Sensor
#include <U8g2lib.h>              // For oled display
#include <Wire.h>                 // For oled Display

//=======================Enable Z-Wave Command Classes ==============================================
ZUNO_ENABLE(WITH_CC_SWITCH_BINARY                 //need for switch 4
            WITH_CC_SWITCH_MULTILEVEL             //need for dimmer 4
            WITH_CC_BASIC WITH_CC_MULTICHANNEL    //need for multichannel works
            WITH_CC_SENSOR_MULTILEVEL             //need for all multilevel sensors
            WITH_CC_NOTIFICATION                  //need for binary sensor (water overflow)
            WITH_CC_TIME_PARAMETERS               //need for time and date 
           )    //Enable support for command clasesses
//===================================================================================================


//======================Encoder==================================================================
#define encoderPinA  1                // outputA digital pin1
#define encoderPinB  2                // outoutB digital pin2
#define encoderPinButton 0
volatile int encoderStepCount = 0;    // variable to store enoder position
volatile int encoderButtonStatus = 0;
int encoderPreviousCount = 0;         // variable to filter encoder
unsigned long encoderFilterlastMillis;// variable to filter encoder
//===============================================================================================


//===================== Outputs relay and PWM ======================================================
#define relay1Pin 22
#define relay2Pin 21
#define relay3Pin 20
#define relay4Pin 19
#define pwm1Pin 13
#define pwm2Pin 14
#define pwm3Pin 15
#define pwm4Pin 16

byte lastRelay1State, relay1State = 0;
byte lastRelay2State, relay2State = 0;
byte lastRelay3State, relay3State = 0;
byte lastRelay4State, relay4State = 0;
byte lastPwm1State,   pwm1State = 0;
byte lastPwm2State,   pwm2State = 0;
byte lastPwm3State,   pwm3State = 0;
byte lastPwm4State,   pwm4State = 0;

// create a structure to store getter & setter handlers for relay and pwm this need for z-wave
static zuno_handler_single_gettersetter_t relay1_handlers = {(void*)&relay1Getter, (void*)&relay1Setter};
static zuno_handler_single_gettersetter_t relay2_handlers = {(void*)&relay2Getter, (void*)&relay2Setter};
static zuno_handler_single_gettersetter_t relay3_handlers = {(void*)&relay3Getter, (void*)&relay3Setter};
static zuno_handler_single_gettersetter_t relay4_handlers = {(void*)&relay4Getter, (void*)&relay4Setter};

static zuno_handler_single_gettersetter_t pwm1_handlers = {(void*)&pwm1Getter, (void*)&pwm1Setter};
static zuno_handler_single_gettersetter_t pwm2_handlers = {(void*)&pwm2Getter, (void*)&pwm2Setter};
static zuno_handler_single_gettersetter_t pwm3_handlers = {(void*)&pwm3Getter, (void*)&pwm3Setter};
static zuno_handler_single_gettersetter_t pwm4_handlers = {(void*)&pwm4Getter, (void*)&pwm4Setter};
//=====================================================================================================



//===================== Water overflow sensor========================================================
#define interruptPin 17
volatile int eStopLast = 0;
byte eStop = 0;
//===================================================================================================



//======================DHT-22=======================================================================
#define DHTPIN 12

// Global variables for DHT-22
DHT dht22(DHTPIN, DHT22);
float dht22TemperatureState = 0;
float  dht22TemperatureStateValue;
//variables for filtering dht-22 data
float  dht22TemperatureStateValueBuf;
byte dht22errLimit = 100;
byte dht22errHolder = 0;
byte dht22HumidityState = 0;
//===================================================================================================



//================================mh-z19=============================================================
word mhz19_ppm;
//===================================================================================================



//======================DS18B20======================================================================
#define DS18B20_BUS_PIN 11                 // Pin to which 1-Wire bus is connected
#define MAX_SENSORS     5                  // Number of DS18B20 sensors supported (equals to maximum number of channels for Z-Uno) 

OneWire ow(DS18B20_BUS_PIN);                // Software 1-Wire BUS
DS18B20Sensor ds18b20(&ow);                 // connect DS18B20 class to it

#define ADDR_SIZE 8                         // Size of address of devices on 1-wire bus
byte addresses[ADDR_SIZE * MAX_SENSORS];    // Here we store all the scanned addresses
#define ADDR(i) (&addresses[i * ADDR_SIZE]) // Macro to simplify our life
byte number_of_sensors;                     // Number of sensors found
byte ds18b20LoopCounter = 0;                //

int16_t temperatures[MAX_SENSORS];          //All temperature from ds18b20
//==================================================================================================



//======================128x64 Oled display==========================================================
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2 (U8G2_R0, 9, 10);

u8g2_uint_t ds18b20_offset;     // current offset for the scrolling text
u8g2_uint_t ds18b20_width;      // pixel width of the scrolling text (must be lesser than 128 unless U8G2_16BIT is defined
char *text; // scroll this text from right to left
//===================================================================================================



void setup(void) {
  u8g2.begin();                     //Display init
  u8g2.clearBuffer();               // clear the internal memory
  u8g2.setFont(u8g2_font_7x14B_tr); // choose a suitable font
  u8g2.setCursor(15, 32);
  u8g2.println("Welcome");
  u8g2.sendBuffer();                // transfer internal memory to the display
  timeInit();                        // initilization of time and date
  outputInit();                     // relay & PWM output init
  encoderInit();                    // encoder init
  dht22.begin();                    // DHT-22 init
  waterOverflowInit();              // water overflow sensor init
  ds18b20Scanner();                 // scaning all ds18b20 sensors and init
  zwave_channelsSetup();            // z-wave channel init
  Serial.begin(115200);             // Serial debug init
  Serial1.begin(9600);              // UART for mh-z19 init
  Serial.println("=====START=====");

}

void loop(void) {
  Serial.println("Loop tick");
  waterOverflowSensor();            //Overflow sensor handler
  outputHandler();                  //Relay & PWM handler
  asyncUpdateSensors();             //Get data from 1 sensor every loop
  printOLED(64);                    //Print all data
  encoderGetPos(10000);
  Serial.print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!GET position of encoder: ");
  Serial.println(encoderStepCount);
}

//====================== Scanner for ds18b20 ====================================================
void ds18b20Scanner() {
  // Scanning sensors on the bus every time we starting a sketch
  number_of_sensors = ds18b20.findAllSensors(addresses);
  if (number_of_sensors > MAX_SENSORS)
    number_of_sensors = MAX_SENSORS;
  // Setting up Z-Uno channels dynamically
  // You have to exclude/include your Z-Uno to this take effect on the controller side
  ds18b20_width = number_of_sensors * 35;    // calculate the pixel width of the text
}
//===============================================================================================


//====================== Setup Z-Wave Channels ==================================================
void zwave_channelsSetup() {

  ZUNO_SETUP_S2ACCESS(SKETCH_FLAG_S2_UNAUTHENTICATED_BIT);
  if (zunoStartDeviceConfiguration()) {
    /*
      channel 0 relay1 + pwm1 + water overflow sensor
      channel 1 relay1
      channel 2 relay2
      channel 3 relay3
      channel 4 relay4
      channel 5 pwm1
      channel 6 pwm2
      channel 7 pwm3
      channel 8 pwm4
      channel 9 DHT-22 Temperature
      channel 10 DHT-22 Humidity
      channel 11 mh-z19 Co2 level
      channel 12 water overflow sensor
      channel 13-31 ds-18b20
    */
    //We have 4 relay and 4 pwm outputs this block generate 9 channels
    for (int i = 0; i < 9; i++) {
      if ( i < 5) {
        zunoAddChannel(ZUNO_SWITCH_BINARY_CHANNEL_NUMBER, 0, 0);   //Channel 0 + 1
        (i == 0) ? zunoSetZWChannel(i, i | ZWAVE_CHANNEL_MAPPED_BIT) : zunoSetZWChannel(i, i);
        zunoAddAssociation(ZUNO_ASSOC_BASIC_SET_NUMBER, i);
      } else {
        zunoAddChannel(ZUNO_SWITCH_MULTILEVEL_CHANNEL_NUMBER, 0, 0);//Channel 0 + 5
        (i == 5) ? zunoSetZWChannel(i, i | ZWAVE_CHANNEL_MAPPED_BIT) : zunoSetZWChannel(i, i);
        zunoAddAssociation(ZUNO_ASSOC_BASIC_SET_AND_DIM_NUMBER, i);
      }
    }

    //DHT-22 Temperature
    zunoAddChannel(ZUNO_SENSOR_MULTILEVEL_CHANNEL_NUMBER,
                   ZUNO_SENSOR_MULTILEVEL_TYPE_TEMPERATURE,
                   SENSOR_MULTILEVEL_PROPERTIES_COMBINER(
                     SENSOR_MULTILEVEL_SCALE_CELSIUS,
                     SENSOR_MULTILEVEL_SIZE_TWO_BYTES,
                     SENSOR_MULTILEVEL_PRECISION_ONE_DECIMAL
                   ));
    zunoSetZWChannel(9, 9);

    //DHT-22 Humidity
    zunoAddChannel(ZUNO_SENSOR_MULTILEVEL_CHANNEL_NUMBER,
                   ZUNO_SENSOR_MULTILEVEL_TYPE_RELATIVE_HUMIDITY,
                   SENSOR_MULTILEVEL_PROPERTIES_COMBINER(
                     SENSOR_MULTILEVEL_SCALE_PERCENTAGE_VALUE,
                     SENSOR_MULTILEVEL_SIZE_ONE_BYTE,
                     SENSOR_MULTILEVEL_PRECISION_ZERO_DECIMALS
                   ));
    zunoSetZWChannel(10, 10);

    //mh-z19 Co2 level
    zunoAddChannel(ZUNO_SENSOR_MULTILEVEL_CHANNEL_NUMBER,
                   ZUNO_SENSOR_MULTILEVEL_TYPE_CO2_LEVEL,
                   SENSOR_MULTILEVEL_PROPERTIES_COMBINER(
                     0,
                     SENSOR_MULTILEVEL_SIZE_TWO_BYTES,
                     SENSOR_MULTILEVEL_PRECISION_ZERO_DECIMALS
                   ));
    zunoSetZWChannel(11, 11);

    //water overflow sensor
    zunoAddChannel(ZUNO_SENSOR_BINARY_CHANNEL_NUMBER, ZUNO_SENSOR_BINARY_TYPE_WATER, 1);
    zunoSetZWChannel(12, 12 | ZWAVE_CHANNEL_MAPPED_BIT);

    // channel generator for ds18b20
    for (byte i = 0; i < (number_of_sensors); i++) {
      // Each channel is temperature sensor
      zunoAddChannel(
        ZUNO_SENSOR_MULTILEVEL_CHANNEL_NUMBER,
        ZUNO_SENSOR_MULTILEVEL_TYPE_TEMPERATURE,
        SENSOR_MULTILEVEL_PROPERTIES_COMBINER(
          SENSOR_MULTILEVEL_SCALE_CELSIUS,
          SENSOR_MULTILEVEL_SIZE_TWO_BYTES,
          SENSOR_MULTILEVEL_PRECISION_TWO_DECIMALS
        ));
      zunoSetZWChannel(i + 13 , i + 13);
    }
  }
  zunoCommitCfg(); // Apply settings ^^^

  // Bind handler functions for 0 channel. You have to setup handler every time device powers on
  zunoAppendChannelHandler(0, 1, CHANNEL_HANDLER_SINGLE_GETTERSETTER, (void*)&relay1_handlers);
  zunoAppendChannelHandler(1, 1, CHANNEL_HANDLER_SINGLE_GETTERSETTER, (void*)&relay1_handlers);
  zunoAppendChannelHandler(2, 1, CHANNEL_HANDLER_SINGLE_GETTERSETTER, (void*)&relay2_handlers);
  zunoAppendChannelHandler(3, 1, CHANNEL_HANDLER_SINGLE_GETTERSETTER, (void*)&relay3_handlers);
  zunoAppendChannelHandler(4, 1, CHANNEL_HANDLER_SINGLE_GETTERSETTER, (void*)&relay4_handlers);
  zunoAppendChannelHandler(5, 1, CHANNEL_HANDLER_SINGLE_GETTERSETTER, (void*)&pwm1_handlers);
  zunoAppendChannelHandler(5, 1, CHANNEL_HANDLER_SINGLE_GETTERSETTER, (void*)&pwm1_handlers);
  zunoAppendChannelHandler(6, 1, CHANNEL_HANDLER_SINGLE_GETTERSETTER, (void*)&pwm2_handlers);
  zunoAppendChannelHandler(7, 1, CHANNEL_HANDLER_SINGLE_GETTERSETTER, (void*)&pwm3_handlers);
  zunoAppendChannelHandler(8, 1, CHANNEL_HANDLER_SINGLE_GETTERSETTER, (void*)&pwm4_handlers);
  zunoAppendChannelHandler(9, 2, CHANNEL_HANDLER_SINGLE_GETTER, (void*)&dht22TemperatureGetter);
  zunoAppendChannelHandler(10, 1, CHANNEL_HANDLER_SINGLE_GETTER, (void*)&dht22HumidityGetter);
  zunoAppendChannelHandler(11, 2, CHANNEL_HANDLER_SINGLE_GETTER, (void*)&co2levelGetter);
  zunoAppendChannelHandler(12, 1, CHANNEL_HANDLER_SINGLE_GETTER, (void*)&eStopGetter);
  for (byte i = 0; i < number_of_sensors; i++) {
    zunoAppendChannelHandler(i + 13, 2, CHANNEL_HANDLER_SINGLE_VALUEMAPPER, &temperatures[i]);
  }
}
//===============================================================================================



//========================== initilization outputs Relay + PWM===================================
void outputInit() {
  pinMode(relay1Pin, OUTPUT);
  relay1State = EEPROM.read(0x0001);
  zunoSendReport(0 + 1);
  pinMode(relay2Pin, OUTPUT);
  relay2State = EEPROM.read(0x0002);
  zunoSendReport(2);
  pinMode(relay3Pin, OUTPUT);
  relay3State = EEPROM.read(0x0003);
  zunoSendReport(3);
  pinMode(relay4Pin, OUTPUT);
  relay4State = EEPROM.read(0x0004);
  zunoSendReport(4);
  pinMode(pwm1Pin, OUTPUT);
  pwm1State = EEPROM.read(0x0011);
  zunoSendReport(0 + 5);
  pinMode(pwm2Pin, OUTPUT);
  pwm2State = EEPROM.read(0x0012);
  zunoSendReport(6);
  pinMode(pwm3Pin, OUTPUT);
  pwm3State = EEPROM.read(0x0013);
  zunoSendReport(7);
  pinMode(pwm4Pin, OUTPUT);
  pwm4State = EEPROM.read(0x0014);
  zunoSendReport(8);
}
//===============================================================================================



//========================== Initilization water level input=====================================
void waterOverflowInit() {
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(interruptPin, waterOverload, FALLING);
}
//===============================================================================================



//=========================Controling relay and pwm==============================================
void outputHandler() {
  if (relay1State != lastRelay1State) {
    zunoSendReport(0 + 1);
    lastRelay1State = relay1State;
    Serial.println("New value for relay 1 set in EEPROM!!!");
    EEPROM.put(0x0001, &relay1State, sizeof(relay1State));
  }
  lastRelay1State = relay1State;
  digitalWrite(relay1Pin, relay1State ? LOW : HIGH);

  if (relay2State != lastRelay2State) {
    zunoSendReport(2);
    lastRelay2State = relay2State;
    Serial.println("New value for relay 2 set in EEPROM!!!");
    EEPROM.put(0x0002, &relay2State, sizeof(relay2State));
  }
  lastRelay2State = relay2State;
  digitalWrite(relay2Pin, relay2State ? LOW : HIGH);

  if (relay3State != lastRelay3State) {
    zunoSendReport(3);
    lastRelay3State = relay3State;
    Serial.println("New value for relay 3 set in EEPROM!!!");
    EEPROM.put(0x0003, &relay3State, sizeof(relay3State));
  }
  lastRelay3State = relay3State;
  digitalWrite(relay3Pin, relay3State ? LOW : HIGH);

  if (relay4State != lastRelay4State) {
    zunoSendReport(4);
    lastRelay4State = relay4State;
    Serial.println("New value for relay 4 set in EEPROM!!!");
    EEPROM.put(0x0004, &relay4State, sizeof(relay4State));
  }
  lastRelay4State = relay4State;
  digitalWrite(relay4Pin, relay4State ? LOW : HIGH);

  if (pwm1State != lastPwm1State) {
    zunoSendReport(0 + 5);
    lastPwm1State = pwm1State;
    Serial.println("New value for relay 1 set in EEPROM!!!");
    EEPROM.put(0x0011, &pwm1State, sizeof(pwm1State));
  }
  lastPwm1State = pwm1State;
  analogWrite(pwm1Pin, (word)pwm1State * 255 / 99);

  if (pwm2State != lastPwm2State) {
    zunoSendReport(6);
    lastPwm2State = pwm2State;
    Serial.println("New value for relay 2 set in EEPROM!!!");
    EEPROM.put(0x0012, &pwm2State, sizeof(pwm2State));
  }
  lastPwm2State = pwm2State;
  analogWrite(pwm2Pin, (word)pwm2State * 255 / 99);

  if (pwm3State != lastPwm3State) {
    zunoSendReport(7);
    lastPwm3State = pwm3State;
    Serial.println("New value for relay 1 set in EEPROM!!!");
    EEPROM.put(0x0013, &pwm3State, sizeof(pwm3State));
  }
  lastPwm3State = pwm3State;
  analogWrite(pwm3Pin, (word)pwm3State * 255 / 99);

  if (pwm4State != lastPwm4State) {
    zunoSendReport(8);
    lastPwm4State = pwm4State;
    Serial.println("New value for relay 4 set in EEPROM!!!");
    EEPROM.put(0x0014, &pwm4State, sizeof(pwm4State));
  }
  lastPwm4State = pwm4State;
  analogWrite(pwm4Pin, (word)pwm4State * 255 / 99);
  Serial.print("R1: ");
  Serial.print(relay1State);
  Serial.print(" R2: ");
  Serial.print(relay2State);
  Serial.print(" R3: ");
  Serial.print(relay3State);
  Serial.print(" R4: ");
  Serial.print(relay4State);
  Serial.print(" D1: ");
  Serial.print(pwm1State);
  Serial.print(" D2: ");
  Serial.print(pwm2State);
  Serial.print(" D3: ");
  Serial.print(pwm3State);
  Serial.print(" D4: ");
  Serial.println(pwm4State);
}



//===========================Assync update sensor data===========================================
void asyncUpdateSensors() {
  if (ds18b20LoopCounter < (number_of_sensors + 6)) {
    Serial.print("Found DS1b20: ");
    Serial.println(number_of_sensors);
    Serial.print("Loop №:");
    Serial.println(ds18b20LoopCounter);

    if (ds18b20LoopCounter < number_of_sensors) {                                          //First we get data from from all findet ds18b20 sensors
      temperatures[ds18b20LoopCounter] =  ds18b20.getTempC100(ADDR(ds18b20LoopCounter));
      Serial.print("GET ds18b20 temperature from T");
      Serial.print(ds18b20LoopCounter);
      Serial.print(": ");
      Serial.println(temperatures[ds18b20LoopCounter]);
      zunoSendReport(number_of_sensors + 12);                                              //report for ds18b20 to last channels


    } else if (ds18b20LoopCounter == (number_of_sensors) ) {                                //Then we get data humidity from DHT-22
      dht22HumidityState = dht22.readHumidity();
      Serial.print("GET DHT-22 Humidity:");
      Serial.print(dht22HumidityState);
      Serial.println("%");

    } else if (ds18b20LoopCounter == (number_of_sensors + 1) ) {                            //Then we get data temperature from DHT-22

      dht22TemperatureState = dht22.readTemperature();
      Serial.print("GET DHT-22 temperature:");
      Serial.print(dht22TemperatureState);
      Serial.println("C");


    } else if (ds18b20LoopCounter == (number_of_sensors + 2) ) {
      if (millis() > 30000) {
        mhz19_ppm = getCo2ppm();
        Serial.print("Getting Co2 Level:");
        Serial.println(mhz19_ppm);
      } else {
        mhz19_ppm = 30 - (millis() / 1000);
      }

    } else if (ds18b20LoopCounter == (number_of_sensors + 3) ) {
      Serial.println("GET data from sensor (DUMMY):");

    } else if (ds18b20LoopCounter == (number_of_sensors + 4) ) {
      Serial.println("GET data from TDS sensor (DUMMY):");

    } else if (ds18b20LoopCounter == (number_of_sensors + 5) ) {
      Serial.println("GET data from ES sensor (DUMMY):");

    } else if (ds18b20LoopCounter == (number_of_sensors + 6) ) {
      Serial.println("GET data from PH sensor (DUMMY):");
    }
    ds18b20LoopCounter++ ;
  } else {
    ds18b20LoopCounter = 0;
  }
}

//this func draw disk 14x14 with number if switchState >0 then disk will draw white
void drawSwitchDisk(byte x, byte y, String number, byte switchState) {
  u8g2.setFont(u8g2_font_7x14B_tr);                                                                       // Change font for draw number
  u8g2.setDrawColor(1);                                                                                   // Change collor to white
  (switchState) ? u8g2.drawDisc(x, y, 7, U8G2_DRAW_ALL) : u8g2.drawCircle(x, y, 7, U8G2_DRAW_ALL);        // if relay on we draw white disk, else we draw hollow circle
  u8g2.setFontMode(1);                                                                                    // Change font draw mode for transparent
  u8g2.setDrawColor(2);                                                                                   // Change font collor to XOR mode
  u8g2.drawStr((x + -3), (y + 5), number.c_str());                                                        // Draw number
  u8g2.setDrawColor(1);                                                                                   // Change collor to white
}

//this func draw disk 14x14 with number filling of circle depends on the position of the dimmer pwmState, if pwm is 99 disk will be white
void drawDimmerDisk(byte x, byte y, String number, byte pwmState) {
  u8g2.setFont(u8g2_font_7x14B_tr);                                                                       // Change font for draw number
  u8g2.setDrawColor(1);                                                                                   // Change collor to white
  u8g2.drawDisc(x, y, 7, U8G2_DRAW_ALL);                                                                  // Draw white disk 14x14 pixels
  u8g2.setDrawColor(0);                                                                                   // Change collor to black
  u8g2.drawBox((x - 6), (y - 6), 13, map(pwmState, -8, 99, 13, 0));                                       // Draw a black box, the size of the box depends on the position of the dimmer and is drawn from top to bottom
  u8g2.setFontMode(1);                                                                                    // Change font draw mode for transparent
  u8g2.setDrawColor(2);                                                                                   // Change font collor to XOR mode
  u8g2.drawStr((x + -3), (y + 5), number.c_str());                                                        // Draw number
  u8g2.setDrawColor(1);                                                                                   // Change collor to white
  u8g2.drawCircle(x, y, 7, U8G2_DRAW_ALL);                                                                // Draw circle 14x14 pixel
}




// main function to draw main page with all sensor data and dimmer and pwm state
void printOLED(byte y) {
  u8g2_uint_t x;
  u8g2.firstPage();
  do {
    //===============================PWM============================================
    u8g2.drawHLine(0, (y + -64), 128);
    u8g2.setFont(u8g2_font_5x7_tf);   // set the target font
    u8g2.drawStr(0, (y + -50), "PWM:");
    drawDimmerDisk(37, (y - 55), "1", pwm1State);
    drawDimmerDisk(55, (y - 55), "2", pwm2State);
    drawDimmerDisk(73, (y - 55), "3", pwm3State);
    drawDimmerDisk(91, (y - 55), "4", pwm4State);
    //==============================================================================

    //===============================Relay==========================================
    u8g2.drawHLine(0, (y + -46), 128);
    u8g2.setFont(u8g2_font_5x7_tf);   // set the target font
    u8g2.drawStr(0, (y + -35), "Relay:");
    drawSwitchDisk(37, (y - 37), "1", relay1State);
    drawSwitchDisk(55, (y - 37), "2", relay2State);
    drawSwitchDisk(73, (y - 37), "3", relay3State);
    drawSwitchDisk(91, (y - 37), "4", relay4State);
    //==============================================================================
    
    //===============================mh-z19=========================================
    u8g2.drawHLine(0, (y + -28), 128);
    u8g2.setFont(u8g2_font_5x7_tf);   // set the target font
    u8g2.drawStr(0, (y + -20), "mh-z10:");
    u8g2.setCursor(40, (y + -20));

    if (millis() > 30000) {
      u8g2.print("Co2: ");
      u8g2.print(mhz19_ppm);
      u8g2.print(" ppm");
    } else {
      u8g2.print(30 - (millis() / 1000));
      u8g2.print(" wait...");
    }
    //==============================================================================

    //===============================DHT-22=========================================
    u8g2.drawHLine(0, (y + -18), 128);  // line is higher by 8 pixel of y possition
    u8g2.setFont(u8g2_font_5x8_mr);   // set the target font
    u8g2.drawStr(0, (y + -10), "DHT22:");
    u8g2.setCursor(40, (y + -10));
    u8g2.print("H:");
    u8g2.print(dht22HumidityState);
    u8g2.print("%, T:");
    u8g2.print(dht22TemperatureState);
    u8g2.print("C");
    //==============================================================================

    //=============================ds18b20 moving text==============================
    u8g2.drawHLine(0, (y + -8), 128);  // line is higher by 8 pixel of y possition
    x = (ds18b20_offset + 40);         //+ 40 is ofset of test "ds18b20:"
    do {
      u8g2.setCursor(x, y);
      for (byte i = 0; i < number_of_sensors; i++) {
        int bufTemp = (temperatures[i] / 100);
        if (bufTemp > 0) {
          u8g2.setFont(u8g2_font_5x8_tr);   // set the target font
          u8g2.print("T");
          u8g2.print(i + 1);
          u8g2.print(":");
          u8g2.print(temperatures[i] / 100);
          u8g2.print("C ");
        } else {
          u8g2.setFont(u8g2_font_5x8_tr);   // set the target font
          u8g2.print("T");
          u8g2.print(i + 1);
          u8g2.print(":");
          u8g2.print("Err ");
        }
        u8g2.setFont(u8g2_font_5x8_mr);
        u8g2.setFontMode(0);               // Change font draw mode for solid (with fixed background)
        u8g2.drawStr(0, y, "ds18b20:");
      }
      x += ds18b20_width;                  // add the pixel width of the scrolling text
      //==============================================================================



      //=========================Output selector======================================
      if (encoderStepCount != 0) {
        if (encoderStepCount <= -1) {
          encoderStepCount = 8;
        } else if (encoderStepCount >= 9) {
          encoderStepCount = 1;
        }

        switch (encoderStepCount) {
          case 1:  //PWM 1
            u8g2.drawFrame((37 - 7), (y - 55 - 7), 16, 16);
            if (encoderButtonStatus) {

              encoderButtonStatus = 0; //reset click status
            }
            break;
          case 2:  //PWM 2
            u8g2.drawFrame((55 - 7), (y - 55 - 7), 16, 16);
            if (encoderButtonStatus) {

              encoderButtonStatus = 0; //reset click status
            }
            break;
          case 3:  //PWM 3
            u8g2.drawFrame((73 - 7), (y - 55 - 7), 16, 16);
            if (encoderButtonStatus) {

              encoderButtonStatus = 0; //reset click status
            }
            break;
          case 4:  //PWM 4
            u8g2.drawFrame((91 - 7), (y - 55 - 7), 16, 16);
            if (encoderButtonStatus) {

              encoderButtonStatus = 0; //reset click status
            }
            break;
          case 5:  //Relay 1
            u8g2.drawFrame((37 - 7), (y - 37 - 7), 16, 16);
            if (encoderButtonStatus) {
              relay1State = !relay1State;
              encoderButtonStatus = 0; //reset click status
            }
            break;
          case 6:  //Relay 2
            u8g2.drawFrame((55 - 7), (y - 37 - 7), 16, 16);
            if (encoderButtonStatus) {
              relay2State = !relay2State;
              encoderButtonStatus = 0; //reset click status
            }
            break;
          case 7:  //Relay 3
            u8g2.drawFrame((73 - 7), (y - 37 - 7), 16, 16);
            if (encoderButtonStatus) {
              relay3State = !relay3State;
              encoderButtonStatus = 0; //reset click status
            }
            break;
          case 8:  //Relay 4
            u8g2.drawFrame((91 - 7), (y - 37 - 7), 16, 16);
            if (encoderButtonStatus) {
              relay4State = !relay4State;
              encoderButtonStatus = 0; //reset click status
            }
            break;
        }
      }
      //==============================================================================


      //===============================Water Overflow message=========================
      if (eStopLast) {
        u8g2.setDrawColor(0);
        u8g2.drawBox(25, 21, 78, 38);
        u8g2.setDrawColor(1);
        u8g2.drawFrame(24, 20, 80, 40);
        u8g2.setFont(u8g2_font_5x7_tf);
        u8g2.drawStr(50, 35, "Water");
        u8g2.drawStr(34, 45, "Overflow!!!");
        //u8g2.drawXBM( 0, 0, canabis_imageWidth, canabis_imageHeight, canabis);
      }
      //==============================================================================


    } while ( x < u8g2.getDisplayWidth());   // draw again until the complete display is filled

  } while ( u8g2.nextPage() );

  ds18b20_offset -= 1;            // scroll by one pixel
  if ( (u8g2_uint_t)ds18b20_offset < (u8g2_uint_t) - ds18b20_width )
    ds18b20_offset = 0;             // start over again
  //==============================================================================

}


//======================Encoder Functions==========================================

// Encoder pin & interrupt setup
void encoderInit() {
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(encoderPinButton, INPUT_PULLUP);
  attachInterrupt(encoderPinA, encoderStepA, RISING);
  attachInterrupt(encoderPinB, encoderStepB, RISING);
  attachInterrupt(encoderPinButton, encoderButton, FALLING);
}

// Function return position of encoder
int encoderGetPos(int resetTime) {
  int  _encoderStepCount = encoderStepCount;
  if (_encoderStepCount != encoderPreviousCount && millis() > (encoderFilterlastMillis + 200)) { //150 milisecounds for protect
    encoderFilterlastMillis = millis();
    encoderPreviousCount = _encoderStepCount;
    attachInterrupt(encoderPinA, encoderStepA, RISING);
    attachInterrupt(encoderPinB, encoderStepB, RISING);
  }
  if ((encoderFilterlastMillis + resetTime) < millis() && (resetTime != 0)) {
    encoderStepCount = 0;
  }
  return encoderStepCount;
}

//interrupt handler A
void encoderStepA() {
  if (digitalRead(encoderPinB) != digitalRead(encoderPinA)) {
    encoderStepCount --;
  } else {
    encoderStepCount ++;
  }
  detachInterrupt(encoderPinA);
  detachInterrupt(encoderPinB);
}

//interrupt handler B
void encoderStepB() {
  if (digitalRead(encoderPinA) == digitalRead(encoderPinB)) {
    encoderStepCount --;
  } else {
    encoderStepCount ++;
  }
  detachInterrupt(encoderPinA);
  detachInterrupt(encoderPinB);
}

//interupt for button
void encoderButton() {
  encoderButtonStatus = 1;
}



//===============================================================================================



//==========================Get value of Co2 in PPM==============================================
word getCo2ppm() {
  zunoSendReport(11);
  byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  int newValue;
  int responseHigh;
  int responseLow;
  unsigned char response[9];
  Serial1.write(cmd, 9);
  memset(response, 0, 9);
  Serial1.readBytes(response, 9);
  int i;
  byte crc = 0;
  for (i = 1; i < 8; i++) crc += response[i];
  crc = 255 - crc;
  crc++;
  if ( !(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc) ) {
    Serial.println("CRC error: " + String(crc) + " / " + String(response[8]));
  } else {
    responseHigh = (int) response[2];
    responseLow = (int) response[3];
    newValue = (256 * responseHigh) + responseLow;
    return newValue;
  }

}
//===============================================================================================



//=========================Get values from DHT-22================================================
void getDHT22() {
  dht22TemperatureStateValueBuf = dht22.readTemperature();
  if (isnan(dht22TemperatureStateValueBuf)) {
    Serial.println("DHT-22 Error № ");
    dht22errHolder++;
    Serial.println(dht22errHolder);
    if (dht22errHolder > dht22errLimit) {
    }
  } else {
    zunoSendReport(9 + 10);
    dht22TemperatureState = dht22TemperatureStateValueBuf * 10;
    dht22TemperatureStateValue = dht22TemperatureStateValueBuf;
    dht22HumidityState = dht22.readHumidity();
    dht22errHolder = 0;
  }
}
//===============================================================================================


//======================Geting fresh data from water overflow sensor=============================
void waterOverflowSensor() {
  if (eStopLast) {                          // when eStop 1
    eStopLast = !digitalRead(interruptPin); // check interuptPin state
    if (eStop != eStopLast) {               // if eStop change state
      eStop = eStopLast;                    // remember status of eStop in eStopLast, this is necessary so that this block is reproduced only 1 time
      zunoSendReport(12 + 0);               // send report
      if (eStop) {                          // we need action only when state of eStop 1
        Serial.println("STOP");             // print in terminal STOP
      }
    }
  }
  Serial.print("waterOverflowSensor");
  Serial.println(eStopLast);
}
//===============================================================================================

//======================time and date====================================================
void myUpdateTimeHandler(){
    Serial.print("*** Time updated:");
    Serial.println((uint32_t)zunoGetTimeStamp());
}

void timeInit(){
  zunoAttachSysHandler(ZUNO_HANDLER_NOTIFICATON_TIME_STAMP, 0, (void*) &myUpdateTimeHandler);
}


//======================Getter & Setter =========================================================
// Getters and setters

static void relay1Setter(byte value) {
  relay1State = value;
}

static byte relay1Getter() {
  return relay1State;
}

static void relay2Setter(byte value) {
  relay2State = value;
}

static byte relay2Getter() {
  return relay2State;
}

static void relay3Setter(byte value) {
  relay3State = value;
}

static byte relay3Getter() {
  return relay3State;
}

static void relay4Setter(byte value) {
  relay4State = value;
}

static byte relay4Getter() {
  return relay4State;
}

static void pwm1Setter(byte value) {
  pwm1State = value;
}

static byte pwm1Getter() {
  return pwm1State;
}

static void pwm2Setter(byte value) {
  pwm2State = value;
}

static byte pwm2Getter() {
  return pwm2State;
}

static void pwm3Setter(byte value) {
  pwm3State = value;
}

static byte pwm3Getter() {
  return pwm3State;
}

static void pwm4Setter(byte value) {
  pwm4State = value;
}

static byte pwm4Getter() {
  return pwm4State;
}

WORD co2levelGetter() {
  return mhz19_ppm;
}

WORD dht22TemperatureGetter() {
  return dht22TemperatureState * 10 ;
}

BYTE dht22HumidityGetter() {
  return dht22HumidityState;
}

void waterOverload() {
  eStopLast = 1;
}

BYTE eStopGetter() {
  return eStop;
}
