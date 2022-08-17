/**
 * Big thanks to Mitch Altman, Limor Fried, Kevin Timmerman, Damien Good, KA1KJZ, Ken Shirriff And Fabriel Staples 
 * for working on TV-B-Gone firmware and porting it to Arduino.
 *  
 * This code is written for Universal IR Blaster and not a 'Arduino TV-B-Gone', only parts of code from Arduino TV-B-Gone were taken
 * to use original compression and decompression of ircodes from program memory. If any other firmware is used on Universal IR Blaster or 
 * vice versa damage of the hardware might occur!
 * 
 * Check License and Changes.txt 
 *
 */

//pin definitions but some of them are not used so double check direct register manipulation in code, all fixed
#define IR_SIG PIN_PD3 // IR output, fixed PD3 becuase we use TIMER2
#define USB_IO3 PIN_PD4 // CP2104 GPIO3 open drain
#define PULLDOWN_RESISTOR PIN_PD5 // pulldown resistor, can be used as input or output
#define WAKE PIN_PD2 // Wake button above usb esd ic
#define OPTION_SW1 PIN_PC2 // left button above wake button 
#define OPTION_SW2 PIN_PC3 // middle button above wake button
#define OPTION_SW3 PIN_PC4 // right button above wake button 
#define IPROG A1 // analog input 0-1V
#define STAT_LED PIN_PB5 // stat led
#define LED STAT_LED
#undef LED_BUILTIN
#define LED_BUILTIN STAT_LED

/*
 * If the chip is brand new or this firmware has never been flashed on target uncomment this below
 * After flashing this firmware with that uncommented then comment it out and flash again
 */

//#define FIRST_WRITE

#include "main.h"
#include "WORLD_IR_CODES.h"
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <EEPROM.h>

#include "libraries/PalatisSoftPWM.h"
SOFTPWM_DEFINE_PIN13_CHANNEL(0);  //Configure Arduino pin 13 as PWM channel 0
SOFTPWM_DEFINE_OBJECT(1);

//eeprom addresses and lenghts
int Serial_Number = -1;
#define INFO_SERIAL_NUMBER_EEPROM_ADDR 0
#define INFO_SERIAL_NUMBER_EEPROM_LEN sizeof(Serial_Number)

bool Boot_Debug_EN = false;
#define INFO_DEBUG_ENABLED_EEPROM_ADDR INFO_SERIAL_NUMBER_EEPROM_ADDR + INFO_SERIAL_NUMBER_EEPROM_LEN
#define INFO_DEBUG_ENABLED_EEPROM_LEN sizeof(Boot_Debug_EN)

int Ref_1V1_Voltage = -1L;
#define INFO_1V1_VOLTAGE_EEPROM_ADDR INFO_DEBUG_ENABLED_EEPROM_ADDR + INFO_DEBUG_ENABLED_EEPROM_LEN
#define INFO_1V1_VOLTAGE_EEPROM_LEN sizeof(Ref_1V1_Voltage)

byte Boot_Region;
#define INFO_REGION_EEPROM_ADDR INFO_1V1_VOLTAGE_EEPROM_ADDR + INFO_1V1_VOLTAGE_EEPROM_LEN
#define INFO_REGION_EEPROM_LEN sizeof(Boot_Region)

double HardwareVersion = 0.0;
#define INFO_HARDWARE_VERSION_ADDR INFO_REGION_EEPROM_ADDR + INFO_REGION_EEPROM_LEN
#define INFO_HARDWARE_VERSION_LEN sizeof(HardwareVersion)

unsigned long NumberOfBoots = 0UL;
#define INFO_NUMBER_OF_BOOTS_ADDR INFO_HARDWARE_VERSION_ADDR + INFO_HARDWARE_VERSION_LEN
#define INFO_NUMBER_OF_BOOTS_LEN sizeof(NumberOfBoots)

unsigned long NumberOfStarts = 0UL;
#define INFO_NUMBER_OF_STARTS_ADDR INFO_NUMBER_OF_BOOTS_ADDR + INFO_NUMBER_OF_BOOTS_LEN
#define INFO_NUMBER_OF_STARTS_LEN sizeof(NumberOfStarts)

byte STAT_LED_PWM_Brightness = 255;
#define INFO_STAT_PWM_BRIGHTNESS_ADDR INFO_NUMBER_OF_STARTS_ADDR + INFO_NUMBER_OF_STARTS_LEN
#define INFO_STAT_PWM_BRIGHTNESS_LEN sizeof(STAT_LED_PWM_Brightness)

String USBSerialNumber = "ERR";
#define INFO_USB_SERIAL_NUMBER_ADDR INFO_STAT_PWM_BRIGHTNESS_ADDR + INFO_STAT_PWM_BRIGHTNESS_LEN
#define INFO_USB_SERIAL_NUMBER_LEN 8 //8 characters

#define BAT_LOW_VOLTAGE 3450 //empty battery threshold voltage in mV
#define IPROG_RESISTOR 4990 //value of IPROG resistor used to set charging current in ohms

#define SLEEP_OVERRIDE_BLINK_DELAY_MS 2000 // delay between flashes of stat led in loop while sleep is overriden

//number of slow flashes for stat led
#define FLASH_SLOW_WARNING 1
#define FLASH_SLOW_NO_CODES 3

//number of quick flashes for stat led
#define FLASH_QUICK_REGION_EU 1
#define FLASH_QUICK_REGION_NA 2
#define FLASH_QUICK_SEQUENCE_END 4

#define DEVELOPMENT 0 // if 1 then just timings are printed out and no code is actually transmitted
//#define DISABLE_DEBUG_SAVE_SPACE // uncomment this if you want to disable few debug messages if youre low on flash
#define FIRMWARE_VERSION_STR "1.5.3" // string firmware version 
#define HARDWARE_VERSION_CMP 0.2 // double hardware version x.x thats checked againt value stored in eeprom
#define SERIAL_BAUD_RATE 1000000

#define SizeOfTimingPairsBuffer 512 // max value of numpairs in ircode * 2 because we have 2 values per pair

void xmitCodeElement(uint16_t ontime, uint16_t offtime, uint8_t PWM_code );
void quickflashLEDx( uint8_t x );
void delay_ten_us(uint16_t us);
void quickflashLED( void );
uint8_t read_bits(uint8_t count);

extern const IrCode* const NApowerCodes[] PROGMEM;
extern const IrCode* const EUpowerCodes[] PROGMEM;
extern const IrCode* const User1Codes[] PROGMEM;
extern const IrCode* const User2Codes[] PROGMEM;
extern const IrCode* const User3Codes[] PROGMEM;
extern uint8_t num_NAcodes, num_EUcodes, num_User1Codes, num_User2Codes, num_User3Codes;

volatile uint16_t TimingPairsBuffer[256][2];

byte SelectedUser = 0;

uint8_t bitsleft_r = 0;
uint8_t bits_r=0;
PGM_P code_ptr;

uint16_t ontime, offtime;
uint8_t i,num_codes;
bool pwmCode = false;

unsigned long previousMillis = 0;  

byte Region;

bool Debug_EN = false;
bool Debug_Override = false;

bool SleepOverride = false;

bool TCommand = false;
bool DCommand = false;
bool ICommand = false;
bool QCommand = false;
bool NCommand = false;
bool PCommand = false;

int NumOfCodes = -1;

bool Low_Battery = false;

unsigned long TransmissionTime = 0;

/**
 Init everything
*/
void setup() {
  ReadFromEEPROM();
  
  //if hardware version defined in software does not match hardware version stored in eeprom then do a WDT reset
  if(HardwareVersion != HARDWARE_VERSION_CMP) { wdt_enable(WDTO_2S); while(1); } // if hardware version does not match restart to prevent damage to hardware
  
  EEPROM.put(INFO_NUMBER_OF_BOOTS_ADDR, ++NumberOfBoots);

  digitalWrite(IR_SIG, LOW); pinMode(IR_SIG, OUTPUT);
  pinMode(USB_IO3, INPUT_PULLUP);
  pinMode(OPTION_SW1, INPUT_PULLUP);
  pinMode(OPTION_SW2, INPUT_PULLUP);
  pinMode(OPTION_SW3, INPUT_PULLUP);
  pinMode(WAKE, INPUT_PULLUP);
  digitalWrite(STAT_LED, LOW); pinMode(STAT_LED, OUTPUT); 

  //10us and 100uS pulse on pulldown_resistor
  digitalWrite(PULLDOWN_RESISTOR, LOW); pinMode(PULLDOWN_RESISTOR, OUTPUT);
  PORTD |= _BV(PORTD5); 
  delay_ten_us(1);
  PORTD &= ~_BV(PORTD5);
  PORTD |= _BV(PORTD5); 
  delay_ten_us(100);
  PORTD &= ~_BV(PORTD5);
  digitalWrite(PULLDOWN_RESISTOR, LOW); pinMode(PULLDOWN_RESISTOR, INPUT);

  //reset timer2, start softpwm and disable wdt
  TCCR2A = 0;
  TCCR2B = 0;
  wdt_disable();
  
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setTimeout(200);

  byte regionSwitchCondition = 0 | (digitalRead(WAKE) << 3) | (digitalRead(OPTION_SW1) << 2) | (digitalRead(OPTION_SW2) << 1) | digitalRead(OPTION_SW3);
  if(regionSwitchCondition == 0)
  {
    if(Boot_Region == NA) EEPROM.update(INFO_REGION_EEPROM_ADDR, EU);
    else if(Boot_Region == EU) EEPROM.update(INFO_REGION_EEPROM_ADDR, NA);
    else
    {
      EEPROM.update(INFO_REGION_EEPROM_ADDR, DEFAULT_REGION);
      Serial.println(F("Error"));
    }
    EEPROM.get(INFO_REGION_EEPROM_ADDR, Boot_Region);
    digitalWrite(STAT_LED, HIGH);
    while(1)
    {
      Serial.println();
      Serial.print(F("Region changed: ")); Serial.println(Boot_Region, DEC);
      Serial.println(F("Please reboot"));
      delay(1000);
    }
  }
  
  PalatisSoftPWM.begin(60);  

  //region from eeprom, we should try not to change boot variables in code unless were reading it from eeprom
  Region = Boot_Region;

  //check if option sw2 is held while booting and if yes then enable debug and set debug override flag to true if not then set debug to state from eeprom
  if(digitalRead(OPTION_SW2) == 0)
  {
    Debug_EN = true;
    Debug_Override = true;
    Serial.println(F("[DBG]: Manual Debug Override Enabled"));
  }
  else
  {
    Debug_EN = Boot_Debug_EN;
  }

  #ifdef DISABLE_DEBUG_SAVE_SPACE
    Serial.println(F("Most of debug messages disabled in firmware to free up flash memory")); 
  #endif

  //select region
  switch(Region)
  {
    case EU: Region = EU; break;
    case NA: Region = NA; break;
    default: Region = DEFAULT_REGION; flashslowLEDx(FLASH_SLOW_WARNING); Serial.print(F("[WARN] Unknown region read from EEPROM, setting region to: ")); 
                                      if(DEFAULT_REGION == EU) Serial.println(F("Europe")); else if(DEFAULT_REGION == NA) Serial.println(F("North America"));
                                      else Serial.println(F("Unknown - check main.h DEFAULT_REGION"));
                                      break;
  }
  
  SelectedUser = 0; // default Region codes
  NumOfCodes = Region ? num_EUcodes : num_NAcodes;

  //if cp2104 gpio3 is held low while booting then enable sleep override
  SleepOverride = false;
  if(digitalRead(USB_IO3) == 0)
  {
    SleepOverride = true;
    if(Debug_EN) Serial.println(F("[DBG]: Manual Sleep Override Via USB Enabled"));
  }

  //if option sw1 is held while booting then enable sleep override
  if(digitalRead(OPTION_SW1) == 0)
  {
    SleepOverride = true;
    if(Debug_EN) Serial.println(F("[DBG]: Manual Sleep Override Via [Option 1] Switch Enabled"));
  }

  ShowInfo();

  #ifndef DISABLE_DEBUG_SAVE_SPACE
    if(Debug_EN)
    {
      PrintDBG(F("EU size: "), false); Serial.println(num_EUcodes, DEC);
      PrintDBG(F("NA size: "), false); Serial.println(num_NAcodes, DEC);
      PrintDBG(F("User1 size: "), false); Serial.println(num_User1Codes, DEC);
      PrintDBG(F("User2 size: "), false); Serial.println(num_User2Codes, DEC);
      PrintDBG(F("User3 size: "), false); Serial.println(num_User3Codes, DEC);
      Serial.println();
    }
  #endif
  
  quickflashLEDx(Region ? FLASH_QUICK_REGION_EU : FLASH_QUICK_REGION_NA);
}

/**
  Checks for Serial input and user input
  Transmits code if needed
*/
void loop() {
  digitalWrite(IR_SIG, LOW);
  BatteryCheck();
  SleepNow(SleepOverride);

  //flash every SLEEP_OVERRIDE_BLINK_DELAY_MS if sleep is overriden
  unsigned long currentMillis = millis();
  if ((currentMillis - previousMillis >= SLEEP_OVERRIDE_BLINK_DELAY_MS) && SleepOverride && !Low_Battery) {
    previousMillis = currentMillis;
    //digitalWrite(STAT_LED, HIGH);
    PalatisSoftPWM.set(0, STAT_LED_PWM_Brightness);
  }
  delay(100);
  PalatisSoftPWM.set(0, 0);
  delay(10);
  digitalWrite(STAT_LED, LOW);

  if(SleepOverride)
  { 
    if(Debug_EN) Serial.println(F("[DBG]: Sleep overriden, disabling debug"));
    Debug_EN = false;
  }

  if(Serial.available() > 1)
  {
    PrintDBG(F("Too much serial data received"));
    ShowHelp();
  }

  if(Serial.available() == 1)
  {
    char tmpChar = Serial.read();
    if(Debug_EN) { PrintDBG(F("Serial char rx: "), false); Serial.println(tmpChar); }
    switch(tmpChar)
    {
      case 'D': 
      PrintDBG(F("Dcmd=1"));
      DCommand = true; break;
      
      case 'I':
      PrintDBG(F("Icmd=1"));
      ICommand = true; break;
      
      case 'T':
      PrintDBG(F("Tcmd=1"));
      TCommand = true; break;
      
      case 'Q': 
      PrintDBG(F("Qcmd=1"));
      QCommand = true; break;
      
      case 'N':
      PrintDBG(F("Ncmd=1"));
      NCommand = true; break;

      case 'P':
      PrintDBG(F("Pcmd=1"));
      PCommand = true; break;
      
      case '0': 
      SelectedUser = 0; Serial.println(F("Selected default region codes")); break;
      
      case '1': 
      SelectedUser = 1; Serial.println(F("Selected User1 codes")); break;
      
      case '2': 
      SelectedUser = 2; Serial.println(F("Selected User2 codes")); break;
      
      case '3': 
      SelectedUser = 3; Serial.println(F("Selected User3 codes")); break;
      
      case 'S': Serial.println(F("[S] command can only be used while IR codes are being transmitted")); break;
      default: ShowHelp(); break;
    }
  }

  SerialFlush();

  if(QCommand)
  {
    QCommand = false;
    SendData();
  }

  if(DCommand)
  {
    DCommand = false;
    Serial.print(F("Debug mode in EEPROM is currently ")); PrintLnEnabledDisabled(Boot_Debug_EN);
    if(Boot_Debug_EN)
    {
      Serial.println(F("Disabling debug"));
      Debug_EN = false;
      Boot_Debug_EN = false;
    }
    else
    {
      Serial.println(F("Enabling debug"));
      Debug_EN = true;
      Boot_Debug_EN = true;
    }
    Serial.println(F("Saving debug mode in EEPROM"));
    EEPROM.put(INFO_DEBUG_ENABLED_EEPROM_ADDR, Boot_Debug_EN);
    Serial.println(F("Debug mode saved"));
  }

  if(PCommand)
  {
    PCommand = false;
    Serial.print(F("STAT LED PWM is currently ")); Serial.println(STAT_LED_PWM_Brightness, DEC);
    Serial.println(F("To update enter [1-255], to exit enter anything else: "));
    SerialFlush();
    while(Serial.available() == 0) wdt_reset();
    delay(100);
    int parsedPWM = Serial.parseInt(SKIP_NONE);
    if(parsedPWM >= 1 && parsedPWM <= 255)
    {
      EEPROM.put(INFO_STAT_PWM_BRIGHTNESS_ADDR, (byte)parsedPWM);
    }
    
    EEPROM.get(INFO_STAT_PWM_BRIGHTNESS_ADDR, STAT_LED_PWM_Brightness);
    Serial.print(F("Set to: ")); Serial.println(STAT_LED_PWM_Brightness, DEC);
    Serial.println();
    SerialFlush();
  }

  if(ICommand)
  {
    ICommand = false;
    ShowInfo();
  }

  if(NCommand)
  {
    PrintDBG(F("Ncmd==1, Tcmd=1"));
    TCommand = true;
  }

  if (digitalRead(WAKE) == 0 || TCommand) 
  {
    Debug_EN = Debug_Override ? true : Boot_Debug_EN;

    if(!TCommand)
    {
      byte optionSW = 0 | (digitalRead(OPTION_SW1) << 2) | (digitalRead(OPTION_SW2) << 1) | digitalRead(OPTION_SW3);
      if(Debug_EN) { Serial.print(F("[DBG]: Selecting user codes, optionSW = 0b")); Serial.println(optionSW, BIN); }
      switch(optionSW)
      {
        case 0b011: SelectedUser = 1; Serial.println(F("[Option 1] User1 selected")); break;
        case 0b101: SelectedUser = 2; Serial.println(F("[Option 2] User2 selected")); break;
        case 0b110: SelectedUser = 3; Serial.println(F("[Option 3] User3 selected")); break;
        case 0b111: SelectedUser = 0; Serial.println(F("No option selected, Region codes selected")); break;
        default: SelectedUser = 0; flashslowLEDx(FLASH_SLOW_WARNING); Serial.println(F("Unconfigured option, Region codes selected and aborting transmission..")); return; break;
      }
    }
    else
    {
      Serial.println(F("T command has been issued, only [S] and [N] commands will be accepted while transmitting, simulating WAKE button press"));
      Serial.print(F("Using already selected codes: ")); Serial.println(SelectedUser, DEC);
    }
    TCommand = false;

    delay(500);
    EEPROM.put(INFO_NUMBER_OF_STARTS_ADDR, ++NumberOfStarts);
    Serial.println(F("Starting transmission of all codes"));
    SendAllCodes();
    Serial.println(F("Sending complete"));
  }
  if(!SleepOverride) Serial.println();
}

/**
  Reads information from eeprom
  Serial Number
  Reference 1V1 voltage
  Boot Region
  Boot Debug Enable
  Hardware Version
  Number Of Boots
  Number Of Starts
*/
void ReadFromEEPROM(){
#ifdef FIRST_WRITE
  EEPROM.put(INFO_SERIAL_NUMBER_EEPROM_ADDR, 0);
  EEPROM.put(INFO_1V1_VOLTAGE_EEPROM_ADDR, 1082);
  EEPROM.put(INFO_REGION_EEPROM_ADDR, DEFAULT_REGION);
  
  EEPROM.put(INFO_DEBUG_ENABLED_EEPROM_ADDR, false);
  EEPROM.put(INFO_HARDWARE_VERSION_ADDR, 0.2);
  EEPROM.put(INFO_NUMBER_OF_BOOTS_ADDR, 0UL);
  EEPROM.put(INFO_NUMBER_OF_STARTS_ADDR, 0UL);
  EEPROM.put(INFO_STAT_PWM_BRIGHTNESS_ADDR, 25);
  WriteUSBSerialToEEPROM("00000000"); // must be 8 chars if chip is brand new, replace with your own value
#endif
  EEPROM.get(INFO_SERIAL_NUMBER_EEPROM_ADDR, Serial_Number);
  EEPROM.get(INFO_1V1_VOLTAGE_EEPROM_ADDR, Ref_1V1_Voltage);
  EEPROM.get(INFO_REGION_EEPROM_ADDR, Boot_Region);
  
  EEPROM.get(INFO_DEBUG_ENABLED_EEPROM_ADDR, Boot_Debug_EN);
  EEPROM.get(INFO_HARDWARE_VERSION_ADDR, HardwareVersion);
  EEPROM.get(INFO_NUMBER_OF_BOOTS_ADDR, NumberOfBoots);
  EEPROM.get(INFO_NUMBER_OF_STARTS_ADDR, NumberOfStarts);
  EEPROM.get(INFO_STAT_PWM_BRIGHTNESS_ADDR, STAT_LED_PWM_Brightness);
  USBSerialNumber = ReadUSBSerialFromEEPROM();
}

/**
  Measures and samples newest data and prints it to Serial in JSON format
*/
void SendData()
{
  Serial.println(F("Measuring and sending newest data"));
  analogReference(INTERNAL);
  analogRead(A1);
  delay(10); // wait for 1.1V reference to stabilise
  long U_IPROG = MeasureIPROG();
  long I_CHG = MeasureChargeCurrent();
  analogReference(DEFAULT);
  analogRead(A1); 
  delay(10); // wait for default reference to stabilis
  long U_AVcc = MeasureAVCC();
  byte adcsra = ADCSRA;
  byte admux = ADMUX;
  byte prr = PRR;
  byte portB = PORTB;
  byte ddrB = DDRB;
  byte pinB = PINB;
  byte portC = PORTC;
  byte ddrC = DDRC;
  byte pinC = PINC;
  byte portD = PORTD;
  byte ddrD = DDRD;
  byte pinD = PIND;
  Serial.println();
  
  Serial.print(F("{\"U_IPROG\":")); Serial.print(U_IPROG, DEC); 
  Serial.print(F(",\"I_CHG\":")); Serial.print(I_CHG, DEC); 
  Serial.print(F(",\"U_AVcc\":")); Serial.print(U_AVcc, DEC); 
  Serial.print(F(",\"LOW_BAT\":")); Serial.print(Low_Battery, DEC);
  
  Serial.print(F(",\"ADCSRA\":")); Serial.print(adcsra, DEC);
  Serial.print(F(",\"ADMUX\":")); Serial.print(admux, DEC);  
  Serial.print(F(",\"PRR\":")); Serial.print(prr, DEC);  
  
  Serial.print(F(",\"PORTB\":")); Serial.print(portB, DEC);  
  Serial.print(F(",\"DDRB\":")); Serial.print(ddrB, DEC);  
  Serial.print(F(",\"PINB\":")); Serial.print(pinB, DEC); 
  
  Serial.print(F(",\"PORTC\":")); Serial.print(portC, DEC);  
  Serial.print(F(",\"DDRC\":")); Serial.print(ddrC, DEC); 
  Serial.print(F(",\"PINC\":")); Serial.print(pinC, DEC);  
  
  Serial.print(F(",\"PORTD\":")); Serial.print(portD, DEC);  
  Serial.print(F(",\"DDRD\":")); Serial.print(ddrD, DEC);   
  Serial.print(F(",\"PIND\":")); Serial.print(pinD, DEC); 
  
  Serial.print(F(",\"DEBUG_EN\":")); Serial.print(Debug_EN, DEC);
  Serial.print(F(",\"DEBUG_DISABLED_IN_FIRMWARE\":"));
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  Serial.print(0);
  #else
  Serial.print(1);
  #endif
  Serial.print(F(",\"BOOT_DEBUG_EN\":")); Serial.print(Boot_Debug_EN, DEC);
  Serial.print(F(",\"DEBUG_OVERRIDE\":")); Serial.print(Debug_Override, DEC);
  Serial.print(F(",\"SLEEP_OVERRIDE\":")); Serial.print(SleepOverride, DEC);
  
  Serial.print(F(",\"SERIAL_NUMBER\":")); Serial.print(Serial_Number, DEC);
  Serial.print(F(",\"USB_SERIAL_NUM\":\"")); Serial.print(USBSerialNumber);
  Serial.print(F("\",\"REFERENCE_1V1_VOLTAGE\":")); Serial.print(Ref_1V1_Voltage, DEC);
  Serial.print(F(",\"FIRMWARE_VERSION\":\"")); Serial.print(F(FIRMWARE_VERSION_STR));
  Serial.print(F("\",\"HARDWARE_VERSION\":\"")); Serial.print(HardwareVersion);
  Serial.print(F("\",\"NUM_OF_BOOTS\":")); Serial.print(NumberOfBoots, DEC);
  Serial.print(F(",\"NUM_OF_STARTS\":")); Serial.print(NumberOfStarts, DEC);
  Serial.print(F(",\"STAT_PWM\":")); Serial.print(STAT_LED_PWM_Brightness, DEC);
  
  Serial.print(F(",\"REGION\":")); Serial.print(Region, DEC);
  Serial.print(F(",\"BOOT_REGION\":")); Serial.print(Boot_Region, DEC);
  Serial.print(F(",\"SELECTED_USER\":")); Serial.print(SelectedUser, DEC);
  Serial.print(F(",\"NUM_OF_CODES\":")); Serial.print(NumOfCodes, DEC);
  Serial.print(F(",\"NUM_OF_CODES_NA\":")); Serial.print(num_NAcodes, DEC);
  Serial.print(F(",\"NUM_OF_CODES_EU\":")); Serial.print(num_EUcodes, DEC);
  Serial.print(F(",\"NUM_OF_CODES_USER1\":")); Serial.print(num_User1Codes, DEC);
  Serial.print(F(",\"NUM_OF_CODES_USER2\":")); Serial.print(num_User2Codes, DEC);
  Serial.print(F(",\"NUM_OF_CODES_USER3\":")); Serial.print(num_User3Codes, DEC);

  Serial.print(F(",\"T_CMD\":")); Serial.print(TCommand, DEC);
  Serial.print(F(",\"D_CMD\":")); Serial.print(DCommand, DEC);
  Serial.print(F(",\"I_CMD\":")); Serial.print(ICommand, DEC);
  Serial.print(F(",\"Q_CMD\":")); Serial.print(QCommand, DEC);
  Serial.print(F(",\"N_CMD\":")); Serial.print(NCommand, DEC);
  Serial.print(F(",\"SERIAL_AVAILABLE\":")); Serial.print(Serial.available(), DEC);
  Serial.print(F(",\"MILLIS\":")); Serial.print(millis(), DEC);
  Serial.println(F("}"));
  
  Serial.println();
  Serial.println(F("Newest measured data has been sent"));
}

/**
  Prints "[DBG]: " and text passed and new line if debug is enabled

  @param text Text to be printed
*/
void PrintDBG(const __FlashStringHelper* text)
{
  PrintDBG(text, true);
}

/**
  Prints "[DBG]: " and text passed if debug is enabled

  @param text Text to be printed
  @param newLine Adds new line after printing
*/
void PrintDBG(const __FlashStringHelper* text, bool newLine)
{
  if(Debug_EN)
  {
    Serial.print(F("[DBG]: "));
    Serial.print(text);
    if(newLine) Serial.println();
  }
}

/**
  Resets WDT and prints "WDT RST" to debug output
*/
void Watchdog_Reset_DBG()
{
  wdt_reset();
  PrintDBG(F("WDT RST"));
}

/**
  Prints help message to Serial
*/
void ShowHelp()
{
  Serial.println(F("Unknown command, please use 'I' command for more info"));
}

/**
  Flushes serial receive buffer
*/
void SerialFlush(){
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("Flushing serial rx buff"));
  #endif
  while(Serial.available() > 0) {
    Serial.read();
  }
} 

/**
  Sends all IR codes that are currently selected by SelectedUser.
  If N command is specified it will wait before transmitting every code
*/
void SendAllCodes() 
{
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("Sending all codes"));
  #endif
  bool endingEarly = false; //will be set to true if the user presses the button during code-sending 

  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("WDTO_1S EN"));
  #endif
  wdt_enable(WDTO_1S); //more protection for ir diodes so that they dont get stuck on and burn off..
  Watchdog_Reset_DBG();

  #ifndef DISABLE_DEBUG_SAVE_SPACE
  if(Debug_EN) { PrintDBG(F("SelectedUser: "), false); Serial.println(SelectedUser, DEC); }
  #endif

  //set num_codes based on selected user, if selected user is 0 or unknown then select default region numofcodes
  switch(SelectedUser)
  {
    case 1: num_codes = num_User1Codes; Serial.print(F("User1")); break;
    case 2: num_codes = num_User2Codes; Serial.print(F("User2")); break;
    case 3: num_codes = num_User3Codes; Serial.print(F("User3")); break;
    case 0: 
    default: num_codes = Region ? num_EUcodes : num_NAcodes; Serial.print(F("Default region")); break;
  }
  Serial.println(F(" codes selected"));

  #ifndef DISABLE_DEBUG_SAVE_SPACE
  if(Debug_EN) { PrintDBG(F("num_codes: "), false); Serial.println(num_codes, DEC); }
  #endif
  if(num_codes < 1)
  {
    Serial.println(F("[ERR] There are no entries in currently selected codes"));
    flashslowLEDx(FLASH_SLOW_NO_CODES);
    return;
  }

  if(NCommand)
  {
    Serial.println(F("[N] command has been issued and will be requred before sending every IR code"));
  }

  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("Start ir code tx loop"));
  #endif
  // for every POWER code in our collection
  for (i=0 ; i<num_codes; i++) 
  {
    if(NCommand)
    {
      repeatNLoop:
      Watchdog_Reset_DBG();
      Serial.println();
      Serial.print(F("Waiting for next [N] command to continue to next code. Current ID: "));
      Serial.print(i); Serial.print(F(" of ")); Serial.println(num_codes);
      Serial.println(F("To skip to specific ID enter the ID as positive integer:"));
      Serial.println();
      repeatNLoopWrongID:
      Watchdog_Reset_DBG();
      SerialFlush();
      
      PrintDBG(F("Wait for serial in"));
      while(Serial.available() == 0) wdt_reset();
      delay(100);
      #ifndef DISABLE_DEBUG_SAVE_SPACE
      PrintDBG(F("Serial rx buff check"));
      #endif
      int availableCount = Serial.available();
      int parsedID = 0;
      if(availableCount > 1) // parsing ID
      {
         tryParseID:
         parsedID = Serial.parseInt(SKIP_NONE);
         if(parsedID <= 0) 
         { 
           Serial.println();
           if(availableCount == 1) Serial.println(F("Only [N],[S] command or ID as positive integer can be accepted"));
           else Serial.println(F("[ERR]: Expected ID as positive integer")); 
           goto repeatNLoop; 
         }
         if(parsedID > num_codes) { Serial.print(F("[ERR]: Max ID is ")); Serial.println(num_codes); goto repeatNLoopWrongID; }
         i = parsedID-1;
         Serial.print(F("ID set to: ")); Serial.println(i+1);
         goto transmitCode;
      }

      if(availableCount == 1)
      {
        char tempReadNLoop = Serial.peek();
        #ifndef DISABLE_DEBUG_SAVE_SPACE
        PrintDBG(F("Serial rx char check"));
        #endif
        if(tempReadNLoop == 'S')
        {
          endingEarly = true;
          Serial.println(F("[S] issued. Breaking transmission loop")); break;
        }
        else if(tempReadNLoop == 'N') goto transmitCode;
        else goto tryParseID;
      }
    }

    transmitCode:
    if(Debug_EN) Serial.println(); 
    
    Serial.print(F("Transmitting code ")); Serial.print(i+1, DEC); Serial.print(F(" of ")); Serial.println(num_codes);
    PGM_P data_ptr;
    
    Watchdog_Reset_DBG();
  
    if(Debug_EN) { Serial.print(F("Selecting codes data pointer for user: ")); Serial.println(SelectedUser, DEC); }
    // set data pointer for selected codes
    switch(SelectedUser)
    {
      case 1: data_ptr = (PGM_P)pgm_read_word(User1Codes+i); break;
      case 2: data_ptr = (PGM_P)pgm_read_word(User2Codes+i); break;
      case 3: data_ptr = (PGM_P)pgm_read_word(User3Codes+i); break;
      case 0: 
      default: data_ptr = (PGM_P)pgm_read_word((Region ? EUpowerCodes : NApowerCodes)+i); if(SelectedUser != 0) Serial.println(F("Default region codes data pointer selected!")); break;
    }
    
    // Read the carrier frequency from the first byte of code structure
    const uint8_t freq = pgm_read_byte(data_ptr++);
    pwmCode = (freq != 0);
    // set OCR for Timer1 to output this POWER code's carrier frequency
    OCR2A = freq;
    OCR2B = freq / 3; // 33% duty cycle

    //do the reverse of freq_to_timerval to calculate frequency thats going to be sent out
    uint16_t x = (freq+1) * 8; // 8 because timer clk divisor is set to divide by 8

    const uint8_t numpairs = pgm_read_byte(data_ptr++);
    
    const uint8_t bitcompression = pgm_read_byte(data_ptr++);
    if(Debug_EN) 
    {
      PrintDBG(F("Code #"), false);
      Serial.println(i, DEC);
      PrintDBG(F("ROM Addr 0x"), false);
      Serial.println((uint16_t)data_ptr, HEX);
      
      PrintDBG(F("OCR1: "), false);
      Serial.println(freq, DEC);
      PrintDBG(F("Freq: "), false);
      Serial.println(F_CPU/x, DEC);
      
      PrintDBG(F("On/Off pairs: "), false);
      Serial.println(numpairs, DEC);
      PrintDBG(F("Compression: "), false);
      Serial.println(bitcompression, DEC);
    }

    // Get pointer (address in memory) to pulse-times table
    // The address is 16-bits (2 byte, 1 word)
    PGM_P time_ptr = (PGM_P)pgm_read_word(data_ptr);
    data_ptr+=2;
    code_ptr = (PGM_P)pgm_read_word(data_ptr);

    // Transmit all codeElements for this POWER code
    // (a codeElement is an onTime and an offTime)
    // transmitting onTime means pulsing the IR emitters at the carrier
    // frequency for the length of time specified in onTime
    // transmitting offTime means no output from the IR emitters for the
    // length of time specified in offTime

//DEVELOPMENTAL TESTING: 
#if DEVELOPMENT
      bool oldDebugEN = Debug_EN;
      Debug_EN = true;
      PrintDBG(F("      \t     \t[On]\t[Off]\t[*10µS]"));
      // print out all of the pulse pairs
      for (uint8_t k=0; k<numpairs; k++) {
        uint8_t ti;
        ti = (Read_bits(bitcompression)) * 4;
        // read the onTime and offTime from the program memory
        ontime = pgm_read_word(time_ptr+ti);
        offtime = pgm_read_word(time_ptr+ti+2);
          PrintDBG(F("ti = "), false);
          Serial.print(ti>>2, DEC);
          Serial.print(F("\tPair = "));
          Serial.print(ontime, DEC);
          Serial.print(F("\t"));
          Serial.println(offtime, DEC);
      }
      Serial.println();
      Debug_EN = oldDebugEN;
      continue;
#endif

    TransmissionTime = 0;
    PrintDBG(F("Decompressing pairs to RAM"));
    //decompress timing pairs into ram and calculate sum of all timings
    for (uint8_t k=0; k<numpairs; k++) {
      uint16_t ti;
      //decompression takes around 25uS on average per pair
      
      // Read the next 'n' bits as indicated by the compression variable
      // The multiply by 4 because there are 2 timing numbers per pair
      // and each timing number is one word long, so 4 bytes total!
      ti = (Read_bits(bitcompression)) * 4;

      // read the onTime and offTime from the program memory
      ontime = pgm_read_word(time_ptr+ti);  // read word 1 - ontime
      offtime = pgm_read_word(time_ptr+ti+2);  // read word 2 - offtime
      
      // put on and off times in RAM
      TimingPairsBuffer[k][0] = ontime;
      TimingPairsBuffer[k][1] = offtime;

      // add up on and off times into transmissiontime so we can keep track if WDTO_500MS is enough
      TransmissionTime += TimingPairsBuffer[k][0];
      TransmissionTime += TimingPairsBuffer[k][1];

      //we should transmit codes directly from ram in another for loop with interrupts disabled so that
      //timing is right and thus we get more precise off time, if we emmited elemnt right here
      //we would get higher on time than expected due to time requred to decompress timing pair
      //XmitCodeElement(ontime, offtime, (freq!=0)); 
    }
    //Flush remaining bits, so that next code starts
    //with a fresh set of 8 bits.
    bitsleft_r=0;

    Watchdog_Reset_DBG();
    #ifndef DISABLE_DEBUG_SAVE_SPACE
    PrintDBG(F("WDT OFF"));
    #endif
    wdt_disable();

    PrintDBG(F("WDTO_500MS EN, CLI, Start tx ir pairs"));
    //enable wdt in case ir leds get stuck on and disable interrupts while transmitting to get precise timing
    //offtime will still be higher than ideal because of time needed to decompress next pair
    wdt_enable(WDTO_500MS);
    Watchdog_Reset_DBG(); 
    cli();  // disable interrupts
    //pwmCode = false; // uncomment only for testing exact on and off times less than 10miliseconds. (ITS BAD FOR IR LEDS)
    //emminting ir code from ram
    for(int tp = 0; tp < numpairs; tp++)
    {
      XmitCodeElement(TimingPairsBuffer[tp][0], TimingPairsBuffer[tp][1], pwmCode);
    }

    sei(); //enable interrupts
    wdt_reset();
    wdt_disable();
    wdt_enable(WDTO_1S);
    wdt_reset();
    
    TransmissionTime *= 10;

    PrintDBG(F("SEI, WDR, WDT OFF, WDTO_1S EN, WDR"));
    if(Debug_EN) { PrintDBG(F("Pairs tx time: "), false); Serial.print(TransmissionTime, DEC); Serial.println(F("µS"));}
    if(TransmissionTime > 300000UL) 
    { 
      Serial.print(F("[WARN]: IR transmission took ")); Serial.print(TransmissionTime, DEC); Serial.print(F("µS ")); 
      Serial.println(F("of 450mS allowed."));
    }
    

    // visible indication that a code has been output.
    quickflashLED();
    
    // delay 205 milliseconds before transmitting next POWER code
    delay(205);

    if(Serial.available() == 1)
    {
      if(Serial.read() == 'S') 
      {
        Serial.println(F("[S] command has been issued, stopping the loop..")); 
        endingEarly = true;
      }
    }
    else
    {
      SerialFlush();
    }

    if (digitalRead(WAKE) == 0 || endingEarly) 
    {
      Serial.println(F("Stopping transmission of all codes early"));
      #ifndef DISABLE_DEBUG_SAVE_SPACE
      PrintDBG(F("Stopping tx early"));
      #endif
      endingEarly = true;
      Watchdog_Reset_DBG();
      delay(500);
      quickflashLEDx(4);
      //pause for ~1.3 sec to give the user time to release the button so that the code sequence won't immediately start again.
      #ifndef DISABLE_DEBUG_SAVE_SPACE
      PrintDBG(F("Pause ~1.3sec"));
      #endif
      delay(655);
      Watchdog_Reset_DBG();
      delay(655);
      Watchdog_Reset_DBG();
      #ifndef DISABLE_DEBUG_SAVE_SPACE
      PrintDBG(F("Break tx ir code loop"));
      #endif
      break; //exit the POWER code "for" loop
    }
  } //end of POWER code for loop

  if(NCommand)
  {
  #ifndef DISABLE_DEBUG_SAVE_SPACE 
    PrintDBG(F("Ncmd=0"));
  #endif
    NCommand = false;
  }

  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("WDT OFF"));
  #endif
  wdt_disable();
  if (endingEarly==false)
  {
    Serial.println(F("All codes have been transmitted"));
    #ifndef DISABLE_DEBUG_SAVE_SPACE
    PrintDBG(F("Pause ~1.3sec"));
    #endif
    //pause for ~1.3 sec, then flash the visible LED 8 times to indicate that we're done
    delay(1310);
    quickflashLEDx(FLASH_QUICK_SEQUENCE_END);
  }
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("End of tx all ir codes"));
  #endif
} //end of SendAllCodes

/**
  Waits for x tens of microseconds. Can be calibrated in main.h

  @param us Tens of uS to wait
*/
void __attribute__((noinline)) delay_ten_us(uint16_t us) {
  while (us != 0) {
    __builtin_avr_delay_cycles(DELAY_CYCLES_CNT); 
    us--;
  }
}

/**
  Sends IR signal pair with specified on time and off time in tens of uS

  @param ontime Amount of time in tens of uS the IR signal will be 'on'
  @param offtime Amount of time in tens of uS the IR singal will be 'off'
  @param PWM_code Enable PWM during ontime
*/
void XmitCodeElement(uint16_t ontime, uint16_t offtime, uint8_t PWM_code )
{
  // Reset timer2 counter
  TCNT2 = 0;
  if(PWM_code) {
    // Fast PWM, setting top limit, divide by 8
    // Output to pin 3
    TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(WGM22) | _BV(CS21);
  }
  else {
    // However some codes dont use PWM in which case we just turn the IR
    // LED on for the period of time.
    PORTD |= _BV(PORTD3); // SET PD3 HIGH
  }
  // Now we wait, allowing the PWM hardware to pulse out the carrier
  // frequency for the specified 'on' time
  delay_ten_us(ontime);
  
  // Now we have to turn it off so disable the PWM output
  TCCR2A = 0;
  TCCR2B = 0;
  // And make sure that the IR LED is off too (since the PWM may have
  // been stopped while the LED is on!)
  PORTD &= ~_BV(PORTD3); // SET PD3 LOW

  // Now we wait specified offtime to keep output LOW for that amount of time
  delay_ten_us(offtime);
}

/**
  Reads bits from program memory at current location of code_ptr then increments code_ptr by 1
  More than 8 bits cannot be read so do not try!
  @param count Number of bits to read
  @returns Byte containing bits read from code_ptr
*/
uint8_t Read_bits(uint8_t count)
{
  uint8_t i;
  uint8_t tmp=0;

  // we need to read back count bytes
  for (i=0; i<count; i++) {
    // check if the 8-bit buffer we have has run out
    if (bitsleft_r == 0) {
      // in which case we read a new byte in
      bits_r = pgm_read_byte(code_ptr++);
      // and reset the buffer size (8 bites in a byte)
      bitsleft_r = 8;
    }
    // remove one bit
    bitsleft_r--;
    // and shift it off of the end of 'bits_r'
    tmp |= (((bits_r >> (bitsleft_r)) & 1) << (count-1-i));
  }
  // return the selected bits in the LSB part of tmp
  return tmp;
}


/**
  Prints word Enabled or Disabled to Serial

  @param value If true prints Enabled othervise Disabled
  @param newLine Print new line after printing
*/
void PrintEnabledDisabled(bool value, bool newLine)
{
  Serial.print(value ? F("Enabled") : F("Disabled"));
  if(newLine) Serial.println();
}

/**
  Prints word Enabled or Disabled to Serial and prints new line

  @param value If true prints Enabled othervise Disabled
*/
void PrintLnEnabledDisabled(bool value)
{
  PrintEnabledDisabled(value, true);
}

/**
  Prints word Enabled or Disabled to Serial

  @param value If true prints Enabled othervise Disabled
*/
void PrintEnabledDisabled(bool value)
{
  PrintEnabledDisabled(value, false);
}

/**
  Disables debug and prints info menu to Serial
  After printing it restores debug state
*/
void ShowInfo()
{
  ShowInfo(false);
}

/**
  Disables debug and prints info menu to Serial
  After printing it restores Debug state

  @param debug Enables debug while printing
*/
void ShowInfo(bool debug)
{
  bool oldDebug = Debug_EN;
  Debug_EN = debug;
  Serial.println();
  Serial.print(F("[Universal IR Blaster TV-B-Gone - Hardware V")); Serial.print(HardwareVersion);  Serial.print(F(" - Firmware V")); Serial.print(F(FIRMWARE_VERSION_STR));  Serial.println(F(" " __DATE__ " " __TIME__ "]" ));
  Serial.println(F("  - [Board designed and firmware written by]: Djordje Mandic - https://youtube.com/c/ncky - https://github.com/DjordjeMandic"));
  Serial.println(F("  - [Thanks to for original Arduino TV-B-Gone(v1.3) development]: Mitch Altman, Limor Fried, Kevin Timmerman, KA1KJZ, Damien Good, Ken Shirriff And Gabriel Staples"));
  Serial.print(F("  - [Debug Disabled In Firmware]: "));
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  Serial.println(F("False"));
  #else
  Serial.println(F("True"));
  #endif
  Serial.print(F("  - [Serial Number]: ")); Serial.println(Serial_Number, DEC);
  Serial.print(F("  - [USB Serial Number]: ")); Serial.println(USBSerialNumber);
  Serial.print(F("  - [Stored Reference 1.1V]: ")); Serial.print(Ref_1V1_Voltage, DEC); Serial.println(F("mV"));
  Serial.print(F("  - [Boots]: ")); Serial.println(NumberOfBoots, DEC);
  Serial.print(F("  - [Starts]: ")); Serial.println(NumberOfStarts, DEC);
  Serial.print(F("  - [STAT PWM]: ")); Serial.println(STAT_LED_PWM_Brightness, DEC);
  Serial.print(F("  - [Boot Debug Setting]: ")); PrintLnEnabledDisabled(Boot_Debug_EN);
  Serial.print(F("  - [Boot Region Setting]: ")); Serial.println(Boot_Region, DEC);
  Serial.print(F("  - [Debug Override]: ")); PrintLnEnabledDisabled(Debug_Override);
  Serial.print(F("  - [Debug]: ")); PrintLnEnabledDisabled(oldDebug);
  Serial.print(F("  - [Debug during info printing]: ")); PrintLnEnabledDisabled(Debug_EN);
  Serial.print(F("  - [Sleep Override]: ")); PrintLnEnabledDisabled(SleepOverride);
  Serial.print(F("  - [IPROG Voltage]: ")); Serial.print(MeasureIPROG(), DEC); Serial.println(F("mV"));
  Serial.print(F("  - [Charge Curret]: ")); Serial.print(MeasureChargeCurrent(), DEC); Serial.println(F("mA"));
  Serial.print(F("  - [AVcc]: ")); Serial.print(MeasureAVCC(), DEC); Serial.println(F("mV"));
  Serial.print(F("  - [Low Battery]: ")); Serial.println(Low_Battery ? F("Yes") : F("No"));
  Serial.println(F("[IR Codes]:"));
  Serial.print(F("  - [Selected]: ")); 
  switch(SelectedUser)
  {
    case 1: Serial.println(F("User1")); break;
    case 2: Serial.println(F("User2")); break;
    case 3: Serial.println(F("User3")); break;
    case 0:
    default: Serial.println(F("Region")); break;
  }
    Serial.print(F("  - [Region]: ")); if(Region) Serial.println(F("Europe")); else Serial.println(F("North America"));
  Serial.println(F("  - [Number of codes]:"));
    Serial.print(F("    - [Region]: ")); Serial.println(NumOfCodes, DEC);
    Serial.print(F("    - [User1]: ")); Serial.println(num_User1Codes, DEC);
    Serial.print(F("    - [User2]: ")); Serial.println(num_User2Codes, DEC);
    Serial.print(F("    - [User3]: ")); Serial.println(num_User3Codes, DEC);
  Serial.println(F("[Commands]:"));
  Serial.println(F("  - [T] Start IR code transmission"));
  Serial.println(F("  - [S] Stop IR code transmission"));
  Serial.println(F("  - [0] Select region codes"));
  Serial.println(F("  - [1] Select User1 codes"));
  Serial.println(F("  - [2] Select User2 codes"));
  Serial.println(F("  - [3] Select User3 codes"));
  Serial.println(F("  - [N] Execute [T] command and wait for [N] command per IR code"));
  Serial.println(F("  - [D] Debug Enable/Disable"));
  Serial.println(F("  - [Q] Query for json data"));
  Serial.println(F("  - [P] Set STAT LED PWM value"));
  Serial.println(F("  - [I] Print this content"));
  Serial.println();
  Debug_EN = oldDebug;
}


/**
  Flashes STAT led for 30ms
*/
void quickflashLED( void ) {
  //digitalWrite(STAT_LED, HIGH);
  PalatisSoftPWM.set(0, STAT_LED_PWM_Brightness);
  delay(30);
  PalatisSoftPWM.set(0, 0);
  delay(10);
  digitalWrite(STAT_LED, LOW);
}

/**
  Flashes STAT led x amount of times slow with 500ms delay between flashes

  @param num_blinks Amount of times led will flash
*/
void flashslowLEDx( uint8_t num_blinks )
{
  uint8_t i;
  for(i=0;i<num_blinks;i++)
    {
      //digitalWrite(STAT_LED, HIGH);  
      PalatisSoftPWM.set(0, STAT_LED_PWM_Brightness);
      delay(500);
      wdt_reset();
      PalatisSoftPWM.set(0, 0);         
      delay(500);
      digitalWrite(STAT_LED, LOW);
      wdt_reset();
    }
}

/**
  Flashes STAT led x amount of times fast with 250ms delay between flashes

  @param x Amount of times led will flash
*/
void quickflashLEDx( uint8_t x ) {
  quickflashLED();
  while(--x) {
    wdt_reset();
    delay(250);
    quickflashLED();
  }
  wdt_reset();
}

/**
  Turns off adc, attaches interrupt to INT0
  and puts mcu to sleep, after its woken up WakeUpNow() is called and
  adc is restored and INT0 detached.
*/
void SleepNow()
{
  SleepNow(false);
}

/**
  If sleep is overriden just puts IR_SIG low and turns off WDT
  If sleep is not overriden, turns off adc, attaches interrupt to INT0
  and puts mcu to sleep, after its woken up WakeUpNow() is called and
  adc is restored and INT0 detached.

  @param sleepoverride Is sleep overriden or not?
*/
void SleepNow(bool sleepoverride)
{
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("Starting sleep procedure"));
  #endif
  digitalWrite(IR_SIG, LOW);
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  Watchdog_Reset_DBG();
  PrintDBG(F("WDT OFF"));
  #endif
  wdt_disable();
  if(sleepoverride)
  {
    PrintDBG(F("Sleep overriden, aborting sleep"));
    return;
  }
  
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("Sleep mode power down"));
  #endif
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);                    // sleep mode is set here

  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("Sleep EN"));
  #endif
  sleep_enable();                             // enables the sleep bit in the mcucr register

  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("INT0 ON LOW"));
  #endif
  attachInterrupt(0, WakeUpNow, LOW);         // use interrupt 0 (pin 2) and run function
  // wakeUpNow when pin 2 gets LOW
  
  Serial.println(F("Going to sleep in ~500ms..."));
  delay(500);

  byte adcsra_save = ADCSRA;
  ADCSRA = 0; // disable adc
  PRR |= _BV(PRADC); // turn off adc

  sleep_mode();                               // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE ON WAKE
  sleep_disable();                            // first thing after waking, disable sleep

  PRR &= ~_BV(PRADC); // turn on adc
  ADCSRA = adcsra_save; //restore adc
  
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("Woken up, INT0 OFF"));
  #endif
  detachInterrupt(0);

  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("End of sleep procedure"));
  #endif
  Serial.println(F("Waked up from sleep")); 
  Serial.println();
}

/**
  Dummy function executed when mcu is woken up from sleep by WAKE button
*/
void WakeUpNow()
{
  // any needed wakeup code can be placed here
}

/**
  Checks if battery is low by measuring avcc. If it is low stat led will fade down 2 times
*/
void BatteryCheck()
{
  digitalWrite(IR_SIG, LOW);
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("Checking bat"));
  #endif
  long currentAVCC = MeasureAVCC();
  if (Low_Battery) 
  {
    if(!SleepOverride) { Serial.print(F("Battery is low, current AVcc = ")); Serial.print(currentAVCC, DEC); Serial.println(F("mV")); }
    for(int i = 0; i < 2; i++)
    {
      for(byte i = STAT_LED_PWM_Brightness; i > 0; i--)
      {
        PalatisSoftPWM.set(0, i);
        delay(750 / STAT_LED_PWM_Brightness);
      }
      PalatisSoftPWM.set(0, 0);
      delay(250);
      digitalWrite(STAT_LED, LOW);
    }
  }
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  else
  {
    PrintDBG(F("Bat is not low"));
  }
  #endif
}

/**
  Measures IPROG and calculates charge current

  @return the numerical value of charging current in miliamps
*/
long MeasureChargeCurrent()
{
  digitalWrite(IR_SIG, LOW);
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("Measureing charge current"));
  #endif
    long result;
    result = MeasureIPROG();
    result *= 1000L;
    result /= IPROG_RESISTOR;
    if(Debug_EN) { PrintDBG(F("Charge current avg of 5 = "), false); Serial.print(result, DEC); Serial.println(F("mA")); }
    return result;
}

/**
  Measures IPROG voltage against internal 1.1v reference

  @return the numerical value of IPROG voltage in milivolts
*/
long MeasureIPROG()
{
  digitalWrite(IR_SIG, LOW);
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("Measuring IPROG"));
  #endif
  long result = 0;
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("AREF = INTERNAL"));
  #endif
  analogReference(INTERNAL);
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  PrintDBG(F("ADC Reading IPROG"));
  #endif
  analogRead(IPROG);
  delay(10);
  for(int i = 0; i < 5; i++)
  {
    result += analogRead(IPROG);
    delay(2);
  }
  result /= 5;
  if(Debug_EN) { PrintDBG(F("IPROG_adc avg of 5 = "), false); Serial.println(result, DEC); }
  result *= Ref_1V1_Voltage;
  result /= 1024L;
  if(Debug_EN) { PrintDBG(F("Calculated IPROG avg of 5 = "), false); Serial.print(result, DEC); Serial.println(F("mV")); }
  return result;
}


/**
  Measures the INTERNAL Reference voltage and comapres it against
  AVcc voltage to calculate the AVcc
  @return the numerical value of AVcc voltage in milivolts
*/
long MeasureAVCC() 
{
  PrintDBG(F("Measuring AVcc, AREF=DEFAULT, 1.1V sample ADC"));
  digitalWrite(IR_SIG, LOW);
  // Read 1.1V reference against AVcc
  long result = 0;
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(10); // Wait for Vref to settle
  for(int i = 0; i < 5; i++)
  {
    ADCSRA |= _BV(ADSC); // Convert
    while (bit_is_set(ADCSRA,ADSC));
    result += ADCH<<8 | ADCL;
    delay(2);
  }
  result /= 5;
  if(Debug_EN) { PrintDBG(F("1V1 against AVcc avg of 5 = "), false); Serial.println(result, DEC); }
  result = (Ref_1V1_Voltage*1024L) / result;
  if(Debug_EN) { PrintDBG(F("Calculated AVcc avg of 5 = "), false); Serial.print(result, DEC); Serial.println(F("mV")); }
  Low_Battery = result < BAT_LOW_VOLTAGE;
  #ifndef DISABLE_DEBUG_SAVE_SPACE
  if(Debug_EN) { PrintDBG(F("Battery low: "), false); Serial.println(Low_Battery ? F("Yes") : F("No")); }
  #endif
  return result;
}

/**
  Writes USB Serial Number to EEPROM.

  @param strToWrite String thats going to be written, must be INFO_USB_SERIAL_NUMBER_LEN chars
  @return Amount of bytes written. If returned 0 then there are no bytes written or String is too short or too long.
*/ 
byte WriteUSBSerialToEEPROM(const String &strToWrite)
{
  if(strToWrite.length() != INFO_USB_SERIAL_NUMBER_LEN) return 0;
  byte written = 0;
  for (int i = 0; i < INFO_USB_SERIAL_NUMBER_LEN; i++)
  {
    EEPROM.write(INFO_USB_SERIAL_NUMBER_ADDR + i, strToWrite[i]);
    written++;
  }
  return written;
}

/**
  Reads USB Serial Number from EEPROM. 

  @return String containing USB Serial Number
*/ 
String ReadUSBSerialFromEEPROM()
{
  char data[INFO_USB_SERIAL_NUMBER_LEN];
  for (int i = 0; i < INFO_USB_SERIAL_NUMBER_LEN; i++)
  {
    data[i] = EEPROM.read(INFO_USB_SERIAL_NUMBER_ADDR + i);
  }
  return String(data);
}