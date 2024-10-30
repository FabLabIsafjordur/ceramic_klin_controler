#include <EEPROM.h>
#include <OneWire.h>
#include <LiquidCrystal.h>
#include <AutoPID.h>

#define TYPE_MAX31850 3
#define ONE_WIRE_BUS 2
OneWire ds(2);       // Thermocouple

byte i;
byte present = 0;
byte data[12];
byte addr[8];

bool PID = true;     // Use binary control (false) or PID control (true)

double readtemp;     // Read temp from sensor
double settemp;
int inttemp;         // Converted reading to int
int intset;          // User-defined setpoint temp
int setdiff = 30;    // Difference from setpoint

int mintemp = 20;    // Lower setpoint limit
int maxtemp = 1250;  // Upper setpoint limit

uint8_t state = 0;   // 0: Ramp to 300, 1: Hold 300, 2: Ramp to 600, 3: Hold 600, 4: Ramp to user setpoint, 5: Hold at user setpoint
unsigned long lastTempUpdate;
unsigned long holdStartTime = 0;

#define TEMP_READ_DELAY 1000
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 20
#define KI 0.3
#define KD 0

double outputVal = 255;  // PWM output after first ramp-up

AutoPID myPID(&readtemp, &settemp, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

// LCD pin to Arduino
const int pin_RS = 8; 
const int pin_EN = 9; 
const int pin_d4 = 4; 
const int pin_d5 = 5; 
const int pin_d6 = 6; 
const int pin_d7 = 7; 
const int pin_BL = 10; 

LiquidCrystal lcd(pin_RS, pin_EN, pin_d4, pin_d5, pin_d6, pin_d7);

void setup() {
  pinMode(12, OUTPUT); // GREEN LED
  pinMode(11, OUTPUT); // RED LED
  pinMode(3, OUTPUT);  // SSR
  
  Serial.begin(9600);

  // Initialize display and sensor
  lcd.begin(16, 2);
  lcd.setCursor(0,0);
  lcd.print("Initializing...");

  if (!ds.search(addr)) {
    lcd.clear();
    lcd.print("No sensor found");
    delay(10000);
  }

  lcd.clear();
  lcd.print("Sensor found");

  delay(3000);
  lcd.clear();

  // Read intset from EEPROM
  cli();
  byte byte1 = EEPROM.read(0);
  byte byte2 = EEPROM.read(1);
  intset = (byte1 << 8) + byte2;
  settemp = intset;
  sei();

  while (!updateTemp()) {}
  
  myPID.setBangBang(setdiff);
  myPID.setTimeStep(4000);
}

bool updateTemp() {
  if((millis() - lastTempUpdate) > TEMP_READ_DELAY){
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 0);
    delay(1000);
    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);

    for (i = 0; i < 9; i++) {  
      data[i] = ds.read();
    }

    int16_t raw = (data[1] << 8) | data[0];
    readtemp = (float)raw / 16.0;
    return true;
  }
  return false;
}

void loop() {
  updateTemp();

  // Handle temperature stages
  switch (state) {
    case 0: // Ramp to 300°C
      settemp = 300;
      if (readtemp >= settemp) {
        state = 1;
        holdStartTime = millis();
      }
      break;

    case 1: // Hold at 300°C for 4 hours
      if (millis() - holdStartTime >= 4 * 3600000) {
        state = 2;
      }
      break;

    case 2: // Ramp to 600°C
      settemp = 600;
      if (readtemp >= settemp) {
        state = 3;
        holdStartTime = millis();
      }
      break;

    case 3: // Hold at 600°C for 3 hours
      if (millis() - holdStartTime >= 3 * 3600000) {
        state = 4;
      }
      break;

    case 4: // Ramp to user-defined setpoint
      settemp = intset;
      if (readtemp >= settemp) {
        state = 5;
        digitalWrite(3, LOW); // Turn off SSR
      }
      break;
  }

  // Control heating element
  if (PID) {
    myPID.run();
    analogWrite(3, outputVal);
  } else {
    digitalWrite(3, (readtemp < settemp) ? HIGH : LOW);
  }

  // Update LCD
  lcd.setCursor(0,0);
  lcd.print("Set: ");
  lcd.print(intset);
  lcd.print("C");

  lcd.setCursor(0,1);
  lcd.print("Current: ");
  lcd.print(readtemp);
  lcd.print("C");

  delay(1000);
}
