#include <EEPROM.h>
#include <OneWire.h>
#include <LiquidCrystal.h>
#include <AutoPID.h>

#define TYPE_MAX31850 3
#define ONE_WIRE_BUS 2
OneWire ds(2);       //Thermocouple

byte i;
byte present = 0;
byte data[12];
byte addr[8];

bool PID = true;     //use binary control (false) or PID control (true)

double readtemp;     //read temp from sensor
double settemp;
int inttemp;         //converted reading to int
int intset;          //setpoint temp
int setdiff = 30;         //difference from setpoint


int mintemp = 20;    //lower setpoint limit
int maxtemp = 1250;  //upper setpoint limit

uint8_t state = 2;   //0 ramp up // 1 PID // 2 cool

unsigned long lastTempUpdate;

#define TEMP_READ_DELAY 1000
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 20       //15
#define KI 0.3     //0.002
#define KD 0        //0

double outputVal = 255;         //PWM out after first ramp-up

AutoPID myPID(&readtemp, &settemp, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

//LCD pin to Arduino
const int pin_RS = 8; 
const int pin_EN = 9; 
const int pin_d4 = 4; 
const int pin_d5 = 5; 
const int pin_d6 = 6; 
const int pin_d7 = 7; 
const int pin_BL = 10; 

//custom "Í" letter for display
byte iii[8] ={
  B00010,
  B00100,
  B01110,
  B00100,
  B00100,
  B00100,
  B01110,
  B00000,
};

//custom "ð" letter for display
byte eth[8] ={
  B10100,
  B01000,
  B10100,
  B00010,
  B01111,
  B10001,
  B01110,
  B00000,
};


LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);

void setup() {
  //Set Digital Outputs
  pinMode(12, OUTPUT); //GREEN LED
  pinMode(11, OUTPUT); //RED LED
  pinMode(3, OUTPUT);  //SSR
  
  Serial.begin(9600);

  //Start Display
  lcd.begin(16, 2);
  lcd.createChar(0, iii);
  lcd.createChar(1, eth);
  lcd.setCursor(0,0);
  lcd.print("Hersluofn");
  lcd.setCursor(0,1);
  lcd.print("FabLab ");
  lcd.write(byte(0));
  lcd.print("sa");

  delay(3000);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Leita a");
  lcd.write(byte(1));
  lcd.setCursor(0,1);
  lcd.print("hitanema");

  delay(2000);
  
  if (!ds.search(addr)){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Fann ekki");
    lcd.setCursor(0,1);
    lcd.print("Hitanemann");
    Serial.println("Unable to find address for Device 0");
    delay(10000);
  }

  lcd.clear();
  
//  Serial.println("starting");
//  Serial.print("ROM =");
//  for( i = 0; i < 8; i++) {
//    Serial.write(' ');
//    Serial.print(addr[i], HEX);
//  }
//  if (OneWire::crc8(addr, 7) != addr[7]) {
//     Serial.println("CRC is not valid!");
//     return;
//  }
  lcd.setCursor(0,0);
  lcd.print("Hitanemi");
  lcd.setCursor(0,1);
  lcd.print("fundinn");

  delay(3000);

  lcd.clear();

  //Display Setup
  lcd.setCursor(0,0);
  lcd.print("Mark");
  lcd.setCursor(0,1);
  lcd.print("Raun");

  delay(50);

  //Read intset from EEPROM
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

bool updateTemp() {   //read sensor
  if((millis() - lastTempUpdate) > TEMP_READ_DELAY){
    ds.reset();
    ds.select(addr);
    ds.write(0x44, 0);        // start conversion, without parasite power
    delay(1000);     // maybe 750ms is enough, maybe not
    present = ds.reset();
    ds.select(addr);    
    ds.write(0xBE);         // Read Scratchpad
    for ( i = 0; i < 9; i++) {           // 9 bytes
      data[i] = ds.read();
    }
    int16_t raw = (data[1] << 8) | data[0];
    if (raw & 0x01) {
        Serial.println("**FAULT!**");
        return;
    } else {
      byte cfg = (data[4] & 0x60);
      // at lower res, the low bits are undefined, so let's zero them
      if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
      //// default is 12 bit resolution, 750 ms conversion time
    }
        //convert to celcius
    readtemp = (float)raw / 16.0;
     return true;
    }
  return false;
}

void loop() {
 
// read "up/down" keys to adjust intset and save to EEPROM and "select" key to switch on/off
  int x;
  int tint5 = 300;            //from what intset change in 5°C intervals
  x = analogRead (0);         
  if (x >= 200 && x < 400 && intset > mintemp){
    if(intset >= tint5 + 5)
      intset=intset-5;
    else
      intset--;
  EEPROM.write(0,intset >> 8);
  EEPROM.write(1,intset & 0xFF);
  //return;
  }else if (x >= 60 && x < 200 && intset < maxtemp){    
    if(intset >= tint5)
      intset=intset+5;
    else
      intset++;
  EEPROM.write(0,intset >> 8);
  EEPROM.write(1,intset & 0xFF);
  //return;
  }else if (x >= 600 && x < 800){
    if(state == 0 || state == 1){
      state = 2;      
      digitalWrite(11,LOW);
      digitalWrite(3,LOW);
      delay(500);
    }else{
      state = 0;
      delay(500);
    }
  //return;
  }else
    updateTemp();

//convert temp reading to int
  inttemp = readtemp;

//save intset to double
  settemp = intset;

// check length of temp reading
  int RLen;
  if(inttemp > 9999)
     RLen = 5;
  else if(inttemp > 999)
     RLen = 4;
  else if(inttemp > 99)
    RLen = 3;
  else if(inttemp > 9)
    RLen = 2;
  else if(inttemp > -1)
    RLen = 1;
  else if(inttemp > -10)
    RLen = 2;
  else if(inttemp > -20)
    RLen = 3;
  else
    RLen = 0;

// display readttemp
  if(RLen>0 && RLen <5){
    lcd.setCursor(5,1);
    for (int i=0; i < 4-RLen ;i++)
    lcd.print(" ");
    lcd.print(inttemp);
    lcd.print("\337C");
    digitalWrite(12,HIGH);
  }
  else{
    lcd.setCursor(5,1);
    lcd.print(" ERROR");
    digitalWrite(12,LOW);
    digitalWrite(11,LOW);
    digitalWrite(3,LOW);
  }

//   Serial.print("State: ");
//   Serial.print(state);

   Serial.print(" Temp: ");
   Serial.print(readtemp);

   Serial.print(" Set: ");
   Serial.print(intset);

   Serial.println();


// check length of intset
  int SLen;
  if(intset > 9999)
     SLen = 5;
  else if(intset > 999)
     SLen = 4;
  else if(intset > 99)
    SLen = 3;
  else if(intset > 9)
    SLen = 2;
  else if(intset > -1)
    SLen = 1;
  else
    SLen = 0;

//display intset
  lcd.setCursor(5,0);
  for (int i=0; i < 4-SLen ; i++)
    lcd.print(" ");
  lcd.print(intset);
  lcd.print("\337C");

//setdiff calculation to trigger Ramp Up 
//  setdiff = intset*0.2; 

// Simple binary control
  if(PID == false && state == 0){
    float tol=0.03; //tolerance for switching
    if(readtemp < intset*(1-tol)){
      digitalWrite(11,HIGH);
      digitalWrite(3,HIGH);
    }
    if(readtemp > intset){
      digitalWrite(11,LOW);
      digitalWrite(3,LOW);
    }
  }


//PID Control
  if(PID == true){
    if(state == 0)
    ramp_up();
    else if(state == 1)
    PID_control();
    else
    cool();
  }
    
  delay(50);
}



//Rising temp to setpoint diff from setpoint
void ramp_up(void){
  
  if(readtemp < (intset - setdiff)){
    digitalWrite(3,HIGH);
    lcd.setCursor(12,0);
    lcd.print("UPPH");
    lcd.setCursor(12,1);
    lcd.print("100%");
    for(i=0; i<255; i++){
      analogWrite(11,i);
      delay(5);
    }
  }
  else{
    state = 1;
    Serial.println("Switching to PID mode");
  }
}


// PID Control
void PID_control(void){
  if(readtemp > (intset - 2*setdiff)){
    myPID.run();
    analogWrite(3, outputVal);
    analogWrite(11, outputVal);
    lcd.setCursor(12,0);
    lcd.print(" PID");
    lcd.setCursor(12,1);
    int outpro = outputVal*100/255;
    if(outpro < 10){
      lcd.print("  ");
      lcd.print(outpro);
    }else if(outpro <100){
      lcd.print(" ");
      lcd.print(outpro);
    }else{
      lcd.print(outpro);
    }
    lcd.print("%");
   // Serial.println(outpro);
  }else{
    digitalWrite(3,HIGH);
    state = 0;
    Serial.println("Switching to Ramp Up Mode");
  }
}

void cool(void){
    lcd.setCursor(12,0);
    lcd.print("STOP");
    lcd.setCursor(12,1);
    lcd.print("  0%");
    digitalWrite(11,LOW);
    digitalWrite(3,LOW);
}
