/* This code works with ACS712 current sensor, it permits the calculation   of the signal TRMS
 * Visit www.surtrtech.com for more details
 */

#include   <Filters.h>                      //This library does a massive work check it's .cpp   file

#define ACS_Pin A2                       //Sensor data pin on A0 analog   input

//libraries to be used
#include <Wire.h>
#include "RTClib.h"
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>


//initializing Global Variables
int RedLED_1 = 53;
int RedLED_2 = 49;
int YellowLED_1 = 51;
int YellowLED_2 = 47;

int Relay_1 = 41;
int Relay_2 = 39;

int CurrentSensor_1 = A2;
int CurrentSensor_2 = A3;

const int Num_readings = 500;
long int sensor_value = 0;  

float voltage = 0;
float current = 0;

int days_1 = 0;
int hours_1 = 0;
int minutes_1 = 0;
int seconds_1 = 0;

int days_2 = 0;
int hours_2 = 0;
int minutes_2 = 0;
int seconds_2 = 0;



//Create software serial object to communicate with SIM800L
SoftwareSerial mySerial(43, 45); //SIM800L Tx & Rx 
//Create software serial object to communicate with LCD
LiquidCrystal_I2C lcd(0x27, 16, 4); // Set the LCD address to 0x27 for a 16 chars and 2 line display

RTC_DS3231 rtc; //declaring object for DS3231

char daysOfTheWeek[7][12] = {"sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Sartuday"};


float ACS_Value;                              //Here we keep the raw   data valuess
float testFrequency = 50;                    // test signal frequency   (Hz)
float windowLength = 40.0/testFrequency;     // how long to average the   signal, for statistist



float intercept = 0; // to be adjusted based   on calibration testing
float slope = 0.0752; // to be adjusted based on calibration   testing
                      //Please check the ACS712 Tutorial video by SurtrTech   to see how to get them because it depends on your sensor, or look below


float   Amps_TRMS; // estimated actual current in amps

unsigned long printPeriod   = 1000; // in milliseconds
// Track time in milliseconds since last reading 
unsigned   long previousMillis = 0;

void setup() {
  Serial.begin( 9600 );    //   Start the serial port
  pinMode(ACS_Pin,INPUT);  //Define the pin mode
  pinMode(RedLED_1, OUTPUT);
  pinMode(RedLED_2, OUTPUT);
  pinMode(YellowLED_1, OUTPUT);
  pinMode(YellowLED_2, OUTPUT);
  
  pinMode(Relay_1, OUTPUT);
  pinMode(Relay_2, OUTPUT);
   
  digitalWrite(RedLED_1, HIGH);
  digitalWrite(RedLED_2, HIGH);
  digitalWrite(YellowLED_1, LOW);
  digitalWrite(YellowLED_2, LOW);
  
  lcd.init();
  lcd.backlight();
  
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SYSTEM BOOTING..");
  lcd.setCursor(0,1);
  lcd.print("GSM TEST: ");
  lcd.setCursor(-4,2);
  lcd.print("RTC TEST: ");
  lcd.setCursor(-1,3);
  lcd.print(" Loading...");
  delay(3000);
  /*while(!mySerial.available()){
    //mySerial.println("AT");
    lcd.setCursor(10,1);
    lcd.print("Wait");
    delay(1000);
  }
  while(rtc){
    lcd.setCursor(6,2);
    lcd.print("Wait");
    delay(1000);
  }*/
  lcd.setCursor(10,1);
  lcd.print("OK");
  lcd.setCursor(6,2);
  lcd.print("OK");
  delay(3000);

  lcd.clear();
  lcd.setCursor(5,1);
  lcd.print("READY");
  delay(2000);
  digitalWrite(Relay_2, HIGH);
}

void   loop() {
  RunningStatistics inputStats;                 // create statistics   to look at the raw test signal
  inputStats.setWindowSecs( windowLength );     //Set   the window length
  if(digitalRead(Relay_1) == HIGH){
    digitalWrite(RedLED_1, LOW);
    digitalWrite(YellowLED_1, HIGH);
  }else if(digitalRead(Relay_1) == LOW){
    digitalWrite(RedLED_1, HIGH);
    digitalWrite(YellowLED_1, LOW);
  }

  if(digitalRead(Relay_2) == HIGH){
    digitalWrite(RedLED_2, LOW);
    digitalWrite(YellowLED_2, HIGH);
  }else if(digitalRead(Relay_2) == LOW){
    digitalWrite(RedLED_2, HIGH);
    digitalWrite(YellowLED_2, LOW);
  }
  
  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print("CHARGING POINT");
  lcd.setCursor(0,1);
  lcd.print("Socket 1: ");
  if(digitalRead(Relay_1) == HIGH){
    lcd.setCursor(10,1);
    lcd.print("ON");
  }else{
    lcd.setCursor(10,1);
    lcd.print("OFF");
  }
  lcd.setCursor(-4,2);
  lcd.print("Socket 2: ");
  if(digitalRead(Relay_2) == HIGH){
    lcd.setCursor(6,2);
    lcd.print("ON");
  }else{
    lcd.setCursor(6,2);
    lcd.print("OFF");
  }
  lcd.setCursor(-4,3);
  lcd.print("T1: ");
  lcd.setCursor(4,3);
  lcd.print("T2: ");
  delay(1000);
  
  //digitalWrite(Relay_1, HIGH);
  
  
  while( true ) {   
    ACS_Value = analogRead(ACS_Pin);   // read the analog in value:
    inputStats.input(ACS_Value);  // log to Stats   function
        
    if((unsigned long)(millis() - previousMillis) >= printPeriod)   { //every second we do the calculation
      previousMillis = millis();   //   update time
      
      Amps_TRMS = intercept + slope * inputStats.sigma();

      Serial.print( "\	 Amps: " ); 
      Serial.println( Amps_TRMS );

     }
  }
}

/* About the slope and intercept
 * First you need to   know that all the TRMS calucations are done by functions from the library, it's   the "inputStats.sigma()" value
 * At first you can display that "inputStats.sigma()"   as your TRMS value, then try to measure using it when the input is 0.00A
 * If   the measured value is 0 like I got you can keep the intercept as 0, otherwise you'll   need to add or substract to make that value equal to 0
 * In other words " remove   the offset"
 * Then turn on the power to a known value, for example use a bulb   or a led that ou know its power and you already know your voltage, so a little math   you'll get the theoritical amps
 * you divide that theory value by the measured   value and here you got the slope, now place them or modify them
 */
