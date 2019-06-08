#include <EEPROM.h>
//#include <TimerOne.h>
//#include <RBDdimmer.h>
#include <LiquidCrystal.h>

#include <DHT.h>
#include <DHT_U.h>

#include <PID_v1.h>

#include <Rotary.h>

#define pwrRelayPin A1
#define spkrPin A2
#define dmrPin 9

#define redBtnPin A0
#define dialBtnPin A3
#define dialClkPin A5
#define dialDtPin A4
#define DHTPIN 8
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

LiquidCrystal lcd(13,12,7,6,5,4);

boolean relayOn = false;
boolean heaterOn = false;
boolean tempOOR = false;
boolean humidOOR = false;
boolean sensorMalfunction = false;
int displayMode = 0;//false for temp, true for humidity
int alarmPitch = 440;
double targetTemp = 37.5;;
double currentTemp = targetTemp;
const float tempTolerance = .2;
const int tempAlarmTolerance = 3;//plus and minus
const float tempAdjustInc = .25;
double dimmerLevel;

float currentHumid = 0;
float targetHumid = 45; //percent
const int humidTolerance = 4; //plus minus
const int humidAlarmTolerance = 10; //plus minus
const float humidAdjustInc = 2.5;

const int maxTempReadings = 20;
float tempReadings[maxTempReadings];
int curTempIndex = 0;

const int maxHumidReadings = 10;
float humidReadings[maxHumidReadings];
int curHumidIndex = 0;

unsigned long currentMillis = 0; 
unsigned long prevTempMillis = 0;
unsigned long prevAlarmMillis = 0;
unsigned long prevHeaterMillis = 0;
unsigned long prevHumidMillis = 0;
unsigned long prevDisplayMillis = 0;
unsigned long prevButtonMillis = 0;

unsigned int tempPolling = 1 * 1000;
unsigned int alarmPolling = 2 * 1000;
unsigned int heaterPolling = 10 * 1000;
unsigned int humidPolling = 20 * 1000;
unsigned int displayPolling = 10 * 1000;
unsigned int buttonPolling = 500;

//PID pid(&currentTemp, &dimmerLevel, &targetTemp, 50.0, 230.0, 0.25, DIRECT);

unsigned char dialRotation = DIR_NONE;

Rotary r = Rotary(2, 3);

//dimmerLamp dimmer(dmrPin);

void setup() {
  Serial.begin (9600);

  //affects pins 9 and 10
  //Timer1.initialize(5000);//5ms = 200hz

  int eeAddress = 0;
  float value;
  EEPROM.get(eeAddress, value);
  if(value > 0)
  {
    targetTemp = value;
    Serial.println((String)"Got target temp from storage " + value);
    eeAddress += sizeof(float);
    EEPROM.get(eeAddress, value);
    if(value > 0)
    {
      targetHumid = value;
      Serial.println((String)"Got target humid from storage " + value);
    }
  }

  dht.begin();
  lcd.begin(16,2);

  delay(1000);

  lcd.print("Initializing");
  
  pinMode(pwrRelayPin, OUTPUT);
  pinMode(spkrPin, OUTPUT);
  pinMode(dmrPin, OUTPUT);
  //analogWrite(dmrPin, 50);

  pinMode(redBtnPin, INPUT_PULLUP);
  pinMode(dialBtnPin, INPUT);

  //play a short tone to make sure the speaker is working
  tone(spkrPin, alarmPitch, 500);

  //Set all readings to the current temp;
  float curTemp = getActualTemp();
  for (int thisReading = 0; thisReading < maxTempReadings; thisReading++) {
    tempReadings[thisReading] = curTemp;
  }

  //init all humidity readings
  float curHumid = getActualHumid();
  for (int thisReading = 0; thisReading < maxHumidReadings; thisReading++) {
    humidReadings[thisReading] = curHumid;
  }

  //pid.SetOutputLimits(0,100);
  //pid.SetMode(AUTOMATIC);//automatic

  
  r.begin();
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();
}

ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result == DIR_NONE) {
    // do nothing
  }
  else if (result == DIR_CW) {
    dialRotation = DIR_CW;
    Serial.println("ClockWise");
  }
  else if (result == DIR_CCW) {
    dialRotation = DIR_CCW;
    Serial.println("CounterClockWise");
  }
}

void loop() {
  currentMillis = millis();   

  checkButtons();
  checkAlarm();
  setCurrentTemp();
  checkHeater();
  checkHumid();
  updateDisplayLoop();
}

void checkButtons()
{
  if((currentMillis - prevButtonMillis) < buttonPolling)
    return;

  prevButtonMillis = currentMillis;

  //Serial.println("check buttons");

  if(digitalRead(redBtnPin)== LOW)
  {
    displayMode = displayMode == 0 ? 1 : 0;
    updateDisplay();
  }

  int dialAdjust = 0;
  if(dialRotation == DIR_CW)
    dialAdjust = +1;
  else if(dialRotation == DIR_CCW)
    dialAdjust = -1;
    
  if(dialAdjust != 0)
  {
    if(displayMode == 0)
    {
      targetTemp += (tempAdjustInc * dialAdjust);
    }
    else
    {
      targetHumid += (humidAdjustInc * dialAdjust);
    }

    writeTargets();
    dialRotation = DIR_NONE;
    updateDisplay();
  }
}

void updateDisplayLoop()
{
  if((currentMillis - prevDisplayMillis) < displayPolling)
    return;

  prevDisplayMillis = currentMillis;

  updateDisplay();
}

void updateDisplay()
{
  lcd.clear();
  if(sensorMalfunction)
  {
    lcd.print("Sensor error");
    return;    
  }
  
  if(displayMode == 0)
    updateTempDisplay();
  else
    updateHumidDisplay();
}

void updateTempDisplay()
{
  lcd.print((String)"Temp " + currentTemp);
  lcd.setCursor(0,1);
  String state = heaterOn ? "ON" : "OFF";
  //lcd.print((String)"T " + targetTemp + " Br " + dimmerLevel + "%"); 
  lcd.print((String)"T " + targetTemp + " He " + state); 
  //lcd.print((String)"Humidity " + currentHumid);  
}

void updateHumidDisplay()
{
  lcd.print((String)"Humid " + currentHumid);
  lcd.setCursor(0,1);
  String humidifier = relayOn ? "ON" : "OFF";
  lcd.print((String)"T " + targetHumid +  " Hu " + humidifier);
}

void checkHeater()
{
  if((currentMillis - prevHeaterMillis) < heaterPolling)
    return;

  prevHeaterMillis = currentMillis;
      
  targetTemp = getTargetTemp();
  
  //pid.Compute();

  //Serial.println((String)"Dimmer level " + dimmerLevel);
  //writeDimmer(dimmerLevel);
  
  float variance = currentTemp - targetTemp;//negative means heat up
  if(abs(variance) > tempTolerance)
  {
    if(variance < 0)
      writeHeater(true);
    else
      writeHeater(false);
  }

  tempOOR = abs(variance) > tempAlarmTolerance;
}

void checkHumidLoop()
{
  if((currentMillis - prevHumidMillis) < humidPolling)
    return;

  prevHumidMillis = currentMillis;
  
  checkHumid();
}

void checkHumid()
{
  float variance = currentHumid - targetHumid;
  
  if(abs(variance) > humidTolerance)
  {
    if(variance < 0)
      writeRelay(true);
    else
      writeRelay(false);
  }
  
  humidOOR = (abs(variance) > humidAlarmTolerance) 
    && ((variance > 0 && relayOn) || (variance < 0));
}

void setCurrentHumid()
{
  float curHumid = getActualHumid();
  sensorMalfunction = isnan(curHumid);
  if(sensorMalfunction){
    Serial.println("Humid sensor failure");
    return;
  }

  humidReadings[curHumidIndex] = curHumid;
  //get the total
  float total = 0;
  for (int i = 0; i < maxHumidReadings; i++) {
    total += humidReadings[i];
  }
  currentHumid = total / maxHumidReadings;
  Serial.println((String)"Actual humid " + currentHumid);

  //advance the index
  curHumidIndex += 1;
  if(curHumidIndex >= maxHumidReadings)
    curHumidIndex = 0;//go to beginning    
}

void setCurrentTemp()
{
  if((currentMillis - prevTempMillis) < tempPolling)
    return;

  prevTempMillis = currentMillis;
  
  float curTemp = getActualTemp();
  sensorMalfunction = isnan(curTemp);
  if(sensorMalfunction){
    Serial.println("Temp sensor failure");
    return;
  }
        
  tempReadings[curTempIndex] = curTemp;
  //get the total
  float total = 0;
  for (int i = 0; i < maxTempReadings; i++) {
    total += tempReadings[i];
  }
  currentTemp = total / maxTempReadings;
  Serial.println((String)"Actual temp " + currentTemp);

  //advance the index
  curTempIndex += 1;
  if(curTempIndex >= maxTempReadings)
    curTempIndex = 0;//go to beginning

  setCurrentHumid();
}

float getActualTemp()
{
    return dht.readTemperature();
}

float getActualHumid()
{
    return dht.readHumidity();
}

float getTargetTemp()
{
  return targetTemp;
}

void checkAlarm(){
  if((currentMillis - prevAlarmMillis) < alarmPolling)
    return;

  prevAlarmMillis = currentMillis;

  boolean shouldAlarm = sensorMalfunction || tempOOR || humidOOR;
  
  if(!shouldAlarm)
  {
    digitalWrite(spkrPin, LOW);
    return;
  }

  //conflicts??
  tone(spkrPin, alarmPitch, 1000);
  if(alarmPitch == 440)
    alarmPitch = 880;
  else 
    alarmPitch = 440;
}

void writeDimmer(int percent)
{
    percent = min(100, max(0, percent));
    //int pinVal = 255.0 * ((float)percent / 100.0);
    Serial.println((String)"Dimmer to " + percent + "%");
    //analogWrite(dmrPin, pinVal);
    
}

void writeRelay(boolean on)
{
  if(relayOn == on)
    return;

  if(on)
  {
    Serial.println("Relay ON");
    digitalWrite(pwrRelayPin, HIGH);
  }
  else
  {
    Serial.println("Relay OFF");
    digitalWrite(pwrRelayPin, LOW);
  }
  
  relayOn = on;
}

void writeHeater(boolean on)
{
  if(heaterOn == on)
    return;

  if(on)
  {
    Serial.println("Heater ON");
    digitalWrite(dmrPin, HIGH);
  }
  else
  {
    Serial.println("Heater OFF");
    digitalWrite(dmrPin, LOW);
  }
  
  heaterOn = on;
}

void writeTargets()
{
  int eeAddress = 0;
  float value = targetTemp;
  EEPROM.put(eeAddress, value);

  eeAddress += sizeof(float);
  value = targetHumid;
  EEPROM.put(eeAddress, value);
  
}
