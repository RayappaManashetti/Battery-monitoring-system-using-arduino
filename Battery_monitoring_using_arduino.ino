
#include <Adafruit_Sensor.h>


#include <DHT.h>


#include <DHT_U.h>


#define DHTTYPE    DHT11     // DHT 11


#define DHTPIN 2


DHT_Unified dht(DHTPIN, DHTTYPE);
#define ANALOG_IN_PIN A1
// Floats for ADC voltage & Input voltage
float adc_voltage = 0.0;
float in_voltage = 0.0;
 
// Floats for resistor values in divider (in ohms)
float R1 = 30000.0;
float R2 = 7500.0; 
 
// Float for Reference Voltage
float ref_voltage = 5.0;
 
// Integer for ADC value
int adc_value = 0;
const int currentPin = A2;
int sensitivity = 66;
int adcValue= 0;
int offsetVoltage = 2500;
double adcVoltage = 0;
double currentValue = 0;
 
int analogInPin  = A0;    // Analog input pin
float sensorValue;          // Analog Output of Sensor
float calibration = 0.36; // Check Battery voltage using multimeter & add/subtract the value
int bat_percentage;
 float voltage1;
 #include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);

void setup()
{
   lcd.init();
    lcd.backlight();
  Serial.begin(9600);
  dht.begin();

 float sensorValue;
  sensor_t sensor;
  sensorValue = analogRead(analogInPin);
  lcd.setCursor(0,0);
  lcd.print("Welcome to Battery");
  lcd.setCursor(0,1);
  lcd.print("Monitoring Sys");
  delay(1000);
}

void loop()
{
  voltage1 = (((sensorValue * 3.3) / 1024) * 2 + calibration); //multiply by two as voltage divider network is 100K & 100K Resistor
 
 if(voltage1 > 3.0){
  lcd.clear();
    lcd.setCursor(0,0);
  lcd.print("Charging");
  
 }
 else{
  //lcd.clear();
    lcd.setCursor(0,0);
  lcd.print("Discharging");
 
  Serial.println("Discharging");
 }
  Serial.println("Analog Value = ");
  Serial.println(sensorValue);
  Serial.println("Output Voltage = ");
  Serial.println(voltage1);
//  Serial.print("\t Battery Percentage = ");
//  Serial.println(bat_percentage);
// Read the Analog Input
   adc_value = analogRead(ANALOG_IN_PIN);
   
   // Determine voltage at ADC input
   adc_voltage  = (adc_value * ref_voltage) / 1024.0; 
   
   // Calculate voltage at divider input
   in_voltage = adc_voltage / (R2/(R1+R2)) ; 
   
   // Print results to Serial Monitor to 2 decimal places
  Serial.print("Input Voltage = ");
  Serial.println(in_voltage, 2);

   adcValue = analogRead(currentPin);
  adcVoltage = (adcValue / 1024.0) * 5200;
  currentValue = ((adcVoltage - offsetVoltage) / sensitivity);
   Serial.print("\t Current = ");
  Serial.println(currentValue,3);

sensors_event_t event;


  dht.temperature().getEvent(&event);


  Serial.print(F("Temperature: "));


  Serial.print(event.temperature);


  Serial.println(F("°C"));


  dht.humidity().getEvent(&event);


  Serial.print(F("Humidity: "));


  Serial.print(event.relative_humidity);


  Serial.println(F("%"));
  //lcd 
//  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp=");
  lcd.print(event.temperature);
  lcd.print("C");

  lcd.setCursor(0, 1);
  lcd.print("Hmdt=");
  lcd.print(event.relative_humidity);
  lcd.print("%");

  lcd.setCursor(10, 0);
  lcd.print("V=");
  lcd.print(in_voltage, 2);
  lcd.print("v|");

  lcd.setCursor(10, 1);
  lcd.print("C=");
  lcd.print(currentValue,2);
  lcd.print("mA");

  delay(1000);
}
 
 
