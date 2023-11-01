//Include libraries for the Arduino and I2C
#include <SPI.h>
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

//Includes the Library for the Analog Digital Converters
#include <Adafruit_ADS1X15.h>

//Creates variables for different ADCs
Adafruit_ADS1115 o2sensor;


//Setting variables for the ADC results
int16_t digital_results;
float multiplier = 0.125F;
float O2_Volt;
float O2_Percentage;
float accuracy_modifer;
float O2_Actual_Percentage;

void setup() {

  Wire.begin();

  //Begins serial communication with 9600 Baud
  Serial.begin(9600);

  //Set the gain for each ADC located on AA0,AA1, AA2, and AA3 on Mayfly Microcontroller
  o2sensor.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  
  //Check if ADC on mayfly is operational at 0x48 address
  if (!o2sensor.begin(0x48)) {
    Serial.println("Failed to initialize ADS1.");
    while (1)
      ;
  }


}

void loop() {




    // Read Oxygen and CO2 results from the Mayfly ADCs
    digital_results = o2sensor.readADC_Differential_0_1();

    //Compute voltage from ADC results
    O2_Volt = digital_results * multiplier / 1000;
 
    //_______________Algorithm Computation_______________________________________

    //Converts Voltage to Percent O2%
    O2_Percentage = (((0.0763 * abs(O2_Volt)) - 0.0841) * 100);

    //Error Reading if negative value
    if (O2_Percentage <= 0) {
      O2_Percentage = -999;
    }

    Serial.print("O2 Volt= ");
    Serial.print(O2_Volt);
    Serial.print("V| ");

    Serial.print("O2% = ");
    Serial.print(O2_Percentage);
    Serial.print("%| ");


    delay(1000);
  
}




