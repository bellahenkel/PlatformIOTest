//Include libraries for the Arduino and I2C
#include <SPI.h>
#include <Arduino.h>

//Includes the Library for the Analog Digital Converters
#include <Adafruit_ADS1X15.h>

//Creates variables for ADC
Adafruit_ADS1115 ads1;

//Setting variables for the ADC results
int16_t results;
float CO2_ppm;

//sets delay between sensor readings (in ms)
const int delay_value = 5000;

//inputs digital value from sensor ADC, outputs CO2 reading in ppm
float get_ppm_value(int16_t digital_value) {
    //converts digital CO2 value to voltage (V)
    float CO2_Voltage = ads1.computeVolts(digital_value);
    //converts voltage to amps (mA)
    float CO2_Amp = CO2_Voltage/250 * 1000;
    //converts amp value to ppm (recommended by sensor manufacturer when reading voltage across 250ohm resistor)
    float CO2_amp_to_ppm = 312.5 * CO2_Amp - 1250;

    return CO2_amp_to_ppm;

}


void setup() {
  //Begins serial communication with 9600 Baud
  Serial.begin(9600);

    //Begins serial communication with 9600 Baud
  Serial.println("Setting ads variables...");
  //Set the gain for ADC located on AA1 on Mayfly Microcontroller
  ads1.setGain(GAIN_ONE);  // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV

  Serial.println("ADS variable setup complete");

  //Check if ADC on mayfly is operational at 0x48 address
  Serial.println("Checking if ads1 is initialized...");
  if (!ads1.begin(0x48)) {
    Serial.println("Failed to initialize ADS1.");
    while (1);
  }
  Serial.println("Initialization complete.");

}

void loop() {
    //voltage difference across AA2 and AA3 from CO2 sensor
    results = ads1.readADC_Differential_2_3();

    CO2_ppm = get_ppm_value(results);

    Serial.println(CO2_ppm);

    delay(delay_value);

    
  }
