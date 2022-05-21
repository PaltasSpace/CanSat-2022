#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_ADXL345_U.h>



#define SEALEVELPRESSURE_HPA (1019.2)

Adafruit_BMP3XX bmp;
/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


String mensaje = "";
String temperatura = "";
String presion = "";
String altitud = "";

int counter = 0;


void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Sender");
  if (!LoRa.begin(915E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  Serial.println("Adafruit BMP388 / BMP390 test");
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);  

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);
  // accel.setRange(ADXL345_RANGE_8_G);
  // accel.setRange(ADXL345_RANGE_4_G);
  // accel.setRange(ADXL345_RANGE_2_G);

}



void loop() {
  sensors_event_t event; 
  accel.getEvent(&event);

 /* 
  //Serial.print("Sending packet: ");
  Serial.println(counter);
  Serial.println(bmp.temperature);
  Serial.println(bmp.pressure / 100.0);
  Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));
*/

  mensaje = String(counter)+","+String(bmp.temperature)+","+String(bmp.pressure / 100.0)+","+ String(bmp.readAltitude(SEALEVELPRESSURE_HPA))+","+String(event.acceleration.x)+","+String(event.acceleration.y)+","+String(event.acceleration.z)+"\n";
  Serial.print(mensaje);

  // send packet
  LoRa.beginPacket();
  LoRa.print(mensaje);
  LoRa.endPacket();

  counter++;
 
}
