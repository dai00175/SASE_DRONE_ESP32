#include <Wire.h>
#include <ScioSense_ENS160.h> 
#include <Adafruit_AHTX0.h>

// Pass the address 0x53 found by your scanner to the constructor
ScioSense_ENS160 ens160(0x53); 
Adafruit_AHTX0 aht;

// void setup() {
//   Serial.begin(115200);
//   while (!Serial) delay(10); 

//   Serial.println("ENS160 + AHT21 System Starting...");

//   // Initialize AHT21
//   if (!aht.begin()) {
//     Serial.println("Could not find AHT21 sensor!");
//     while (1) delay(10);
//   }

//   // Initialize ENS160 using the pre-set address
//   if (!ens160.begin()) { 
//     Serial.println("Could not find ENS160 sensor!");
//     while (1) delay(10);
//   }

//   // Set to Standard Operating Mode
//   ens160.setMode(ENS160_OPMODE_STD);
//   Serial.println("Both sensors found! Starting measurements...");
// }

// void loop() {
//   sensors_event_t humidity, temp;
//   aht.getEvent(&humidity, &temp); 

//   // Provide temperature and humidity for internal compensation
//   ens160.set_envdata(temp.temperature, humidity.relative_humidity);

//   if (ens160.available()) {
//     ens160.measure(true); 

//     Serial.println("--- Air Quality Data ---");
//     Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" C");
//     Serial.print("Hum:  "); Serial.print(humidity.relative_humidity); Serial.println(" %");

//     // Display Air Quality Index, TVOC, and eCO2
//     Serial.print("AQI:  "); Serial.println(ens160.getAQI());
//     Serial.print("TVOC: "); Serial.print(ens160.getTVOC()); Serial.println(" ppb");
//     Serial.print("eCO2: "); Serial.print(ens160.geteCO2()); Serial.println(" ppm");
//     Serial.println("------------------------\n");
//   }

//   delay(2000); 
// }