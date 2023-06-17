#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);
  while (!Serial);

  Serial.println("\nI2C Scanner");
}

void loop() {
  byte error, address;
  int deviceCount = 0;

  Serial.println("Scanning...");

  for (address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("");
      deviceCount++;
    }
  }

  if (deviceCount == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("Scan complete\n");
  }

  delay(5000); // Wait 5 seconds before next scan
}
