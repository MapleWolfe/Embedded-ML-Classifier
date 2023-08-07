#include <ArduinoBLE.h>


void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
  while (!Serial);
  // initialize the BLE hardware
  BLE.begin();
  Serial.println("BLE Central - glove control");
  BLE.scanForUuid("98ac7d69-32e9-4dd9-8ea8-0d1ace729b1c");
}

void loop() {
  // put your main code here, to run repeatedly:
  BLEDevice peripheral = BLE.available();
  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();
    Serial.println("Connected");
    peripheral.connect();
    BLECharacteristic ClfCharacteristic = peripheral.characteristic("98ac7d69-32e9-4dd9-8ea8-0d1ace729b1c");
    byte value = ClfCharacteristic.read();
    ClfCharacteristic.readValue(value);
    Serial.println(value);

  }
  
    // peripheral disconnected, start scanning again
delay(1000);
}
