#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino_LSM6DSOX.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <WiFiNINA.h>

// wifi server


// here we create our global variables
char ssid[] = "Yash 2.4GHz";
char password[] = "12345678";
float accX, accY, accZ, gyroX, gyroY, gyroZ;
float sensor_1_ax = 0, sensor_1_ay = 0, sensor_1_az = 0, sensor_1_gx = 0, sensor_1_gy = 0, sensor_1_gz = 0;
float sensor_2_ax = 0, sensor_2_ay = 0, sensor_2_az = 0, sensor_2_gx = 0, sensor_2_gy = 0, sensor_2_gz = 0;
float onboard_ax = 0, onboard_ay = 0, onboard_az = 0, onboard_gx = 0, onboard_gy = 0, onboard_gz = 0;
int received_data = 0;

WiFiServer server(8888);
WiFiClient client;

// Here we create objects for the sensor
Adafruit_MPU6050 mpu1; 
Adafruit_MPU6050 mpu2;

void setup(void) {
  Serial.begin(9600);
  while (!Serial)
    delay(10);

  Serial.println("MPU6050 test!");

  // Initializing MPU6050 with address 0x68
  if (!mpu1.begin(0x68)) {
    Serial.println("Failed to find MPU6050 chip at address 0x68");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 at address 0x68 found!");

  // Initialize MPU6050 with address 0x69
  if (!mpu2.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip at address 0x69");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 at address 0x69 found!");

  // Initialize onboard IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {
      delay(10);
    }
  }
  Serial.println("OnBoard IMU found!");

  // Set accelerometer range for MPU6050 at address 0x68
  mpu1.setAccelerometerRange(MPU6050_RANGE_8_G);

  // Set accelerometer range for MPU6050 at address 0x69
  mpu2.setAccelerometerRange(MPU6050_RANGE_8_G);
  
    // Initialize Wi-Fi module
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  // Scan for Wi-Fi networks
  Serial.println("Scanning Wi-Fi networks...");
  int numNetworks = WiFi.scanNetworks();

  if (numNetworks == 0) {
    Serial.println("No Wi-Fi networks found.");
  } else {
    Serial.print("Number of Wi-Fi networks found: ");
    Serial.println(numNetworks);

    for (int i = 0; i < numNetworks; i++) {
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      delay(10);
    }
  }

  // Connect to the Wi-Fi network
  Serial.print("Attempting to connect to network: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, password) != WL_CONNECTED) {
    Serial.println("Connection failed. Retrying...");
    delay(5000);  // Wait 5 seconds before retrying
  }

  server.begin();
  Serial.println("Server started");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  delay(1000);
  /* Get new sensor 1 events with the readings */
  sensors_event_t a1, g1, temp1;
  mpu1.getEvent(&a1, &g1, &temp1);
  sensor_1_ax = a1.acceleration.x;
  sensor_1_ay = a1.acceleration.y;
  sensor_1_az = a1.acceleration.z;
  sensor_1_gx = g1.gyro.x;
  sensor_1_gy = g1.gyro.y;
  sensor_1_gz = g1.gyro.z;
  
  /* Get new sensor 2 events with the readings */
  sensors_event_t a2, g2, temp2;
  mpu2.getEvent(&a2, &g2, &temp2);
  sensor_2_ax = a2.acceleration.x;
  sensor_2_ay = a2.acceleration.y;
  sensor_2_az = a2.acceleration.z;
  sensor_2_gx = g2.gyro.x;
  sensor_2_gy = g2.gyro.y;
  sensor_2_gz = g2.gyro.z;

  if (IMU.accelerationAvailable()){
    IMU.readAcceleration(accX, accY, accZ);
    onboard_ax = accX;
    onboard_ay = accY;
    onboard_az = accZ;

  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    onboard_gx = gyroX;
    onboard_gy = gyroY;
    onboard_gz = gyroZ;
  }
  /* Print out the values in dictionary-like format */

  client = server.available();
  if (client) {
    while (client.connected()) {
      // Check if there's data available
      if (client.available()) {
        // Read the data from the client
        String data = client.readStringUntil('\n');
        int receivedData = data.toInt();
        String str_output = String(receivedData)+"\t"+String(sensor_1_ax)+"\t"+String(sensor_1_ay)+"\t"+String(sensor_1_az)+"\t"+
        String(sensor_1_gx)+"\t"+String(sensor_1_gy)+"\t"+String(sensor_1_gz)+"\t"+
        String(sensor_2_ax)+"\t"+String(sensor_2_ay)+"\t"+String(sensor_2_az)+"\t"+
        String(sensor_2_gx)+"\t"+String(sensor_2_gy)+"\t"+String(sensor_2_gz)+"\t"+
        String(onboard_ax)+"\t"+String(onboard_ay)+"\t"+String(onboard_az)+"\t"+
        String(onboard_gx)+"\t"+String(onboard_gy)+"\t"+String(onboard_gz);

        client.print(str_output);
        }
      }
    }
  }
