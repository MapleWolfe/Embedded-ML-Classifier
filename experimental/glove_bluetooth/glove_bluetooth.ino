#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino_LSM6DSOX.h>
#include <Wire.h>
#include <best_rfc_model.h>
#include <math.h>
#include <ArduinoBLE.h>
Eloquent::ML::Port::RandomForest clf;
// XGBClassifier
// RandomForest

// here we create our global variables
const int numSamples = 25;
float accX, accY, accZ, gyroX, gyroY, gyroZ;
float sensor_1_ax = 0, sensor_1_ay = 0, sensor_1_az = 0, sensor_1_gx = 0, sensor_1_gy = 0, sensor_1_gz = 0;
float sensor_2_ax = 0, sensor_2_ay = 0, sensor_2_az = 0, sensor_2_gx = 0, sensor_2_gy = 0, sensor_2_gz = 0;
float onboard_ax = 0, onboard_ay = 0, onboard_az = 0, onboard_gx = 0, onboard_gy = 0, onboard_gz = 0; int iteration = 0;

// create buffer lists
float sensor_1_ax_list[numSamples], sensor_1_ay_list[numSamples], sensor_1_az_list[numSamples], sensor_1_gx_list[numSamples], sensor_1_gy_list[numSamples], sensor_1_gz_list[numSamples];
float sensor_2_ax_list[numSamples], sensor_2_ay_list[numSamples], sensor_2_az_list[numSamples], sensor_2_gx_list[numSamples], sensor_2_gy_list[numSamples], sensor_2_gz_list[numSamples];
float onboard_ax_list[numSamples], onboard_ay_list[numSamples], onboard_az_list[numSamples], onboard_gx_list[numSamples], onboard_gy_list[numSamples], onboard_gz_list[numSamples];
float combined_output_list[108];

int sampleIndex = 0;

// bluetooth set up
BLEService ClfService("98ac7d69-32e9-4dd9-8ea8-0d1ace729b1c"); // BLE LED Service UIUD is the text
BLEByteCharacteristic ClfCharacteristic("98ac7d69-32e9-4dd9-8ea8-0d1ace729b1c", BLERead | BLEWrite);


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
  
  // setup bluetooth
  if (BLE.begin()) {
    Serial.println("Bluetooth started!");
  } else {
    Serial.println("Bluetooth failed to start!");
  }

  BLE.setLocalName("Glove");
  BLE.setAdvertisedService(ClfService);
  ClfService.addCharacteristic(ClfCharacteristic);
  // add service
  BLE.addService(ClfService);
  // start advertising
  BLE.advertise();
  Serial.println("device is now advertising");

  }

void loop() {
  BLEDevice central = BLE.central();
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

  }
  // Get new sensor 1 events with the readings */
  sensors_event_t a1, g1, temp1;
  mpu1.getEvent(&a1, &g1, &temp1);
  sensor_1_ax = a1.acceleration.x;
  sensor_1_ay = a1.acceleration.y;
  sensor_1_az = a1.acceleration.z;
  sensor_1_gx = g1.gyro.x;
  sensor_1_gy = g1.gyro.y;
  sensor_1_gz = g1.gyro.z;

  sensor_1_ax_list[sampleIndex] = sensor_1_ax;
  sensor_1_ay_list[sampleIndex] = sensor_1_ay;
  sensor_1_az_list[sampleIndex] = sensor_1_az;
  sensor_1_gx_list[sampleIndex] = sensor_1_gx;
  sensor_1_gy_list[sampleIndex] = sensor_1_gy;
  sensor_1_gz_list[sampleIndex] = sensor_1_gz;
  
  // Get new sensor 2 events with the readings */
  sensors_event_t a2, g2, temp2;
  mpu2.getEvent(&a2, &g2, &temp2);
  sensor_2_ax = a2.acceleration.x;
  sensor_2_ay = a2.acceleration.y;
  sensor_2_az = a2.acceleration.z;
  sensor_2_gx = g2.gyro.x;
  sensor_2_gy = g2.gyro.y;
  sensor_2_gz = g2.gyro.z;

  sensor_2_ax_list[sampleIndex] = sensor_2_ax;
  sensor_2_ay_list[sampleIndex] = sensor_2_ay;
  sensor_2_az_list[sampleIndex] = sensor_2_az;
  sensor_2_gx_list[sampleIndex] = sensor_2_gx;
  sensor_2_gy_list[sampleIndex] = sensor_2_gy;
  sensor_2_gz_list[sampleIndex] = sensor_2_gz;

  if (IMU.accelerationAvailable()){
    IMU.readAcceleration(accX, accY, accZ);
    onboard_ax = accX;
    onboard_ay = accY;
    onboard_az = accZ;
    
    onboard_ax_list[sampleIndex] = onboard_ax;
    onboard_ay_list[sampleIndex] = onboard_ay;
    onboard_az_list[sampleIndex] = onboard_az;

  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    onboard_gx = gyroX;
    onboard_gy = gyroY;
    onboard_gz = gyroZ;

    onboard_gx_list[sampleIndex] = onboard_gx;
    onboard_gy_list[sampleIndex] = onboard_gy;
    onboard_gz_list[sampleIndex] = onboard_gz;

    }
    // this statement creates a buffer list using index
    sampleIndex = (sampleIndex + 1) % numSamples;
  
    if (sampleIndex == 0){
        calculateStatistics(sensor_1_ax_list, numSamples, combined_output_list, iteration);
        
        calculateStatistics(sensor_1_ay_list, numSamples, combined_output_list, iteration);
        calculateStatistics(sensor_1_az_list, numSamples, combined_output_list, iteration);
        calculateStatistics(sensor_1_gx_list, numSamples, combined_output_list, iteration);
        calculateStatistics(sensor_1_gy_list, numSamples, combined_output_list, iteration);
        calculateStatistics(sensor_1_gz_list, numSamples, combined_output_list, iteration);

        calculateStatistics(sensor_2_ax_list, numSamples, combined_output_list, iteration);
        calculateStatistics(sensor_2_ay_list, numSamples, combined_output_list, iteration);
        calculateStatistics(sensor_2_az_list, numSamples, combined_output_list, iteration);
        calculateStatistics(sensor_2_gx_list, numSamples, combined_output_list, iteration);
        calculateStatistics(sensor_2_gy_list, numSamples, combined_output_list, iteration);
        calculateStatistics(sensor_2_gz_list, numSamples, combined_output_list, iteration);

        calculateStatistics(onboard_ax_list, numSamples, combined_output_list, iteration);
        calculateStatistics(onboard_ay_list, numSamples, combined_output_list, iteration);
        calculateStatistics(onboard_az_list, numSamples, combined_output_list, iteration);
        calculateStatistics(onboard_gx_list, numSamples, combined_output_list, iteration);
        calculateStatistics(onboard_gy_list, numSamples, combined_output_list, iteration);
        calculateStatistics(onboard_gz_list, numSamples, combined_output_list, iteration);
        iteration = 0;
//        for (int i = 0; i < 108; i++) {
//          Serial.print(String(combined_output_list[i])+"\t");
//        }
        int clf_output = clf.predict(combined_output_list);
        Serial.println("prediction: "+ String(clf_output));
        ClfCharacteristic.writeValue((byte)0x03);
        
        delay(2000);
    
  }
}
  
void calculateStatistics(float data[], int dataSize, float data_store[], int iterator) {
  float sum = 0.0;
  float sumSquared = 0.0;
  float skewnessSum = 0.0;
  float kurtosisSum = 0.0;
  float minimum = data[0];
  float maximum = data[0];

  for (int i = 0; i < dataSize; i++) {
    sum += data[i];
    sumSquared += data[i] * data[i];
    skewnessSum += data[i] * data[i] * data[i];
    kurtosisSum += data[i] * data[i] * data[i] * data[i];
    if (data[i] < minimum) {
      minimum = data[i];
      }
    if (data[i] > maximum) {
      maximum = data[i];
      }
    }

    float mean = sum / dataSize;
    float variance = (sumSquared / dataSize) - (mean * mean);
    float standardDeviation = sqrt(variance);

    skewnessSum /= dataSize;
    skewnessSum -= 3 * mean * variance - mean * mean * mean;
    skewnessSum /= standardDeviation * standardDeviation * standardDeviation;

    kurtosisSum /= dataSize;
    kurtosisSum -= 4 * mean * skewnessSum * variance + 6 * mean * mean * variance - 3 * mean * mean * mean * mean;
    kurtosisSum /= variance * variance;


    data_store[iterator] = mean;
    data_store[iterator+1] = standardDeviation;
    data_store[iterator+2] = skewnessSum;
    data_store[iterator+3] = kurtosisSum;
    data_store[iterator+4] = minimum;
    data_store[iterator+5] = maximum;

    iteration = iterator+6;
    // Serial.println(iteration);

  }


