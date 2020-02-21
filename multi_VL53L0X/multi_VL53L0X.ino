#include <Wire.h>
#include <VL53L0X.h> // VL53L0X lib from polulu version 1.2.0

VL53L0X sensorA;
VL53L0X sensorB;
VL53L0X sensorC;
VL53L0X sensorD;

byte sensor_a_xshut_pin = 4;
byte sensor_b_xshut_pin = 5;
byte sensor_c_xshut_pin = 6;
byte sensor_d_xshut_pin = 7;


int sensor_a_distance;
int sensor_b_distance;
int sensor_c_distance;
int sensor_d_distance;

uint8_t sensor_address = 1; // start address (dosent have to be changed as long you're only unsing these sensors on the i2s bus)


void setup()
{
  Wire.begin();
  Serial.begin(115200);

  setupSensorPin(sensor_a_xshut_pin);
  setupSensorPin(sensor_b_xshut_pin);
  setupSensorPin(sensor_c_xshut_pin);
  setupSensorPin(sensor_d_xshut_pin);
  
  sensorA = initializeSensor(sensor_a_xshut_pin, sensorA);
  sensorB = initializeSensor(sensor_b_xshut_pin, sensorB);
  sensorC = initializeSensor(sensor_c_xshut_pin, sensorC);
  sensorD = initializeSensor(sensor_d_xshut_pin, sensorD);

  sensorA.startContinuous();
  sensorB.startContinuous();
  sensorC.startContinuous();
  sensorD.startContinuous();
}

void loop() {
  sensor_a_distance = cleanSensorData(sensorA.readRangeContinuousMillimeters());
  Serial.print(sensor_a_distance);
  Serial.print(",");

  sensor_b_distance = cleanSensorData(sensorB.readRangeContinuousMillimeters());
  Serial.print(sensor_b_distance);
  Serial.print(",");

  sensor_c_distance = cleanSensorData(sensorC.readRangeContinuousMillimeters());
  Serial.print(sensor_c_distance);
  Serial.print(",");

  sensor_d_distance = cleanSensorData(sensorD.readRangeContinuousMillimeters());
  Serial.print(sensor_d_distance);
  Serial.println("");
}

void setupSensorPin(byte pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

int cleanSensorData(int raw_input) {
  if (raw_input > 1300) { // capping raw input
    return 1300;
  }
  return raw_input;
}

VL53L0X initializeSensor(byte xshut_pin, VL53L0X sensor) {
  digitalWrite(xshut_pin, HIGH);
  delay(50);
  sensor.init(true);
  delay(50);
  sensor.setAddress(uint8_t(sensor_address));
  sensor_address += 1;
  return sensor;
}
