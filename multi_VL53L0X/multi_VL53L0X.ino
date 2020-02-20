#include <Wire.h>
#include <VL53L0X.h> // VL53L0X lib from polulu version 1.2.0

VL53L0X sensorA;
VL53L0X sensorB;
VL53L0X sensorC;
VL53L0X sensorD;

int sensor_a_xshut_pin = 4;
int sensor_b_xshut_pin = 5;
int sensor_c_xshut_pin = 6;
int sensor_d_xshut_pin = 7;

int sensor_a_distance;
int sensor_b_distance;
int sensor_c_distance;
int sensor_d_distance;

uint8_t sensor_address = 1; // start address (dosent have to be changed as long you're only unsing these sensors on the i2s bus)


void setup()
{
  Wire.begin();
  Serial.begin(115200);

  pinMode(sensor_a_xshut_pin, OUTPUT);
  pinMode(sensor_b_xshut_pin, OUTPUT);
  pinMode(sensor_c_xshut_pin, OUTPUT);
  pinMode(sensor_d_xshut_pin, OUTPUT);

  digitalWrite(sensor_a_xshut_pin, LOW);
  digitalWrite(sensor_b_xshut_pin, LOW);
  digitalWrite(sensor_c_xshut_pin, LOW);
  digitalWrite(sensor_d_xshut_pin, LOW);

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

int cleanSensorData(int raw_input) {
  if (raw_input > 1300) {
    return 1300;
  }
  return raw_input;
}

VL53L0X initializeSensor(int xshut_pin, VL53L0X sensor) {
  digitalWrite(xshut_pin, HIGH);
  delay(50);
  sensor.init(true);
  delay(50);
  sensor.setAddress(uint8_t(sensor_address));
  sensor_address += 1;
  return sensor;
}
