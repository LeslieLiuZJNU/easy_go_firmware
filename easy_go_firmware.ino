#include<EMGFilters.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#define PIN_RTT A0
#define PIN_EMG A1

SoftwareSerial s(2, 3);
EMGFilters myFilter;

unsigned long threshold = 600;
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_500HZ;
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;

const int MPU = 0x68;
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;
float angle[12] = { 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45};

void setup() {
  s.begin(115200);
}

void loop() {
  if (analogRead(PIN_RTT) <= 450)  mode_emg();
  else  mode_angle();
}

void mode_emg() {
  pinMode(PIN_RTT, INPUT);
  pinMode(PIN_EMG, INPUT);
  myFilter.init(sampleRate, humFreq, true, true, true);
  s.println("arm run 1 -60 2500");
  delay(4500);
  while (true) {
    int data = analogRead(PIN_EMG);
    int dataAfterFilter = myFilter.update(data);
    int envelope = sq(dataAfterFilter);
    envelope = (envelope > threshold) ? envelope : 0;
    if (threshold > 0)
    {
      if (getEMGCount(envelope)) {
        s.println("arm run 1 60 2500");
        delay(2500);
        break;
      }
    }
    delayMicroseconds(500);
  }
  while (true);
}

void mode_angle() {
  for (int i = 0; i < 12; i++)
    pinMode(angle[i], OUTPUT);
  pinMode(47,OUTPUT);
  digitalWrite(47,LOW);
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  calculate_IMU_error();
  delay(20);
  s.println("arm run 1 60 2500");
  delay(2500);
  int a;
  int ap = -10;
  while (true) {
    float sum = 0;
    for(int i=1;i<=10;i++){
      calculate_roll();
      delay(1);
      sum+=roll;
    }
    int a = 6 - sum/10 * 2;
    if (a != ap) {
      for (int i = 0; i <= 12; i++) {
        if (i != a) {
          digitalWrite(angle[a], 0);
          delay(3);
        }
        else {
          digitalWrite(angle[a], 1);
          delay(3);
        }
      }
      ap = a;
    }

    if (a >=9) {
        s.println("arm run 1 -60 1000");
        delay(1000);
        s.println("arm run 1 60 1000");
        delay(1500);
    }
  }
}

int getEMGCount(int gforce_envelope) {
  static long integralData = 0;
  static long integralDataEve = 0;
  static bool remainFlag = false;
  static unsigned long timeMillis = 0;
  static unsigned long timeBeginzero = 0;
  float angle[5] = { -1.5, -0.5, 0.5, 1.5, 1.6};
  static long fistNum = 0;
  static int  TimeStandard = 200;
  integralDataEve = integralData;
  integralData += gforce_envelope;
  if ((integralDataEve == integralData) && (integralDataEve != 0)) {
    timeMillis = millis();
    if (remainFlag) {
      timeBeginzero = timeMillis;
      remainFlag = false;
      return 0;
    }
    if ((timeMillis - timeBeginzero) > TimeStandard) {
      integralDataEve = integralData = 0;
      return 1;
    }
    return 0;
  }
  else {
    remainFlag = true;
    return 0;
  }
}

void calculate_IMU_error() {
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
}

void calculate_roll() {
  gyroAngleX = 0;
  gyroAngleY = 0;
  yaw = 0;
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58;
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroX = GyroX + 0.56;
  GyroY = GyroY - 2;
  GyroZ = GyroZ + 0.79;
  gyroAngleX = gyroAngleX + GyroX * elapsedTime;
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
}
