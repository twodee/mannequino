#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu(0x69);

bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
unsigned long lastTime;

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  mpu.initialize();

  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read());
  while (!Serial.available());
  while (Serial.available() && Serial.read());

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(-1093);
  mpu.setYAccelOffset(1662);
  mpu.setZAccelOffset(1526);
  mpu.setXGyroOffset(37);
  mpu.setYGyroOffset(-40);
  mpu.setZGyroOffset(57);

  if (devStatus == 0) {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  lastTime = millis();
}

void loop() {
  if (!dmpReady) return;

  fifoCount = mpu.getFIFOCount();

  if (fifoCount == 1024) {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  } else {
    while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    unsigned long presentTime = millis();
    unsigned long elapsedTime = presentTime - lastTime;
    if (elapsedTime > 100) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      Serial.print("[");
      Serial.print(q.w, 5);
      Serial.print("\t");
      Serial.print(q.x, 5);
      Serial.print("\t");
      Serial.print(q.y, 5);
      Serial.print("\t");
      Serial.print(q.z, 5);
      Serial.println("]");
      lastTime = presentTime;
    }
  }
}
