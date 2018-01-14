#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// CONSTANTS ------------------------------------------------------------------

#define MPU_COUNT 2
const int ADO_PINS[MPU_COUNT] = {8, 9};

#define READ_ADDRESS 0x68 // could have been 0x69
#define MILLIS_TILL_RESET 1000
#define MILLIS_TILL_SHIP 100

// TYPES ----------------------------------------------------------------------

class MultiMPU : public MPU6050 {
  public:
    MultiMPU(int id) :
      MPU6050(READ_ADDRESS),
      id(id) {
    }

    bool initialize() {
      MPU6050::initialize();
      pinMode(ADO_PINS[id], OUTPUT);
      last_serial_at = millis();

      dev_status = dmpInitialize();

      if (dev_status == 0) {
        Serial.println(F("Enabling DMP..."));
        setXAccelOffset(-1093);
        setYAccelOffset(1662);
        setZAccelOffset(1526);
        setXGyroOffset(37);
        setYGyroOffset(-40);
        setZGyroOffset(57);
        setDMPEnabled(true);
        packet_size = dmpGetFIFOPacketSize();
      } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(dev_status);
        Serial.println(F(")"));
      }

      return dev_status == 0;
    }

    void deactivate() {
      int drop_state = READ_ADDRESS == 0x68 ? HIGH : LOW;
      digitalWrite(ADO_PINS[id], drop_state);
    }

    void activate() {
      int read_state = READ_ADDRESS == 0x68 ? LOW : HIGH;
      digitalWrite(ADO_PINS[id], read_state);
    }

    void read() {
      if (dev_status != 0) {
        Serial.print("reinitializing ");
        Serial.println(id);
        initialize();
      }

      if (dev_status != 0) {
        return;
      }

      // Wait until we have enough data. Once we do, let's push out a
      // quarternion onto the serial port! After abandoning interrupts,
      // which didn't make sense with multiple MPU6050s issuing them,
      // this code would occasionally hang. The workaround for the time
      // being is to reset the MPU if it takes to long to acquire a 
      // full packet.
      fifo_size = getFIFOCount();
      if (fifo_size == 1024) {
        resetFIFO();
        Serial.println(F("FIFO overflow!"));
      } else {
        bool is_reset = false;
        unsigned long polling_started_at = millis();
        while (!is_reset && fifo_size < packet_size) {
          fifo_size = getFIFOCount();
          if (millis() - polling_started_at > MILLIS_TILL_RESET) {
            Serial.print("resetting ");
            Serial.println(id);
            is_reset = true;
          }
        }

        if (is_reset) {
          initialize();
          resetFIFO();
        } else {
          getFIFOBytes(fifo_buffer, packet_size);
          fifo_size -= packet_size;
          emit_quaternion();
        }
      }
    }

    void emit_quaternion() {
      // The serial reading in Unity seems to get bogged down if the data
      // comes in too quickly. We issue a serial write only once every
      // MILLIS_TILL_SHIP milliseconds.
      unsigned long presentTime = millis();
      unsigned long elapsedTime = presentTime - last_serial_at;
      if (elapsedTime > MILLIS_TILL_SHIP) {
        dmpGetQuaternion(&q, fifo_buffer);
        Serial.print("[");
        Serial.print(id);
        Serial.print("\t");
        Serial.print(q.w, 5);
        Serial.print("\t");
        Serial.print(q.x, 5);
        Serial.print("\t");
        Serial.print(q.y, 5);
        Serial.print("\t");
        Serial.print(q.z, 5);
        Serial.println("]");
        last_serial_at = presentTime;
      }
    }

  private:
    int id;
    uint16_t fifo_size;
    uint8_t fifo_buffer[64];
    uint8_t dev_status;
    uint16_t packet_size;
    unsigned long last_serial_at;
    Quaternion q;
};

// GLOBALS --------------------------------------------------------------------

bool is_dmp_ready = false; 

MultiMPU mpus[] = {
  MultiMPU(0),
  MultiMPU(1)
};

// ----------------------------------------------------------------------------

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  Serial.println("Gimme a char, please, and then I'll start up the MPUs.");
  while (Serial.available() && Serial.read());
  while (!Serial.available());
  while (Serial.available() && Serial.read());

  Serial.println("Initializing DMP...");

  is_dmp_ready = true;
  for (int i = 0; i < MPU_COUNT; ++i) {
    is_dmp_ready = is_dmp_ready && mpus[i].initialize();
  }
}

// ----------------------------------------------------------------------------

void loop() {
  if (!is_dmp_ready) return;

  for (int i = 0; i < MPU_COUNT; ++i) {
    // Associate this MPU with the read address.
    mpus[i].activate();

    // Divert all other MPUs to the ignored address.
    for (int oi = 0; oi < MPU_COUNT; ++oi) {
      if (i != oi) {
        mpus[oi].deactivate();
      }
    }

    mpus[i].read();
  }
}
