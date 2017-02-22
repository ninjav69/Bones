/*
 * Bones is a system of MPU6050 motion sensors behind a I2C multiplexer. This program handles IRQs from
 * the MPU6050, reads the Yaw Pitch Roll information the MP6050 DPU for the request, and forwards it
 * over the serial port tagged with the device index of the source MPU.
 * 
 * Behind the serial port sits a Raspberry PI Model 1B that reads our packets, parses them and formats
 * a UDP datagram in the likeness of the Freepie IMU android program. These datagrams are sent off to
 * a listening instance of Freepie, which is running a python script that captures the information via
 * its Android plugin.
 * 
 * The Yaw Pitch Roll values along with the device index is used in an algorithm that determines the
 * position and rotation of the in-hand Motion Sensor. The result of the transform is mapped to the
 * Freepie Hydra emulation driver which gets picked up by Steam VR.
 * 
 * That is why this project is called bones - it's a VR skeleton.
 */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

class MPU {
public:
    MPU();
    MPU(char id, char channel);

    void update();
    void transmit();
    
    void setup(char id, char channel);
    char channel();
private:
    char id;            // Device ID transmitted for Android IMU emulation
    char muxChannel;    // Channel on Mux where this MPU is installed
    char ypr[3];
};

MPU::MPU() {
    MPU(0, 0);
}

MPU::MPU(char id, char channel) {
    this->id = id;
    this->muxChannel = channel;
    ypr[0] = 0;
    ypr[1] = 0;
    ypr[2] = 0;
}

void MPU::setup(char id, char channel) {
    this->id = id;
    this->muxChannel = channel;
}

char MPU::channel() {
    return this->muxChannel;
}

void MPU::update() {
  // Read values from sensors
  // ...
}

void MPU::transmit() {
    Serial.print(id);
    Serial.print(":");
    Serial.print(ypr[0]);
    Serial.print(":");
    Serial.print(ypr[1]);
    Serial.print(":");
    Serial.print(ypr[2]);
    Serial.print("\n");
}

#define MPU_COUNT 2
MPU mpus[MPU_COUNT];

class Mux {
public:
    void select(char channel);
private:
    void switchChannel();
    char channel;
};

void Mux::select(char channel) {
    if (this->channel != channel) {
        this->channel = channel;
        switchChannel();
    }
}

void Mux::switchChannel() {
}

Mux mux;

#define QUEUE_SIZE 32
#define QUEUE_MASK (QUEUE_SIZE - 1)
class Work {
public:
    Work();

    MPU *get();
    void put(MPU* mpu);
    char size();
    
private:
    MPU* queue[QUEUE_SIZE];
    char head;
    char tail;
};

Work::Work() {
    this->head = 0;
    this->tail = 0;
}

char Work::size() {
    return abs((QUEUE_SIZE - head) - (QUEUE_SIZE - tail));
}

MPU* Work::get() {
    MPU* rv;
    rv = queue[head];
    ++head;
    head &= QUEUE_MASK;
    return rv;
}

void Work::put(MPU* mpu) {
    queue[tail] = mpu;
    int index = (tail + 1) & QUEUE_MASK;
    if (index != head) {
        tail = index;
    }
}

Work queue;

// Interrupt handlers
void interruptMpu1() {
    queue.put(&mpus[0]);
}

void interruptMpu2() {
    queue.put(&mpus[1]);
}

void setup() {
    mpus[0] = MPU(1, 1);
    mpus[1] = MPU(2, 2);
    mux.select(1);
}

#define EVER ;;
void loop() {
  if (queue.size() > 0) {
    MPU* mpu = queue.get();
    mux.select(mpu->channel());
    mpu->update();
    mpu->transmit();
  }
}

