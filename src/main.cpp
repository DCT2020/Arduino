#include "Arduino.h"
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//Error Check
const int None = -1;
bool g_bIsRunning = true;

//Function
static bool WireSetup(const int I2CAddress = None);

namespace GyroSens
{
    MPU6050 accelgyro;

    const int X = 0;
    const int Y = 1;
    const int Z = 2;
    const int MaxAxisCount = 3;

    int16_t AcAxis[MaxAxisCount] = {0};
    int16_t GyAxis[MaxAxisCount] = {0};

    uint8_t packetSize = 0;
    uint8_t FifoBuffer[64] = {0};
    uint8_t fifoCount = 0;

    uint8_t mpuIntStatus;  

    Quaternion q;
    VectorFloat gravity;
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    int Temp = 0;

    bool GyroSetup()
   {
       accelgyro.initialize();
       pinMode(2, INPUT);

       accelgyro.dmpInitialize();

       accelgyro.setXGyroOffset(220);
       accelgyro.setYGyroOffset(76);
       accelgyro.setZGyroOffset(-85);
       accelgyro.setZAccelOffset(1788); // 1688 factory default for my test chip

       accelgyro.setIntDMPEnabled(true);
       packetSize = accelgyro.dmpGetFIFOPacketSize();

       mpuIntStatus = accelgyro.getIntStatus();
   }

   void UpdateGyro()
   {
    mpuIntStatus = accelgyro.getIntStatus();
    
        fifoCount = accelgyro.getFIFOCount();
        
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                // reset so we can continue cleanly
                accelgyro.resetFIFO();
                Serial.println(F("FIFO overflow!"));
        
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();
    
            // read a packet from FIFO
            accelgyro.getFIFOBytes(FifoBuffer, packetSize);
            
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
    
    
                // display Euler angles in degrees
                accelgyro.dmpGetQuaternion(&q, FifoBuffer);
                accelgyro.dmpGetGravity(&gravity, &q);
                accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
                Serial.print("ypr\t");
                Serial.print(ypr[0] * 180/M_PI);
                Serial.print("\t");
                Serial.print(ypr[1] * 180/M_PI);
                Serial.print("\t");
                Serial.println(ypr[2] * 180/M_PI);
        }
};

// Servo Info
namespace ServoMotor
{
    Servo g_servo;
    int g_servoPin = 9;

    int g_angle = 0;

    bool ServoSetup(int servoPin = None)
    {
        if(servoPin != None)
        {
            g_servoPin = servoPin;
        }

        if(g_servoPin == None)
        {
            return false;
        }    
        else
        {
            g_servo.attach(g_servoPin);
        }

        return true;
    }
};


void setup() {
    Serial.begin(9600);
    
    // put your setup code here, to run once:
    ServoMotor::ServoSetup();
    GyroSens::GyroSetup();
}

void loop() {
    GyroSens::UpdateGyro();
}