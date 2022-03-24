#include "I2Cdev.h"
#include <PID_v1.h>                     //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

/** MPU6050 I2C device class **/
MPU6050 mpu;

/** Data from the MPU6050 sensor **/
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// Orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/** PID algorithm values **/
double setpoint = 176; // set the value when the bot is perpendicular to ground using serial monitor.
double Kp = 21;  // Set this first
double Kd = 0.8; // Set this secound
double Ki = 140; // Finally set this
double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

/** Initial Setup **/
void setup()
{
    Serial.begin(115200);
    mpu.initialize();

    // Verifying connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // Supplying gyro offsets
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688);

    // Checking if mpu is initialised
    if (devStatus == 0)
    {
        // Turn on the DMP
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Setting the DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // Get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        // Setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // (if it's going to break, usually the code will be 1)
        // 2 = DMP configuration updates failed
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    /** Initialising the Digital PWM pins **/
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);

    // By default turn off both the motors
    analogWrite(6, LOW);
    analogWrite(9, LOW);
    analogWrite(10, LOW);
    analogWrite(11, LOW);
}

void loop()
{
    // if programming failed, don't do anything
    if (!dmpReady)
        return;

    /** Initialise the PID algorithm **/
    PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

    /** Initial Setup **/
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        // No mpu data - performing PID calculations and output to motors
        pid.Compute();

        // Print the value of Input and Output on serial monitor to check how it is working.
        Serial.print(input);
        Serial.print(" =>");
        Serial.println(output);

        if (input > 150 && input < 200)
        {                        // If the Bot is falling
            if (output > 0)      // Falling towards front
                Forward();       // Rotate the wheels forward
            else if (output < 0) // Falling towards back
                Reverse();       // Rotate the wheels backward
        }
        else        // If Bot not falling
            Stop(); // Hold the wheels still
    }

    // Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // Check for overflow (this should never happen unless code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // Reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    }
    // Otherwise, check for DMP data ready interrupt (this should happen frequently)
    else if (mpuIntStatus & 0x02)
    {
        // Wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        // Read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // Track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);      // get value for q
        mpu.dmpGetGravity(&gravity, &q);           // get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // get value for ypr

        input = ypr[1] * 180 / M_PI + 180;
    }
}

// Rotate the wheel forward
void Forward()
{
    analogWrite(6, output);
    analogWrite(9, 0);
    analogWrite(10, output);
    analogWrite(11, 0);
    Serial.print("F"); // Debugging information
}

// Rotate the wheel backward
void Reverse()
{
    analogWrite(6, 0);
    analogWrite(9, output * -1);
    analogWrite(10, 0);
    analogWrite(11, output * -1);
    Serial.print("R");
}

// Stop both the wheels
void Stop()
{
    analogWrite(6, 0);
    analogWrite(9, 0);
    analogWrite(10, 0);
    analogWrite(11, 0);
    Serial.print("S");
}
