#pragma once
#include <elapsedMillis.h>
#include <Servo.h>
#include <Wire.h>
#include <PID_v1.h>
#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location


// communication over serial
struct dataPacket 
{
    char message[64] = {0};
    float first = 0.0f;
    float second = 0.0f;

};


//Timing
// elapsedMillis elapsedTime;

const uint16_t ANALOG_WRITE_BITS = 8;  
const uint16_t MAX_PWM = pow(2, ANALOG_WRITE_BITS)-1;  
const uint16_t MIN_PWM = MAX_PWM / 4; 
// constants for motor driver
const uint8_t IN1=25;
const uint8_t IN2=26;
const uint8_t ENA=23;
const uint8_t PWMA=23;

const uint8_t IN3=27;
const uint8_t IN4=28;
const uint8_t ENB=22;
const uint8_t PWMB=22;

double KpA = 0.20, KiA = 0.20, KdA = 0;  
double KpB = 0.20, KiB = 0.20, KdB = 0;  

float yawErrorAccumulated = 0;
float pitchErrorAccumulated = 0;

unsigned long nowTime = 0;       // updated on every loop  
unsigned long startTimeA = 0;    // start timing A interrupts  
unsigned long startTimeB = 0;    // start timing B interrupts  
unsigned long countIntA = 0;     // count the A interrupts  
unsigned long countIntB = 0;     // count the B interrupts  
double periodA = 0;              // motor A period  
double periodB = 0;              // motor B period  
// PID   
const unsigned long SAMPLE_TIME = 100;  // time between PID updates  
const unsigned long INT_COUNT = 20;     // sufficient interrupts for accurate timing  
double setpointA = 150;         // setpoint is rotational speed in Hz  
double inputA = 0;              // input is PWM to motors  
double outputA = 0;             // output is rotational speed in Hz  
double setpointB = 150;         // setpoint is rotational speed in Hz  
double inputB = 0;              // input is PWM to motors  
double outputB = 0;             // output is rotational speed in Hz 
//============
PID motorA(&inputA, &outputA, &setpointA, KpA, KiA, KdA, DIRECT);  
PID motorB(&inputB, &outputB, &setpointB, KpB, KiB, KdB, DIRECT);  

//============

// enum direction {
//     dirForward,
//     dirBackwards,
//     dirRotatingLeft,
//     dirRotatingRight
// };

// direction currentDirection;
bool isBreaking = false;
bool isStopped = false;


//constants for ESP8266
const uint8_t ESP8266_RST = 29, ESP8266_CH_PD = 30;


//Constants for Lidar 
// VL53L1X distanceSensor;

const uint8_t Lidar_XSHUT = 36;
enum LidarDistanceMode 
{
    near = 0,
    medium = 1,
    far = 2
};


enum class EngineSelector
{
	Left,
	Right
};

enum class ServoSelector{
    Yaw,
    Pitch
};


//constants for debug LED
const uint8_t led = 13;
boolean ledState = HIGH;


//constants for servos
//left-right
Servo servoYaw;
int yawMin = 45, yawMax = 135, yawCenter = 90, yawCurrent = 90;
//up-down
Servo servoPitch;
int pitchMin = 45, pitchMax = 135, pitchCenter = 90, pitchCurrent = 90;

// Theoretical speed of the servo
// 0.23s / 60 degrees (at 4.8V, no load)
float servoMillisecondsPerDegree = 20;
float damping = 0.01;
int deadZone = 5;

elapsedMillis yawMoveTimer;
elapsedMillis pitchMoveTimer;

bool isYawServoMoving = false;
bool isPitchServoMoving = false;


unsigned int servoWaitTimeYaw = 0;
unsigned int servoWaitTimePitch = 0;




