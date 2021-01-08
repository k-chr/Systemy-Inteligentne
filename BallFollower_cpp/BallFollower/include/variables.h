#pragma once
#include <elapsedMillis.h>
#include <Servo.h>
#include <Wire.h>
#include <PID_v1.h>

#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location

/*
ENKODERY:
    2 i 3   
	4 i 5
	14 i 15 **DZIALA LB
	16 i 17 **DZIALA RB
    3 łopatki: 12 impulsów na obrót silnika*
    5 łopatek: 20 impulsów na obrót silnika*
*/
// communication over serial
struct dataPacket 
{
    char message[64] = {0};
    float first = 0.0f;
    float second = 0.0f;

};

const int maxDistance = 200;


//Timing
// elapsedMillis elapsedTime;

const uint16_t ANALOG_WRITE_BITS = 8;  
const uint16_t MAX_PWM = pow(2, ANALOG_WRITE_BITS)-1;  
const uint16_t MIN_PWM = 0; 
// constants for motor driver
const uint8_t IN_LEFT1=25;
const uint8_t IN_LEFT2=26;
const uint8_t EN_LEFT_BACK=14;
const uint8_t PWM_LEFT=23;

const uint8_t IN_RIGHT1=27;
const uint8_t IN_RIGHT2=28;
const uint8_t EN_RIGHT_BACK=16;
const uint8_t PWM_RIGHT=22;

double KpM = 0.02, KiM = 0.20, KdM = 0;  
 
float Kp = 20.0f;
float Ki = 8.0f;
float Kd = 0.5f;


//PID SERVOS
float yawErrorAccumulated = 0;
float pitchErrorAccumulated = 0;
double inputSA = 0;              // input is PWM to motors  
double outputSA = 0;             // output is rotational speed in Hz  
double setpointSB = 150;         // setpoint is rotational speed in Hz  
double inputSB = 0;              // input is PWM to motors  
double outputSB = 0; 
double setpointSA = 150;         // setpoint is rotational speed in Hz  

//PID TIME DC
unsigned long nowTime = 0;       // updated on every loop  
unsigned long startTimeA = 0;    // start timing A interrupts  
unsigned long startTimeB = 0;    // start timing B interrupts  
unsigned long countIntA = 0;     // count the A interrupts  
unsigned long countIntB = 0;     // count the B interrupts  
double periodA = 0;              // motor A period  
double periodB = 0;              // motor B period  
// PID  motors DC
const unsigned long SAMPLE_TIME = 100;  // time between PID updates  
const unsigned long INT_COUNT = 20;     // sufficient interrupts for accurate timing  
double setpointRightBack = 0;         // setpoint is rotational speed in Hz  
double inputRightBack = 0;              // input is PWM to motors  
double outputRightBack = 0;             // output is rotational speed in Hz  
double setpointLeftBack = 0;         // setpoint is rotational speed in Hz  
double inputLeftBack = 0;              // input is PWM to motors  
double outputLeftBack = 0;             // output is rotational speed in Hz 
//============
PID motorRightBack(&inputRightBack, &outputRightBack, &setpointRightBack, KpM, KiM, KdM, DIRECT);  
PID motorLeftBack(&inputLeftBack, &outputLeftBack, &setpointLeftBack, KpM, KiM, KdM, DIRECT);  


PID servoY(&inputSA, &outputSA, &setpointSA, Kp, Ki, Kd, DIRECT);
PID servoP(&inputSB, &outputSB, &setpointSB, Kp, Ki, Kd, DIRECT);

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
int yawMin = 30, yawMax = 120, yawCenter = 90, yawCurrent = 90;
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




