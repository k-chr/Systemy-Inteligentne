#include <Arduino.h>

/*
This code is based on the examples at http://forum.arduino.cc/index.php?topic=396450
*/


// Example 5 - parsing text and numbers with start and end markers in the stream

#include "variables.h"
#include "functions.hpp"
#include "NewPing.h"
#include <Encoder.h>
#include "ISAMobile.h"

float YawCalibrationCenter = 73.0f;
float PitchCalibrationCenter = 58.0f;

const byte numChars = 64;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

dataPacket packet;


boolean newData = false;
boolean obstacleTooClose = false;
boolean obstacleVeryClose = false;

float yawRequested = 0;
float pitchRequested = 0;
float distanceRequested = 0;
float velocityLeftRequested = 0;
float velocityRightRequested = 0;
float motorOutputLimit = 150.0f;

NewPing sonarMiddle(ultrasound_trigger_pin[(int)UltraSoundSensor::Front], 
                     ultrasound_echo_pin[(int)UltraSoundSensor::Front], 
                     maxDistance);

unsigned int pingDelay = 30; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.


// ##########
Encoder rightSide(ENCODER_REAR_RIGHT_1, ENCODER_REAR_RIGHT_2);
Encoder leftSide(ENCODER_REAR_LEFT_1, ENCODER_REAR_LEFT_2);

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//============

dataPacket parseData() {      // split the data into its parts

    dataPacket tmpPacket;

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    strcpy(tmpPacket.message, strtokIndx); // copy it to messageFromPC

    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    tmpPacket.first = atof(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    tmpPacket.second = atof(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    if(NULL != strtokIndx)
    {
        tmpPacket.third = atof(strtokIndx);
    }

    return tmpPacket;
}


void showParsedData(dataPacket packet) {
    Serial.print("Message ");
    Serial.println(packet.message);
    Serial.print("Yaw ");
    Serial.println(packet.first);
    Serial.print("Pitch ");
    Serial.println(packet.second);
}

void slowDownSatan()
{
    inputLeftBack = setpointLeftBack + 5;
    inputRightBack = setpointRightBack + 5;
    motorRightBack.Compute();  
    motorLeftBack.Compute(); 

    SetPowerLevel(EngineSelector::Left, outputLeftBack);
    SetPowerLevel(EngineSelector::Right, outputRightBack);
}

void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  // Don't do anything here!
  if (sonarMiddle.check_timer()) { // This is how you check to see if the ping was received.
    // Here's where you can add code.
    // Serial.print("Ping: ");
    int computedDistanceInCm = sonarMiddle.ping_result / US_ROUNDTRIP_CM;
    obstacleTooClose = computedDistanceInCm >= 0 && computedDistanceInCm < 30;
    obstacleVeryClose = computedDistanceInCm >= 0 && computedDistanceInCm < 10;
    // Serial.print(computedDistanceInCm); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
    // Serial.println("cm");
  }
  // Don't do anything here!
}

void setup() {

    initSerial(115200);
    Serial.println("This demo expects 3 pieces of data - text, an integer and a floating point value");
    Serial.println("Enter data in this style <HelloWorld, 12, 24.7>  ");
    Serial.println();
    pingTimer = millis(); // Start now.

    // Each platform has to do this independently, checked manually
    calibrateServo(ServoSelector::Yaw, (int)YawCalibrationCenter);
    calibrateServo(ServoSelector::Pitch, (int)PitchCalibrationCenter);

    initMotors();
    initPWM();
    initServos();
    centerServos();

    servoY.SetMode(AUTOMATIC);
    servoP.SetMode(AUTOMATIC);
    inputSA = 0;
    servoY.SetOutputLimits(-90, 90);
    motorRightBack.SetOutputLimits(-0, motorOutputLimit);
    motorLeftBack.SetOutputLimits(-0, motorOutputLimit);
    setpointLeftBack = 20;
    setpointRightBack = 20;
    initESP826();
    // initLED();
    Brake();
    delay(500);

    // Serial.println("Initalization ended");

}

//============

long positionLeft  = 0;
long positionRight = 0;
void loop() {
         // Notice how there's no delays in this sketch to allow you to do other processing in-line while doing distance pings.
   if (millis() >= pingTimer)
   {   // pingSpeed milliseconds since last ping, do another ping.
      pingTimer += pingDelay;      // Set the next ping time.
      sonarMiddle.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
      positionLeft = leftSide.read();
      positionRight = rightSide.read();
     // Serial.println("Left = " + String(positionLeft) + "; Right = " + String(positionRight));

   }
    // parse input data
    recvWithStartEndMarkers();
    {
        nowTime = millis();
        // if(nowTime > 10000 && setpointLeftBack)
        // {
        //     setpointLeftBack = 0;
        //     // Serial.println("SetPointLeftBack changed");
        // }  
    }
    if (obstacleVeryClose == true && (outputLeftBack >= 0.1 || outputRightBack >= 0.1)) {
        Brake();
    } else if (obstacleTooClose == true && (outputLeftBack >= 0.1 || outputRightBack >= 0.1)) {
        slowDownSatan();
    } else {
        if (newData == true) {
            strcpy(tempChars, receivedChars);
                // this temporary copy is necessary to protect the original data
                //   because strtok() used in parseData() replaces the commas with \0
            packet = parseData();
            // showParsedData(packet);

            if (strcmp(packet.message, "servo") == 0)
            {

                if (isStopped == false)
                {
                    yawRequested = packet.first;
                    pitchRequested = packet.second;
                    distanceRequested = packet.third;

                    {
                        //float yawError = -(yawRequested / (HorizontalFOV/2) );
                        
                        float yawError = - yawRequested;
                        float Kp = 25.0f;
                        float Ki = 4.0f;

                        float output = Kp * yawError + Ki * yawErrorAccumulated;
                        output *= 0.8f;
                        yawErrorAccumulated += yawError;
                        
                        // move servo
                        moveServo(ServoSelector::Yaw,  (int)(YawCalibrationCenter + output));
                    }
                    {
                        
                        // float pitchError = -(pitchRequested / (VerticalFOV/2));
                        float pitchError = - pitchRequested;
                        float Kp = 15.0f;
                        float Ki = 3.0f;

                        float output = Kp * pitchError + Ki * pitchErrorAccumulated;
                        output *= 0.8f;
                        pitchErrorAccumulated += pitchError;
                        
                        // move servo
                        moveServo(ServoSelector::Pitch,  (int)(PitchCalibrationCenter + output));
                    }
                    
                    {
                        const double yawInputMotorsError = (YawCalibrationCenter - yawCurrent) * 0.5;
                        const double inputMotors = distanceRequested * 1000;
                        // inputLeftBack = -inputMotors;
                        // inputRightBack = inputMotors;

                        inputLeftBack = inputMotors - yawInputMotorsError;
                        inputRightBack = inputMotors + yawInputMotorsError;


                        motorRightBack.Compute();  
                        motorLeftBack.Compute(); 

                        
                        // SetPowerLevel(EngineSelector::Left, outputLeftBack);
                        // SetPowerLevel(EngineSelector::Right, outputRightBack);
                        SetPowerLevel(EngineSelector::Left, outputLeftBack);
                        SetPowerLevel(EngineSelector::Right, outputRightBack);
                        

                        // Serial.printf("yaw: %d, inputMotors: %f\n", yawCurrent, inputMotors);
                        Serial.printf("leftOutput: %f, rightOutput: %f\n", outputLeftBack, outputRightBack);
                        // Serial.printf("Computed A output %d\n", leftSide.read());
                        // Serial.printf("Computed B output %d\n", rightSide.read());
                    }
                }
            }

            if (strcmp(packet.message, "pause") == 0)
            {
                slowDownSatan();
            }

            if (strcmp(packet.message, "stop") == 0)
            {
                isStopped = true;
            }

            if (strcmp(packet.message, "start") == 0)
            {
                isStopped = false;
            }

            newData = false;
        } 
    }

}
