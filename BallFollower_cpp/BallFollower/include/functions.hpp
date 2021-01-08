#pragma once
#include "variables.h"



void _softRestart() 
{
  Serial.end();  //clears the serial monitor  if used
  SCB_AIRCR = 0x05FA0004;  //write value for restart
}


// Motor Control Functions

void MotorL_Brake()      
{
  digitalWrite(IN_LEFT1,LOW); 
  digitalWrite(IN_LEFT2,LOW); 
}  

void MotorR_Brake()
{
  digitalWrite(IN_RIGHT1,LOW); 
  digitalWrite(IN_RIGHT2,LOW); 
}

void Brake()
{
  MotorL_Brake();
  MotorR_Brake();
}


void MotorL_Move(int level) 
{
  if(level < 0)
  {
    digitalWrite(IN_LEFT1,HIGH); 
    digitalWrite(IN_LEFT2,LOW);  
    analogWrite(PWM_LEFT,-level);
  }
  else if (level > 0)
  {
    digitalWrite(IN_LEFT1,LOW);
    digitalWrite(IN_LEFT2,HIGH);  
    analogWrite(PWM_LEFT,level);
  }
  else
  {
    MotorL_Brake();
  }

}

void MotorR_Move(int level) 
{
  if(level < 0)
  {
    digitalWrite(IN_RIGHT1,HIGH); 
    digitalWrite(IN_RIGHT2,LOW);  
    analogWrite(PWM_RIGHT,-level);
  }
  else if (level > 0)
  {
    digitalWrite(IN_RIGHT1,LOW); 
    digitalWrite(IN_RIGHT2,HIGH);  
    analogWrite(PWM_RIGHT,level);
  }
  else
  {
    MotorR_Brake();
  }

}


void SetPowerLevel(EngineSelector side, int level, int levelConstaint = 150)
{
	level = constrain(level, -levelConstaint, levelConstaint);

	if (side == EngineSelector::Left) 
  {
		MotorL_Move(level);
	}
	if (side == EngineSelector::Right) 
  {
		MotorR_Move(level);
	}	
}

// Servo Control Functions

void calibrateServo(ServoSelector servo, int centerPosition)
{
  if (servo == ServoSelector::Yaw)
  {
    yawCenter = centerPosition;
  }
  if (servo == ServoSelector::Pitch)
  {
    pitchCenter = centerPosition;
  }

}


void calibrateServo(ServoSelector servo, int centerPosition, int minPosition, int maxPosition)
{
  if (servo == ServoSelector::Yaw)
  {
    yawMin = minPosition;
    yawMax = maxPosition;
    yawCenter = centerPosition;
  }
  if (servo == ServoSelector::Pitch)
  {
    pitchMin = minPosition;
    pitchMax = maxPosition;
    pitchCenter = centerPosition;
  }

}

void centerServos()
{

  int m_yawCenter = constrain(yawCenter, yawMin, yawMax);
  int m_pitchCenter = constrain(pitchCenter, pitchMin, pitchMax);

  servoYaw.write(m_yawCenter);
  yawCurrent = m_yawCenter;
  servoPitch.write(m_pitchCenter);
  pitchCurrent = m_pitchCenter;

}

void moveServo(ServoSelector servo, int position)
{

  int yawConstraint = constrain(position, yawMin, yawMax);
  int pitchConstrain = constrain(position, pitchMin, pitchMax);

  if (servo == ServoSelector::Yaw)
  {
    servoYaw.write(yawConstraint);
    yawCurrent = yawConstraint;
  }
  if (servo == ServoSelector::Pitch)
  {
    servoPitch.write(pitchConstrain);
    pitchCurrent = pitchConstrain;
  }

}


//Initialization functions
void initServos()
{
    servoYaw.attach(20);
    servoPitch.attach(21);
    centerServos();
}


void initESP826()
{
    //Activate ESP8266
    pinMode(ESP8266_RST, OUTPUT);
    pinMode(ESP8266_CH_PD, OUTPUT);

    digitalWrite(ESP8266_RST, HIGH);
    digitalWrite(ESP8266_CH_PD, HIGH);
}

void initMotors()
{
    pinMode(IN_LEFT1, OUTPUT);
    pinMode(IN_LEFT2, OUTPUT);
    pinMode(PWM_LEFT, OUTPUT);

    pinMode(IN_RIGHT2, OUTPUT);
    pinMode(IN_RIGHT1, OUTPUT);
    pinMode(PWM_RIGHT, OUTPUT);

    analogWriteResolution(ANALOG_WRITE_BITS);  
    
}

void isr_RIGHT_BACK()
{
  //Serial.printf("Interrupt RIGHT_BACK handler called\n");
  countIntA++;  
  if (countIntA == INT_COUNT){  
    inputRightBack = (float) INT_COUNT * 1000 / (float)(nowTime - startTimeA);  
    startTimeA = nowTime;  
    countIntA = 0;  
  }  
}

void isr_LEFT_BACK()
{
  //Serial.printf("Interrupt LEFT_BACK handler called\n");

  countIntB++;  
  if (countIntB == INT_COUNT){  
    inputLeftBack = (float) INT_COUNT * 1000 / (float)(nowTime - startTimeB);  
    startTimeB = nowTime;  
    countIntB = 0;  
  }  
}

// void initEncoders()
// {  
//   Serial.printf("Encoders init\n");
//   pinMode(EN_LEFT_BACK, INPUT_PULLUP);  
//   pinMode(EN_RIGHT_BACK, INPUT_PULLUP);  
//   attachInterrupt(digitalPinToInterrupt(EN_LEFT_BACK), isr_LEFT_BACK, FALLING);  
//   attachInterrupt(digitalPinToInterrupt(EN_RIGHT_BACK), isr_RIGHT_BACK, FALLING);  
// } 



void initPWM()
{  
  startTimeA = millis();  
  startTimeB = millis();  
  motorRightBack.SetOutputLimits(MIN_PWM, MAX_PWM);  
  motorLeftBack.SetOutputLimits(MIN_PWM, MAX_PWM);  
  motorRightBack.SetSampleTime(SAMPLE_TIME);  
  motorLeftBack.SetSampleTime(SAMPLE_TIME);  
  motorRightBack.SetMode(AUTOMATIC);  
  motorLeftBack.SetMode(AUTOMATIC);  
}  



void initLED()
{
    pinMode(led, OUTPUT); 
}

void initSerial(int baudRate = 9600)
{
    Serial.begin(baudRate);
    delay(1000);
}


