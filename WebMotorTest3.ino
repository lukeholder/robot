/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-esp8266-web-server-outputs-momentary-switch/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#ifdef ESP32
  #include <WiFi.h>
  #include <AsyncTCP.h>
#else
  #include <ESP8266WiFi.h>
  #include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"


//Motor Pins
#define m1Pin1        14
#define m1Pin2        26
#define m2Pin1        16
#define m2Pin2        17
//Indicator LED Pin
#define indicatorPin  2
//Motor Constants
#define pwmFreq       30000
#define pwmResolution 8
#define pwmChannelM1  0
#define pwmChannelM2  1
#define minDutyCycle  155
#define maxDutyCycle  255
#define minSpeed      0
#define maxSpeed      maxDutyCycle - minDutyCycle
#define defaultSpeed  30
#define minRamp       1
#define maxRamp       10
#define minTurnRatio  1
#define maxTurnRatio  4



typedef enum {
  mCoast,
  mForward,
  mReverse,
  mBrake,
} MotorState_t;

struct Motor_t {
  MotorState_t state;
  int pwmChannel;
  int fwdPin;
  int revPin;
  int currSpeed;
  int goalSpeed;
  int rampRate;
};


// REPLACE WITH YOUR NETWORK CREDENTIALS
const char* ssid = "Internat";
const char* password = "p1kachu1";

// replace staticIP and gateway address based on your home router settings
IPAddress staticIP(192, 168, 1, 200);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

Motor_t rightMotor = {.state = mCoast, .pwmChannel = pwmChannelM1, .fwdPin = m1Pin1, .revPin = m1Pin2, .currSpeed = minSpeed, .goalSpeed = minSpeed, .rampRate = 1};
Motor_t leftMotor = {.state = mCoast, .pwmChannel = pwmChannelM2, .fwdPin = m2Pin1, .revPin = m2Pin2, .currSpeed = minSpeed, .goalSpeed = minSpeed, .rampRate = 1};

AsyncWebServer server(80);

void setup() {
  //Initialise Indicator LED
  pinMode(indicatorPin, OUTPUT);
  digitalWrite(indicatorPin, LOW);
  
  // Initialise Motor Pins and PWM channels
  InitMotorDrive(rightMotor);
  InitMotorDrive(leftMotor);

    // Initialize SPIFFS
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.config(staticIP, gateway, subnet);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("ESP IP Address: http://");
  Serial.println(WiFi.localIP());
  
  // Send web page to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false);
  });

    // Send web page to client
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/style.css", String(), false);
  });


  // Receive an HTTP GET request
  server.on("/F", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("Forward");
    MoveForward(50, 5, 0);
    request->send(200, "text/plain", "ok");
  });

    // Receive an HTTP GET request
  server.on("/B", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("Back");
    MoveReverse(50, 5, 0);
    request->send(200, "text/plain", "ok");
  });

  // Receive an HTTP GET request
  server.on("/S", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.println("Off");
    moveCoast();
    request->send(200, "text/plain", "ok");
  });
  
 // server.onNotFound(handleNotFound);
  server.begin();
}

//with this calculation it is going to jump either side of the desired motor speed
void loop() {
  //function to update speed
  if(rightMotor.goalSpeed != rightMotor.currSpeed)
  {
    if(rightMotor.goalSpeed > rightMotor.currSpeed)
    {
      rightMotor.currSpeed = rightMotor.currSpeed + rightMotor.rampRate;
      ledcWrite(rightMotor.pwmChannel, rightMotor.currSpeed);
    }else
    {
      rightMotor.currSpeed = rightMotor.currSpeed - rightMotor.rampRate;
      ledcWrite(rightMotor.pwmChannel, rightMotor.currSpeed);
    }
  }

    if(leftMotor.goalSpeed != leftMotor.currSpeed)
  {
    if(leftMotor.goalSpeed > leftMotor.currSpeed)
    {
      leftMotor.currSpeed = leftMotor.currSpeed + leftMotor.rampRate;
      ledcWrite(leftMotor.pwmChannel, leftMotor.currSpeed);
    }else
    {
      leftMotor.currSpeed = leftMotor.currSpeed - leftMotor.rampRate;
      ledcWrite(leftMotor.pwmChannel, leftMotor.currSpeed);
    }
  }
  
  delay(10);
}

//Motor Control functions
void MoveForward(int motSpeed, int ramp, int turnRatio)
{
  int motSpeedDCRight,motSpeedDCLeft, motRampDCRight, motRampDCLeft, leftTurnRatio = 0, rightTurnRatio = 0;

  //If motors in reverse stop before changing to forward
  if((rightMotor.state == mReverse) || (rightMotor.state == mReverse))
  {
    moveBrake();
    delay(10);
  }
 
  if(turnRatio > 0)
  {
    leftTurnRatio = GetTurnRatio(turnRatio);
  } else if(turnRatio < 0)
  {
    rightTurnRatio = GetTurnRatio(abs(turnRatio));
  }

  motSpeedDCRight = getSpeedDC(motSpeed >> rightTurnRatio);
  motSpeedDCLeft = getSpeedDC(motSpeed >> leftTurnRatio);
  motRampDCRight = GetRampDC(ramp >> rightTurnRatio);
  motRampDCLeft = GetRampDC(ramp >> leftTurnRatio);
  
  rightMotor.goalSpeed = motSpeedDCRight;
  leftMotor.goalSpeed = motSpeedDCLeft;
  rightMotor.rampRate = motRampDCRight;
  leftMotor.rampRate = motRampDCLeft;
  MotorSetForward(rightMotor);
  MotorSetForward(leftMotor);
}

void MoveReverse(int motSpeed, int ramp, int turnRatio)
{
  int motSpeedDCRight,motSpeedDCLeft, motRampDCRight, motRampDCLeft, leftTurnRatio = 0, rightTurnRatio = 0;

  //If motors moving forward stop before changing to reverse
  if((rightMotor.state == mForward) || (rightMotor.state == mForward))
  {
    moveBrake();
    delay(10);
  }
 
  if(turnRatio > 0)
  {
    leftTurnRatio = GetTurnRatio(turnRatio);
  } else if(turnRatio < 0)
  {
    rightTurnRatio = GetTurnRatio(abs(turnRatio));
  }
  
  motSpeedDCRight = getSpeedDC(motSpeed >> rightTurnRatio);
  motSpeedDCLeft = getSpeedDC(motSpeed >> leftTurnRatio);
  motRampDCRight = GetRampDC(ramp >> rightTurnRatio);
  motRampDCLeft = GetRampDC(ramp >> leftTurnRatio);
  
  rightMotor.goalSpeed = motSpeedDCRight;
  leftMotor.goalSpeed = motSpeedDCLeft;
  rightMotor.rampRate = motRampDCRight;
  leftMotor.rampRate = motRampDCLeft;
  MotorSetReverse(rightMotor);
  MotorSetReverse(leftMotor);
}

void moveCoast()
{
  rightMotor.goalSpeed = minDutyCycle;
  leftMotor.goalSpeed = minDutyCycle;
  rightMotor.rampRate = minRamp;
  leftMotor.rampRate = minRamp;
  MotorSetCoast(rightMotor);
  MotorSetCoast(leftMotor);
}

void moveBrake()
{
  rightMotor.goalSpeed = minDutyCycle;
  leftMotor.goalSpeed = minDutyCycle;
  rightMotor.rampRate = minRamp;
  leftMotor.rampRate = minRamp;
  MotorSetBrake(rightMotor);
  MotorSetBrake(leftMotor);
}

//Hardware Motor Configurations
void MotorSetCoast(Motor_t motor)
{
  motor.state = mCoast;
  ledcDetachPin(motor.fwdPin);
  ledcDetachPin(motor.revPin);
  digitalWrite(motor.fwdPin, LOW);
  digitalWrite(motor.revPin, LOW);
}

void MotorSetForward(Motor_t motor)
{
  motor.state = mForward;
  ledcDetachPin(motor.revPin);
  digitalWrite(motor.revPin, LOW);
  ledcAttachPin(motor.fwdPin, motor.pwmChannel);
}

void MotorSetReverse(Motor_t motor)
{
  motor.state = mReverse;
  ledcDetachPin(motor.fwdPin);
  digitalWrite(motor.fwdPin, LOW);
  ledcAttachPin(motor.revPin, motor.pwmChannel);
}

void MotorSetBrake(Motor_t motor)
{
  motor.state = mBrake;
  ledcDetachPin(motor.fwdPin);
  ledcDetachPin(motor.revPin);
  digitalWrite(motor.fwdPin, HIGH);
  digitalWrite(motor.revPin, HIGH);
}

void InitMotorDrive(Motor_t motor)
{
  pinMode(motor.fwdPin, OUTPUT);
  pinMode(motor.revPin, OUTPUT);
  digitalWrite(motor.fwdPin, LOW);
  digitalWrite(motor.revPin, LOW);
  
  // configure LED PWM functionalitites
  ledcSetup(motor.pwmChannel, pwmFreq, pwmResolution);
  ledcWrite(motor.pwmChannel, motor.currSpeed);
}

//Variable Control
//set default speed if outside limits
int getSpeedDC(int motSpeed)
{
  if((motSpeed < minSpeed) || (motSpeed > maxSpeed))
  {
    return minDutyCycle + defaultSpeed;
  }
  return minDutyCycle + motSpeed;
}

//set default ramp if outside limits
int GetRampDC(int ramp)
{
  if(ramp < minRamp)
  {
    return minRamp;
  } else if(ramp > maxRamp)
  {
    return maxRamp;
  }
  return ramp;
}

//set default ramp if outside limits
int GetTurnRatio(int turnRatio)
{
  if(turnRatio < minTurnRatio)
  {
    return minTurnRatio;
  } else if(turnRatio > maxTurnRatio)
  {
    return maxTurnRatio;
  }

  return turnRatio;
}
