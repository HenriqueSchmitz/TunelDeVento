#include"MotorController.h"
#include"UltrasonicSensor.h"
#include"PidControl.h"

const unsigned int pwmFrequency = 100;
const unsigned char pwmChannel = 0;
const unsigned char pwmPin = 27;
const unsigned char pwmResolution = 12;

const int echoPin = 21;
const int trigPin = 22;
const double ultrasonicZero = 34.1;
const unsigned int numberOfMovingAverageSamples = 15;

const unsigned int cycleTimeMicros = 100000;
unsigned long cycleStartTime = 0;

double target = 0;
double startingTarget = 0;
double targetPosition = 0;
unsigned long rampStartTime = 0;
const int microsInSecond = 1000000;
const int rampTimeMicros = 5*microsInSecond;

MotorController *motorController;
UltrasonicSensor *ultrasonicSensor;
PidControl *pidControl;

TaskHandle_t pidLoopTask;

void setup() {
  motorController = new MotorController(pwmPin, pwmChannel, pwmFrequency, pwmResolution);
  ultrasonicSensor = new UltrasonicSensor(echoPin, trigPin, numberOfMovingAverageSamples, true);
  ultrasonicSensor->setOffset(ultrasonicZero);
  pidControl = new PidControl(motorController, ultrasonicSensor);
  pidControl->setConstants(0.02, 0.005, 0.005, 0.55, 0); //@100Hz & 100ms & 15ma & 0.45 (0)
  pidControl->enableProcessVariableLogging(true);
  pidControl->enableOutputLogging(true);
  pidControl->setTarget(target);
  Serial.begin(115200);
  xTaskCreatePinnedToCore(
      pidLoop, /* Function to implement the task */
      "pidLoop", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &pidLoopTask,  /* Task handle. */
      0); /* Core where the task should run */
  cycleStartTime = micros();
}

void loop() {
    unsigned long currentTime = micros();
    if(currentTime >= (cycleStartTime + cycleTimeMicros)) {
      Serial.print(" LowerBound:");
      Serial.print(target-1);
      Serial.print(" UpperBound:");
      Serial.print(target+1);
      if(target > 0){
        pidControl->refresh();
      } else {
        motorController->setOutput(0.45);
        Serial.print(" ProcessVariable:");
        Serial.println(ultrasonicSensor->getValue());
      }
    }
    vTaskDelay(1);
}

void pidLoop(void * pvParameters) {
  while(1) {
  unsigned long currentTime = micros();
  readSerialControl(currentTime);
  implementRamp(currentTime);
  pidControl->setTarget(target);
  vTaskDelay(1);
  }
}

void readSerialControl(unsigned long currentTime) {
  if (Serial.available() > 0) {
    startingTarget = target;
    String incomingValue = Serial.readString();
    targetPosition = incomingValue.toFloat();
    if(target == 0) {
      cycleStartTime = currentTime;
    }
    rampStartTime = currentTime;
  }
}

void implementRamp(unsigned long currentTime) {
  if(currentTime < (rampStartTime + rampTimeMicros)) {
    float rampProgression = ((float)(currentTime - rampStartTime))/(float)rampTimeMicros;
    target = startingTarget + (targetPosition - startingTarget)*rampProgression;
  } else {
    target = targetPosition;
  }
}

// Setpoints Antigos
//  pidControl->setConstants(0.05, 0.01, 0.03, 0.60, 0);
//  pidControl->setConstants(0.1, 0.1, 0.05, 0.35, 0.1); @500Hz & 500u & 10ma
//  pidControl->setConstants(1, 0.1, 0.3, 0.35, 0); //@20kHz & 300u & 10ma
//  pidControl->setConstants(2, 0.5, 0.5, 0.35, 0); //@20kHz & 100u & 10ma (+-)
//  pidControl->setConstants(0.05, 0.005, 0.005, 0.55, 0); //@100Hz & 100ms & 15ma & 0.45 (0) (bom)
