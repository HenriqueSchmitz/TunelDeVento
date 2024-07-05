#include"esp32-hal.h"
#include"MotorController.h"


MotorController::MotorController(unsigned char pwmPin, unsigned char channel, unsigned int frequency, unsigned char resolution){
  this->pwmPin = pwmPin;
  this->channel = channel;
  this->frequency = frequency;
  this->resolution = resolution;
  ledcSetup(channel, frequency, resolution);
  ledcAttachPin(pwmPin, channel);
  this->maxDutyCycle = (unsigned int)(pow(2, resolution) - 1);
}

void MotorController::setOutput(double output){
  if(output >= 1){
    ledcWrite(this->channel, 0);
  } else if(output <= 0){
    ledcWrite(this->channel, this->maxDutyCycle);
  } else {
    unsigned int dutyCycle = (1 - output) * this->maxDutyCycle;
    ledcWrite(this->channel, dutyCycle);
  }
}
