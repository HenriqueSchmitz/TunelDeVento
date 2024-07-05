#include"UltrasonicSensor.h"

#include<Arduino.h>

UltrasonicSensor::UltrasonicSensor(unsigned char echoPin, unsigned char trigPin, unsigned int numberOfSamples) {
  this->echoPin = echoPin;
  this->trigPin = trigPin;
  pinMode(this->echoPin, INPUT);
  pinMode(this->trigPin, OUTPUT);
  this->isMeasurementInverted = false;
  this->offset = 0;
  this->numberOfSamples = numberOfSamples;
  this->measurements = new double[numberOfSamples];
  this->currentSamplePosition = 0;
  for(int zeroingPosition = 0; zeroingPosition < numberOfSamples; zeroingPosition++){
    this->measurements[zeroingPosition] = 0;
  }
}

UltrasonicSensor::UltrasonicSensor(unsigned char echoPin, unsigned char trigPin, unsigned int numberOfSamples, bool isMeasurementInverted) {
  this->echoPin = echoPin;
  this->trigPin = trigPin;
  pinMode(this->echoPin, INPUT);
  pinMode(this->trigPin, OUTPUT);
  this->isMeasurementInverted = isMeasurementInverted;
  this->offset = 0;
  this->numberOfSamples = numberOfSamples;
  this->measurements = new double[numberOfSamples];
  this->currentSamplePosition = 0;
  for(int zeroingPosition = 0; zeroingPosition < numberOfSamples; zeroingPosition++){
    this->measurements[zeroingPosition] = 0;
  }
}

UltrasonicSensor::~UltrasonicSensor(){
  delete this->measurements;
}

void UltrasonicSensor::setOffset(double offset) {
  this->offset = offset;
}

double UltrasonicSensor::getValue() {
  double measurement = this->readSensor();
  this->measurements[this->currentSamplePosition] = measurement;
  this->currentSamplePosition++;
  if(this->currentSamplePosition >= this->numberOfSamples){
    this->currentSamplePosition = 0;
  }
  double totalMeasurement = 0;
  double maxMeasurement = this->measurements[0];
  double minMeasurement = this->measurements[0];
  for(int averagingPosition = 0; averagingPosition < this->numberOfSamples; averagingPosition++){
    double currentMeasurement = this->measurements[averagingPosition];
    totalMeasurement += currentMeasurement;
    if(currentMeasurement > maxMeasurement){
      maxMeasurement = currentMeasurement;
    }
    if(currentMeasurement < minMeasurement){
      minMeasurement = currentMeasurement;
    }
  }
  totalMeasurement -= maxMeasurement;
  totalMeasurement -= minMeasurement;
  return totalMeasurement / (this->numberOfSamples - 2);
}

double UltrasonicSensor::readSensor() {
  digitalWrite(this->trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(this->trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(this->trigPin, LOW);
  unsigned long duration = pulseIn(this->echoPin, HIGH);
  double measurementInCm = duration * 0.034 / 2;
  if (this->isMeasurementInverted){
    return this->offset - measurementInCm;
  }
  return this->offset + measurementInCm;
}
