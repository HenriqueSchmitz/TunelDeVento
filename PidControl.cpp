#include"PidControl.h"

#include<Arduino.h>
#include"Actuator.h"
#include"Sensor.h"

PidControl::PidControl(Actuator *actuator, Sensor *sensor){
  this->actuator = actuator;
  this->actuator->setOutput(0);
  this->sensor = sensor;
  this->kp = 0;
  this->ki = 0;
  this->kd = 0;
  this->kf = 0;
  this->deadBand = 0;
  this->targetValue = 0;
  this->lastRefreshTime = micros();
  this->lastExecutionError = 0;
  this->integralAccumulator = 0;
  this->shouldLogP = false;
  this->shouldLogI = false;
  this->shouldLogD = false;
  this->shouldLogF = false;
  this->shouldLogOutput = false;
  this->shouldLogError = false;
  this->shouldLogProcessVariable = false;
}

void PidControl::setConstants(double kp, double ki, double kd, double kf, double deadBand){
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->kf = kf;
  this->deadBand = deadBand;
}

void PidControl::setTarget(double targetValue){
  this->targetValue = targetValue;
}

void PidControl::refresh(){
  unsigned long currentTime = micros();
  double error = this->measureError();
  double output = this->calculateOutput(currentTime, error);
  this->actuator->setOutput(output);
  this->lastExecutionError = error;
  this->lastRefreshTime = currentTime;
}

double PidControl::measureError(){
  double processValue = this->sensor->getValue();
  if(this->shouldLogProcessVariable) {
    Serial.print(" ProcessVariable:");
    Serial.print(processValue);
  }
  double error = this->targetValue - processValue;
  if(error < this->deadBand && error > (-1*this->deadBand)){
    return 0;
  }
  return error;
}

double PidControl::calculateOutput(unsigned long currentTime, double error){
  double timeDelta = ((double)(currentTime - this->lastRefreshTime))/this->MICROS_IN_SECOND;
  double p = this->calculatePValue(error);
  double i = this->calculateIValue(error, timeDelta);
  double d = this->calculateDValue(error, timeDelta);
  double f = this->calculateFValue();
  double output = (p + i + d + f);
  if(this->shouldLogOutput){
//    Serial.print(" Output:");
//    Serial.println(output);
    Serial.println();
  }
  return output;
}

double PidControl::calculatePValue(double error){
  double p = this->kp * error;
  Serial.print(" p:");
  Serial.print(p);
  return p;
}

double PidControl::calculateIValue(double error, double timeDelta){
  // This uses both the current error and last error to create a trapezoidal
  // profile instead of a "square" profile, thus better estimating the integral
  // of this error.
  double currentI = this->ki * (error + this->lastExecutionError) * timeDelta / 2;
  this->integralAccumulator += currentI;
  double i = this->integralAccumulator;
  Serial.print(" i:");
  Serial.print(i);
  return i;
  
}

double PidControl::calculateDValue(double error, double timeDelta){
  double errorDelta = error - this->lastExecutionError;
  double d = this->kd * errorDelta / timeDelta;
//  Serial.print(" d:");
//  Serial.print(d);
  return d;
}

double PidControl::calculateFValue(){
  double f = 0;
  if(this->targetValue > 0){
    f = this->kf;
  } else if(this->targetValue){
    f = -1 * this->kf;
  }
  Serial.print(" f:");
  Serial.print(f);
  return f;
}

void PidControl::logComponentValues(){
  
}

void PidControl::logControlValues(){
  Serial.print(" output:");
  Serial.print(0);
}

void PidControl::enablePLogging(bool shouldLogP){
  this->shouldLogP = shouldLogP;
}

void PidControl::enableILogging(bool shouldLogI){
  this->shouldLogI = shouldLogI;
}

void PidControl::enableDLogging(bool shouldLogD){
  this->shouldLogD = shouldLogD;
}

void PidControl::enableFLogging(bool shouldLogF){
  this->shouldLogF = shouldLogF;
}

void PidControl::enableOutputLogging(bool shouldLogOutput){
  this->shouldLogOutput = shouldLogOutput;
}

void PidControl::enableErrorLogging(bool shouldLogError){
  this->shouldLogError = shouldLogError;
}

void PidControl::enableProcessVariableLogging(bool shouldLogProcessVariable){
  this->shouldLogProcessVariable = shouldLogProcessVariable;
}
