#ifndef UltrasonicSensor_h
#define UltrasonicSensor_h

#include"Sensor.h"

class UltrasonicSensor : public Sensor {
  public:
    UltrasonicSensor(unsigned char echoPin, unsigned char trigPin, unsigned int numberOfSamples);
    UltrasonicSensor(unsigned char echoPin, unsigned char trigPin, unsigned int numberOfSamples, bool isMeasurementInverted);
    ~UltrasonicSensor();
    void setOffset(double offset);
    double getValue();
    
  private:
    double readSensor();
    
    unsigned char echoPin;
    unsigned char trigPin;

    bool isMeasurementInverted;
    double offset;
    unsigned int numberOfSamples;
    unsigned int currentSamplePosition;
    double *measurements;
};

#endif
