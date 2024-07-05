#ifndef MotorController_h
#define MotorController_h

#include"Actuator.h"

class MotorController : public Actuator {
public:
    MotorController(unsigned char pwmPin, unsigned char channel, unsigned int frequency, unsigned char resolution);
    void setOutput(double output);

private:
    unsigned char pwmPin;
    unsigned char channel;
    unsigned int frequency;
    unsigned char resolution;
    
    unsigned int maxDutyCycle;

};

#endif
