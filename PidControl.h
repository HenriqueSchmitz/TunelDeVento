#ifndef PidControl_h
#define PidControl_h

#include"Actuator.h"
#include"Sensor.h"

class PidControl {
public:
    PidControl(Actuator *actuator, Sensor *sensor);
    void setConstants(double kp, double ki, double kd, double kf, double deadBand);
    void setTarget(double targetValue);
    void refresh();

    void enablePLogging(bool shouldLogP);
    void enableILogging(bool shouldLogI);
    void enableDLogging(bool shouldLogD);
    void enableFLogging(bool shouldLogF);
    void enableOutputLogging(bool shouldLogOutput);
    void enableErrorLogging(bool shouldLogError);
    void enableProcessVariableLogging(bool shouldLogProcessVariable);

private:
    Actuator *actuator;
    Sensor *sensor;
    
    double kp;
    double ki;
    double kd;
    double kf;
    double deadBand;
    
    double targetValue;
    unsigned long lastRefreshTime;
    double lastExecutionError;
    double integralAccumulator;

    bool shouldLogP;
    bool shouldLogI;
    bool shouldLogD;
    bool shouldLogF;
    bool shouldLogOutput;
    bool shouldLogError;
    bool shouldLogProcessVariable;

    const double MICROS_IN_SECOND = 1000000;

    
    double measureError();
    double calculateOutput(unsigned long currentTime, double error);
    double calculatePValue(double error);
    double calculateIValue(double error, double timeDelta);
    double calculateDValue(double error, double timeDelta);
    double calculateFValue();
    void logComponentValues();
    void logControlValues();
};

#endif
