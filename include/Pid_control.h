#ifndef PID_CONTROL_H
#define PID_CONTROL_H
#include "Arduino.h"
#include "Pid.h"


void Pid_controller_init();

void Pid_controller_run(float TARGET);//一次控制所有电机


class PID_run
{
public:
    PID_run() = default;
   
private:
    int ID_;
    float TARGET_;

public:
    void Pid_run_();
    void Pid_setgoal(int ID,float TARGET);
};




extern PID_run run[4];
extern PIDController pid_controller[4];
#endif
