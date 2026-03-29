#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H 


class My_motor
{
public:
    My_motor() = default;  
private:
    int ID_=0;
    int V_=0;
    int pwm_=0;
    int in_=1;
    int out_=0;

public:
    void Motor_Init(int ID,int turn);
    void Motor_Run(int ID,float pwm  );
    void Motor_Speed(int V);

};


extern My_motor my_motor[4];
#endif
