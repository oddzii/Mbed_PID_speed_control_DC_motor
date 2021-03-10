/**
 * Drive a robot forwards or backwards by using a PID controller to vary
 * the PWM signal to H-bridges connected to the motors to attempt to maintain
 * a constant velocity.
 */
 
#include "Motor.h"
#include "QEI.h"
#include "PID.h"
//#include <cstdint>


Motor MotorA(D10, D3, D4);  //pwm, inB, inA

QEI MotorQei(D7, D8, NC , 224);  //chanA, chanB, index, ppr

//Tuning parameters calculated from step tests;
//see http://mbed.org/cookbook/PID for examples.

PID MotorPid(1, 0.01, 0, 0.01);  //Kc, Ti, Td
using namespace std::chrono;
Timer t; 
int main() {
 
    MotorA.period(0.00005);  //Set motor PWM periods to 20KHz. 0.00005
 
    //Input and output limits have been determined
    //empirically with the specific motors being used.
    //Change appropriately for different motors.
    //Input  units: counts per second.
    //Output units: PwmOut duty cycle as %.
    //Default limits are for moving forward.
    int flag;
    float userInput;
    int x;   
    do{ 
        printf("Enter SetPoint(start 80 pps to 600 pps) \r\n");
        flag = scanf("%f", &userInput);
    }while(flag == EOF);
    x = userInput;
    printf("SetPoint:%d \r\n ", x);


    MotorPid.setInputLimits(0, 1000); 
    MotorPid.setOutputLimits(0, 0.9);
    //MotorPid.setBias(0.5); //0.45
    MotorPid.setMode(AUTO_MODE);
    //Velocity to mantain in pulses per second.
    MotorPid.setSetPoint(userInput); //We want the process variable to be 
 

    int MotorPulses      = 0; //How far the left wheel has travelled.
    int MotorPrevPulses  = 0; //The previous reading of how far the left wheel
    //has travelled.
    float MotorVelocity  = 0.0; //The velocity of the left wheel in pulses per
    //second.
    int flag2;
    int distance; //Number of pulses to travel.
    int direction;
    do{ 
        printf("Enter Direction(1 = CW and 0 = CCW) \r\n");
        direction = scanf("%f", &userInput);
    }while(direction == EOF);
    direction = userInput;
    if (direction == 1)
        printf("Clockwise \r\n ");
    else {
        printf("Counter Clockwise \r\n ");
    }

    do{ 
        printf("Enter distance(Pulses) \r\n");
        flag2 = scanf("%f", &userInput);
    }while(flag2 == EOF);
    distance = userInput;
    printf("distance:%d \r\n ", distance);

    //thread_sleep_for(1);  //Wait a few seconds before we start moving.
  
    
    t.start();
    while (MotorPulses < distance) {
        
        //Get the current pulse count and subtract the previous one to
        //calculate the current velocity in counts per second.
        if (direction==0) {
            MotorPulses = MotorQei.getPulses();
        }
        else {
            MotorPulses = -MotorQei.getPulses();
        }

        MotorVelocity = (MotorPulses - MotorPrevPulses) / 0.01;
        MotorPrevPulses = MotorPulses;
        
        //Use the absolute value of velocity as the PID controller works
        //in the % (unsigned) domain and will get confused with -ve values.
        
        MotorPid.setProcessValue(fabs(MotorVelocity));
        if (direction==0) {
            MotorA.speed(MotorPid.compute());
        }
        else {
            MotorA.speed(-MotorPid.compute());
        }


        thread_sleep_for(10); 

    //    printf("%d \r\n ", MotorPulses);
    }
    t.stop(); 
    

    MotorA.brake();
    printf("Pulses:%d \r\n ", MotorPulses);
    printf("The time taken was %llu milliseconds\r\n", duration_cast<milliseconds>(t.elapsed_time()).count());
    printf("Avg.Vilocity %llu Pulses/sec\r\n", MotorPulses*1000/duration_cast<milliseconds>(t.elapsed_time()).count());
}