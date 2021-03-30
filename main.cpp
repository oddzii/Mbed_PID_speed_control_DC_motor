#include "Motor.h"
#include "QEI.h"
#include "PID.h"

using namespace std::chrono;
int userInput;
int setpoint;   
int length; //Number of pulses to travel.
int direction;
int MotorAPulses      = 0; //How far the left wheel has travelled.
int MotorAPrevPulses  = 0; //The previous reading of how far the left wheel has travelled.
float MotorAVelocity  = 0.0; //The velocity of the left wheel in pulses per second.
float Kc = 1;
float tauI = 0.01;
float tauD = 0;
float interval = 0.01 ; //  second unit
uint64_t time_l ; // time of movement
Timer t; 

PID MotorAPid(Kc, tauI, tauD, interval);  //Kc, Ti, Td , interval time
Motor MotorA(D10, D3, D4);  //pwm, inB, inA
QEI MotorAQei(D7, D8, NC , 224);  //chanA, chanB, index, ppr

void getDir();
void getSpeed();
void UserIn();
void setUp();

int main() {

    UserIn();
    setUp();
    thread_sleep_for(1000);  //Wait a few seconds before we start moving.
    
    t.start();
    while (MotorAPulses < length) {
        getDir();
        getSpeed();
        thread_sleep_for(interval*1000);  // interval in miliseconds
        
    }
    t.stop(); 
    
    MotorA.brake();

    printf("Final Pulses:%d \r\n ", MotorAPulses);
    time_l = duration_cast<milliseconds>(t.elapsed_time()).count();
    printf("The time taken was %llu milliseconds\r\n", time_l );
    printf("Avg.Vilocity: %llu Pulses/sec\r\n", MotorAPulses*1000/time_l);
}


void getDir(){
        if (direction==0) {
            MotorAPulses = MotorAQei.getPulses();
        }
        else {
            MotorAPulses = -MotorAQei.getPulses();
        }
    }

void getSpeed(){
        MotorAVelocity = (MotorAPulses - MotorAPrevPulses) / interval;
        MotorAPrevPulses = MotorAPulses;
        MotorAPid.setProcessValue(fabs(MotorAVelocity));
        if (direction==0) {
            MotorA.speed(MotorAPid.compute());
        }
        else {
            MotorA.speed(-MotorAPid.compute());
        }
    }

void UserIn(){

    do{ 
        printf("Enter SetPoint(start 80 pps to 600 pps) \r\n");
        setpoint = scanf("%d", &userInput);
    }while(setpoint == EOF);
    setpoint = userInput;
    do{ 
        printf("Enter Direction(1 = CW and 0 = CCW) \r\n");
        direction = scanf("%d", &userInput);
    }while(direction == EOF);
    direction = userInput; 
    do{ 
        printf("Enter length(Pulses) \r\n");
        length = scanf("%d", &userInput);
    }while(length == EOF);
    length = userInput; 

    printf("SetPoint:%d \r\n ", setpoint);
    if (direction == 1)
        printf("Direction : Clockwise \r\n ");
    else {
        printf("Direction : Counter Clockwise \r\n ");
    }
    printf("length : %d \r\n ", length);
    }

void setUp(){
    MotorA.period(0.00005);  //Set motor PWM periods to 20KHz. 0.00005
    MotorAPid.setInputLimits(0, 1000); 
    MotorAPid.setOutputLimits(0, 0.9);  
    MotorAPid.setMode(AUTO_MODE);
    MotorAPid.setSetPoint(setpoint); ////Velocity to mantain in pulses per second.We want the process variable to be 
    }
