#include "mbed.h"
#include "IRSensor.h"
#include <array> 
#include "Controller.h"
#include "EncoderCounter.h"

#define ACTIVATION_DISTANCE 0.2
#define NUM_SENSORS 6

// leds
DigitalOut led0(PD_4);
DigitalOut led1(PD_3);
DigitalOut led2(PD_6);
DigitalOut led3(PD_2);
DigitalOut led4(PD_7);
DigitalOut led5(PD_5);
DigitalOut leds[] = { led0, led1, led2, led3, led4, led5 };

// distance sensor
AnalogIn distance_in(PA_0); 
DigitalOut enable(PG_1);
DigitalOut bit0(PF_0);
DigitalOut bit1(PF_1);
DigitalOut bit2(PF_2);

IRSensor irSensor0(distance_in, bit0, bit1, bit2, 0);
IRSensor irSensor1(distance_in, bit0, bit1, bit2, 1);
IRSensor irSensor2(distance_in, bit0, bit1, bit2, 2);
IRSensor irSensor3(distance_in, bit0, bit1, bit2, 3);
IRSensor irSensor4(distance_in, bit0, bit1, bit2, 4);
IRSensor irSensor5(distance_in, bit0, bit1, bit2, 5);
IRSensor irSensors[] = { irSensor0, irSensor1, irSensor2, irSensor3, irSensor4, irSensor5 };

// motors
DigitalOut enableMotorDriver(PG_0);  
DigitalIn motorDriverFault(PD_1); 
DigitalIn motorDriverWarning(PD_0); 
PwmOut pwmLeft(PF_9); 
PwmOut pwmRight(PF_8);
EncoderCounter counterLeft(PD_12, PD_13); 
EncoderCounter counterRight(PB_4, PC_7);
Controller controller(pwmLeft, pwmRight, counterLeft, counterRight); 

int main()
{    
    // distance sensors
    enable = 1;
    enableMotorDriver = 1;

    // motor driver
    pwmLeft.period(0.00005);
    pwmRight.period(0.00005); 
    pwmLeft = 0.5;
    pwmRight = 0.5; 


    float t = 0;

    while(true){
        IRSensor sensorLeft = irSensor2;
        IRSensor sensorCenter = irSensor3;
        IRSensor sensorRight = irSensor4;

        led2 = irSensor2.read() < 0.2;
        led3 = irSensor3.read() < 0.2;
        led4 = irSensor4.read() < 0.2;


        if(sensorLeft.read() < 0.3 || sensorCenter.read() < 0.3){
            controller.setRotationalVelocity(-1.5);
        }
        else if(sensorRight.read() < 0.3){
            controller.setRotationalVelocity(1.5);
        }
        else {
            controller.setRotationalVelocity(0);
            controller.setTranslationalVelocity(0.1);
        }

        //t += 0.010;
        ThisThread::sleep_for(10ms);
    }
}
