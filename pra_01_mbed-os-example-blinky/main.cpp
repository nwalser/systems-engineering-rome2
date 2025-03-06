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

    // motor driver
    pwmLeft.period(0.00005);
    pwmRight.period(0.00005); 
    pwmLeft = 0.5;
    pwmRight = 0.5; 

    
    Timer timer = Timer();
    bool state = 0;
    timer.start();

    while(true){
        printf("distance: ");

        if(timer.elapsed_time() > 500ms){
            state = !state;
            timer.reset();
        }

        for(int i = 0; i < NUM_SENSORS; i++){
            float distance = irSensors[i].read();

            leds[i] = (distance < ACTIVATION_DISTANCE) & state;
            
            printf("%05d ", (int)(1000.0f*distance));
        }

        printf("[mm] \r\n");


        enableMotorDriver = 1;
        controller.setDesiredSpeedLeft(50.0);
        controller.setDesiredSpeedRight(-50.0);

        //printf("actual speed (left/right): %.3f / %.3f [rpm]\r\n", controller.getActualSpeedLeft(), controller.getActualSpeedRight());
   
        // angle_velocity
        // velocity

        // R = 150mm

        // vr = R * angle_velocity + velocity
        // vl = R * -angle_velocity + velocity

        // velocity =  (vl + vr) / 2
        // angle_velocity = 1 / (2*R) * (vr-vl)
    }
}
