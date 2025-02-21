#include "mbed.h"
#include "IRSensor.h"
#include <array> 

#define ACTIVATION_DISTANCE 0.2
#define NUM_SENSORS 6

DigitalOut led0(PD_4);
DigitalOut led1(PD_3);
DigitalOut led2(PD_6);
DigitalOut led3(PD_2);
DigitalOut led4(PD_7);
DigitalOut led5(PD_5);

DigitalOut leds[] = {
    led0,
    led1,
    led2,
    led3,
    led4,
    led5,
};

AnalogIn distance2(PA_0); 
DigitalOut enable(PG_1);
DigitalOut bit0(PF_0);
DigitalOut bit1(PF_1);
DigitalOut bit2(PF_2);

IRSensor irSensor0(distance2, bit0, bit1, bit2, 0);
IRSensor irSensor1(distance2, bit0, bit1, bit2, 1);
IRSensor irSensor2(distance2, bit0, bit1, bit2, 2);
IRSensor irSensor3(distance2, bit0, bit1, bit2, 3);
IRSensor irSensor4(distance2, bit0, bit1, bit2, 4);
IRSensor irSensor5(distance2, bit0, bit1, bit2, 5);

IRSensor irSensors[] = {
    irSensor0,
    irSensor1,
    irSensor2,
    irSensor3,
    irSensor4,
    irSensor5,
};

int main()
{    
    enable = 1;

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
    }
}
