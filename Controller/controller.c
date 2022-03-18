#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <math.h>
#include <time.h>
#include "IMU.h"

/*Robot chassis specifications
*/
#define d 2.25 //distance from center of the chasis to the wheel
#define r 1.1875 // radius of the wheel
#define PI 3.14159265

/*STEP MULTIPLIER 
/ Determined by driver configuration (see driver wiring diagram)
*/
#define Step3_Size .5
#define Step2_Size .5
#define Step1_Size .5

/* TIMER CONFIGURATION
/  TIMER (ms) sets the time between of motor callbacks
/  dt (ms) sets the loop sleep time, which allows the timer to run.
/  Experimentally, the control is optimal when TIMER * dt = 200
/  Together, dt and TIMER are parameters that trade off between controller sensitivity
/  and the wheels' maximum rotational velocity. 
*/
#define TIMER 2
#define dt 100

/* PID CONTROL  
/  Convert error in roll and pitch (any measurement above 0)
/  to twist commands.
/  Inputs: Roll, Pitch (degrees)
/  Outputs: Vx, Vy (in./sec.)
*/
#define Kp .29 //.29 //0.69
#define Kd .27 //.27 //0.35 //0.04 //.25 //.3 //.3
#define Ki 0

/*Microcontroller configuration*/
const int stepPin3 = 19;
const int dirPin3 = 20;

const int stepPin1 = 22;
const int dirPin1 = 21;

const int stepPin2 = 18;
const int dirPin2 = 17;

/* Global Variables for motor controll */
float rps_motor1;
float rps_motor2;
float rps_motor3;

int motor1_count = 0;
int motor2_count = 0;
int motor3_count = 0;
int print_count = 0;

float delay1;
float delay2;
float delay3;

float e_int_vx = 0;
float saved_e_vx = 0;
float e_int_vy = 0;
float saved_e_vy = 0;

int dir1;
int dir2;
int dir3;

double roll_acc;
double pitch_acc;
double roll_gyro;
double pitch_gyro;

struct Twist{
    float vx;
    float vy;
    float w;
}; //in. per sec.

typedef struct Twist Twist;

Twist twist_cmd;

/* Global variables for measurement updates*/
IMU_Data raw_IMU;
IMU_Data saved_IMU[4];
IMU_Data filtered_IMU;
Orientation calibration;
Orientation calced_RPY;
Orientation saved_RPY;

/* Helper function prototypes */
Twist get_twist(Orientation RPY);
IMU_Data filter_IMU(IMU_Data IMU);
void update_wheels(Twist twist);
void update_motors();
bool motor_callback(void);

/* filter_IMU
/  Calculates a rolling average for the IMU measurement 
*/
IMU_Data filter_IMU(IMU_Data IMU){
    IMU_Data filtered;
    IMU_Data sum;

    sum.accelX =  IMU.accelX;
    sum.accelY = IMU.accelY;
    sum.accelZ = IMU.accelZ;
    sum.gyroX = IMU.gyroX;
    sum.gyroY = IMU.gyroY;
    sum.gyroZ = IMU.gyroZ;

    for (int i=0; i<3; i++){
        sum.accelX +=  saved_IMU[i].accelX;
        sum.accelY += saved_IMU[i].accelY;
        sum.accelZ += saved_IMU[i].accelZ;
        sum.gyroX += saved_IMU[i].gyroX;
        sum.gyroY += saved_IMU[i].gyroY;
        sum.gyroZ += saved_IMU[i].gyroZ;
    }

    saved_IMU[0] = IMU;
    for (int i=0; i<3; i++){
        saved_IMU[i+1] = saved_IMU[i];
    }

    filtered.accelX = sum.accelX/4;
    filtered.accelY = sum.accelY/4;
    filtered.accelZ = sum.accelZ/4;
    filtered.gyroX = sum.gyroX/4;
    filtered.gyroY = sum.gyroY/4;
    filtered.gyroZ = sum.gyroZ/4;

    return filtered;
}

/* get_twist
/  The primary PID controller. 
/  Receives the calculated orientation and outputs a twist command using the set gains 
/  and assuming a set-point of 0 roll and 0 pitch.
*/
Twist get_twist(Orientation RPY){
    Twist twist_cmd;

    //printf("Inputs -- yaw: %6.2f, Kp: %6.2f",RPY.yaw, Kp);
    float e_vx = 0 - RPY.pitch;
    e_int_vx += e_vx;

    float e_vy = 0 - RPY.roll;
    e_int_vy += e_vy;

    twist_cmd.vx = Kp*e_vx + Kd*(saved_e_vx-e_vx) + Ki*e_int_vx;
    twist_cmd.vy = Kp*e_vy + Kd*(saved_e_vy-e_vy) + Ki*e_int_vy;
    twist_cmd.w = 0;


    //printf("%6.2f, %6.2f",twist_cmd.vx,twist_cmd.vy);

    // printf("Vx: %6.2f Vy: %6.2f \n",twist_cmd.vx,twist_cmd.vy);

    //saved_RPY = RPY;
    saved_e_vx = e_vx;
    saved_e_vy = e_vy;

    return twist_cmd;
}

/* update_wheels
/  Calculates the motor step rate starting with the commanded twist.
/  Wheel speed equation reference: "Modern Robotics: Mechanics, Planning, and Control," Kevin M. Lynch and Frank C. Park, Cambridge University Press, 2017,
*/
void update_wheels(Twist twist){

/*
    u1 = (-d*w + vx)/r
    u2 = (-d*w - vx/2 + vy*-sin(pi/3))/r
    u3 = (-d*w - vx/2 + vy*sin(pi/3))/r
*/

    rps_motor1 = ((-1*d*twist.w + twist.vx)/r)/(2*PI);
    rps_motor2 = ((-1*d*twist.w - twist.vx/2 + twist.vy*-sin(PI/3))/r)/(2*PI);
    rps_motor3 = ((-1*d*twist.w - twist.vx/2 + twist.vy*sin(PI/3))/r)/(2*PI);

    delay1 = (1 / (rps_motor1 * 200)); // x rotations/sec * 200 steps/rotation * 1 sec / 1000 ms
    delay2 = (1 / (rps_motor2 * 200));
    delay3 = (1 / (rps_motor3 * 200));

    if (delay1 < 0){
        delay1 = delay1 * -1.0;
    }
    if (delay2 < 0){
        delay2 = delay2 * -1.0;
    }
    if (delay3 < 0){
        delay3 = delay3 * -1.0;
    }

    delay1 *= Step1_Size;
    delay2 *= Step2_Size;
    delay3 *= Step3_Size;

    if (delay1 < TIMER){
        delay1 = TIMER;
    }
    if (delay2 < TIMER){
        delay2 = TIMER;
    }
    if (delay3 < TIMER){
        delay3 = TIMER;
    }

    // if ((delay1 - saved_delay1)/saved_delay1 > 1.00 || ((delay1 - saved_delay1)/saved_delay1 < -1.00)) {
    //     delay1 = saved_delay1;
    // }

    // if ((delay2 - saved_delay2)/saved_delay2 > 1.00 || ((delay2 - saved_delay2)/saved_delay2 < -1.00)) {
    //     delay2 = saved_delay2;
    // }

    // if ((delay3 - saved_delay3)/saved_delay3 > 1.00 || ((delay3 - saved_delay3)/saved_delay3 < -1.00)) {
    //     delay3 = saved_delay3;
    // }

    // saved_delay1 = delay1;
    // saved_delay2 = delay2;
    // saved_delay3 = delay3;

    // counts1 = floor(delay1 / TIMER);
    // counts2 = floor(delay2 / TIMER);
    // counts3 = floor(delay3 / TIMER);

    // print_count++; //Use a print count to slow down print speed and increase legibility
    // if (print_count == 1000/250){
    printf("%6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f", rps_motor1, rps_motor2, rps_motor3, twist_cmd.vy, twist_cmd.vx, calced_RPY.roll, calced_RPY.pitch, calced_RPY.yaw);
        print_count = 0;
    // }
}

/* motor_callback
/  The function paced by TIMER to step each motor, using a polling algorithm to the number 
/  of timer fires to the calculated step delay. 
*/
bool motor_callback(void){
    
    motor1_count++;
    motor2_count++;
    motor3_count++;

    if((delay1 - motor1_count * TIMER) < .0001){//
        // if (delay1 < 50){
        gpio_put(stepPin1,1);
        // }
        motor1_count = 0;
    }

    if((delay2 - motor2_count * TIMER) < .0001){//
        // if (delay2 < 50){
        gpio_put(stepPin2,1);
        // }
        motor2_count = 0;
    }

    if((delay3 - motor3_count * TIMER) < .0001){
        // if (delay3 < 50){
        gpio_put(stepPin3,1);
        // }
        motor3_count = 0;
    }

    gpio_put(stepPin1,0);
    gpio_put(stepPin2,0);
    gpio_put(stepPin3,0);

    return true;
}

/* update_motors
/  The main control loop organize each step to update motor controlls from the measurement values
*/
void update_motors(){
        
        raw_IMU = read_IMU();
        filtered_IMU = filter_IMU(raw_IMU);
        
        calced_RPY = calc_RPY(filtered_IMU, calibration, .250);

        twist_cmd = get_twist(calced_RPY);
        
        update_wheels(twist_cmd);

        if (rps_motor1 < 0){
            gpio_put(dirPin1,0);
        }
        else{
            gpio_put(dirPin1,1);
        }

        if (rps_motor2 < 0){
            gpio_put(dirPin2,0);
        }
        else{
            gpio_put(dirPin2,1);
        }

        if (rps_motor3 < 0){
            gpio_put(dirPin3,0);
        }   
        else{
            gpio_put(dirPin3,1);
        } 

}


int main()
{
    stdio_init_all();

    gpio_init(stepPin1);
    gpio_init(dirPin1);
    gpio_init(stepPin2);
    gpio_init(dirPin2);
    gpio_init(stepPin3);
    gpio_init(dirPin3);

    gpio_set_dir(stepPin1, GPIO_OUT);
    gpio_set_dir(dirPin1, GPIO_OUT);
    gpio_set_dir(stepPin2, GPIO_OUT);
    gpio_set_dir(dirPin2, GPIO_OUT);
    gpio_set_dir(stepPin3, GPIO_OUT);
    gpio_set_dir(dirPin3, GPIO_OUT);

    IMU_init();

    calibration = calibrate_IMU();
    saved_IMU[0] = read_IMU(calibration);
    saved_IMU[1] = read_IMU(calibration);
    saved_IMU[2] = read_IMU(calibration);
    saved_IMU[3] = read_IMU(calibration);

    delay1 = 1000.0;
    delay2 = 1000.0;
    delay3 = 1000.0;
    
    /* Timer Set-Up */
    struct repeating_timer motor_timer;
    add_repeating_timer_ms(-1*TIMER, motor_callback, NULL, &motor_timer);
    
    /* CONTROL LOOP */
    while(true){
        update_motors();
        sleep_ms(dt); 
    }
}