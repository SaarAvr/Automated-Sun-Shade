/*
 * motorFuncs.cpp
 *
 *  Created on: Apr 14, 2025
 *      Author: saar
 */

#include "motorFuncs.hpp"
#include "main.h"
#include <stdbool.h>
#include <iostream>

#define IN1_PIN GPIO_PIN_8
#define IN1_PORT GPIOA
#define IN2_PIN GPIO_PIN_10
#define IN2_PORT GPIOB
#define IN3_PIN GPIO_PIN_4
#define IN3_PORT GPIOB
#define IN4_PIN GPIO_PIN_5
#define IN4_PORT GPIOB

uint16_t maxAngleMap = 950;
uint16_t minAngleMap = 250;

const int16_t servo_bias1 = 105;
const int16_t servo_bias2 = 95;
const int16_t servo_bias3 = 60;
//const int16_t servo_bias1 = 0;
//const int16_t servo_bias2 = 0;
//const int16_t servo_bias3 = 0;
int16_t servo_ang1 = servo_bias1;
int16_t servo_ang2 = servo_bias2;
int16_t servo_ang3 = servo_bias3;
//int16_t servo3_limit = 35;
int16_t stepper_ang = 0;
float stepper_ang_float = 0;
float stepperPhi = 0.0878906;
bool servoMoved = false;

// Step sequence for motor control (half-drive)
const uint8_t step_sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

// Function for a microsecond delay using Timer 1
//With the microcontroller running at 80Mhz, the TIM1 needs to be initialized..
//..with a prescaler of 79 (80-1) and the default maximum counter period of 65535..
//.. so that it can run at 1MHz for us to get the 1 microsecond delay
void microDelay(uint16_t delay) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

// Function to set GPIO pins according to step sequence
void setPins(uint8_t step[4]) {
    HAL_GPIO_WritePin(IN1_PORT, IN1_PIN, step[0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_PORT, IN2_PIN, step[1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_PORT, IN3_PIN, step[2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_PORT, IN4_PIN, step[3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// Function to move the motor in either clockwise or counterclockwise direction
//void stepMotor(int steps, uint16_t delay, int direction) {
//    int seq_len = sizeof(step_sequence) / sizeof(step_sequence[0]);
//    for (int x = 0; x < steps; x++) {
//        int step_index = (direction == 1) ? x % seq_len : (seq_len - (x % seq_len)) % seq_len;
//        setPins((uint8_t*)step_sequence[step_index]);
//        microDelay(delay);
//    }
//}

void stepMotor(int steps, uint16_t delay, int direction) {
    static int current_step_index = 0;  // store where we left off
    int seq_len = sizeof(step_sequence) / sizeof(step_sequence[0]);

    for(int i = 0; i < steps; i++) {
        if (direction == 1) {
            // Move forward by 1 in the sequence
            current_step_index = (current_step_index + 1) % seq_len;
        } else {
            // Move backward by 1 in the sequence
            current_step_index = (current_step_index + seq_len - 1) % seq_len;
        }
        setPins((uint8_t*)step_sequence[current_step_index]);
        microDelay(delay);
    }
}

void setServo (TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle){
	uint32_t pulseWidth = minAngleMap + (angle * (maxAngleMap - minAngleMap) / 180);
	__HAL_TIM_SET_COMPARE(htim, channel, pulseWidth);
}

void setServoToInit(){
	setServo (&htim3, TIM_CHANNEL_1, servo_bias1);
	setServo (&htim3, TIM_CHANNEL_2, servo_bias2);
	setServo (&htim3, TIM_CHANNEL_3, servo_bias3);
}

void moveMotors(int anglesRec[4]){
	int angles[4] = {anglesRec[0],anglesRec[1],anglesRec[2],anglesRec[3]};
	bool anglesSet[4] = {false, false, false, false};
	printf("Received angles: %d, %d, %d, %d\r\n", angles[0], angles[1], angles[2], angles[3]);
	while (!(anglesSet[0] && anglesSet[1] && anglesSet[2] && anglesSet[3])){
		printf(">>>>WHILE LOOP<<<<\r\n\n");
		printf("Current Angles: %d, servo_ang1: %d, servo_ang2: %d,servo_ang3: %d\r\n",stepper_ang,servo_ang1,servo_ang2,servo_ang3);
		printf("Received angles: %d, %d, %d, %d\r\n", angles[0], angles[1], angles[2], angles[3]);
		printf("States:__%d__%d__%d__%d__\r\n", anglesSet[0],anglesSet[1],anglesSet[2],anglesSet[3]);
		if (servo_ang1 - servo_bias1 > -angles[1]){ // FIX BACK TO '-'
		  servo_ang1 -= 1;
		  servoMoved = true;
		} else if (servo_ang1 - servo_bias1 < -angles[1]){
		  servo_ang1 += 1;
		  servoMoved = true;
		}
		setServo (&htim3, TIM_CHANNEL_1, servo_ang1);
		if (servo_ang1 == -(angles[1] - servo_bias1)) anglesSet[1] = true;

		if (servo_ang2 - servo_bias2 > angles[2]){
		  servo_ang2 -= 1;
		  servoMoved = true;
		} else if(servo_ang2 - servo_bias2 < angles[2]){
		  servo_ang2 += 1;
		  servoMoved = true;
		}
		setServo (&htim3, TIM_CHANNEL_2, servo_ang2);
		if (servo_ang2 == angles[2] + servo_bias2) anglesSet[2] = true;

		if (servo_ang3 - servo_bias3 > angles[3]){
		  servo_ang3 -= 1;
		  servoMoved = true;
		} else if(servo_ang3 - servo_bias3 < angles[3]){
		  servo_ang3 += 1;
		  servoMoved = true;
		}
		setServo (&htim3, TIM_CHANNEL_3, servo_ang3);
		if (servo_ang3 == angles[3] + servo_bias3) anglesSet[3] = true;

		if (stepper_ang_float < angles[0]){
		  stepper_ang_float += stepperPhi*12;
		  stepMotor(12, 1000, 1);
		} else if (stepper_ang > angles[0]){
		  stepper_ang_float -= stepperPhi*12;
		  stepMotor(12, 1000, -1);
		}
		stepper_ang = (int)stepper_ang_float;
		if (stepper_ang == angles[0]) anglesSet[0] = true;


		if (servoMoved == true){
		  HAL_Delay(30);
		  servoMoved = false;
		}
	}
}

void testFunc(){
	printf("TEST\r\n");
//	int counterA = 0;
//	while (counterA < 180){
//		printf("counterA = %d\r\n", counterA);
//		stepMotor(11, 1000, -1);
//		counterA++;
//		HAL_Delay(10);
//	}
//	stepMotor(2048, 1000, -1);

}



