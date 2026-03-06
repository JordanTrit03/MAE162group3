#ifndef INTERRUPT_AND_TIMER_SETUP_H
#define INTERRUPT_AND_TIMER_SETUP_H
#define EncoderCounterPerRotation 1440
// This library is wriiten by Shuo-Wu (Will) Shih for the UCLA MAE 162D/E Capstone Class

#include <Arduino.h>

////////////////////////////////////////////////////////////////
#define Priority0 0
#define Priority1 1
#define Priority2 2
#define Priority3 3
#define Priority4 4
#define Priority5 5
#define Priority6 6
#define Priority7 7


//extern unsigned char InterruptRunningFlag; // All eight flags are clear, higher bit has higher priority
// When running flag is set, means that interrupt is currently running
//extern int TimerTimeout; // if not zero, means that certain timer is up before being able to finish the code.

void TimerAndInterruptSetup();
void InterruptFunction1();
void InterruptFunction2();
void InterruptFunction3();
//extern unsigned long Interrupt1Period, Interrupt1SpentTime, TimerTemp1A, TimerTemp1B;
//extern unsigned long Interrupt2Period, Interrupt2SpentTime, TimerTemp2A, TimerTemp2B;
//extern unsigned long Interrupt3Period, Interrupt3SpentTime, TimerTemp3A, TimerTemp3B;


typedef void (*Timer_Callback_t)(void);
void HighPriorityInterruptInit(Timer_Callback_t cb, unsigned long Freq); // 31Hz - 2MHz
void MediumPriorityInterruptInit(Timer_Callback_t cb, unsigned long Freq); // 4Hz - 250kHz
void LowerPriorityInterruptInit(Timer_Callback_t cb, float Freq); // 0.24 Hz - 15625 Hz
///


void Encoder1ExtISRSetup(int PA, int PB, int32_t *Counter, int Mode=0); // PA, PB need to be 2, 3, 18, or 19
void Encoder1_Reading(void);
void Encoder2ExtISRSetup(int PA, int PB, int32_t *Counter, int Mode=0); // PA, PB need to be 2, 3, 18, or 19
void Encoder2_Reading(void);
float EncoderCountToRad(long count);


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
////////////////////Motor Part///////////////////////////////////


typedef struct
{
    uint8_t IN1_Pin;
    uint8_t IN2_Pin;
    uint8_t ENA_Pin;
    float PID_PreviousError;
    float PID_integral;
    float PID_integral_Max = 0.5;
    int32_t CurrentEncoderCount;
    float MotorSpeed;
    float MotorPos;
    float LastMotorPos;
    float LastMotorPosForRoverCoords;
    
}DC_Motor;

void DC_Motor_Init(DC_Motor *motor, uint8_t Pin_IN1, uint8_t Pin_IN2, uint8_t Pin_ENA);
float DC_PID_Controller(DC_Motor *motor, float DesiredVel,float CurrentVel, float dt, float Kp, float Ki, float Kd);
void DC_Motor_Go(DC_Motor *motor,float Input); // -1 to 1
float FF_Controller(float DesiredVel, float Kff);
float DC_Motor_GetPos(DC_Motor *motor);  // Return Motor Position in rad
float DC_Motor_GetVel(DC_Motor *motor,float dt, float LOWPASS_Const = 1000);

void DC_Motor_ResetPos(DC_Motor *motor, float MotorPos = 0);// Reset the motor position to somewhere (rad)
void RoverToMotorCmd(float *ReturnedMotorSpeed, float TranslationalSpeed, float SteeringSpeed, float WheelDistance, float WheelDiameter);
void CalculateRoverStates(float *RoverStates, DC_Motor *motor1, DC_Motor *motor2, float WheelDistance, float WheelDiameter);
void MotorSpeedControl(DC_Motor *motor, float DesiredSpeed, float dt, float kp, float ki, float kd, float kff, float SpeedLowPassFactor);

#endif // INTERRUPT_AND_TIMER_SETUP_H
