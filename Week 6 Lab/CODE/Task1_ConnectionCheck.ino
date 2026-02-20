#include "InterruptAndTimerSetup.h"
float MotorSpeedCalculationLowPassFactor = 0.5;
//////////////////////////////////////Parameters for DC Motor 1/////////////////////////////////////
#define Motor1_ENA_PIN 45
#define Motor1_IN1_PIN 30
#define Motor1_IN2_PIN 31
DC_Motor Motor1;
float M1_Kp = 0.2; float M1_Ki = 0.1; float M1_Kd = 0.00; float M1_Kff = 0.04;
// P gain, I gain, D gain, Feedfoward controller (p) gain
#define Encoder1_PhaseA_Pin 2 // must be 2, 3, 18, or 19
#define Encoder1_PhaseB_Pin 3 // must be 2, 3, 18, or 19
float Motor1_Cmd, Motor1Pos, LastMotor1Pos, Motor1Vel, DesiredMotor1Vel;        /////////////////////
//////////////////////////////////////Parameters for DC Motor 2 /////////////////////////////////////
#define Motor2_ENA_PIN 44
#define Motor2_IN1_PIN 32
#define Motor2_IN2_PIN 33
DC_Motor Motor2;
float M2_Kp = 0.2; float M2_Ki = 0.1; float M2_Kd = 0.00; float M2_Kff = 0.04;
// P gain, I gain, D gain, Feedfoward controller (p) gain
#define Encoder2_PhaseA_Pin 18 // must be 2, 3, 18, or 19
#define Encoder2_PhaseB_Pin 19 // must be 2, 3, 18, or 19
float Motor2_Cmd, Motor2Pos, LastMotor2Pos, Motor2Vel, DesiredMotor2Vel;/////////////////////////////
//////
float ISR1_dt = 0.01;  float ISR2_dt = 0.05; float ISR3_dt = 0.1;// sec
float RoverGlobalDirection = 0;
float RoverGlobalCoordsX = 0; float RoverGlobalCoordsY = 0;
float DesiredForwardSpeed = 0; // inch/sec
float DesiredRotationSpeed = 0; // rad/sec
float CircularMotionTimeToFinish, CalculatedCircularOmega;
float WheelDiameter = 3; // inch
float WheelDistance = 13; // inch
float DrawingCircleRadius = 15; // inch
float DrawingCircleSpeed = 4; // inch/sec
float InitialWait = 3;// sec  : How long to wait before the rover starts moving


void setup() {
  Serial.begin(115200);
  DC_Motor_Init(&Motor1, Motor1_IN1_PIN, Motor1_IN2_PIN, Motor1_ENA_PIN);
  Motor1_Cmd = 0;
  Encoder1ExtISRSetup(Encoder1_PhaseA_Pin, Encoder1_PhaseB_Pin, &Motor1.CurrentEncoderCount);// PA, PB, couter to be updated
  DC_Motor_Init(&Motor2, Motor2_IN1_PIN, Motor2_IN2_PIN, Motor2_ENA_PIN);
  Motor2_Cmd = 0;
  Encoder2ExtISRSetup(Encoder2_PhaseA_Pin, Encoder2_PhaseB_Pin, &Motor2.CurrentEncoderCount);// PA, PB, couter to be updated
  delay(100);
  HighPriorityInterruptInit(ISR1, 1/ISR1_dt); // Callback function, Triggering frequency between 31Hz - 2MHz
  MediumPriorityInterruptInit(ISR2, 1/ISR2_dt); // Callback function, Triggering frequency between 4Hz - 250kHz
  LowerPriorityInterruptInit(ISR3, 1/ISR3_dt); // Callback function, Triggering frequency between 0.24 Hz - 15625 Hz
  CalculatedCircularOmega = CircularMotionSteeringSpeed(DrawingCircleRadius, DrawingCircleSpeed);
  Motor1Pos = (Motor1.CurrentEncoderCount / 1440.0) * 2.0 * PI;
  Motor2Pos = (Motor2.CurrentEncoderCount / 1440.0) * 2.0 * PI;
  LastMotor1Pos = Motor1Pos;
  LastMotor2Pos = Motor2Pos;
}

void loop() { 
  Serial.print(Motor1Vel);
  Serial.print("\t");
  Serial.print(Motor2Vel);
  Serial.print("\t");
  Serial.print(RoverGlobalDirection);
  Serial.print("\t");
  Serial.print(RoverGlobalCoordsX);
  Serial.print("\t");
  Serial.print(RoverGlobalCoordsY);
  Serial.print("\n");
  delay(10);
  
  }
float timeCounter = 0;
void ISR1(void){ // Higher Priority ISR
    
    //DesiredMotor1Vel = 5;
    //DesiredMotor2Vel = 3;
    //Motor1SpeedControl(DesiredMotor1Vel); // Send speed to PID controller
    //Motor2SpeedControl(DesiredMotor2Vel); // Send speed to PID controller
    const float COUNTS_PER_REV = 1440.0;
    Motor1Pos = (Motor1.CurrentEncoderCount / COUNTS_PER_REV) * 2.0 * PI;
    Motor2Pos = (Motor2.CurrentEncoderCount / COUNTS_PER_REV) * 2.0 * PI;
    
    // 1. Calculate how much time has passed
    timeCounter += ISR1_dt;
  
    // 2. Wait for InitialWait period before moving
    if (timeCounter < InitialWait) {
      Motor1SpeedControl(0);
      Motor2SpeedControl(0);
      return; 
    }
  
    // 3. Check if we finished the circle
    if (timeCounter > (InitialWait + CircularMotionTimeToFinish)) {
      Motor1SpeedControl(0);
      Motor2SpeedControl(0);
      return;
    }

    
    CalculateRoverDynamics(Motor1Pos, Motor2Pos, WheelDistance, WheelDiameter);
    float TargetSpeeds[2];
    RoverToMotorCmd(TargetSpeeds, DrawingCircleSpeed, CalculatedCircularOmega, WheelDistance, WheelDiameter);
    Motor1SpeedControl(TargetSpeeds[0]); // Right Motor
    Motor2SpeedControl(TargetSpeeds[1]); // Left Motor
  }

void ISR2(void){  // Medium Priority ISR

  }
void ISR3(void){  // Lower Priority ISR

  }

void RoverToMotorCmd(float *ReturnedMotorSpeed, float TranslationalSpeed, float SteeringSpeed, float WheelDistance, float WheelDiameter)
{
  float MotorR_Speed, MotorL_Speed;
  // Use TranslationalSpeed, SteeringSpeed, WheelDistance, WheelDiameter to calculate each motor's required speed
  //================= Start your code here============================//
  // Hints:
  // Step 1: TranslationalSpeed = average speed of right wheel and left wheel tangent line speed
  float vR_trans = TranslationalSpeed;
  float vL_trans = TranslationalSpeed;
  
  // Step 2: Transform right wheel and left wheel tangent line speed to angular speed using WheelDiameter
  float wR_trans = vR_trans / (WheelDiameter/2);
  float wL_trans = vL_trans / (WheelDiameter/2);
  
  // Step 3: SteeringSpeed = difference of right wheel and left wheel tangent line speed divided by WheelDistance
  float vR_steer = (SteeringSpeed * WheelDistance) / 2.0;
  float vL_steer = -(SteeringSpeed * WheelDistance) / 2.0;
  
  // Step 4: Transform right wheel and left wheel tangent line speed to angular speed using WheelDiameter
  float wR_steer = vR_steer / (WheelDiameter/2);
  float wL_steer = vL_steer / (WheelDiameter/2);
  
  // Step 5: Add the right wheel and left wheel abgular speed respectively from Step 2 and 4
  MotorR_Speed = wR_trans + wR_steer;
  MotorL_Speed = wL_trans + wL_steer;
  
  //================= End your code here============================//
   ReturnedMotorSpeed[0] = MotorR_Speed; // send back calculated right motor speed
   ReturnedMotorSpeed[1] = MotorL_Speed; // send back calculated left motor speed
}

void CalculateRoverDynamics(float Motor1Pos, float Motor2Pos, float WheelDistance, float WheelDiameter)
{
  // Note that global variables are used here only for demostration
  // Later this will be changed to use local variable and pointer for better structure
  //Use these variables: 
  Motor1Pos;LastMotor1Pos;Motor2Pos;LastMotor2Pos;WheelDistance;WheelDiameter;
  RoverGlobalCoordsX;RoverGlobalCoordsY;RoverGlobalDirection;
  // Hints:
  // Step 1: Use Motor1Pos and LastMotor1Pos, Motor2Pos and LastMotor2Pos to find d_theta1, d_theta2
  float d_theta1 = Motor1Pos - LastMotor1Pos; // Change in Right wheel angle
  float d_theta2 = Motor2Pos - LastMotor2Pos; // Change in Left wheel angle
  
  // Step 2: Find d_l1 and d_l2 from d_theta1, d_theta2 with WheelDiameter
  float d_l1 = (WheelDiameter/2) * d_theta1;
  float d_l2 = (WheelDiameter/2) * d_theta2;
  
  // Step 3: Find d_l from the average of d_l1 and d_l2
  float d_l = (d_l1 + d_l2) / 2.0;
  
  // Step 4: Find d_alpha from d_l1 and d_l2 and WheelDistance
  float d_alpha = (d_l1 - d_l2) / WheelDistance;
  
  // Step 5: Add d_alpha to the current RoverGlobalDirection to update the new direction
  RoverGlobalDirection += d_alpha;
  
  // Step 6: Calculate d_x, d_y from d_l, cos(RoverGlobalDirection), sin(RoverGlobalDirection) 
  float d_x = d_l * cos(RoverGlobalDirection);
  float d_y = d_l * sin(RoverGlobalDirection);
  
  // Step 7: update coords x and y by adding d_x, d_y to the current RoverGlobalCoordsX and Y
  RoverGlobalCoordsX += d_x;
  RoverGlobalCoordsY += d_y;
  
  // Step 8: Remember to update LastMotor1Pos and LastMotor2Pos with Motor1Pos and Motor2Pos
  LastMotor1Pos = Motor1Pos;
  LastMotor2Pos = Motor2Pos;
  
}

void Motor1SpeedControl(float DesiredSpeed)
{
  Motor1Vel = DC_Motor_GetVel(&Motor1, ISR1_dt, MotorSpeedCalculationLowPassFactor);
  Motor1_Cmd = 0;
  Motor1_Cmd += FF_Controller(DesiredSpeed, M1_Kff); // Replace this with Motor1_Cmd += PID_Controller(DesiredMotor1Vel, CurrentMotor1Vel);
  Motor1_Cmd += DC_PID_Controller(&Motor1, DesiredSpeed, Motor1Vel, ISR1_dt, M1_Kp, M1_Ki, M1_Kd);
  DC_Motor_Go(&Motor1, Motor1_Cmd);
}
void Motor2SpeedControl(float DesiredSpeed)
{
  Motor2Vel = DC_Motor_GetVel(&Motor2, ISR1_dt, MotorSpeedCalculationLowPassFactor);
  Motor2_Cmd = 0;
  Motor2_Cmd += FF_Controller(DesiredSpeed, M2_Kff); // Replace this with Motor1_Cmd += PID_Controller(DesiredMotor1Vel, CurrentMotor1Vel);
  Motor2_Cmd += DC_PID_Controller(&Motor2, DesiredSpeed, Motor2Vel, ISR1_dt, M2_Kp, M2_Ki, M2_Kd);
  DC_Motor_Go(&Motor2, Motor2_Cmd);
}


float CircularMotionSteeringSpeed(float CircleRadius, float DrawingSpeed)
{
  float PathLength = 2*CircleRadius*PI;
  CircularMotionTimeToFinish = PathLength/DrawingSpeed; // sec
  float SteeringSpeed = 2*PI/CircularMotionTimeToFinish;
  return SteeringSpeed;
}
