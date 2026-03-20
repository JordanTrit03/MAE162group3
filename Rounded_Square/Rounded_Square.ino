#include "InterruptAndTimerSetup.h"
float MotorSpeedCalculationLowPassFactor = 0.5;
//////////////////////////////////////Parameters for DC Motor 1/////////////////////////////////////
#define Motor1_ENA_PIN 6
#define Motor1_IN1_PIN 8
#define Motor1_IN2_PIN 43
DC_Motor Motor1;
float M1_Kp = 0.2; float M1_Ki = 0.3; float M1_Kd = 0.00; float M1_Kff = 0.00;
// P gain, I gain, D gain, Feedfoward controller (p) gain
#define Encoder1_PhaseA_Pin 3 // must be 2, 3, 18, or 19
#define Encoder1_PhaseB_Pin 2 // must be 2, 3, 18, or 19
float Motor1_Cmd, Motor1Pos, LastMotor1Pos, Motor1Vel, DesiredMotor1Vel;        /////////////////////
//////////////////////////////////////Parameters for DC Motor 2 /////////////////////////////////////
#define Motor2_ENA_PIN 7
#define Motor2_IN1_PIN 30
#define Motor2_IN2_PIN 4
DC_Motor Motor2;
float M2_Kp = 0.2; float M2_Ki = 0.3; float M2_Kd = 0.00; float M2_Kff = 0.00;
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
float WheelDiameter = 3; // inch
float WheelDistance = 13.2; // inch
float RoverAccumulativeL = 0;
float RoverStates[4] = {0}; // RoverGlobalCoordsX, RoverGlobalCoordsY, RoverGlobalDirection, RoverAccumulativeL (For Week 9 lab)
#define pi PI

// 75" rounded-corner square with corner radius r = 10"
// Straight segments: 75 - 2*10 = 55" each
// Corner arcs: quarter circle = (pi/2)*10 = 15.708" each, turning 90 deg (pi/2 rad)
// Cumulative odometer (L) and heading angle at the START of each phase:
//   Phase 0: L=0,       angle=0        (start)
//   Phase 1: L=55,      angle=0        (end of straight 1 / start of arc 1)
//   Phase 2: L=70.708,  angle=pi/2     (end of arc 1    / start of straight 2)
//   Phase 3: L=125.708, angle=pi/2     (end of straight 2 / start of arc 2)
//   Phase 4: L=141.416, angle=pi       (end of arc 2    / start of straight 3)
//   Phase 5: L=196.416, angle=pi       (end of straight 3 / start of arc 3)
//   Phase 6: L=212.124, angle=1.5*pi   (end of arc 3    / start of straight 4)
//   Phase 7: L=267.124, angle=1.5*pi   (end of straight 4 / start of arc 4)
//   Phase 8: L=282.832, angle=2*pi     (end of arc 4 / back to start heading)
float TranslationalPosArray[] = { 0,  55,      70.708,  125.708, 141.416,  196.416, 212.124, 267.124, 282.832 };
float RotationalAngleArray[]  = { 0,   0,  pi/2,     pi/2,      pi,       pi, 1.5*pi,  1.5*pi,  2*pi    };
float CommandTimeInteval = 10; 
float L_kp = 1; float theta_kp = 1;
float TranslationalVelCmdSaturation = 8; // inch/sec
float RotationalVelCmdSaturation = 2; // rad/sec

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
}

void loop() { 
  Serial.print(RoverGlobalCoordsX);
  Serial.print("\t");
  Serial.print(RoverGlobalCoordsY);
  Serial.print("\t");
  Serial.print(RoverGlobalDirection);
  Serial.print("\n");
  delay(10);
  }

size_t CommandArrayLength1 = sizeof(TranslationalPosArray) / sizeof(TranslationalPosArray[0]);
size_t CommandArrayLength2 = sizeof(RotationalAngleArray) / sizeof(RotationalAngleArray[0]);
int CommandArrayLength = min(CommandArrayLength1, CommandArrayLength2);

void ISR1(void){ // Higher Priority ISR
    //////////////////////////////// Update rover states/////////////////////////////////////////////////
    Motor1Pos = DC_Motor_GetPos(&Motor1); // Get motor 1 angle in rad
    Motor2Pos = DC_Motor_GetPos(&Motor2); // Get motor 1 angle in rad
    CalculateRoverStates(RoverStates, &Motor1, &Motor2, WheelDistance, WheelDiameter); // Get rover states from encoders
    RoverGlobalCoordsX = RoverStates[0];  // Rover X position in global coords
    RoverGlobalCoordsY = RoverStates[1];  // Rover Y position in global coords
    RoverGlobalDirection = RoverStates[2]; // Rover heading direction in global coords
    RoverAccumulativeL = RoverStates[3]; // Rover accumulative moved distance (odometer) for Week 9 lab
    /////////////////////////////////////////////////////////////////////////////// END
    
    //////////////////////////////// Get rover speed command/////////////////////////////////////////////////
    float DesiredRovTransVel = 0; // Desired Rover Translational Velocity
    float DesiredRovAngVel = 0; // Desired Rover angular (steering) Velocity (CCW as positive)
    int CommandIndex = int(millis()/1000/CommandTimeInteval);  // Get the index for which the commands should be taken out
    if(CommandIndex > CommandArrayLength - 1) CommandIndex = CommandArrayLength - 1; // Make sure index doesn't exceed the command array length
    float DesiredTranslationalPos = TranslationalPosArray[CommandIndex]; // Get the translational pos
    float DesiredRotationalAngle = RotationalAngleArray[CommandIndex]; // Get the angle
    DesiredRovTransVel = TranslationalSpeedGenerator(RoverAccumulativeL, DesiredTranslationalPos); // Get speed command from pos error
    DesiredRovAngVel = RotationalSpeedGenerator(RoverGlobalDirection, DesiredRotationalAngle); // Get rotational speed command from angle error
    /////////////////////////////////////////////////////////////////////////////// END
    
    //////////////////////////////// Send Speed Commands to run the motors////////////////////////////////////////////
    float WheelSpeed[2] = {0};
    RoverToMotorCmd(WheelSpeed, DesiredRovTransVel, DesiredRovAngVel, WheelDistance, WheelDiameter);
    DesiredMotor1Vel = WheelSpeed[0]; // Get the right wheel speed (motor 1)
    DesiredMotor2Vel = WheelSpeed[1]; // Get the left wheel speed (motor 2)
    //DesiredMotor1Vel = 0;
    //DesiredMotor2Vel = 0;
    MotorSpeedControl(&Motor1, DesiredMotor1Vel, ISR1_dt, M1_Kp, M1_Ki, M1_Kd, M1_Kff, MotorSpeedCalculationLowPassFactor); // Send speed to PID controller
    MotorSpeedControl(&Motor2, DesiredMotor2Vel, ISR1_dt, M2_Kp, M2_Ki, M2_Kd, M2_Kff, MotorSpeedCalculationLowPassFactor); // Send speed to PID controller
    /////////////////////////////////////////////////////////////////////////////// END
  }

void ISR2(void){  // Medium Priority ISR

  }
void ISR3(void){  // Lower Priority ISR

  }


float TranslationalSpeedGenerator(float CurrentPos, float DesiredPos)
{
  float kp = L_kp;
  // Hint: Implement the p controller from position error to generator speed cmd
  // Step 1: caculate the position error from DesiredPos and CurrentPos
  float PosError = DesiredPos - CurrentPos;
  float Cmd;
  // Step 2: Calculate the command from PosError and kp
  Cmd = kp * PosError;
  // Step 3: If Cmd is grater than saturation, set it equal to saturation
  if (Cmd > TranslationalVelCmdSaturation) Cmd = TranslationalVelCmdSaturation;
  if (Cmd < -TranslationalVelCmdSaturation) Cmd = -TranslationalVelCmdSaturation;
  return Cmd;
}
float RotationalSpeedGenerator(float CurrentAngle, float DesiredAngle)
{
  float kp = theta_kp;
  // Hint: Implement the p controller from angle error to generator angular speed cmd
  // Step 1: caculate the angle error from DesiredAngle and CurrentAngle
  float AngleError = DesiredAngle - CurrentAngle;
  float Cmd;
  // Step 2: Calculate the command from AngleError and kp
  Cmd = kp * AngleError;
  // Step 3: If Cmd is grater than saturation, set it equal to saturation
  if (Cmd > RotationalVelCmdSaturation) Cmd = RotationalVelCmdSaturation;
  if (Cmd < -RotationalVelCmdSaturation) Cmd = -RotationalVelCmdSaturation;
  return Cmd;
}
