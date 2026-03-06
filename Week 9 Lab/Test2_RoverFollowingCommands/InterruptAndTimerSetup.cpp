
#include "InterruptAndTimerSetup.h"
#include <Arduino.h>

unsigned char InterruptRunningFlag; // All eight flags are clear, higher bit has higher priority
// When running flag is set, means that interrupt is currently running
int TimerTimeout; // if not zero, means that certain timer is up before being able to finish the code.
unsigned long Interrupt1Period, Interrupt1SpentTime, TimerTemp1A, TimerTemp1B;
unsigned long Interrupt2Period, Interrupt2SpentTime, TimerTemp2A, TimerTemp2B;
unsigned long Interrupt3Period, Interrupt3SpentTime, TimerTemp3A, TimerTemp3B;

static volatile Timer_Callback_t Timer1overflowCallback = 0;
static volatile Timer_Callback_t Timer3overflowCallback = 0;
static volatile Timer_Callback_t Timer4overflowCallback = 0;

static float CountToRadRatio = 2*PI/EncoderCounterPerRotation;

//////////////////////////////////////////////////////////////

void InterruptAndTimerInit()
{
  InterruptRunningFlag = 0b00000000;
  TimerTimeout = 0;
}

void HighPriorityInterruptInit(Timer_Callback_t cb, unsigned long Freq)
{
  Timer1overflowCallback = cb; // Attach the callback function
  //Timer 1 setup    // Control pin 11, 12
  TCCR1A = 0b00000010;
  TCCR1B = 0b00011010; // Set WGM3:0 to be 0b1110: Fast PWM mode; CS2:0 to be 0b010, Prescaler = 8 -> timer ticks at 16Mhz/8 = 2Mhz
  //ICR1 = 10000; // Timer counter overflow at 2Mhz/10000 = 200 Hz
  //ICR1 = 2000; // Timer counter overflow at 2Mhz/2000 = 1000 Hz
  if(Freq > 2000000) Freq = 2000000;
  else if(Freq < 31) Freq = 31;
  ICR1 = uint16_t(2000000/float(Freq));
  TIMSK1 = 0b00000001; // Enable Timer Interrupt
  //End of Timer 1 setup
}

void MediumPriorityInterruptInit(Timer_Callback_t cb, unsigned long Freq)
{
  Timer3overflowCallback = cb; // Attach the callback function
  //Timer 3 setup  //  control pin 2, 3, 5
  TCCR3A = 0b00000010;
  TCCR3B = 0b00011011; // Set WGM3:0 to be 0b1110: Fast PWM mode; CS2:0 to be 0b011, Prescaler = 64 -> timer ticks at 16Mhz/64 = 250kHz
  //ICR3 = 2500;  // 100Hz 
  //ICR3 = 12500;   // 20Hz
  //ICR3 = 5000;   // 50Hz
  if(Freq > 250000) Freq = 250000;
  else if(Freq < 4) Freq = 4;
  ICR3 = uint16_t(250000/float(Freq));
  TIMSK3 = 0b00000001; // Enable Timer Interrupt
  //End of Timer 3 setup
}

void LowerPriorityInterruptInit(Timer_Callback_t cb, float Freq)
{
  Timer4overflowCallback = cb; // Attach the callback function
  //Timer 4 setup  //  control pin 6, 7, 8
  TCCR4A = 0b00000010;
  TCCR4B = 0b00011101; // Set WGM3:0 to be 0b1110: Fast PWM mode; CS2:0 to be 0b101, Prescaler = 1024 -> timer ticks at 16Mhz/1024 = 15625Hz
  //ICR4 = 31250; // Timer counter overflow at 15625/31250 = 0.5Hz
  //ICR4 = 1562; // Timer counter overflow at 15625/1562 ~= 10Hz
  if(Freq > 15625) Freq = 15625;
  else if(Freq < 0.24) Freq = 0.24;
  ICR4 = uint16_t(15625/float(Freq));
  TIMSK4 = 0b00000001; // Enable Timer Interrupt
  //End of Timer 4 setup
}


static volatile unsigned char Encoder1_current_state; 
static int32_t *Encoder1_counter_Ptr = 0;
static int Encoder1_PA_Pin, Encoder1_PB_Pin;

void Encoder1ExtISRSetup(int PA, int PB, int32_t *Counter, int Mode=0)
{
  Encoder1_counter_Ptr = Counter;
  Encoder1_PA_Pin = PA;
  Encoder1_PB_Pin = PB;
  pinMode(Encoder1_PA_Pin, INPUT_PULLUP);
  pinMode(Encoder1_PB_Pin, INPUT_PULLUP);
  bool StateA = digitalRead(Encoder1_PA_Pin);
  bool StateB = digitalRead(Encoder1_PB_Pin);
  Encoder1_current_state = (StateA<<1)|StateB;
  attachInterrupt(digitalPinToInterrupt(Encoder1_PA_Pin), Encoder1_Reading, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder1_PB_Pin), Encoder1_Reading, CHANGE);
}

static volatile unsigned char Encoder2_current_state; 
static int32_t *Encoder2_counter_Ptr = 0;
static int Encoder2_PA_Pin, Encoder2_PB_Pin;

void Encoder2ExtISRSetup(int PA, int PB, int32_t *Counter, int Mode=0)
{
  Encoder2_counter_Ptr = Counter;
  Encoder2_PA_Pin = PA;
  Encoder2_PB_Pin = PB;
  pinMode(Encoder2_PA_Pin, INPUT_PULLUP);
  pinMode(Encoder2_PB_Pin, INPUT_PULLUP);
  bool StateA = digitalRead(Encoder2_PA_Pin);
  bool StateB = digitalRead(Encoder2_PB_Pin);
  Encoder2_current_state = (StateA<<1)|StateB;
  attachInterrupt(digitalPinToInterrupt(Encoder2_PA_Pin), Encoder2_Reading, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Encoder2_PB_Pin), Encoder2_Reading, CHANGE);
}

void Encoder1_Reading(void)
{
  unsigned char StateA = digitalRead(Encoder1_PA_Pin);
  unsigned char StateB = digitalRead(Encoder1_PB_Pin);
  unsigned char Encoder1_state = (StateA<<1)|StateB;
  int Diff = 0;
  switch(Encoder1_current_state)
  {// 013201320132
    case 0:
      if(Encoder1_state==1) Diff = 1;
      else if(Encoder1_state==2) Diff = -1;
      //else Encoder1Error = 1;
      break;
    case 1:   
      if(Encoder1_state==3) Diff = 1;
      else if(Encoder1_state==0) Diff = -1;
      //else Encoder1Error = 1;
      break;
    case 2:
      if(Encoder1_state==0) Diff = 1;
      else if(Encoder1_state==3) Diff = -1;
      //else Encoder1Error = 1;
      break;
    case 3:
      if(Encoder1_state==2) Diff = 1;
      else if(Encoder1_state==1) Diff = -1;
      //else Encoder1Error = 1;
      break;
    default:
      break;
  }
  Encoder1_current_state = Encoder1_state;
  //Encoder1_counter += Diff;
  *Encoder1_counter_Ptr += Diff;
}


void Encoder2_Reading(void)
{
  unsigned char StateA = digitalRead(Encoder2_PA_Pin);
  unsigned char StateB = digitalRead(Encoder2_PB_Pin);
  unsigned char Encoder2_state = (StateA<<1)|StateB;
  int Diff = 0;
  switch(Encoder2_current_state)
  {// 013201320132
    case 0:
      if(Encoder2_state==1) Diff = 1;
      else if(Encoder2_state==2) Diff = -1;
      //else Encoder1Error = 1;
      break;
    case 1:   
      if(Encoder2_state==3) Diff = 1;
      else if(Encoder2_state==0) Diff = -1;
      //else Encoder1Error = 1;
      break;
    case 2:
      if(Encoder2_state==0) Diff = 1;
      else if(Encoder2_state==3) Diff = -1;
      //else Encoder1Error = 1;
      break;
    case 3:
      if(Encoder2_state==2) Diff = 1;
      else if(Encoder2_state==1) Diff = -1;
      //else Encoder1Error = 1;
      break;
    default:
      break;
  }
  Encoder2_current_state = Encoder2_state;
  //Encoder2_counter += Diff;
  *Encoder2_counter_Ptr += Diff;
}


ISR(TIMER1_OVF_vect) // Timer1 overflow will automatically trigger this function
{
  sei(); // Allow this function to be interrupted
  TimerTemp1A = micros();
  if(InterruptRunningFlag>>Priority2) 
  {
    TCNT1 = ICR1 - 1; // Set the timer counter very close to the TOP so it could be triggered as soon as possible once the higher priority function is done
    //Serial.println(TCNT1);
    //CheckBit = 1;
    return 0; //if there are higher or same priority interrupt not done yet
  }
  else // No higher or same prioirity interrupt running, we can proceed.
  {
    InterruptRunningFlag |= (1<<Priority2); // Set the running flag for bit 2
    //InterruptFunction1();
    Timer1overflowCallback();
    InterruptRunningFlag &= ~(1<<Priority2); // clear the running flag for bit 2
  }
  Interrupt1SpentTime = micros() - TimerTemp1A;
  Interrupt1Period = micros() - TimerTemp1B;
  TimerTemp1B = micros();
  return 1;
}
ISR(TIMER3_OVF_vect) // Timer3 overflow will automatically trigger this function
{
  //TCNT3 = 34826; // has to be declared in the code
  sei(); // Allow this function to be interrupted
  TimerTemp2A = micros();
  if(InterruptRunningFlag>>Priority1) 
  {
    TCNT3 = ICR3 - 1; // Set the timer counter very close to the TOP so it could be triggered as soon as possible once the higher priority function is done
    return 0; //if there are higher or same priority interrupt not done yet
  }
  else // No higher or same prioirity interrupt running, we can proceed.
  {
    InterruptRunningFlag |= (1<<Priority1); // Set the running flag for bit 1
    //InterruptFunction2();
    Timer3overflowCallback();
    InterruptRunningFlag &= ~(1<<Priority1); // clear the running flag for bit 1
  }
  Interrupt2SpentTime = micros() - TimerTemp2A;
  Interrupt2Period = micros() - TimerTemp2B;
  TimerTemp2B = micros();
  return 1;
}
ISR(TIMER4_OVF_vect) // Timer4 overflow will automatically trigger this function
{
  sei(); // Allow this function to be interrupted
  TimerTemp3A = micros();
  if(InterruptRunningFlag>>Priority0) 
  {
    //CheckBit = 1;
    TCNT4 = ICR4 - 1; // Set the timer counter very close to the TOP so it could be triggered as soon as possible once the higher priority function is done
    return 0; //if there are higher or same priority interrupt not done yet
  }
  else // No higher or same prioirity interrupt running, we can proceed.
  {
    InterruptRunningFlag |= (1<<Priority0); // Set the running flag for bit 0
    //InterruptFunction3();
    Timer4overflowCallback();
    InterruptRunningFlag &= ~(1<<Priority0); // clear the running flag for bit 0
  }
  Interrupt3SpentTime = micros() - TimerTemp3A;
  Interrupt3Period = micros() - TimerTemp3B;
  TimerTemp3B = micros();
  return 1;
}



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
////////////////////Motor Part///////////////////////////////////

#define MotorDeadZone 0.02 // Below this number, stop motor
#define MotorSaturation 0.9 // Upper Limit for motor

void DC_Motor_Init(DC_Motor *motor, uint8_t Pin_IN1, uint8_t Pin_IN2, uint8_t Pin_ENA)
{
  motor->IN1_Pin = Pin_IN1;
  motor->IN2_Pin = Pin_IN2;
  motor->ENA_Pin = Pin_ENA;
  pinMode(motor->IN1_Pin, OUTPUT);
  pinMode(motor->IN2_Pin, OUTPUT);
  pinMode(motor->ENA_Pin, OUTPUT);
  digitalWrite(motor->IN1_Pin, LOW);
  digitalWrite(motor->IN2_Pin, LOW);
  digitalWrite(motor->ENA_Pin, LOW);
  motor->CurrentEncoderCount = 0;
  motor->LastMotorPos = 0;
  motor->LastMotorPosForRoverCoords = 0;
}


float DC_Motor_GetPos(DC_Motor *motor)
{
  motor->MotorPos = EncoderCountToRad(motor->CurrentEncoderCount);
  return motor->MotorPos;
}

float DC_Motor_GetVel(DC_Motor *motor,float dt, float LOWPASS_Const = 1000)
{
  motor->MotorPos = motor->CurrentEncoderCount*CountToRadRatio;
  float PosDiff = motor->MotorPos - motor->LastMotorPos;
  motor->LastMotorPos = motor->MotorPos;
  float RawSpeed = PosDiff/dt;
  motor->MotorSpeed = (motor->MotorSpeed + RawSpeed* LOWPASS_Const)/(1+ LOWPASS_Const);
  return motor->MotorSpeed;
}

void DC_Motor_Go(DC_Motor *motor,float Input) // -1 to 1
{
  if(Input > 1 || Input > MotorSaturation) Input = min(1,MotorSaturation);
  else if (Input < -1 || Input < -1*MotorSaturation) Input = -1*min(1,MotorSaturation);
  
  if(Input >= MotorDeadZone)
  {
    digitalWrite(motor->IN2_Pin, LOW);
    digitalWrite(motor->IN1_Pin, HIGH);
    analogWrite(motor->ENA_Pin, int(Input*255));
  }
  else if(Input < MotorDeadZone && Input > -1*MotorDeadZone)
  {
    digitalWrite(motor->IN1_Pin, LOW);
    digitalWrite(motor->IN2_Pin, LOW);
    digitalWrite(motor->ENA_Pin, LOW);
  }
  else if(Input <= -1*MotorDeadZone)
  {
    digitalWrite(motor->IN1_Pin, LOW);
    digitalWrite(motor->IN2_Pin, HIGH);
    analogWrite(motor->ENA_Pin, int(Input*-255));
  }
}

void DC_Motor_ResetPos(DC_Motor *motor, float MotorPos = 0)
{
  motor->LastMotorPos = MotorPos;
  motor->MotorPos = MotorPos;
  motor->CurrentEncoderCount = MotorPos/CountToRadRatio;
}




float FF_Controller(float DesiredVel, float Kff)
{
  float output = Kff*DesiredVel;
  return output;
}

float DC_PID_Controller(DC_Motor *motor, float DesiredVel,float CurrentVel, float dt, float Kp, float Ki, float Kd)
{
  float error = DesiredVel - CurrentVel;
  float NewIntegral = motor->PID_integral += error*dt;
  if(motor->PID_integral_Max > NewIntegral && -1*motor->PID_integral_Max  < NewIntegral) motor->PID_integral = NewIntegral;
  float output = Kp*error + Kd*(error - motor->PID_PreviousError)/dt + Ki * motor->PID_integral;
  motor->PID_PreviousError = error;
  return output;
}



float EncoderCountToRad(long count)
{
  return float(count)*CountToRadRatio;// Save some time
}


void MotorSpeedControl(DC_Motor *motor, float DesiredSpeed, float dt, float kp, float ki, float kd, float kff, float SpeedLowPassFactor)
{
  float MotorVel = DC_Motor_GetVel(motor, dt, SpeedLowPassFactor);
  float Motor_Cmd = 0;
  Motor_Cmd += FF_Controller(DesiredSpeed, kff); // Replace this with Motor1_Cmd += PID_Controller(DesiredMotor1Vel, CurrentMotor1Vel);
  Motor_Cmd += DC_PID_Controller(motor, DesiredSpeed, MotorVel, dt, kp, ki, kd);
  DC_Motor_Go(motor, Motor_Cmd);
}

void RoverToMotorCmd(float *ReturnedMotorSpeed, float TranslationalSpeed, float SteeringSpeed, float WheelDistance, float WheelDiameter)
{
  float MotorR_Speed, MotorL_Speed;
  MotorR_Speed = TranslationalSpeed/(WheelDiameter/2);
  MotorL_Speed = TranslationalSpeed/(WheelDiameter/2);
  float RoverToMotorAngleRatio = WheelDistance/(WheelDiameter/2);
  MotorR_Speed += 0.5*SteeringSpeed*RoverToMotorAngleRatio;
  MotorL_Speed -= 0.5*SteeringSpeed*RoverToMotorAngleRatio;
  ReturnedMotorSpeed[0] = MotorR_Speed; // send back calculated right motor speed
  ReturnedMotorSpeed[1] = MotorL_Speed; // send back calculated left motor speed
}


void CalculateRoverStates(float *RoverStates, DC_Motor *motor1, DC_Motor *motor2, float WheelDistance, float WheelDiameter)
{
  float Motor1Pos = motor1->MotorPos;
  float LastMotor1Pos = motor1->LastMotorPosForRoverCoords;
  float Motor2Pos = motor2->MotorPos;
  float LastMotor2Pos = motor2->LastMotorPosForRoverCoords;
  float d_theta1 = Motor1Pos - LastMotor1Pos;
  float d_theta2 = Motor2Pos - LastMotor2Pos;
  float d_l1 = (WheelDiameter/2)*d_theta1;
  float d_l2 = (WheelDiameter/2)*d_theta2;
  float d_l = (d_l1 + d_l2)/2;
  float d_alpha = (d_l1 - d_l2)/WheelDistance;
  //RoverGlobalDirection += d_alpha;
  RoverStates[2] += d_alpha;
  float d_x = d_l*cos(RoverStates[2]);
  float d_y = d_l*sin(RoverStates[2]);
  //RoverGlobalCoordsX += d_x;
  RoverStates[0] += d_x;
  //RoverGlobalCoordsY += d_y;
  RoverStates[1] += d_y;
  motor1->LastMotorPosForRoverCoords = Motor1Pos;
  motor2->LastMotorPosForRoverCoords = Motor2Pos;
  //RoverAccumulativeL += d_l; // Add for Week 9 simple translational and rotation position controller
  RoverStates[3] += d_l;
}
