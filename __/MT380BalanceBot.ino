
#include "MeMegaPi.h"
#include <Wire.h>


#define UPRIGHT -1.52

#define MOTORTYPE 1  //set 1 for DC motor, 0 for EncoderMotor.
//#define TILTFWD -1.83

enum States{STOP, RUN, TURN, PAUSE};
enum States currentState = STOP;
enum States prevState = STOP;

MeGyro gyro;
MeUltrasonicSensor ultraSensor(PORT_7);
#if MOTORTYPE
MeMegaPiDCMotor motor1(PORT1B);
MeMegaPiDCMotor motor2(PORT2B);
#else
MeEncoderOnBoard motor1(SLOT1);
MeEncoderOnBoard motor2(SLOT2);
#endif
//PID params
double pidBias = 0.0; //0: angle only, 1: speed only, 0.5: equal weight
    //Angle
double setPoint1 = UPRIGHT; //deg
#if MOTORTYPE
double kP1 = 5;
double kI1 = 0.0;
double kD1 = 0.0;
double pidMax1 = 50; //Max PID value before saturation
#else
double kP1 = 10;
double kI1 = 0.02;
double kD1 = 0.0;
double pidMax1 = 50; //Max PID value before saturation
#endif
int maxAngle = 15; //+- angle range that will attempt to balance
    //Speed
double setPoint2 = 0;  //rpm
double kP2 = 1.0;
double kI2 = 0.0;
double kD2 = 0.0;
double pidMax2 = 1000; //Max PID value before saturation
//Motor params
#if MOTORTYPE
int motorMax = 255; // motorMin-255
int motorMin = 20; // 0 - motorMax
int motorOffset = 0;
#else
double motorMaxRPM = 255;
#endif

//flags
unsigned int flagEnterState = FALSE;
unsigned int flagExitState = FALSE;
unsigned int flagHasTravelled5m = FALSE;
unsigned int flagHasTipped = FALSE;
unsigned int flagHasTurned180 = FALSE;
unsigned int flagObstacleDetected = FALSE;
unsigned int flagRunEnded = FALSE;
//PID vars
unsigned long prevTime = millis();
unsigned long currentTime = 0;
double pidOutFinal = 0;
    //Angle
double P1 = 0;
double I1 = 0;
double D1 = 0;
double currentError1 = 0;
double prevError1 = 0;
double pidOut1 = 0;
    //Speed
double P2 = 0;
double I2 = 0;
double D2 = 0;
double currentError2 = 0;
double prevError2 = 0;
double pidOut2 = 0;
//Gyro vars
double angX = 0;
double angY = 0;
double angZ = 0;
double accY = 0;
//Motor vars
int motorSpeed = 0;
double motorAvSpeed = 0;
//functions
void resetFlags(){
    //flagEnterState = FALSE;
    //flagExitState = FALSE;
    flagHasTravelled5m = FALSE;
    flagHasTurned180 = FALSE;
    flagObstacleDetected = FALSE;
}
//int checkDistance();
//int checkTurnAngle();
//int checkObstacle();
void turn180();
#if !MOTORTYPE
void isr_process_encoder1(void)
{
  if(digitalRead(motor1.getPortB()) == 0)
  {
    motor1.pulsePosMinus();
  }
  else
  {
    motor1.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(motor2.getPortB()) == 0)
  {
    motor2.pulsePosMinus();
  }
  else
  {
    motor2.pulsePosPlus();
  }
}
#endif
void setup(){
    #if !MOTORTYPE
    attachInterrupt(motor1.getIntNum(), isr_process_encoder1, RISING);
    attachInterrupt(motor2.getIntNum(), isr_process_encoder2, RISING);

    //Set PWM 8KHz
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(CS11) | _BV(WGM12);

    TCCR2A = _BV(WGM21) | _BV(WGM20);
    TCCR2B = _BV(CS21);
    #endif
    Serial.begin(115200);
    gyro.begin();
}

void loop(){
    //Loop Functions:
    gyro.update();
    #if !MOTORTYPE
    motor1.updateSpeed();
    motor2.updateSpeed();
    #endif
    angX = gyro.getAngleX();
    angY = gyro.getAngleY();
    angZ = gyro.getAngleZ();
    accY = gyro.getAccY();

    //update Flags
    flagHasTipped = (angY< -maxAngle || angY > maxAngle)? TRUE : FALSE;
    flagHasTravelled5m = 0 ? TRUE : FALSE;
    flagHasTurned180 = 0 ? TRUE : FALSE;
    // flagHasTravelled5m = checkDistance() ? TRUE : FALSE;
    // flagHasTurned180 = checkTurnAngle() ? TRUE : FALSE;
    flagObstacleDetected = (ultraSensor.distanceCm() < 10.0) ? TRUE : FALSE;


    currentTime = millis();
    // PID CONTROL BALANCE
    currentError1 = setPoint1 - angY;
    P1 = kP1 * currentError1;
    I1 = I1 + kI1 * currentError1 * (currentTime - prevTime);
    D1 = kD1 * (currentError1 - prevError1)/(currentTime - prevTime);
    pidOut1 = P1 + I1 + D1;
    // Angle I Saturation
    if(I1 > pidMax1) I1 = pidMax1; 
    else if(I1 < -pidMax1) I1 = -pidMax1;

    // PID CONTROL SPEED
    #if !MOTORTYPE
    motorAvSpeed = (motor1.getCurrentSpeed()+motor2.getCurrentSpeed())/2;
    currentError2 = setPoint2 - motorAvSpeed;
    P2 = kP2 * currentError2;
    I2 = I2 + kI2 * currentError2 * (currentTime - prevTime);
    D2 = kD2 * (currentError2 - prevError2)/(currentTime - prevTime);
    pidOut2 = P2 + I2 + D2;
    //Speed I Saturation
    if(I2 > pidMax2) I2 = pidMax2; 
    else if(I2 < -pidMax2) I2 = -pidMax2;
    //Mix PID Outputs
    pidOutFinal = (1-pidBias)*pidOut1/pidMax1 + pidBias*pidOut2/pidMax2;
    #endif

    #if 1
    Serial.print("Angle:");
    Serial.print(angY);
    Serial.print(" PIDang:");
    Serial.print(pidOut1);
    Serial.print(" PIDspd:");
    Serial.print(pidOut2);
    Serial.print(" PIDfnl:");
    Serial.print(pidOutFinal);
    Serial.print(" MotorSpd: ");
    Serial.println(motorAvSpeed);
    #endif

    // Give up if over max angle
    // Else adjust speed according to PID
    if(flagHasTipped) motorSpeed = 0;
    #if MOTORTYPE
    else motorSpeed = pidOut1 * (motorMax - motorMin)/pidMax1 + motorMin+motorOffset;
    motor1.run(-motorSpeed); /* value: between -255 and 255. */
    motor2.run(motorSpeed); /* value: between -255 and 255. */
    #else
    else motorSpeed = pidOutFinal*motorMaxRPM;
    motor1.setMotorPwm(motorSpeed);
    motor2.setMotorPwm(-motorSpeed);
    #endif
    // prevOffset = currentOffset;
    prevTime = currentTime;
    prevError1 = currentError1;
    ///////////////////////////////////////////////////////////////////////////

    
    //Finite State Machine:
    switch (currentState)
    {
    case STOP:
        //Transition Conditions:
        if (!flagRunEnded){
            prevState = currentState;
            currentState = RUN;
            flagExitState = TRUE;
        }
        //Enter Action:
        if (flagEnterState){
            //add actions below
            resetFlags();
//            Serial.println("entering STOP State");
            if (prevState == RUN) flagRunEnded = TRUE;
            // setLED(LEFT,colour);
            //add actions above
            flagEnterState = FALSE;
        }
        //During Actions:
            //add actions below
            if (flagHasTipped && flagRunEnded) flagRunEnded = FALSE; //Tip to reset after run
            //add actions above

        //Exit Actions:
        if (flagExitState){
            //add actions below

            //add actions above
            flagExitState = FALSE;
            flagEnterState = TRUE;
        }
        break;
    
    case RUN:
        //Transition Conditions:
        if (flagHasTravelled5m && !flagHasTurned180){
            prevState = currentState;
            currentState = TURN;
            flagExitState = TRUE;
        }
        else if (flagObstacleDetected){
            prevState = currentState;
            currentState = PAUSE;
            flagExitState = TRUE;
        }

        //Enter Action:
        if (flagEnterState){
            //add actions below
//            Serial.println("entering RUN State");
            // setLED(LEFT,colour);
            //add actions above
            flagEnterState = FALSE;
        }
        //During Actions:
            //add actions below
            //add actions above

        //Exit Actions:
        if (flagExitState){
            //add actions below

            //add actions above
            flagExitState = FALSE;
            flagEnterState = TRUE;
        }
        break;
    
    case TURN:
        //Transition Conditions:
        if (flagHasTurned180){
            prevState = currentState;
            currentState = RUN;
            flagExitState = TRUE;
        }
        else if (flagObstacleDetected){
            prevState = currentState;
            currentState = PAUSE;
            flagExitState = TRUE;
        }

        //Enter Action:
        if (flagEnterState){
//            Serial.println("entering TURN State");
            //add actions below
            // setLED(LEFT,colour);
//            turn180();
            //add actions above
            flagEnterState = FALSE;
        }
        //During Actions:
            //add actions below
            setPoint1 = UPRIGHT;
            //add actions above

        //Exit Actions:
        if (flagExitState){
            //add actions below

            //add actions above
            flagExitState = FALSE;
            flagEnterState = TRUE;
        }
        break;
    
    case PAUSE:
        //Transition Conditions:
        if (!flagObstacleDetected && prevState == RUN){
            prevState = currentState;
            currentState = RUN;
            flagExitState = TRUE;
        }
        else if (!flagObstacleDetected && prevState == TURN){
            prevState = currentState;
            currentState = TURN;
            flagExitState = TRUE;
        }

        //Enter Action:
        if (flagEnterState){
            //add actions below
//            Serial.println("entering PAUSE State");
            // setLED(LEFT,colour);
            //add actions above
            flagEnterState = FALSE;
        }
        //During Actions:
            //add actions below
            setPoint1 = UPRIGHT;
            //add actions above

        //Exit Actions:
        if (flagExitState){
            //add actions below

            //add actions above
            flagExitState = FALSE;
            flagEnterState = TRUE;
        }
        break;
    
    default:
        prevState = currentState;
        currentState = STOP;
        break;
    }
}
