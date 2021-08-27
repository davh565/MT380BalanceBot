
#include "balance.h"
#include "io.h"

#define TRUE 1
#define FALSE 0

enum States{STOP, RUN, TURN, PAUSE, MANUAL};
enum States currentState = STOP;
enum States previousState = STOP;
enum Modes{SELECT, AUTO, MAN};
enum Modes Mode = SELECT;
enum Leds{LEFT, RIGHT, UP, DOWN};          //move to io.h

int colour = -1; //dummy variable, delete   move to io.h

//flags
unsigned int flagHasTipped = FALSE;
unsigned int flagObstacleDetected = FALSE;
unsigned int flagHasTravelled5m = FALSE;
unsigned int flagHasTurned18FALSE = FALSE;
unsigned int flagEnterState = FALSE;
unsigned int flagExitState = FALSE;
//functions
void controlBalance(double balanceSP);      //move to balance.h
void detectObstacle();                      //move to io.h
void updateDistance();                      //move to io.h

void turn180(double zHeading);              //move to io.h
void setBalanceSP(double setpoint);         //move to balance.h
void setTurnDirection(double direction);    //move to io.h
void setLED(int LED, int colour);           //move to io.h
void modeSelect();

void resetFlags(){
    flagHasTipped = FALSE;
    flagObstacleDetected = FALSE;
    flagHasTravelled5m = FALSE;
    flagHasTurned18FALSE = FALSE;
    //flagEnterState = FALSE;
    //flagExitState = FALSE;
}

void setup(){}

void loop(){
    switch (currentState)
    {
    case STOP:
        //Transition Conditions:
        if (Mode == AUTO && !flagObstacleDetected){
            previousState = currentState;
            currentState = RUN;
            flagExitState = TRUE;
        }
        else if (Mode == MANUAL){
            previousState = currentState;
            currentState = MANUAL;
            flagExitState = TRUE;
        }

        //Enter Action:
        if (flagEnterState){
            //add actions here
            resetFlags();
            setLED(LEFT,colour);
            flagEnterState = FALSE;
        }
        //During Actions:
            //add actions here
            setBalanceSP(0.0);
            modeSelect();

        //Exit Actions:
        if (flagExitState){
            //add actions here
            flagExitState = FALSE;
            flagEnterState = TRUE;
        }
        break;
    
    case RUN:
        //Transition Conditions:
        if (FALSE){     //add condition
            previousState = currentState;
            currentState = RUN;
            flagExitState = TRUE;
        }
        else if (FALSE){     //add condition
            previousState = currentState;
            currentState = RUN;
            flagExitState = TRUE;
        }

        //Enter Action:
        if (flagEnterState){
            //add actions here
            flagEnterState = FALSE;
        }
        //During Actions:
            //add actions here

        //Exit Actions:
        if (flagExitState){
            //add actions here
            flagExitState = FALSE;
            flagEnterState = TRUE;
        }
        break;
    
    case TURN:
        //Transition Conditions:
        if (FALSE){     //add condition
            previousState = currentState;
            currentState = RUN;
            flagExitState = TRUE;
        }
        else if (FALSE){     //add condition
            previousState = currentState;
            currentState = RUN;
            flagExitState = TRUE;
        }

        //Enter Action:
        if (flagEnterState){
            //add actions here
            flagEnterState = FALSE;
        }
        //During Actions:
            //add actions here

        //Exit Actions:
        if (flagExitState){
            //add actions here
            flagExitState = FALSE;
            flagEnterState = TRUE;
        }
        break;
    
    case PAUSE:
        //Transition Conditions:
        if (FALSE){     //add condition
            previousState = currentState;
            currentState = RUN;
            flagExitState = TRUE;
        }
        else if (FALSE){     //add condition
            previousState = currentState;
            currentState = RUN;
            flagExitState = TRUE;
        }

        //Enter Action:
        if (flagEnterState){
            //add actions here
            flagEnterState = FALSE;
        }
        //During Actions:
            //add actions here

        //Exit Actions:
        if (flagExitState){
            //add actions here
            flagExitState = FALSE;
            flagEnterState = TRUE;
        }
        break;
    
    default:
        previousState = currentState;
        currentState = STOP;
        break;
    }
}