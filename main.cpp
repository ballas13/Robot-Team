#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>



FEHMotor left_motor(FEHMotor::Motor0,9.0);
FEHMotor right_motor(FEHMotor::Motor1,9.0);
DigitalInputPin leftFrontMicro(FEHIO::P0_0);
DigitalInputPin rightFrontMicro(FEHIO::P0_1);

//Write function prototypes
void performanceTest(void);
void setMotorSpeed(int leftSpeed, int rightSpeed);
void stopBoth(void);
void moveTime(int leftMotorPower, int rightMotorPower,double time);
void moveTillWall(int leftMotorPower, int rightMotorPower);

int main(void)
{


    performanceTest();


    float x,y;

    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    while( true )
    {
        if( LCD.Touch(&x,&y) )
        {
            LCD.WriteLine( "Hello World!" );
            Sleep( 100 );
        }
    }
    return 0;
}

void performanceTest(void){
    //Go forward
    moveTime(1,1,1);

    //Turn Right
    moveTime(34,5,6);

    //Go formward
    moveTime(4,5,5);

    //Turn left
    moveTime(4,4,4);

    //Run into wall
    moveTillWall(4,4);

    //Turn back towards switch
    moveTime(4,4,4);

    //Run into switch


    //Turn back towards top
    moveTime(4,4,4);

    //Go forward and increase power up hill
    moveTime(4,4,4);
    moveTime(4,4,4);

    //Turn right
    moveTime(4,4,4);

    //Go forward
    moveTime(4,4,4);

    //Turn Right
    moveTime(4,4,4);

    //Run into control panel
    moveTillWall(4,4);
}

void setMotorSpeed(int leftSpeed, int rightSpeed){
    right_motor.SetPercent(rightSpeed);
    left_motor.SetPercent(leftSpeed);
}
\
void stopBoth(void){
    right_motor.Stop();
    left_motor.Stop();
}


void moveTime(int leftMotorPower, int rightMotorPower,double time){
    setMotorSpeed(leftMotorPower,rightMotorPower);
    Sleep(time);
    stopBoth();
}

void moveTillWall(int leftMotorPower, int rightMotorPower){
    setMotorSpeed(leftMotorPower,rightMotorPower);
    while(leftFrontMicro.Value()&&rightFrontMicro.Value()){}
    stopBoth();
}
