#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>


FEHMotor right_motor(FEHMotor::Motor0,7.2);
FEHMotor left_motor(FEHMotor::Motor1,7.2);
FEHServo arm_lift(FEHServo::Servo0);
DigitalInputPin BackRightMicro(FEHIO::P0_0);
DigitalInputPin BackLeftMicro(FEHIO::P1_0);
AnalogInputPin CdS_cell(FEHIO::P0_7);
/*int main(void)
{
left_motor.SetPercent(50);
right_motor.SetPercent(50);
Sleep(5.0);
left_motor.Stop();
right_motor.Stop();
return 0;
}
*/

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

    int MainMotorPower=50;
    double nintyDegreeTurn=1.4;
    int Zero=0;

    LCD.Write(CdS_cell.Value());

    while(CdS_cell.Value()>0.5){}
    //Go forward
    moveTime(MainMotorPower,MainMotorPower,1.0);

    //Turn Left
    moveTime(Zero,MainMotorPower,nintyDegreeTurn);

    //Go forward
    moveTime(MainMotorPower,MainMotorPower,.7);

    //Turn Left
    moveTime(Zero,MainMotorPower,nintyDegreeTurn);

    //Move forward
    moveTime(MainMotorPower,MainMotorPower,3.0);

    //Move backwards
    moveTime(-1*MainMotorPower, -1*MainMotorPower, 2.5);

    
    //Turn Left
    moveTime(Zero,-1*MainMotorPower,0.6);

    moveTime(-1*MainMotorPower, -1*MainMotorPower, 1.0);

    //Turn Right
    moveTime(-1*MainMotorPower, Zero, 0.4);

    //Move backward into button
    moveTillWall(-1*MainMotorPower,-1*MainMotorPower);

    //Move forward
    moveTime(MainMotorPower,MainMotorPower,1.0);

    //orient towards wall
    moveTime(-1*MainMotorPower,Zero,nintyDegreeTurn);

    //Move backward into wall
    moveTillWall(-1*MainMotorPower,-1*MainMotorPower);



    //Move forward across course
    moveTime(MainMotorPower,MainMotorPower,3.25);

    //turn forward right, robot facing grassy incline
    moveTime(MainMotorPower, Zero, nintyDegreeTurn);

    //move backwards a little past striped line
    moveTime(-1*MainMotorPower, -1*MainMotorPower, 1.7);

    //turn backwards left
    moveTime(Zero, -1*MainMotorPower, nintyDegreeTurn+.3);

    //align with clear wall
    moveTillWall(-1*MainMotorPower, -1*MainMotorPower);

    //move forward to lift car jack
   moveTime(MainMotorPower, MainMotorPower, 1.4);

    //end run
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
    Sleep(0.5);
}

void moveTillWall(int leftMotorPower, int rightMotorPower){
    setMotorSpeed(leftMotorPower,rightMotorPower);
    double start = TimeNow();
    bool end=false;
    while(BackRightMicro.Value()&&BackLeftMicro.Value()&&!end){
        if(TimeNow()-start>3.0){
           end=true;
        }
    }
    if(!BackRightMicro.Value()&&!BackLeftMicro.Value()){
        stopBoth();
        Sleep(0.5);
        LCD.WriteLine("BOTH");
    }

    else if (!BackRightMicro.Value()){
        stopBoth();
        Sleep(0.5);
        setMotorSpeed(leftMotorPower,0);
        LCD.WriteLine("RIGHT");
    }
    else if(!BackLeftMicro.Value()){
        stopBoth();
        Sleep(0.5);
        setMotorSpeed(0,rightMotorPower);
        LCD.WriteLine("LEFT");
}
    stopBoth();
    Sleep(0.5);
}
