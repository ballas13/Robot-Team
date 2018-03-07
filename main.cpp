#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>

#define FORWARD 1
#define BACKWARD -1
#define MOTORPOW 50
#define TPR 360
#define RADIUS 2.75
#define PI 3.14159265359
#define SLEEPTIME .5
#define DISTANCEBETWEENWHEELS 7.25
#define ZEROPOINT 1
#define REGULARTURN 2
#define CORRECTIONFACTOR .98
#define LEFTMID 12
#define RIGHTMID 23
#define MID 2
#define LEFT 1
#define RIGHT 3
#define OFF 0
#define SLOW 0
#define NORMAL 20
#define FAST 30
#define LEFTVAL 2.5
#define MIDVAL 2.5
#define RIGHTVAL 2.5
#define SCORRECTION 2

FEHMotor right_motor(FEHMotor::Motor0,7.2);
FEHMotor left_motor(FEHMotor::Motor1,7.2);
FEHServo arm_lift(FEHServo::Servo0);
DigitalInputPin BackRightMicro(FEHIO::P3_6);
DigitalInputPin BackLeftMicro(FEHIO::P3_7);
DigitalInputPin frontMicro(FEHIO::P0_0);
AnalogInputPin CdS_cell(FEHIO::P0_7);
DigitalEncoder right_encoder(FEHIO::P3_0);
DigitalEncoder left_encoder(FEHIO::P2_0);
AnalogInputPin rightOpt(FEHIO::P0_6);
AnalogInputPin middleOpt(FEHIO::P0_4);
AnalogInputPin leftOpt(FEHIO::P0_2);
FEHServo servo(FEHServo::Servo0);

//Write function prototypes

void performanceTest1(void);
void performanceTest2(void);
void performanceTest3(void);
void setMotorSpeed(int leftSpeed, int rightSpeed);
void stopBoth(void);
void moveTime(int leftMotorPower, int rightMotorPower,double time);
void moveTillWall(int leftMotorPower, int rightMotorPower);
void moveForward(double inches);
void moveBackward(double inches);
void right(double angle, int direction, int zero);
void left(double angle, int direction, int zero);
void followLine(void);
void setServo(int angle);
void highPower(double inches,int power);

int main(void)
{


    //performanceTest1();




    //performanceTest2();

    performanceTest3();

    float x,y;

    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    while( true )
    {
        if( LCD.Touch(&x,&y) )
        {
            LCD.WriteLine( left_encoder.Counts());
            LCD.WriteLine( right_encoder.Counts());
            Sleep( 100 );
        }
    }
    return 0;
}
/*
void performanceTest1(void){

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
*/
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
    //CHANGE IS HERE
    setMotorSpeed(leftMotorPower+SCORRECTION,rightMotorPower);
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
        Sleep(0.5);
        LCD.WriteLine("RIGHT");
    }
    else if(!BackLeftMicro.Value()){
        stopBoth();
        Sleep(0.5);
        setMotorSpeed(0,rightMotorPower);
        Sleep(0.5);
        LCD.WriteLine("LEFT");
}
    LCD.WriteLine("STOP BOTH");
    stopBoth();
    Sleep(0.5);
}

void performanceTest2(){
    //Run while loop until light turn on.  If light does not turn on in 10 seconds exit while loop.
    LCD.WriteLine("START");
    double start1=TimeNow();
    bool keepWaiting=true;
    while(CdS_cell.Value()>0.5&&keepWaiting){
        if (TimeNow()-start1>5.0){
            keepWaiting=false;
        }
    }

    //Call forward function with parameter of 8 inches.
    moveForward(10.4);

    //Call turn left function with parameters of 90 degrees and forward
    left(90,FORWARD,ZEROPOINT);
    LCD.WriteLine(CdS_cell.Value());
    moveForward(6.33);
    Sleep(.1);


    //Call turn right function with parameters of  90 degrees and forward

    //Call turn right function with parameters of 90 degrees and forward.
    //A. If red, call turn left 45 degrees and then turn right 45 degrees and then call forward function for 3 inches.
    //B. If blue, call turn right 45 degrees and then turn left 45 degrees and then call forward function for 3 inches
    LCD.WriteLine(CdS_cell.Value());
    Sleep(0.5);
    LCD.WriteLine(CdS_cell.Value());
    bool red=false;
    if(CdS_cell.Value()<.24) {
        //red
        LCD.WriteLine("RED");
        right(90,FORWARD,ZEROPOINT);
        moveBackward(4);
        right(45, FORWARD,REGULARTURN);
        left(45,FORWARD,REGULARTURN);
        red=true;

    }
    else{
        //blue
        LCD.WriteLine("BLUE");
        right(90,FORWARD,ZEROPOINT);
        moveBackward(7);
        left(60, FORWARD,REGULARTURN);
        right(70,FORWARD,REGULARTURN);
        moveForward(3);

    }
    LCD.WriteLine(CdS_cell.Value());
    moveBackward(2);
    LCD.WriteLine(CdS_cell.Value());
    left(90,BACKWARD,ZEROPOINT);
    LCD.WriteLine(CdS_cell.Value());
    moveTillWall(-1*MOTORPOW, -1*MOTORPOW);
    moveForward(26);
    moveBackward(12);
    left(90,BACKWARD,ZEROPOINT);
    moveForward(9.5);
}

void moveForward(double inches){
    //Convert inches to counts
    double correctionFactor=1.0;
    int counts=((inches*TPR)/(2.0*PI*RADIUS))*correctionFactor;

    //Reset encodor counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    right_motor.SetPercent(MOTORPOW);
    left_motor.SetPercent(MOTORPOW-SCORRECTION);
    //While the average of the left and right encoder is less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts){}


    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    //Sleep .5 seconds
    Sleep(SLEEPTIME);
    }

void highPower(double inches,int power){
    //Convert inches to counts
    double correctionFactor=1.0;
    int counts=((inches*TPR)/(2.0*PI*RADIUS))*correctionFactor;

    //Reset encodor counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    right_motor.SetPercent(power);
    left_motor.SetPercent(power);
    //While the average of the left and right encoder is less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts){}


    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    //Sleep .5 seconds
    Sleep(SLEEPTIME);
    }

void moveBackward(double inches){
    //Convert inches to counts
    double correctionFactor=.97;
    int counts=((inches*TPR)/(2.0*PI*RADIUS))*correctionFactor;

    //Reset encodor counts
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Set both motors to desired percent
    right_motor.SetPercent(-1*MOTORPOW);
    left_motor.SetPercent((-1*MOTORPOW)+SCORRECTION);
    //While the average of the left and right encoder is less than counts,
    //keep running motors
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts){}

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    //Sleep .5 seconds
    Sleep(SLEEPTIME);
    }

void right(double angle,int direction,int zero) //using encoders
{

    double dist=zero*DISTANCEBETWEENWHEELS*PI*(angle/360.0);
    int counts=((dist*TPR)/(2.0*PI*RADIUS))*CORRECTIONFACTOR;
    //Reset encoder counts
    LCD.WriteLine("RESET COUNTS");
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    if(zero==1){//zero point turn
        //Set both motors to desired percent
        LCD.WriteLine("ZERO POINT TURN");
        right_motor.SetPercent(-1*direction*MOTORPOW);
        left_motor.SetPercent(direction*MOTORPOW);


        //While the left encoder is less than counts,
        //keep running motors
        while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts){};
    }
    else{
    //Set both motors to desired percent
    LCD.WriteLine("ELSE IN RIGHT METHOD");
    right_motor.SetPercent(0);
    left_motor.SetPercent(direction*MOTORPOW);


    //While the left encoder is less than counts,
    //keep running motors
    while(left_encoder.Counts() < counts){};
}

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    //Sleep for .5 seconds
    Sleep(SLEEPTIME);
}

void left(double angle,int direction,int zero) //using encoders
    {

        double dist=zero*DISTANCEBETWEENWHEELS*PI*(angle/360.0);
        int counts=((dist*TPR)/(2.0*PI*RADIUS))*CORRECTIONFACTOR;
        //Reset encoder counts
        right_encoder.ResetCounts();
        left_encoder.ResetCounts();

        if(zero==1){//zero point turn
            //Set both motors to desired percent
            right_motor.SetPercent(direction*MOTORPOW);
            left_motor.SetPercent(-1*direction*MOTORPOW);


            //While the left encoder is less than counts,
            //keep running motors
            while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts){};
        }
        else{
        //Set both motors to desired percent
        right_motor.SetPercent(direction*MOTORPOW);
        left_motor.SetPercent(0);


        //While the left encoder is less than counts,
        //keep running motors
        while(right_encoder.Counts() < counts){};
    }
        //Turn off motors
        right_motor.Stop();
        left_motor.Stop();

        //Sleep for .5 seconds
        Sleep(SLEEPTIME);
    }

void followLine(void)
{
    float Leftvalue = leftOpt.Value();
    float Rightvalue = rightOpt.Value();
    float Middlevalue = middleOpt.Value();



    int position=MID;
    double start=TimeNow();
     while((frontMicro.Value()&&TimeNow()-start<3.0)){
         if (Leftvalue>LEFTVAL){
             if((Middlevalue>MIDVAL)) {
                         position=LEFTMID;


                     }
                     else{
                         position=LEFT;


                     }
                 }
             else if (Rightvalue>RIGHTVAL) {
                     if((Middlevalue>MIDVAL)){
                         position=RIGHTMID;


                     }
                     else{
                         position=RIGHT;


                     }
            }

            else if (Middlevalue>MIDVAL) {
                     position=MID; }
            else{
                     right_motor.Stop();
                     left_motor.Stop();
                 }



                 switch(position) {
                     case LEFT:
                         right_motor.SetPercent(FAST);
                         left_motor.SetPercent(SLOW);
                         LCD.WriteLine("LEFT");
                         break;

                     case LEFTMID:
                         right_motor.SetPercent(FAST);
                         left_motor.SetPercent(SLOW);
                         LCD.WriteLine("LEFTMID");
                         break;
                     case RIGHT:
                         right_motor.SetPercent(SLOW);
                         left_motor.SetPercent(FAST);
                         LCD.WriteLine("RIGHT");
                         break;
                     case RIGHTMID:
                         right_motor.SetPercent(SLOW);
                         left_motor.SetPercent(FAST);
                         LCD.WriteLine("RIGHTMID");
                         break;
                     case MID:
                         right_motor.SetPercent(NORMAL);
                         left_motor.SetPercent(NORMAL);
                         LCD.WriteLine("MID");
                         break;

                 }

     }

}

void setServo(int angle){
    servo.SetMin(500);
    servo.SetMax(2500);
    //Set arm servo to 0 degrees
    servo.SetDegree(angle);
    Sleep(1.0);
}

void performanceTest3(void){
    LCD.WriteLine("START");
    double start1=TimeNow();
    bool keepWaiting=true;
    while(CdS_cell.Value()>0.5&&keepWaiting){
        if (TimeNow()-start1>5.0){
            keepWaiting=false;
        }
    }
    setServo(180);
    moveForward(8.5);
    right(45,FORWARD,ZEROPOINT);
    moveForward(6);
    right(45,FORWARD,REGULARTURN);
    setMotorSpeed(48,50);
    while(frontMicro.Value()){
    }
    moveForward(.2);
    stopBoth();
    Sleep(.1);
    moveBackward(.92);
    setServo(80);
    Sleep(2.0);
    setServo(180);
    moveBackward(5);
    right(45,BACKWARD,ZEROPOINT);
    moveBackward(6);
    left(45,BACKWARD,ZEROPOINT);
    moveTillWall(-50,-50);
    moveForward(3.5);
    right(85,FORWARD,ZEROPOINT);
    highPower(20.5,60);
    left(45-,FORWARD,REGULARTURN);
    moveForward(28);
    setServo(67);

    setServo(180);
    moveBackward(12);

    right(90,FORWARD,ZEROPOINT);
    moveForward(12);



}



