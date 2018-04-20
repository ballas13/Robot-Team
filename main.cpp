#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHRPS.h>
#include <math.h>

#define FORWARD 1
#define BACKWARD -1
#define MOTORPOW 80
#define TPR 360
#define RADIUS 2.75
#define PI 3.14159265359
#define SLEEPTIME .1
#define DISTANCEBETWEENWHEELS 7.25
#define ZEROPOINT 1
#define REGULARTURN 2
#define CORRECTIONFACTOR .94
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
#define SCORRECTION 1

FEHMotor right_motor(FEHMotor::Motor0,7.2);
FEHMotor left_motor(FEHMotor::Motor1,7.2);
DigitalInputPin frontMicro(FEHIO::P0_0);
DigitalInputPin BackLeftMicro(FEHIO::P0_5);
DigitalInputPin BackRightMicro(FEHIO::P0_2);
AnalogInputPin CdS_cell(FEHIO::P0_7);
AnalogInputPin CdS_cell2(FEHIO::P1_1);
AnalogInputPin CdS_cell3(FEHIO::P1_2);
AnalogInputPin CdS_cell4(FEHIO::P1_3);
AnalogInputPin CdS_cell5(FEHIO::P1_4);
AnalogInputPin CdS_cell6(FEHIO::P1_6);
AnalogInputPin CdS_cell7(FEHIO::P1_7);
DigitalEncoder left_encoder(FEHIO::P2_0);
DigitalEncoder right_encoder(FEHIO::P3_0);

FEHServo servo(FEHServo::Servo0);
FEHServo microServo(FEHServo::Servo7);

//Write function prototypes

void final(void);
void final2(void);
void test(void);
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
void setMicroServo(int angle);
void highPower(double inches,int power);
void check_heading(float heading);
void forwardTillY(float yPosition);
bool checkCds(void);
int main(void)
{


    final2();
    //final();

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

void setMotorSpeed(int leftSpeed, int rightSpeed){
    right_motor.SetPercent(rightSpeed);
    left_motor.SetPercent(leftSpeed);
}

void stopBoth(void){
    right_motor.Stop();
    left_motor.Stop();
}

void moveTime(int leftMotorPower, int rightMotorPower,double time){
    setMotorSpeed(leftMotorPower,rightMotorPower);
    Sleep(time);
    stopBoth();
    Sleep(0.1);
}

void moveTillWall(int leftMotorPower, int rightMotorPower){
    //CHANGE IS HERE
    setMotorSpeed(leftMotorPower-SCORRECTION,rightMotorPower);
    double start = TimeNow();
    while(BackRightMicro.Value()&&BackLeftMicro.Value()&&TimeNow()-start<3.0){

    }
    Sleep(.2);

    if(!BackRightMicro.Value()&&!BackLeftMicro.Value()){
        stopBoth();
        Sleep(SLEEPTIME);
        setMotorSpeed(leftMotorPower,rightMotorPower);
        Sleep(SLEEPTIME);
        LCD.WriteLine("BOTH");
    }

    else if (!BackRightMicro.Value()){
        stopBoth();
        Sleep(SLEEPTIME);
        setMotorSpeed(leftMotorPower,0);
        Sleep(SLEEPTIME);
        LCD.WriteLine("RIGHT");
    }
    else if(!BackLeftMicro.Value()){
        stopBoth();
        Sleep(SLEEPTIME);
        setMotorSpeed(0,rightMotorPower);
        Sleep(SLEEPTIME);
        LCD.WriteLine("LEFT");
}
    LCD.WriteLine("STOP BOTH");

    stopBoth();
    Sleep(SLEEPTIME);
}

bool checkCds(void){
    bool red=false;
    int i=0;
    while(i<100){
        red=red||CdS_cell2.Value()<.5||CdS_cell3.Value()<.5||CdS_cell4.Value()<.5||CdS_cell5.Value()<.5||CdS_cell6.Value()<.5||CdS_cell7.Value()<.5;
        i++;
        Sleep(.01);
    }
    return red;
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
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts*.8){}

    //Set both motors to desired percent
    right_motor.SetPercent(MOTORPOW-30);
    left_motor.SetPercent(MOTORPOW-30-SCORRECTION);

    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts){}

    //Turn off motors
    right_motor.Stop();
    left_motor.Stop();

    //Sleep .1 seconds
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
    right_motor.SetPercent(power+3);
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
    while((left_encoder.Counts() + right_encoder.Counts()) / 2. < counts*.8){}

    //Set both motors to desired percent
    right_motor.SetPercent((-1*MOTORPOW)+30);
    left_motor.SetPercent((-1*MOTORPOW)+30+SCORRECTION);

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

void setServo(int angle){
    servo.SetMin(500);
    servo.SetMax(2500);
    //Set arm servo to 0 degrees
    servo.SetDegree(angle);
    Sleep(SLEEPTIME);
}

void setMicroServo(int angle){
    microServo.SetMin(500);
    microServo.SetMax(2500);
    //Set arm servo to 0 degrees
    microServo.SetDegree(angle);
    Sleep(SLEEPTIME);
}

void check_heading(float heading){ //using RPS

    //you will need to fill out this one yourself and take into account
    //the edge conditions (when you want the robot to go to 0 degrees
    //or close to 0 degrees)
    //check whether the robot is within an acceptable range
    while(RPS.Heading() < heading - 1 || RPS.Heading() > heading + 1)
    {
        LCD.WriteLine(RPS.Heading());
        if(heading!=0){
            if(RPS.Heading() > heading)
            {
            //pulse the motors for a short duration in the correct direction

            setMotorSpeed(20,-20);
            Sleep(.014);
            stopBoth();
            Sleep(.01);
            }
            else if(RPS.Heading() < heading)
            {
            //pulse the motors for a short duration in the correct direction

            setMotorSpeed(-20,20);
            Sleep(.014);
            stopBoth();
            Sleep(.01);
            }
        }
        else if(heading==0&&RPS.Heading()<=180){
            setMotorSpeed(20,-20);
            Sleep(.01);
            stopBoth();
            Sleep(.01);
        }
        else if(heading==0&&RPS.Heading()>180){
            setMotorSpeed(-20,20);
            Sleep(.01);
            stopBoth();
            Sleep(.01);
        }

    }
}

void test(void){
    moveForward(10);
    right(90,FORWARD,ZEROPOINT);
    moveForward(5);
    left(90,FORWARD,ZEROPOINT);
    moveBackward(10);
}

void final(void){
    //Run while loop until light turn on.  If light does not turn on in 10 seconds exit while loop.
    RPS.InitializeTouchMenu();
    LCD.WriteLine("START");

    double start1=TimeNow();
    bool keepWaiting=true;
    while(CdS_cell.Value()>1.0&&keepWaiting){
        if (TimeNow()-start1>2.0){
            keepWaiting=false;
        }
    }

    //Rotate arm up
    setServo(178);
    setMicroServo(170);

    //First FORWARD
    moveForward(8.4);

    //Turn left and go forward towards light
    left(90,FORWARD,ZEROPOINT);
    LCD.WriteLine(CdS_cell.Value());
    moveForward(8);
    Sleep(SLEEPTIME);


    //Call turn right function with parameters of  90 degrees and forward

    //Call turn right function with parameters of 90 degrees and forward.
    //A. If red, call turn left 45 degrees and then turn right 45 degrees and then call forward function for 3 inches.
    //B. If blue, call turn right 45 degrees and then turn left 45 degrees and then call forward function for 3 inches
    Sleep(SLEEPTIME);
    LCD.WriteLine(CdS_cell.Value());
    if(CdS_cell.Value()<.5) {
        //red
        LCD.WriteLine("RED");
        left(90,BACKWARD,REGULARTURN);
        //ADJUST SERVO TO AVOID WALL
        setMicroServo(90);
        //HIT RED BUTTON
        moveForward(6);
        //NAVIGATE TOWARD SWITCH
        moveBackward(2);
        right(90,FORWARD, ZEROPOINT);
        moveForward(12);

    }
    else{
        //blue
        LCD.WriteLine("BLUE");
        left(3,FORWARD,ZEROPOINT);
        moveTillWall(50,50);
        //MAYBE AT A SMALL FORWARD HERE??????????????????
        moveBackward(.5);
        right(90,FORWARD,ZEROPOINT);
        //ADJUST SERVO TO AVOID WALL
        setMicroServo(90);
        //HIT BLUE BUTTON
        moveForward(6);
        //NAVIGATE TOWARD SWITCH
        moveBackward(2);
        right(80,FORWARD, ZEROPOINT);
        moveForward(20.5);
    }
    //RESET SERVO POSITION
    setMicroServo(170);
    left(75,FORWARD,ZEROPOINT);
    moveForward(14);
    left(105,FORWARD,ZEROPOINT);
    //RUN INTO SWITCH
    moveForward(4);
    //NAVIGATE TO WRENCH
    moveBackward(1);
    right(90,FORWARD,ZEROPOINT);
    moveTillWall(50,50);
    moveBackward(2);
    right(20,FORWARD,ZEROPOINT);
    moveBackward(9);
    right(58,FORWARD,ZEROPOINT);
    //Go forward until wrench is hit and square up
    setMotorSpeed(50-SCORRECTION,50);
    double start2=TimeNow();
    while(frontMicro.Value()&&TimeNow()-start2<2.0){}
    stopBoth();
    //moveForward(.2);
    Sleep(SLEEPTIME);
    //Move backward correct distance to pick up wrench
    moveBackward(.6);
    //Pick up wrench
    setServo(80);
    Sleep(.5);
    setServo(178);
    //Navigate to other wall
    moveBackward(5);
    right(45,BACKWARD,ZEROPOINT);
    moveBackward(6);
    left(45,BACKWARD,ZEROPOINT);
    //THIS TURN MAY NEED TO BE INCREASED!?!?!?!?!?
    right(180,FORWARD,ZEROPOINT);
    moveForward(12);
    //COULD BE CHANGED BACK TO 50
    moveTillWall(30,30);
    //GO UP RAMP
    moveBackward(.5);
    //CHANGE WAS 95
    left(93,FORWARD,ZEROPOINT);
    moveForward(22);
    //Run into wall
    right(90,FORWARD,ZEROPOINT);
    moveTillWall(50,50);
    moveBackward(2);
    left(138,FORWARD,ZEROPOINT);
    //Navigate to garage
    moveForward(25);
    //GO FORWARD UNTIL FRONT MIDDLE BUMP SWITCH GETS HIT
    setMotorSpeed(50-SCORRECTION,50);
    double start3=TimeNow();
    while(frontMicro.Value()&&TimeNow()-start3<2.0){}
    stopBoth();
    Sleep(SLEEPTIME);
    //MOVE BACK AND DROP WRENCH
    //CHANGE HERE WAS .3
    moveBackward(.15);
    //Dispose of wrench
    //CHANGE WAS 55
    setServo(50);
    Sleep(.5);
    moveBackward(2);
    setServo(178);
    //NAVIGATE TO WHEEL
    moveBackward(9);
    left(90,FORWARD,ZEROPOINT);
    //clockwise 180-0
    if(RPS.FuelType()==1){
        setMicroServo(10);
        moveBackward(15.4);
        Sleep(.2);
        setMicroServo(178);
        Sleep(.5);
        LCD.WriteLine("1");
    }
    else if(RPS.FuelType()==2){
        setMicroServo(170);
        moveBackward(15.4);
        Sleep(.2);
        setMicroServo(0);
        Sleep(.5);
        LCD.WriteLine("2");
    }
    else{
        LCD.WriteLine("error");
    }
    //GO HIT END BUTTON
    moveForward(14.1);
    left(90,FORWARD,ZEROPOINT);
    moveForward(17);
    right(45,FORWARD,ZEROPOINT);
    highPower(17,60);
    //moveForward(18);
    right(90,FORWARD,REGULARTURN);
    moveForward(3);
    right(90,FORWARD,REGULARTURN);
    moveForward(12);

}

void final2(void){

    RPS.InitializeTouchMenu();

    //Run while loop until light turn on.  If light does not turn on in 10 seconds exit while loop.


    LCD.WriteLine("START");

    double start1=TimeNow();
    bool keepWaiting=true;
    while(CdS_cell.Value()>1.0&&keepWaiting){
        if (TimeNow()-start1>25.0){
            keepWaiting=false;
        }
    }
    setServo(178);
    setMicroServo(170);
    //was 12
    right(13,FORWARD,ZEROPOINT);
    moveForward(12);
    setServo(10);
    left(13,FORWARD,ZEROPOINT);
    //LOOK AT THIS
    moveForward(3);
    setServo(90);
    Sleep(.2);
    setServo(10);
    //WAS 2.8
    moveBackward(2.6);
    right(83,FORWARD,ZEROPOINT);
    setServo(178);
    //Go forward until wrench is hit and square up
    setMotorSpeed(50-SCORRECTION,50);
    double start2=TimeNow();
    while(frontMicro.Value()&&TimeNow()-start2<2.0){}
    stopBoth();
    Sleep(SLEEPTIME);
    //Move backward correct distance to pick up wrench
    moveBackward(.6);
    //Pick up wrench
    setServo(80);
    Sleep(.5);
    setServo(178);
    moveBackward(1);
    right(40,FORWARD,ZEROPOINT);
    //was 6.5
    moveForward(6.0);
    right(30,FORWARD,ZEROPOINT);
    moveForward(31);
    left(90,FORWARD,ZEROPOINT);
    moveForward(6);
    //moveTillWall(50,50);
    moveBackward(6);
    right(60,BACKWARD,REGULARTURN);
    //was 8
    moveBackward(8.7);
    //was 92
    right(92,FORWARD,ZEROPOINT);
    //setServo(110);
    //GO FORWARD UNTIL FRONT MIDDLE BUMP SWITCH GETS HIT
    setMotorSpeed(50-SCORRECTION,50);
    double start3=TimeNow();
    while(frontMicro.Value()&&TimeNow()-start3<2.0){}
    stopBoth();
    Sleep(SLEEPTIME);
    //MOVE BACK AND DROP WRENCH
    //CHANGE HERE WAS .3
    moveBackward(.15);
    //Dispose of wrench
    //CHANGE WAS 55
    setServo(45);
    Sleep(.5);
    setServo(178);
    moveBackward(11);

    left(90,FORWARD,ZEROPOINT);
    if(RPS.FuelType()==1){
        setMicroServo(10);
        moveBackward(15.4);
        Sleep(.2);
        //WAS 178
        setMicroServo(177);
        Sleep(.5);
        LCD.WriteLine("1");
    }
    else if(RPS.FuelType()==2){
        setMicroServo(170);
        moveBackward(15.4);
        Sleep(.2);
        //WAS 0
        setMicroServo(5);
        Sleep(.5);
        LCD.WriteLine("2");
    }
    else{
        LCD.WriteLine("error");
    }

    moveForward(13.9);
    left(90,FORWARD,ZEROPOINT);
    moveForward(15.5);
    right(45,FORWARD,REGULARTURN);
    highPower(16.5,60);

    setMotorSpeed(60,0);
    bool red=checkCds();
    stopBoth();

    LCD.WriteLine(CdS_cell2.Value());
    if(red) {
        //red
        LCD.WriteLine("RED");
        setMotorSpeed(-10,80);
        Sleep(1.0);
        stopBoth();
        //left(120,FORWARD,REGULARTURN);
        //ADJUST SERVO TO AVOID WALL
        setMicroServo(90);
        //HIT RED BUTTON
        LCD.WriteLine(RPS.X());
        moveForward(6);
        //NAVIGATE TOWARD SWITCH
        setMotorSpeed(-95,-5);
        Sleep(.52);
        setMotorSpeed(-80,-80);
        Sleep(.4);
        setMotorSpeed(-10,-80);
        Sleep(.6);
        setMotorSpeed(-80,-80);
        Sleep(1.0);
        stopBoth();



    }
    else{
        //blue
        LCD.WriteLine("BLUE");
        setMotorSpeed(-60,0);
        Sleep(1.0);
        stopBoth();
        LCD.WriteLine(RPS.X());
        moveForward(8);
        moveBackward(.25);
        setMotorSpeed(-95,-5);
        Sleep(1.0);
        setMotorSpeed(-80,-80);
        Sleep(.2);
        setMotorSpeed(-10,-80);

        Sleep(1.2);
        setMotorSpeed(-80,-80);
        Sleep(2.0);
        stopBoth();

    }
    int i=0;
    while(i<3){
    float x = RPS.X();
    Sleep(1.0);
    if(x>20){
       moveBackward(5);
       left(30,FORWARD,ZEROPOINT);
       moveForward(8);
    }else{
        moveBackward(5);
        right(30,FORWARD,ZEROPOINT);
        moveForward(8);
    }
    i++;
    }

}

void forwardTillY(float yPosition){
    setMotorSpeed(49,50);
    double start5=TimeNow();
    while(RPS.Y()>yPosition+2&&RPS.Y()<yPosition-2&&TimeNow()-start5<2.0){}
    stopBoth();



}

