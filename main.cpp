#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHRPS.h>

int main(void)
{
    float touch_x, touch_y;

    //Call this function to initialize the RPS to a course
    RPS.InitializeTouchMenu();

    //Wait for touchscreen to be pressed
    LCD.WriteLine("Press Screen to Start");
    while(!LCD.Touch(&touch_x, &touch_y));

    LCD.Clear();

    //Write initial screen info
    LCD.WriteRC("RPS Test Program",0,0);
    LCD.WriteRC("X Position:",2,0);
    LCD.WriteRC("Y Position:",3,0);
    LCD.WriteRC("   Heading:",4,0);

    LCD.WriteRC("Satellite:",6,0);

    while( true )
    {
        LCD.WriteRC(RPS.X(),2,12); //update the x coordinate
        LCD.WriteRC(RPS.Y(),3,12); //update the y coordinate
        LCD.WriteRC(RPS.Heading(),4,12); //update the heading
        LCD.WriteRC(RPS.FuelType(),6,12);

        Sleep(10); //wait for a 10ms to avoid updating the screen too quickly
    }

    //we will never get here because of the infinite while loop
    return 0;
}
