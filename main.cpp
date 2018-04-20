#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>

DigitalInputPin frontMicro(FEHIO::P0_0);
DigitalInputPin BackLeftMicro(FEHIO::P0_5);
DigitalInputPin BackRightMicro(FEHIO::P0_2);
int main(void)
{

    float x,y;

    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );
while(true){
  if(!frontMicro.Value()){
    LCD.WriteLine("mid");
  }

  if(!BackLeftMicro.Value()){
    LCD.WriteLine("left");
  }

  if(!BackRightMicro.Value()){
    LCD.WriteLine("right");
  }
}
    return 0;
}
