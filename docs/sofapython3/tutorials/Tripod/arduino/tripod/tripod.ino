#include <Servo.h>

Servo myservo [3]; //instances of the servo object
int ServoPin []={9, 10, 11};
int pos [] = {0, 0, 0};
int header;

void setup()
{
  for (int i = 0; i < 3; i++)
  {
    myservo[i].attach(ServoPin[i]);
  }
  Serial.begin(115200);
}
  
void loop()
{
  if(Serial.available())
  {
   header=Serial.read();
   while (header!=255)
    { 
     while(!Serial.available());
     header=Serial.read();
    }
   for (int i = 0; i < 3; i++)
   {
    while(!Serial.available());
    pos [i] = Serial.read();
   }
   for (int i = 0; i < 3; i++)
   {
    myservo[i].write(pos[i]); 
   }
  }
}
    
  
