#include<Servo.h>
#include <Pixy2.h>
/*#include <PIDLoop.h>
#include <Pixy2CCC.h>
#include <Pixy2I2C.h>
#include <Pixy2Line.h>
#include <Pixy2SPI_SS.h>
#include <Pixy2UART.h>
#include <Pixy2Video.h>
#include <TPixy2.h>
#include <ZumoBuzzer.h>
#include <ZumoMotors.h>*/

Servo A;
int fr=13;
int bk=12;

Pixy2 p;

void setup() {
  Serial.begin(9600);
 /* pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(13,OUTPUT);*/
  p.init();

 /* digitalWrite(10,HIGH);
  digitalWrite(11,HIGH);

   analogWrite(fr,0);
   analogWrite(bk,0);
   A.attach(9);
   A.write(87);
   delay(500);*/
    
}


void loop() {/*
  //analogWrite(fr,50);
  //analogWrite(bk,0);
  int numg = 0;
  int numr = 0;
  int i;
  p.ccc.getBlocks();
  int a=(p.ccc.blocks[0].m_height)*(p.ccc.blocks[0].m_width);
  //Serial.println(p.ccc.numBlocks);
  //delay(350);
  for (i = 0; i < p.ccc.numBlocks; i++)
  {
    if (p.ccc.blocks[i].m_signature == 1)
    {
      numg++;
    }
    if (p.ccc.blocks[i].m_signature == 2 || p.ccc.blocks[i].m_signature == 3 || p.ccc.blocks[i].m_signature == 4)
    {
      numr++;
    }
  }
  if (numg >= 1 && numr >= 1)
  {
    Serial.println("Green & Red found");

  }
  else if (numg >= 1 && a>=250) 
  {
    Serial.println("Green found");
    A.write(60);
    delay(1200);
    A.write(103);
    delay(800);
    A.write(87);

  }
  else if (numr >= 1 && a>=250)
  {
    Serial.println("Red found");
    A.write(114);
    delay(1200);
    A.write(73);
    delay(800);
    A.write(87);

  }
  else
  {
    Serial.println("Nothing");
  }
  A.write(87);*/
  if(read_cam()=='g') //pixy cam read green pillar
    startup('g');  //avoid pillar from left side
  else if(read_cam()=='r') //pixy cam read red pillar
    startup('r');  //avoid pillar from right side
  else
  {while(1)
  {Motor_forward(40);
  Front_MPU_pid();
  if(sl>100) //when the servo left ultrasonic read more than 100cm ,the vehicle turns left
    {Turn_left();
    break;}
  if(sr>100) //when the servo right ultrasonic read more than 100cm ,the vehicle turns right
    {Turn_right();
    break;}
  }
}
rest_means();
update_sensors();
if(read_cam()=='g') //pixy cam read green pillar
    startup('g');  //avoid pillar from left side
  else if(read_cam()=='r') //pixy cam read red pillar
    startup('r');  //avoid pillar from right side
  else
  {while(1)
  {Motor_forward(40);
  if(read_cam()!='n')
  break;
  }
}
rest_means();
update_sensors();

if(read_cam()=='g') //pixy cam read green pillar
    startup('g');  //avoid pillar from left side
  else if(read_cam()=='r') //pixy cam read red pillar
    startup('r');  //avoid pillar from right side
  else
  {while(1)
  {Motor_forward(40);
  Front_MPU_pid();
  if(sl>100) //when the servo left ultrasonic read more than 100cm ,the vehicle turns left
    {Turn_left();
    break;}
  if(sr>100) //when the servo right ultrasonic read more than 100cm ,the vehicle turns right
    {Turn_right();
    break;}
  }
}
rest_means();
update_sensors();
}


char read_cam()
{ int numg = 0;
  int numr = 0;
  int i;
  p.ccc.getBlocks();
  double a=(p.ccc.blocks[0].m_height)*(p.ccc.blocks[0].m_width);
  for (i = 0; i < p.ccc.numBlocks; i++)//this loop allows pixy cam to read
  {
    if (p.ccc.blocks[i].m_signature == 1)
    {
      numg++;
    }
    if (p.ccc.blocks[i].m_signature == 2 || p.ccc.blocks[i].m_signature == 3 || p.ccc.blocks[i].m_signature == 4)
    {
      numr++;
    }
  }
  if (numg >= 1 && a>=250) 
  {
    return('g');//when pixy cam see green pillar
  }
  else if (numr >= 1 && a>=250)
  {
    return('r');//when pixy cam see red pillar
  }
  else
  {
    return('n');//when pixy cam didn't see any thing
  }}
