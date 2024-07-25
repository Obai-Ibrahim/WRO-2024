#include <Servo.h>
#include <NewPing.h>
#include <Pixy2.h>
#include <ArduinoQueue.h>
#include "impu.h"
long t;
int x = 0;
long time_delay;
// MPU6050 module values
int number_of_turns = 0;
float yaw, last_yaw = 0;
double delta_yaw, real_value = 0;
double imu_overall_val = 0;         //yaw overall value from the starting position range from -360 * 3 to 360 * 3
double imu_current_val = 0;         //yaw straight section value range from -90 to 90
int mx = 41, my = 20, mz = -2;              //imu mpu6050 offsets used for calibration
impu imu(mx, my);

// enc
volatile byte reading = 0;
static int A_R = 2 ;
volatile int aFlag_R = 0;
static int B_R = 3;
volatile int bFlag_R = 0;
int Pos_R = 0;
// enc

Pixy2 p;
double m = 0.0185;
int left_g = A2, left_y = A3, right_g = A0, right_y = A1;
int Servo_Pin = 3;
int ServoStraight = 105;
int Servo_Left_Range = 40;
int Servo_Right_Range = 40;
int ServoCritical_Left_Range = 60;
int ServoCritical_Right_Range = 35;
double duration_R1, duration_R2, duration_L1, duration_L2, duration_F, duration_B, duration_SL, duration_SR, f, b, fl, fr, bl, br, sr, sl  , Real_left = 0, Real_right = 0;
double front = 0, back = 0, right1 = 0, right2 = 0, left1 = 0, left2 = 0, servo_left = 0, servo_right = 0;
int front_id = 0, servor_id = 7, right1_id = 2, right2_id = 3, back_id = 4, left2_id = 5, left1_id = 6, servol_id = 7;
int echo[9] = { 26 , 30 , 42 , 44 , 46 , 50 , 52 , 22  };
int trig[9] = { 27 , 31 , 43 , 45 , 47 , 51 , 53 , 23 };
int Filter_Size = 7;
int threshold = 13;
double temp = 0;
double mean_bl = 0;
ArduinoQueue <double> Means_BL;
double mean_fl = 0;
ArduinoQueue <double> Means_FL;
double mean_br = 0;
ArduinoQueue <double> Means_BR;
double mean_fr = 0;
ArduinoQueue <double> Means_FR;
double mean_f = 0;
ArduinoQueue <double> Means_F;
double mean_b = 0;
ArduinoQueue<double> Means_B;
double mean_sl = 0;
ArduinoQueue <double> Means_SL;
double mean_sr = 0;
ArduinoQueue <double> Means_SR;
int MAX_DISTANCE = 300;


NewPing ultra[8] = {     // Sensor object array.
  NewPing(trig[0], echo[0], MAX_DISTANCE),
  NewPing(trig[1], echo[1], MAX_DISTANCE),
  NewPing(trig[2], echo[2], MAX_DISTANCE),
  NewPing(trig[3], echo[3], MAX_DISTANCE),
  NewPing(trig[4], echo[4], MAX_DISTANCE),
  NewPing(trig[5], echo[5], MAX_DISTANCE),
  NewPing(trig[6], echo[6], MAX_DISTANCE),
  NewPing(trig[7], echo[7], MAX_DISTANCE)
};
int MPU = 0;
int mpu_reference = 0;
double mpu_kp = 3, mpu_kd = 1.5, mpu_error, mpu_prev_error, mpu_time, mpu_last_time, ultra_error, ultra_kpL = 2.5, ultra_kpR = 4;

long t1;
int R_EN = 11;
int L_EN = 10;
int R_PWM = 9;
int L_PWM = 8;
Servo A;
Servo R;
Servo L;
int thresh = 90;
int deg = 87;

double ang, distance;
int left_turn = 0, right_turn = 0, turn_counter = 0;
int turn_range = 40;
int speed_turn_back = 30, speed_turn = 30, speed_befor_turn = 20, speed_normal = 30, speed_danger = 15;




double Filter(double read, double &last_mean, ArduinoQueue <double> &Means)
{
  if (Means.isEmpty())
  {
    last_mean = read;
    for (int i = 0; i < Filter_Size; i++)
    {
      Means.enqueue(read);
    }
  }
  else
  {
    temp = Means.dequeue();
    last_mean -= (double(temp / Filter_Size));
    last_mean += (double(read / Filter_Size));
    Means.enqueue(read);
  }
  return last_mean;
}

double get_distance_F()
{
  duration_F = ultra[0].ping();
  f = (duration_F / 2) * 0.0343;// Calculate the distance based on utlra sonic wave speed and time taken by the wave.
  if (f == 0)
    f = MAX_DISTANCE;
  mean_f = Filter(f, mean_f, Means_F);// Take the filtered value
  return mean_f;
}

double get_distance_B() {
  duration_B = ultra[4].ping();
  b = ((duration_B / 2) * 0.0343);
  if (b == 0)
    b = MAX_DISTANCE;
  mean_b = Filter(b, mean_b, Means_B);
  return mean_b;
}

double get_distance_SL ()
{
  duration_SL = ultra[7].ping();
  sl = (duration_SL / 2) * 0.0343;// Calculate the distance based on utlra sonic wave speed and time taken by the wave.
  if (sl == 0)
    sl = MAX_DISTANCE;
  mean_sl = Filter(sl, mean_sl, Means_SL);// Take the filtered value
  return mean_sl;
}

double get_distance_SR ()
{
  duration_SR = ultra[1].ping();
  sr = (duration_SR / 2) * 0.0343;// Calculate the distance based on utlra sonic wave speed and time taken by the wave.
  if (sr == 0)
    sr = MAX_DISTANCE;
  mean_sr = Filter(sr, mean_sr, Means_SR);// Take the filtered value
  return mean_sr;
}

double get_distance_L1 ()
{
  duration_L1 = ultra[6].ping();
  fl = ((duration_L1 / 2) * 0.0343)  ;
  if (fl == 0)
    fl = MAX_DISTANCE;
  mean_fl = Filter(fl, mean_fl, Means_FL);
  return mean_fl ;
}

double get_distance_L2()
{
  duration_L2 = ultra[5].ping();
  bl = ((duration_L2 / 2) * 0.0343)  ;
  if (bl == 0)
    bl = MAX_DISTANCE;
  mean_bl = Filter(bl, mean_bl, Means_BL);
  return mean_bl;
}

double get_distance_R1 ()
{
  duration_R1 = ultra[2].ping();
  fr = ((duration_R1 / 2) * 0.0343);
  if (fr == 0)
    fr = MAX_DISTANCE;
  mean_fr = Filter(fr, mean_fr, Means_FR);
  return mean_fr;
}

double get_distance_R2()
{
  duration_R2 = ultra[3].ping();
  br = ((duration_R2 / 2) * 0.0343) ;
  if (br == 0)
    br = MAX_DISTANCE;
  mean_br = Filter(br, mean_br, Means_BR);

  return mean_br ;
}


void reset_means()
{
  for (int i = 0 ; i < Filter_Size ; i++)
  {
    Means_B.dequeue();
    Means_BL.dequeue();
    Means_BR.dequeue();
    Means_FL.dequeue();
    Means_FR.dequeue();
    Means_F.dequeue();
    Means_SL.dequeue();
    Means_SR.dequeue();
  }
}

double Real_distance(double read_1, double read_2) {
  return (read_1 + read_2) / 2 * cos((yaw - mpu_reference) * PI / 180);
}

double Fast_dist(int id) {
  double duration = ultra[id].ping();
  double fast_distance = ((duration / 2) * 0.0343);
  if (fast_distance == 0)
    fast_distance = MAX_DISTANCE;
  return fast_distance;
}


void my_delay(int t) {
  time_delay = millis();
  while (millis() - time_delay < t)
  {
    read_yaw();
  }
}

void update_sensors()
{
  front = get_distance_F();
  my_delay(10);
  back = get_distance_B();
  my_delay(10);
  left1 = get_distance_L1();
  my_delay(10);
  right1 = get_distance_R1();
  my_delay(10);
  left2 = get_distance_L2();
  my_delay(10);
  right2 = get_distance_R2();
  my_delay(10);
  servo_right = get_distance_SR();
  my_delay(10);
  servo_left = get_distance_SL();
  //Real_left = Real_distance(left1, left2);
  //Real_right = Real_distance(right1, right2);
}


void Motor_forward(int percent) {
  int PWM = percent * 254 / 100;
  analogWrite(R_PWM, PWM);
  analogWrite(L_PWM, 0);
}


void Motor_backward(int percent) {
  int PWM = percent * 254 / 100;
  analogWrite(L_PWM, PWM);
  analogWrite(R_PWM, 0);
}


void Motor_stop() {
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
}



void Ser(int deg) {
  if (deg < ServoStraight - Servo_Left_Range)
    A.write(ServoStraight - Servo_Left_Range);
  else if (deg > ServoStraight + Servo_Right_Range)
    A.write(ServoStraight + Servo_Right_Range);
  else
    A.write(deg);
}

void Ser_Critical(int deg) {
  if (deg < ServoStraight - ServoCritical_Left_Range)
    A.write(ServoStraight - ServoCritical_Left_Range);
  else if (deg > ServoStraight + ServoCritical_Right_Range)
    A.write(ServoStraight + ServoCritical_Right_Range);
  else
    A.write(deg);
  delay(50);
}



void Back_MPU_pid() {
  mpu_error = imu_current_val - mpu_reference;
  Ser(ServoStraight + (mpu_kp * mpu_error));
}

void Front_MPU_pid() {
  mpu_error = imu_current_val - mpu_reference;
  Ser(ServoStraight - (mpu_kp * mpu_error));
}


void servo_ultra_pid() {
  double error = imu_current_val - mpu_reference;
  delay(10);
  if (error > 80) {
    L.write(10);
    R.write(10);
  }
  else if (error < -80) {
    L.write(170);
    R.write(170);
  }
  else {
    L.write(90 - error);
    R.write(90 - error);
  }
  //  mpu.update();
}



void setup() {
  //Serial.begin(115200);
  imu.init();
  Wire.setWireTimeout(3000, true);
  Serial.begin(9600);
  p.init();
  delay(30);
  Serial.println("Ok");
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  //Serial.println("hello");

  /*   pinMode(A0, OUTPUT);
     pinMode(A1, OUTPUT);
     pinMode(A2, OUTPUT);
     pinMode(A3, OUTPUT);
     pinMode(A7, INPUT);*/
  //setup motors states


  //enc
  //  attachInterrupt(0, PinA_R, RISING);
  // pinMode(A_R, INPUT);
  // attachInterrupt(1, PinB_R, RISING);
  // pinMode(B_R, INPUT);
  //enc
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);
  A.attach(3);
  R.attach(5);
  L.attach(4);
  A.write(ServoStraight);
  R.write(90);
  L.write(90);
  //analogWrite(R_PWM, 0);
  //analogWrite(L_PWM, 0);
  delay(500);
  //delay(500);
  //Serial.println("hello");
  //mpu_reference = mpu.getAngleZ();
  read_yaw();
  mpu_reference = yaw;
}



void Turn_right() {
  L.write(90);
  reset_means();
  while (get_distance_F() > 60)
  {
    my_delay(30);
    if (Fast_dist(7) > 65)
      A.write(ServoStraight - 5);
    else
      Front_MPU_pid();
    update_sensors();
    Motor_forward(40);
    read_yaw();
  }
  Motor_stop();
  my_delay(100);
  while ( ( Fast_dist(2) < 25 || Fast_dist(3) < 25 ) && ( Fast_dist(5) < 35 || Fast_dist(6) < 35) )
  {
    A.write(ServoStraight - 5 );
    Motor_forward(40);
    read_yaw();
    update_sensors();
  }
  Motor_stop();
  A.write(ServoStraight);
  my_delay(100);
  
  
  double d=get_distance_SL();
  if (d < 25)
  {
    while (get_distance_F() < 67)
    { Motor_backward(40);
      Back_MPU_pid();
      my_delay(30);
    }
    Motor_stop();
    A.write(ServoStraight + 35);
    R.write(20);
    L.write(160);
    read_yaw();
    reset_means();
    my_delay(100);
    Motor_forward(40);

    while (imu_current_val < mpu_reference + 90)
    { my_delay(30);
      if (Fast_dist(0) < 20 || Fast_dist(1) < 20 || Fast_dist(7) < 20)
        break;
    }
    mpu_reference += 90;
    Motor_stop();
    A.write(ServoStraight);
    R.write(90);
    L.write(90);
    my_delay(50);
    while (Fast_dist(4) > 20)
    { my_delay(30);
      Motor_backward(40);
      Back_MPU_pid();
    }
    while (Fast_dist(4) < 20)
    { my_delay(30);
      Motor_forward(40);
      Front_MPU_pid();
    }
    Motor_stop();
    A.write(ServoStraight);
    my_delay(100);
  }


  else if (d < 45)
    {
    while (Fast_dist(0) < 65)
    { Motor_backward(40);
      Back_MPU_pid();
      my_delay(30);
    }
    while ( get_distance_SL() > 25)
    {
      A.write(ServoStraight - 35);
      Motor_forward(40);
      servo_ultra_pid();
      my_delay(30);
    }
    Motor_stop();
    A.write(ServoStraight + 35);
    L.write(90);
    R.write(10);
    my_delay(100);
    Motor_forward(40);
    while (imu_current_val < mpu_reference)
    {
      my_delay(30);
      if (Fast_dist(0) < 15 || Fast_dist(1) < 15)
        break;
    }
    
    Motor_stop();
    R.write(90);
    A.write(ServoStraight);
    my_delay(50);

    while (get_distance_F() < 67)
    { Motor_backward(40);
      Back_MPU_pid();
      my_delay(30);
    }

    Motor_stop();
    A.write(ServoStraight + 35);
    R.write(20);
    L.write(160);
    read_yaw();
    reset_means();
    my_delay(100);
    Motor_forward(40);

    while (imu_current_val < mpu_reference + 90)
    { my_delay(30);
      if (Fast_dist(0) < 20 || Fast_dist(1) < 20 || Fast_dist(7) < 20)
        break;
    }
    mpu_reference += 90;
    Motor_stop();
    A.write(ServoStraight);
    R.write(90);
    L.write(90);
    my_delay(50);
    while (Fast_dist(4) > 20)
    { my_delay(30);
      Motor_backward(40);
      Back_MPU_pid();
    }
    while (Fast_dist(4) < 20)
    { my_delay(30);
      Motor_forward(40);
      Front_MPU_pid();
    }
    Motor_stop();
    A.write(ServoStraight);
    my_delay(100);
    }

    
    else
    {
    if (Fast_dist(7) > 70)
    {
      while (Fast_dist(0) > 30)
      {
        Motor_forward(40);
        A.write(ServoStraight-5);
        my_delay(30);
      }
    }
    else {
      while (Fast_dist(0) > 30)
      {
        Motor_forward(40);
        Front_MPU_pid();
        my_delay(30);
      }
    }
    Motor_stop();
    A.write(ServoStraight+25);
    my_delay(100);
    while (Fast_dist(0) > 15)
      {
        Motor_forward(40);
        my_delay(30);
      }

    Motor_stop();
    A.write(ServoStraight - 55);
    my_delay(100);
    while (imu_current_val < mpu_reference + 83)
    { Motor_backward(40);
    my_delay(30);
      if (Fast_dist(4) < 15)
        break;
    }
    Motor_stop();
    A.write(ServoStraight);
    my_delay(50);
    mpu_reference+=90;
    while (Fast_dist(4) > 20)
    { my_delay(30);
      Motor_backward(40);
      Back_MPU_pid();
    }
    while (Fast_dist(4) < 20)
    { my_delay(30);
      Motor_forward(40);
      Front_MPU_pid();
    }
    Motor_stop();
    A.write(ServoStraight);
    my_delay(100);
    }
  reset_means();
}






void Turn_left() {
  R.write(90);
  reset_means();
  while (get_distance_F() > 60) //to read the correct distance from the wall 
  {
    my_delay(30);
    if (Fast_dist(1) > 65)
      A.write(ServoStraight + 5);
    else
      Front_MPU_pid();
    update_sensors();
    Motor_forward(40);
    read_yaw();
  }
  Motor_stop();
  my_delay(100);
  while ( ( Fast_dist(2) < 35 || Fast_dist(3) < 35 ) && ( Fast_dist(5) < 25 || Fast_dist(6) < 25) )
  {
    A.write(ServoStraight+5 );
    Motor_forward(40);
    read_yaw();
    update_sensors();
  }
  Motor_stop();
  A.write(ServoStraight);
  my_delay(100);


  double d=get_distance_SR();
  if (d < 25)
  {
    while (get_distance_F() < 69)
    { Motor_backward(40);
      Back_MPU_pid();
      my_delay(30);
    }
    
    Motor_stop();
    A.write(ServoStraight - 40);
    R.write(20);
    L.write(160);
    read_yaw();
    reset_means();
    my_delay(100);
    Motor_forward(40);

    
    while (imu_current_val > mpu_reference - 90)
    { my_delay(30);
      if (Fast_dist(0) < 20 || Fast_dist(1) < 20 || Fast_dist(7) < 20)
        break;
    }
    mpu_reference -= 90;
    Motor_stop();
    A.write(ServoStraight);
    R.write(90);
    L.write(90);
    my_delay(50);
    while (Fast_dist(4) > 20)
    { my_delay(30);
      Motor_backward(40);
      Back_MPU_pid();
    }
    while (Fast_dist(4) < 20)
    { my_delay(30);
      Motor_forward(40);
      Front_MPU_pid();
    }
    Motor_stop();
    A.write(ServoStraight);
    my_delay(100);
  }

  
  else if (d < 45)
    {
    while (Fast_dist(0) < 65)
    { Motor_backward(40);
      Back_MPU_pid();
      my_delay(30);
    }
    while ( get_distance_SR() > 25)
    {
      A.write(ServoStraight + 35);
      Motor_forward(40);
      servo_ultra_pid();
      my_delay(30);
    }
    Motor_stop();
    A.write(ServoStraight - 40);
    R.write(90);
    L.write(170);
    my_delay(100);
    Motor_forward(40);
    while (imu_current_val > mpu_reference)
    {
      my_delay(30);
      if (Fast_dist(0) < 15 || Fast_dist(7) < 15)
        break;
    }
    
    Motor_stop();
    L.write(90);
    A.write(ServoStraight);
    my_delay(50);
    
    while (get_distance_F() < 69)
    { Motor_backward(40);
      Back_MPU_pid();
      my_delay(30);
    }
    
    Motor_stop();
    A.write(ServoStraight - 40);
    R.write(20);
    L.write(160);
    read_yaw();
    reset_means();
    my_delay(100);
    Motor_forward(40);
    
    while (imu_current_val > mpu_reference - 90)
    { my_delay(30);
      if (Fast_dist(0) < 20 || Fast_dist(1) < 20 || Fast_dist(7) < 20)
        break;
    }
    mpu_reference -= 90;
    Motor_stop();
    A.write(ServoStraight);
    R.write(90);
    L.write(90);
    my_delay(50);
    while (Fast_dist(4) > 20)
    { my_delay(30);
      Motor_backward(40);
      Back_MPU_pid();
    }
    while (Fast_dist(4) < 20)
    { my_delay(30);
      Motor_forward(40);
      Front_MPU_pid();
    }
    Motor_stop();
    A.write(ServoStraight);
    my_delay(100);
  }

  
  else
    {
     if (Fast_dist(1) > 70)
    {
      while (Fast_dist(0) > 30)
      {
        Motor_forward(40);
        A.write(ServoStraight+5);
        my_delay(30);
      }
    }
    else {
      while (Fast_dist(0) > 30)
      {
        Motor_forward(40);
        Front_MPU_pid();
        my_delay(30);
      }
    }
    Motor_stop();
    A.write(ServoStraight-25);
    my_delay(100);
    while (Fast_dist(0) > 15)
      {
        Motor_forward(40);
        my_delay(30);
      }

    Motor_stop();
    A.write(ServoStraight + 40);
    my_delay(100);
    while (imu_current_val > mpu_reference - 83)
    { Motor_backward(40);
    my_delay(30);
      if (Fast_dist(4) < 15)
        break;
    }
    Motor_stop();
    A.write(ServoStraight);
    my_delay(50);
    mpu_reference-=90;
    while (Fast_dist(4) > 20)
    { my_delay(30);
      Motor_backward(40);
     Back_MPU_pid();
    }
    while (Fast_dist(4) < 20)
    { my_delay(30);
      Motor_forward(40);
      Front_MPU_pid();
    }
    Motor_stop();
    A.write(ServoStraight);
    my_delay(100);
    }
  reset_means();
}



void startup(char c) {
  if (c == 'R' || c == 'r')
  {
    update_sensors();
    A.write(ServoStraight + 30);//to avoid the red pillar from right side
    my_delay(50);
    Motor_forward(40);
    if (Fast_dist(2) < 50)
    { while (Fast_dist(1) > 20)
      {my_delay(30);
      servo_ultra_pid();}
    }
    else {
      while (Fast_dist(1) > 25)
      {my_delay(30);
      servo_ultra_pid();}
    }
    Motor_stop();
    A.write(ServoStraight - 50);
    my_delay(100);
    Motor_forward(40);
    while(imu_current_val > mpu_reference)//return the vehicle to the front direction
    { my_delay(30);
      servo_ultra_pid();
      Front_MPU_pid();}
    Motor_stop();
    A.write(ServoStraight);
    my_delay(100);

    
    while (1)   //wait for left turn or right turn
    {
      Motor_forward(40);
    if (get_distance_SL() > 90 || get_distance_L1() > 90 || get_distance_L2() > 90)
    {
      Turn_left();
      break;
    }

    if (get_distance_SR() > 90 || get_distance_R1() > 90 || get_distance_R2() > 90)
    {
      Turn_right();
      break;
    }
  }
  }
  else if (c == 'G' || c == 'g')
  {
    A.write(ServoStraight - 30);//to avoid the green pillar from left side
    my_delay(50);
    Motor_forward(40);
    if (Fast_dist(6) < 50) {
      while (Fast_dist(7) > 20)
      { servo_ultra_pid();
      my_delay(30);
      }
    }
    else
    { while (Fast_dist(7) > 25)
      { servo_ultra_pid();
      my_delay(30);
      }
    }
    Motor_stop();
    A.write(ServoStraight + 40);
    L.write(90);
    my_delay(50);
    Motor_forward(40);
    while(imu_current_val < mpu_reference)  //return the vehicle to the front direction
    { my_delay(30);
      servo_ultra_pid();
      Front_MPU_pid();}
    Motor_stop();
    A.write(ServoStraight);
    my_delay(100);
    while ( 1) //wait for left turn or right turn 
    {
      Motor_forward(40);
      if (Fast_dist(6) > 90 || Fast_dist(7) > 90 || Fast_dist(5) > 90)
      {
        Turn_left();
        break;
      }

      if (Fast_dist(1) > 90 || Fast_dist(2) > 90 || Fast_dist(3) > 90)
      {
        Turn_right();
        break;
      }
    }
  }
  else {
    while (1) // when there is no green or red pillar ,wait for left turn or right turn 
    {
      Motor_forward(40);
      if (Fast_dist(6) > 90 || Fast_dist(7) > 90 || Fast_dist(5) > 90)
      {
        Turn_left();
        break;
      }

      if (Fast_dist(1) > 90 || Fast_dist(2) > 90 || Fast_dist(3) > 90)
      {
        Turn_right();
        break;
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
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
Motor_stop();
delay(10000);
}




void read_yaw() {
  imu.getyaw(yaw);        //get the value of yaw

  if (Wire.getWireTimeoutFlag()) {
    Wire.end();
    delay(10);
    Wire.begin();
    delay(10);
    Wire.clearWireTimeoutFlag();
  }
  //delay(5);         //so it won't ask the dmp for too many values in a short time
  delta_yaw = yaw - last_yaw ;    //get the delta of yaw value
  if (delta_yaw < 150 && delta_yaw > -150) { //solving the problem that the dmp return values 180 and -180 in a random
    real_value += delta_yaw;
    imu_overall_val = real_value; // kalman.updateEstimate(real_value);
  }
  last_yaw = yaw;
  imu_current_val = imu_overall_val ;//+ 90 * number_of_turns;   // get the yaw straight section value range to be  from -90 to 90
}



void DistL_pid(double distance) {
  if (distance < threshold) {
    ultra_error = threshold - distance;
    Ser(ServoStraight + ultra_kpL * ultra_error);
    delay(10);
  }
}

void DistR_pid(double distance) {
  if (distance < threshold) {
    ultra_error = threshold - distance;
    Ser(ServoStraight - ultra_kpR * ultra_error);
    delay(10);
  }
}





void Sensor_disp() {
  Serial.print("  front:");
  Serial.print(front);
  Serial.print("  s_r:");
  Serial.print(servo_right);
  Serial.print("  right1:");
  Serial.print(right1);
  Serial.print("  right2:");
  Serial.print(right2);
  Serial.print("  back:");
  Serial.print(back);
  Serial.print(" left2:");
  Serial.print(left2);
  Serial.print("  left1:");
  Serial.print(left1);
  Serial.print("  s_l:");
  Serial.println(servo_left);
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
