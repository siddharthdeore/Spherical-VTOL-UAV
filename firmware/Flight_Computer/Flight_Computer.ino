#include<Servo.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (10)

#define BEEP 11
#define HWS Serial
// ************** CDH
String s1, s2, s3, s4;
char Str1[3] = "X";
char Str2[3] = "Y";
char Str3[3] = "Z";
char Str4[3] = "B";
int Buttons; //Digital Buttons
int x = 4095;
int y = 4095;
int z = 4095;
int btn = 0;
int la = 4095;
int lo = 4095;
int al = 4095;

int X, Y, Z; //Analog
boolean buttons[10];
boolean flag = 0;
//****************

Adafruit_BNO055 bno = Adafruit_BNO055(55);

boolean state;

Servo F1, F2, F3, F4, R1, R2, R3, R4, ESC;

short rcPin[] = {13, 14, 15, 16};

void setup()
{
  Serial.begin(115200);

  ESC.attach(6, 900, 2200);
  ESC.writeMicroseconds(900);
  R1.attach(2, 750, 2200); //1450
  R2.attach(4, 750, 2200); //1360
  R3.attach(5, 750, 2200); //1490
  R4.attach(3, 750, 2200); //1430 -550 to + 550
  F1.attach(20, 750, 2200);
  F2.attach(21, 750, 2200);
  F3.attach(22, 750, 2200);
  F4.attach(23, 750, 2200);

  R1.writeMicroseconds(1450);
  R2.writeMicroseconds(1360);
  R3.writeMicroseconds(1490);
  R4.writeMicroseconds(1430);

  F1.writeMicroseconds(1250); //1250
  F2.writeMicroseconds(1350); //1350
  F3.writeMicroseconds(1370); //1350
  F4.writeMicroseconds(1280); //1280
  delay(100);

  pinMode(13, INPUT);
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);

  pinMode(BEEP, OUTPUT);
  /* Initialise the sensor */
  if (!bno.begin())
  {
    while (!bno.begin())
    {
      digitalWrite(BEEP, state);
      state = !state;
      delay(500);
    }
    digitalWrite(BEEP, 1);
    delay(200);
    digitalWrite(BEEP, 0);
  }
}

uint16_t ch[4], chMax[] = {1800, 1800, 1800, 1800}, chMin[] = {1100, 1100, 1100, 1100};
uint16_t SPD = 900;

int roll, pitch, yaw;
float PIDroll, I_roll, D_roll, p_roll, rkp = 24.02, rki, rkd = 0;
float PIDpitch, I_pitch, D_pitch, p_pitch, pkp = 24.02, pki, pkd = 0;
float PIDyaw, I_yaw, D_yaw, p_yaw, ykp = 50.2, yki = 0.2, ykd = 3;
float PID, P, I, D, kp, ki, kd, p_P;
uint32_t t, p_t, dt, slp;

int a, b, c;
int np, nr;
boolean rcflag;
void loop()
{
  t = millis();
  dt = t - p_t;
  p_t = t; //time

  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> GY = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  if (1)
  {
    x = (euler.z() + 180);//roll
    y = (euler.y() + 180);//pitch
    z = (euler.x() + 180);//yaw

    digitalWrite(BEEP, 0);
    roll = euler.z();  // roll
    pitch = euler.y(); // pitch
    yaw = euler.x() - 180;
    P = GY.z();
  }
  else
  {
    ESC.writeMicroseconds(900);
    state = !state;
    digitalWrite(BEEP, state);
  }

  for (short i = 0; i < 4; i++)
  {
    ch[i] = pulseIn(rcPin[i], HIGH, 20000);
    if (ch[i] > 800)
    {
      chMin[i] = chMin[i] > ch[i] ? ch[i] : chMin[i];
      chMax[i] = chMax[i] < ch[i] ? ch[i] : chMax[i];
    }
  }

  if (ch[0] > 0)
  {
    SPD = map(ch[2], chMin[2], chMax[2], 900, 2200);
    rcflag = 1;
  }
  else
  {
    SPD = 900;
    //rcflag = 0;

  }

  I += P;
  D = P - p_P;
  PID = P * 85 + I * 0.021 * dt + 10 * D / dt;
  p_P = P;


  D_roll = roll - p_roll;
  I_roll += roll;
  PIDroll = 30 * roll + rki * I_roll + 0.21 * D_roll / dt;
  p_roll = roll;

  D_pitch = pitch - p_pitch;
  I_pitch += pitch;
  PIDpitch = 30 * pitch + pki * I_pitch + 0.2 * D_pitch / dt;
  p_pitch = pitch;

  D_yaw = yaw - p_yaw;
  I_yaw += yaw;
  PIDyaw = 0 * yaw + 0.02 * I_yaw + 0.01 * D_yaw / dt;
  p_yaw = yaw;

  PIDyaw = PIDyaw > 200 ? 200 : PIDyaw;
  PIDyaw = PIDyaw < -200 ? -200 : PIDyaw;

/*
 * IMU is placed 45 Degree ie aprox 0.7854 rad rotated about z axis(yaw). 
 * nr and np are new rotated readings roll and pitch respectively
*/
  np = roll * cos(0.7854) - pitch * cos(0.7854);
  nr = roll * sin(0.7854) + pitch * sin(0.7854);

  float PIDnroll = 20 * nr;
  float PIDnpitch = 20 * np;

  a = PID;
  b = PIDpitch ;
  c = PIDroll ;

  a = a > 600 ? 600 : a;
  a = a < -600 ? -600 : a;

  b = b > 600 ? 600 : b;
  b = b < -600 ? -600 : b;

  int zang = map(ch[0], chMin[0], chMax[0], -400, 400);
  int yang = map(ch[1], chMin[1], chMax[1], -400, 400);
  int xang = map(ch[3], chMin[3], chMax[3], -400, 400);

  int p = a - PIDyaw * 0;

  //  R1.writeMicroseconds(1450 - PIDpitch * 0  + p + xang);
  //  R3.writeMicroseconds(1490 + PIDpitch * 0  + p - xang);
  //  R2.writeMicroseconds(1360 + PIDroll * 0 + p + yang);
  //  R4.writeMicroseconds(1430 - PIDroll + p - yang);
  R1.writeMicroseconds(1450 + a);
  R3.writeMicroseconds(1490 + a); 
  R2.writeMicroseconds(1360 + a);
  R4.writeMicroseconds(1430 + a);

  //  R1.writeMicroseconds(1450 - PIDnroll + p + xang);
  //  R3.writeMicroseconds(1490 + PIDnroll + p - xang);
  //  R2.writeMicroseconds(1360 + PIDnpitch + p + yang);
  //  R4.writeMicroseconds(1430 - PIDnpitch + p - yang);
  //

  F1.writeMicroseconds(1250 + b); //1250
  F2.writeMicroseconds(1350 - b); //1350
  F3.writeMicroseconds(1370 - c); //1350
  F4.writeMicroseconds(1280 + c); //1280



  ESC.writeMicroseconds(SPD);

  //  Serial.print(b);  Serial.print(" c");
  //  Serial.print(c);  Serial.print(" ");
  //  Serial.print(roll);  Serial.print(" ");
  //  Serial.print(pitch);  Serial.print(" ");
  //
  if ((millis() - slp) > 100)
  {
    sendPacket();
    slp = millis();
  }
  //  Serial.println();

}


void printRC()
{
  Serial.print(ch[0]);  Serial.print(" ");
  Serial.print(ch[1]);  Serial.print(" ");
  Serial.print(ch[2]);  Serial.print(" ");
  Serial.print(ch[3]);  Serial.print(" | ");
  Serial.print(chMax[0]);  Serial.print(" ");
  Serial.print(chMax[1]);  Serial.print(" ");
  Serial.print(chMax[2]);  Serial.print(" ");
  Serial.print(chMax[3]);  Serial.print(" | ");
  Serial.print(chMin[0]);  Serial.print(" ");
  Serial.print(chMin[1]);  Serial.print(" ");
  Serial.print(chMin[2]);  Serial.print(" ");
  Serial.print(chMin[3]);;
}
