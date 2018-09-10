/*
 * CT Asia Robotics Co.,Ltd.
 * Bangkok - Thailand.
 * Dinsow 4 Waist Balance using PID Control
 * by: Muchammad Sobrun Ibnu Atfal
 * Research and Developtment
 * last modified: 12 July 2017.
*/

 
#include  <EEPROM.h>
#include  <Wire.h>
#include  "Kalman.h"
#define   RESTRICT_PITCH 

double  position_dot_filter;
double  accX, accY, accZ;
double  gyroX, gyroY, gyroZ;
double  gyroXangle, gyroYangle;                          // Gyroscope angle
double  compAngleX, compAngleY;                          // Complementary filter angle 
double  kalAngleX, kalAngleY;                            // Angle after Kalman filter
double  corrected_x, corrected_y;                        // Corrected with offset
double  kp = 30;
double  ki = 10;
double  kd = 0.5;
double  P = 0;
double  I = 0;
double  D = 0;
double  error = 0;

float   angle, angular_rate;
bool    blinkState = false;
byte    buf_tmp=0;

int     speed_real_l;
int     Setpoint;
int     rx_count=0;
int     RPWM_Output = 5;                                  // Arduino PWM output pin 5; connect to IBT-2 pin 4 (RPWM)
int     LPWM_Output = 6;                                  // Arduino PWM output pin 6; connect to IBT-2 pin 3 (LPWM)
int     speedmotor;

Kalman kalmanX;
Kalman kalmanY;

int16_t tempRaw;
uint32_t timer;
uint8_t i2cData[14];


void pwm_out(int speedmotor)
{
  if (speedmotor > 0)
  {
    speedmotor=speedmotor;
    analogWrite(RPWM_Output, speedmotor);
    analogWrite(LPWM_Output, 0);
  }

  else if ( speedmotor < 0)
  {
    int result = speedmotor*-1;
    analogWrite(RPWM_Output, 0);
    analogWrite(LPWM_Output, result);
  }   
}

void init_IO()
{
  pinMode(RPWM_Output, OUTPUT);
  pinMode(LPWM_Output, OUTPUT);
}

void setup() {
  init_IO();
 
  Serial.begin(9600);
  delay(50);
  Wire.begin();

  TWBR = ((F_CPU / 400000L) - 16) / 2;                  // Set I2C frequency to 400kHz

  i2cData[0] = 7;                                       // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;                                    // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00;                                    // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00;                                    // Set Accelerometer Full Scale Range to ±2g

  while (i2cWrite(0x19, i2cData, 4, false));            // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true));                   // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) {                             // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100); // Wait for sensor to stabilize

  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
// It is then converted from radians to degrees

  #ifdef RESTRICT_PITCH
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
    
  #else
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();
}

//**************************************************************** PID Setting *******************************************************************//
void controlPID()
{
  Serial.available();
  int key = Serial.read();
  
  while (i2cRead(0x3B, i2cData, 14));
    accX =    ((i2cData[0]   << 8) | i2cData[1]);
    accY =    ((i2cData[2]   << 8) | i2cData[3]);
    accZ =    ((i2cData[4]   << 8) | i2cData[5]);
    tempRaw = ((i2cData[6]   << 8) | i2cData[7]);
    gyroX =   ((i2cData[8]   << 8) | i2cData[9]);
    gyroY =   ((i2cData[10]  << 8) | i2cData[11]);
    gyroZ =   ((i2cData[12]  << 8) | i2cData[13]);
    
// Calculate delta time
    double dt = (double)(micros() - timer) / 1000000; 
    timer = micros();

  #ifdef RESTRICT_PITCH
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else 
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } 
  else
     kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
  
// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) 
    {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } 
    
  else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);                                              // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate;                                                                          // Invert rate, so it fits the restriced accelerometer reading
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);                                               // Calculate the angle using a Kalman filter
  #endif

    gyroXangle += gyroXrate * dt;                                                                      // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;  // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

// Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;
  delay(2);
        
  corrected_x=kalAngleX-171,746;
  corrected_y=kalAngleY-81,80;
  corrected_y = corrected_y+84;
  
  double data   = corrected_y;
  double data2  = corrected_x;
  double lastdata = data;
  float  times = millis();
  float  deltatime =(millis()-times);
  
  if (key == '1')
  {
    Setpoint = 0;
  }
  
  else if (key == '2')
  {
    Setpoint = -30;
  }
  
  else if (key == '3')
  {
    Setpoint = -50;
  }
  
  error = Setpoint - data;                                      // e(t) = error value
  
  I += error * (dt/1000.0);
  I = constrain(I, -100.0, 100.0);
  float last_error = error;                                     // Proportional
  float Integ = ki * I;                                         // Integral
  float rate = (error - last_error)*1000/dt;                    // Derivative
  
  int pid = (int) ((error * kp)+ (Integ * ki) + (rate * kd));   // PID Calculation https://en.wikipedia.org/wiki/File:PID_Compensation_Animated.gif
  int movemotor = speed_real_l + pid;
  int maxspeed = 255;
  int minspeed = -255;
  
  movemotor = constrain(pid,minspeed,maxspeed);
  int a = map(movemotor, -50, 50, -255, 255);
  pwm_out(movemotor); 
  
  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print("\t\t");
  Serial.print("IMU Data Y axis: ");
  Serial.print(corrected_y);
  Serial.print("\t\t");
  Serial.print("IMU Data X axis: ");
  Serial.println(corrected_x);
}
//************************************************************************************************************************************************//
void test()
{
    Serial.available();
  int key = Serial.read();
  
  while (i2cRead(0x3B, i2cData, 14));
    accX =    ((i2cData[0]   << 8) | i2cData[1]);
    accY =    ((i2cData[2]   << 8) | i2cData[3]);
    accZ =    ((i2cData[4]   << 8) | i2cData[5]);
    tempRaw = ((i2cData[6]   << 8) | i2cData[7]);
    gyroX =   ((i2cData[8]   << 8) | i2cData[9]);
    gyroY =   ((i2cData[10]  << 8) | i2cData[11]);
    gyroZ =   ((i2cData[12]  << 8) | i2cData[13]);
    
// Calculate delta time
    double dt = (double)(micros() - timer) / 1000000; 
    timer = micros();

  #ifdef RESTRICT_PITCH
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else 
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } 
  else
     kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
  
// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) 
    {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } 
    
  else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);                                              // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate;                                                                          // Invert rate, so it fits the restriced accelerometer reading
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);                                               // Calculate the angle using a Kalman filter
  #endif

    gyroXangle += gyroXrate * dt;                                                                      // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;  // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

// Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;
  delay(2);
        
  corrected_x=kalAngleX-171,746;
  corrected_y=kalAngleY-81,80;
  corrected_y = corrected_y+84;

  double data = corrected_y;
  double lastdata = data;
  float  times = millis();
  float  deltatime =(millis()-times);

int x;
for (x = 0; x <= 1000; x++)
{
  Serial.println(x);
  //delay(1000);
  if (x >= 0 && x < 500)
  {
    Setpoint=0;
    Serial.println(Setpoint);
    error = Setpoint - data;                                      // e(t) = error value

  I += error * (dt/1000.0);
  I = constrain(I, -100.0, 100.0);
  float last_error = error;                                     // Proportional
  float Integ = ki * I;                                         // Integral
  float rate = (error - last_error)*1000/dt;                    // Derivative
  
  int pid = (int) ((error * 10)+ (Integ * 3) + (rate * 0.1));   // PID Calculation https://en.wikipedia.org/wiki/File:PID_Compensation_Animated.gif
  int movemotor = speed_real_l + pid;
  int maxspeed = 90;
  int minspeed = -90;

  movemotor = constrain(pid,minspeed,maxspeed);
  int a = map(movemotor, -50, 50, -255, 255);
  pwm_out(movemotor);
  }
  else// if (x > 5 && x <10)
  {
    Setpoint=-20;
    Serial.println(Setpoint); 
    error = Setpoint - data;                                      // e(t) = error value

  I += error * (dt/1000.0);
  I = constrain(I, -100.0, 100.0);
  float last_error = error;                                     // Proportional
  float Integ = ki * I;                                         // Integral
  float rate = (error - last_error)*1000/dt;                    // Derivative
  
  int pid = (int) ((error * 10)+ (Integ * 3) + (rate * 0.1));   // PID Calculation https://en.wikipedia.org/wiki/File:PID_Compensation_Animated.gif
  int movemotor = speed_real_l + pid;
  int maxspeed = 90;
  int minspeed = -90;

  movemotor = constrain(pid,minspeed,maxspeed);
  int a = map(movemotor, -50, 50, -255, 255);
  pwm_out(movemotor); 
  }
}
}

void sensor2()
{
  Serial.available();
  int key = Serial.read();
  
  while (i2cRead(0x3B, i2cData, 14));
    accX =    ((i2cData[0]   << 8) | i2cData[1]);
    accY =    ((i2cData[2]   << 8) | i2cData[3]);
    accZ =    ((i2cData[4]   << 8) | i2cData[5]);
    tempRaw = ((i2cData[6]   << 8) | i2cData[7]);
    gyroX =   ((i2cData[8]   << 8) | i2cData[9]);
    gyroY =   ((i2cData[10]  << 8) | i2cData[11]);
    gyroZ =   ((i2cData[12]  << 8) | i2cData[13]);
    
// Calculate delta time
    double dt = (double)(micros() - timer) / 1000000; 
    timer = micros();

  #ifdef RESTRICT_PITCH
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else 
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } 
  else
     kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
  
// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) 
    {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } 
    
  else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);                                              // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate;                                                                          // Invert rate, so it fits the restriced accelerometer reading
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);                                               // Calculate the angle using a Kalman filter
  #endif

    gyroXangle += gyroXrate * dt;                                                                      // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;  // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

// Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;
  delay(2);
        
  corrected_x=kalAngleX-171,746;
  corrected_y=kalAngleY-81,80;
  corrected_y = corrected_y+84;
  
  double data   = corrected_y;
  double data2  = corrected_x;
  double lastdata = data;
  float  times = millis();
  float  deltatime =(millis()-times);
  
  if (key == '1')
  {
    Setpoint = 0;
  }
  
  else if (key == '2')
  {
    Setpoint = -30;
  }
  
  else if (key == '3')
  {
    Setpoint = -50;
  }
  
  error = Setpoint - data;                                      // e(t) = error value
  
  I += error * (dt/1000.0);
  I = constrain(I, -100.0, 100.0);
  float last_error = error;                                     // Proportional
  float Integ = ki * I;                                         // Integral
  float rate = (error - last_error)*1000/dt;                    // Derivative
  
  int pid = (int) ((error * kp)+ (Integ * ki) + (rate * kd));   // PID Calculation https://en.wikipedia.org/wiki/File:PID_Compensation_Animated.gif
  int movemotor = speed_real_l + pid;
  int maxspeed = 255;
  int minspeed = -255;
  
  movemotor = constrain(pid,minspeed,maxspeed);
  int a = map(movemotor, -50, 50, -255, 255);
  pwm_out(movemotor); 
  
  Serial.print("Setpoint: ");
  Serial.print(Setpoint);
  Serial.print("\t\t");
  Serial.print("IMU Data Y axis: ");
  Serial.print(corrected_y);
  Serial.print("\t\t");
  Serial.print("IMU Data X axis: ");
  Serial.println(corrected_x);
}

void loop() 
{
controlPID();
//test();
//pwm_out(-100);
//delay(1000);
//pwm_out(100);
//delay(1000);

/*
for (x = -10; x >= 0; x--)
{
  Serial.println(x);
  delay(1000);
    if (x > -10)
  {
    Setpoint=1;
    Serial.println("1");
  }
}
*/
}
