#include <ECE3.h>
#define Kp 0.018
#define Kd 0.0028

uint16_t sensorValues[8]; // right -> left, 0 -> 7

const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;

const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

int prevPos;
int blackBar;
int p,d;
int default_speed = 75;

void setup()
{
  ECE3_Init();

  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);
  
  Serial.begin(9600); // set the data rate in bits per second for serial data transmission
  prevPos = 0;
  p = d = 0;
  blackBar = 0;
  delay(3000); //Wait 3 seconds before starting 
}

int getPos()
{
  // read raw sensor values
  ECE3_read_IR(sensorValues);
      
  double s0 = (sensorValues[0] - 778) * 1000/1722;
  double s1 = (sensorValues[1] - 686) * 1000/1747;
  double s2 = (sensorValues[2] - 709) * 1000/1791;
  double s3 = (sensorValues[3] - 663) * 1000/1557;
  double s4 = (sensorValues[4] - 709) * 1000/1791;
  double s5 = (sensorValues[5] - 640) * 1000/1790;
  double s6 = (sensorValues[6] - 550) * 1000/1532;
  double s7 = (sensorValues[7] - 686) * 1000/1814;
  //fusion process
  return ((-8 * s0) + (-4 * s1) + (-2 * s2) + (-1 * s3) +
          (1 * s4) + (2 * s5) + (4 * s6) + (8 * s7))/4;
}

void loop()
{
  //if at solid black line (not the start)
  if ((sensorValues[0] == sensorValues[1]) && (sensorValues[1] == sensorValues[2])
  && (sensorValues[2] == sensorValues[3]) && (sensorValues[3] == sensorValues[4]) 
  && (sensorValues[4] && sensorValues[5]) && (sensorValues[5] == sensorValues[6])
  && (sensorValues[6] == sensorValues[7]))
  {
    //donut
    digitalWrite(right_dir_pin,HIGH);
    analogWrite(left_pwm_pin,80);
    analogWrite(right_pwm_pin,80);
    delay(810);
    digitalWrite(right_dir_pin,LOW);
  }

  
  //get PID values
  p = getPos();
  d = p - prevPos;

  //get speed from PID
  double speed = (Kp * p) + (Kd * d);
  prevPos = p; 

  //get left and right motor speeds
  //must be within range for analogWrite
  double leftSpeed = default_speed - speed;
  double rightSpeed = default_speed + speed;

  if (rightSpeed < 0)
  {
    rightSpeed *= -1;
    digitalWrite(right_dir_pin, HIGH);
  }
  else
    digitalWrite(right_dir_pin,LOW);
  if (rightSpeed > 255)
  {
    rightSpeed = 255;
  }
  if (leftSpeed < 0)
  {
    leftSpeed *= -1;
    digitalWrite(left_dir_pin,HIGH);
  }
  else 
    digitalWrite(left_dir_pin,LOW);
  if (leftSpeed > 255)
  {
    leftSpeed = 255;
  }
  //double leftSpeed = constrain(default_speed - speed,0,255);
  //double rightSpeed = constrain(default_speed + speed,0,255);

  analogWrite(left_pwm_pin,leftSpeed);
  analogWrite(right_pwm_pin,rightSpeed);
  
}
