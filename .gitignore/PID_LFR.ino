#define LM1 2
#define LM2 3
#define RM1 4
#define RM2 5

#define E1 A14
#define E2 A15
long int ir[6]={A1,A2,A3,A4,A5,A6};
long int ir_s[6] = {0,0,0,0,0,0}; // for storing input
long int ir_s1[6] = {1, 2, 3, 4,5,6}; // multiplying storing value
long int ir_s2[6] = {0, 0, 0, 0,0, 0};


double tt,temp;
#define max_speed 255
#define TS 120
#define base_speed 150
double set_point;
double currentpoint;
double last_proportional;
double Kp, Kd, Ki;
double PID_output;
double sum;
int right_motor_speed;
int left_motor_speed;
double proportional, integral, derivative;
long ir_avg;
long ir_sum;

void setup()
{ // put your setup code here, to run once:
  Serial.begin(9600);
  delay(100);
  //Kp=kp,Ki=ki,Kd=kd;
}

void read_ir()
{
  for(int i=0; i<5; i++)
  {
   ir_s[i]=analogRead(ir[i]);
  }
  for(int i=0; i<5;i++)
  {
    Serial.print(ir_s[i]);
    Serial.print(" ");
  }
  Serial.println();

}
void calc_ir()
{
  ir_sum=0;
  tt=0;
  temp=0;
  /*for(int =0; i<6; i++)
  {
  ir_s2[i]=ir_s[i]*ir_s1[i];
  }
  Serial.print("calculated sensor value:");
  for (int i = 0; i < 5; i++)
  {
    Serial.print(ir_s2[i]);
    Serial.print(" ");
  }*/
  /*Serial.println();
  for (int i = 0; i < 5; i++)
  {
    ir_sum+=ir_s2[i];
  }
  Serial.print("ir_sum:");
  Serial.print(ir_sum);
  Serial.println();*/
  for(int i=0; i<5;i++)
  {
    tt+=ir_s[i]*(i+1)*10;
    temp+=ir_s[i]*10;
    Serial.print(tt);
    Serial.print(" ");
  }

  Serial.println();
  Serial.print("tt:");
  Serial.print(tt);
  Serial.println();
  Serial.print("temp:");
  Serial.print(temp);
  Serial.println();
  Serial.print("tt/temp:");
  Serial.print(tt/temp);
  Serial.println();
  
}
void pid_calc()
{
  Kp = 6;//Ki=0.1;//Kd=50;
  set_point =3.07;
  currentpoint = double(tt/temp);
  proportional =set_point-currentpoint;//-set_point;
  integral = integral + proportional;
  derivative = proportional - last_proportional;
  PID_output = double(proportional * Kp + integral * Ki + derivative * Kd);
  last_proportional = proportional;

  Serial.print(" ");
  Serial.print("ir_avg:");
  Serial.print(ir_avg);
  Serial.println(" ");
  /*Serial.print("sensor sum:");Serial.print(ir_sum);*/
  Serial.println(" ");
  Serial.print("setpoint:");
  Serial.print(set_point); 
  Serial.println(" ");
  Serial.print("currentpoint:");
  Serial.print(currentpoint);
  Serial.println(" "); 
  //Serial.print("tt/temp:");
  //Serial.print(tt/temp);
 // Serial.println(" "); 
  Serial.print("proportional:"); Serial.print(proportional); Serial.println(" ");
  //Serial.print("derivative:");Serial.print(derivative);Serial.println(" ");Serial.print("integral:");Serial.print(integral);Serial.println(" ");
  Serial.print("pid output:"); Serial.print(PID_output); Serial.println(" ");
    
}


void calc_turn()
{
  //Restricting the error value between +256.
  if (PID_output < -256)
  {
    PID_output = -256;
  }
  if (PID_output > 256)
  {
    PID_output =   256;
  }
  // If PID_output is less than zero calculate right turn speed values
  if (PID_output < 0)
  {
    right_motor_speed = max_speed + PID_output ;
    left_motor_speed = max_speed;
  }
  // If error_value is greater than zero calculate left turn values
  else
  {
    right_motor_speed = max_speed;
    left_motor_speed = max_speed- PID_output;
  }
}


void motor_drive(int right_motor_speed, int left_motor_speed)
{
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1,HIGH);
  digitalWrite(RM2,LOW);
  analogWrite(E1, right_motor_speed);
  analogWrite(E2, left_motor_speed);
  Serial.println(" ");
  Serial.print("right motor speed:"); 
  Serial.print(right_motor_speed); 
  Serial.println(" ");
  Serial.print("left motor speed:"); 
  Serial.print(left_motor_speed); 
  Serial.println(" ");
}

void loop()
{
  read_ir();
  calc_ir();
 pid_calc();
 calc_turn();
 motor_drive(right_motor_speed,left_motor_speed);
 //motor_drive(right_motor_speed,left_motor_speed+15);

  delay(3000);
}
