#define LM1 30
#define LM2 32
#define RM1 34
#define RM2 36 

#define E1 A14  /// right motor
#define E2 A15 // left motor
 
long int ir_sensor[6]   = {A5, A4, A3, A2, A1, A0}; // ir_sensor pin#
long int ir_s[6]; //also can use unsigned int
#define left_turn_pin A5
#define right_turn_pin A0
double ir_weight, ir_sum;
#define max_speed 140
#define TS 130 // turn speed
#define base_speed 150

double ir_setvalue;
double ir_currentvalue;
double last_proportional;
double Kp, Kd, Ki;
double PID_output;

int right_motor_speed;
int left_motor_speed;
double proportional, integral, derivative;
long ir_avg;

void setup()
{
  Serial.begin(115200);
  delay(8);
  pinMode(left_turn_pin, INPUT);
  pinMode(right_turn_pin,INPUT);
  
  for(int i=0; i<6; i++)
  {
    pinMode(ir_sensor[i],INPUT);
  }
  pinMode(LM1,OUTPUT);
  pinMode(LM2,OUTPUT);
  pinMode(RM1,OUTPUT);
  pinMode(RM2,OUTPUT);
  pinMode(E1,OUTPUT);
  pinMode(E2,OUTPUT);
     
  // put your setup code here, to run once:
  
  //Kp=kp,Ki=ki,Kd=kd;
  ir_setvalue=36.42;
  Serial.print("set value:");
  Serial.print(ir_setvalue);
  Serial.println();
}

void read_ir()
{
  for(int i=0; i<6; i++)
  {
    ir_s[i] = analogRead(ir_sensor[i]);
  }
  for(int i=0; i<6; i++)
  {
    Serial.print(ir_s[i]);
    Serial.print(" ");
  }
  Serial.println();

  ir_sum = 0;
  ir_weight = 0;
  
  for(int i=0; i<6; i++)
  {
    ir_weight += ir_s[i] * (i + 1) * 10;
    ir_sum += ir_s[i] * 1;
    Serial.print(ir_weight);
    Serial.print(" ");
  }
  ir_currentvalue=ir_weight/ir_sum;

  Serial.println();
  Serial.print("ir_weight:");
  Serial.print(ir_weight);
  Serial.println();
  Serial.print("ir_sum:");
  Serial.print(ir_sum);
  Serial.println();
  Serial.print("ir_weight/ir_sum:");
  Serial.print(ir_weight / ir_sum);
  Serial.println();
  Serial.print("ir current value:");
  Serial.print(ir_currentvalue);
  Serial.println();

}
void left_turn()
{
  digitalWrite(LM1,LOW);
  digitalWrite(LM2, HIGH);
   digitalWrite(RM1, HIGH);
  digitalWrite(RM2,LOW);
  analogWrite(E1,130);
  analogWrite(E2,130);
  delay(50);
  //analogWrite(E1,130);
  //analogWrite(E2,200);
}
void right_turn()
{
  digitalWrite(LM1,HIGH);
  digitalWrite(LM2, LOW);
   digitalWrite(RM1, LOW);
  digitalWrite(RM2,HIGH);
  analogWrite(E1,130 );
  analogWrite(E2,130 );
  //delay(50);
  
  //analogWrite(E1,140 );
  //analogWrite(E2,130 );
}
void pid_calc(double set_point, double currentpoint)
{
  Kp = 5;//Ki=0.055;//Kd=300;//  
  proportional = currentpoint - set_point;
  integral = integral + proportional;
  derivative = proportional - last_proportional;
  PID_output = int(proportional * Kp + integral * Ki + derivative * Kd);
  last_proportional = proportional;

  /*Serial.print(" ");
  Serial.print("ir_avg:");
  Serial.print(ir_avg);
  Serial.println(" ");
  Serial.println(" ");
  Serial.print("setpoint:");
  Serial.print(set_point);
  Serial.println(" ");
  Serial.print("currentpoint:");
  Serial.print(currentpoint);
  Serial.println(" ");
  //Serial.print("ir_weight/ir_sum:");
  //Serial.print(ir_weight/ir_sum);
  // Serial.println(" ");*/
  Serial.print("proportional:"); Serial.print(proportional); Serial.println(" ");
  //Serial.print("derivative:");Serial.print(derivative);Serial.println(" ");
  //Serial.print("integral:");Serial.print(integral);Serial.println(" ");
  Serial.print("pid .output:"); Serial.print(PID_output); Serial.println(" ");

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
    right_motor_speed = max_speed;//+PID_output;
    left_motor_speed = max_speed + PID_output;
  }
  // If error_value is greater than zero calculate left turn values
  else
  {
    right_motor_speed = max_speed - PID_output;
    left_motor_speed = max_speed; //-PID_output;
  }
}

void motor_drive1(int right_motor_speed, int left_motor_speed)
{
  digitalWrite(LM1,HIGH);
  digitalWrite(LM2, LOW);
   digitalWrite(RM1, HIGH);
  digitalWrite(RM2,LOW);
  analogWrite(E1, right_motor_speed);
  analogWrite(E2, left_motor_speed);
  Serial.println (" ");
  Serial.print("right motor speed:");
  Serial.print(right_motor_speed);
  Serial.println(" ");
  Serial.print("left motor speed:");
  Serial.print(left_motor_speed);
  Serial.println(" ");
}
/*
void motor_drive2(int right_motor_speed, int left_motor_speed)
{
  analogWrite(LM1, left_motor_speed);
  analogWrite(LM2, 120);
analogWrite(RM1,120);
  analogWrite(RM2, SFS);
  //analogWrite(E1, right_motor_speed);
  //analogWrite(E2, left_motor_speed);
  Serial.println (" ");
  Serial.print("right motor speed:");
  Serial.print(right_motor_speed);
  Serial.println(" ");
  Serial.print("left motor speed:");
  Serial.print(left_motor_speed);
  Serial.println(" ");
}
void motor_drive3(int right_motor_speed, int left_motor_speed)
{
  analogWrite(LM1, LOW);
  analogWrite(LM2, SFS);
analogWrite(RM1, right_motor_speed);
  analogWrite(RM2, 120);
  //analogWrite(E1, right_motor_speed);
  //analogWrite(E2, left_motor_speed);
  Serial.println (" ");
  Serial.print("right motor speed:");
  Serial.print(right_motor_speed);
  Serial.println(" ");
  Serial.print("left motor speed:");
  Serial.print(left_motor_speed);
  Serial.println(2" ");
}
*/
void md()
{
   digitalWrite(LM1,LOW);
  digitalWrite(LM2, HIGH);
   digitalWrite(RM1, HIGH);
  digitalWrite(RM2,LOW);
  analogWrite(E2,130);
  delay(150);
  analogWrite(E1,130);
  delay(150);
  //analogWrite(E2,0);
  //delay(8);
  //analogWrite(E1,10);
  //delay(1500);
  //delay(3000);
}

void md1()
{
  digitalWrite(LM1,HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2,HIGH);
  analogWrite(E1,130);
  //delay(150);
  analogWrite(E2,130);
  //delay(8);
  //analogWrite(E2,0);
  //delay(8);
  //analogWrite(E1,10);
  //delay(1500);
  //delay(3000);
}

void loop()
{
  
  Serial.print("ir_set_value:");
  Serial.print(ir_setvalue);
  Serial.println();
  read_ir();
  Serial.println();
  int sensor[6];
  for(int i=0; i<6; i++)
  {
    sensor[i]=digitalRead(ir_sensor[i]);
  }
  for(int i=0; i<6; i++)
  {
    Serial.print(sensor[i]);
    Serial.print(" ");
  }
  Serial.println();
  int rp=digitalRead(right_turn_pin);
  int lp=digitalRead(left_turn_pin);
  Serial.print("left turn pin:");
  Serial.print(lp);
  Serial.println();
  Serial.print("right_turn pin:");
  Serial.print(rp);
  Serial.println();
  

  if( sensor[1]==HIGH && sensor[2]!=HIGH && sensor[3]!=HIGH && sensor[4]==HIGH)// && sensor[5]==HIGH)// || sensor[6]==LOW)
  {
    //Serial.print("1111");
    //Serial.println();
    motor_drive1(0, 0);
   delay(100);
  digitalWrite(LM1,LOW);
  digitalWrite(LM2, HIGH);
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2,LOW);
  
  analogWrite(E2,0);
  analogWrite(E1,140);
  delay(800);
  //analogWrite(E1,130);
  //analogWrite(E2,0);
  //delay(100);
  }
  
else if( sensor[2]==HIGH && sensor[3]!=HIGH && sensor[4]!=HIGH && sensor[5]==HIGH)// && sensor[5]==HIGH)// || sensor[6]==LOW)
  {
    //Serial.print("2222");
    //Serial.println();
    motor_drive1(0, 0); 
   delay(100);
  digitalWrite(LM1,HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2,HIGH);
  analogWrite(E1,130);
  analogWrite(E2,130);
  delay(800);
  analogWrite(E1,0);
  analogWrite(E2,130);
  delay(100);
  }
  
  else if(digitalRead(left_turn_pin)==HIGH && !(digitalRead(right_turn_pin)==HIGH))
  {
    //break;
    //Serial.print("3333");
    //Serial.println();
    motor_drive1(0, 0);
   delay(4);
   left_turn();
   Serial.print("Left_ turn");
   Serial.println();
  }
  
  else if(digitalRead(right_turn_pin)==HIGH && !(digitalRead(left_turn_pin)==HIGH ))
  {
    //Serial.print("4444");
    //Serial.println();
    motor_drive1(0, 0);
   delay(8);
    //delay(4);
  right_turn();
  Serial.print("Right_ turn");
   Serial.println();
  }



  
   else if(digitalRead(left_turn_pin)==HIGH && digitalRead(right_turn_pin)==HIGH)
  {
    //Serial.print("5555");
    //Serial.println();
    motor_drive1(0, 0);
   delay(8);
    
    pid_calc(ir_setvalue,ir_currentvalue);
  calc_turn();
  motor_drive1(right_motor_speed, left_motor_speed);
  }

  else if(sensor[1]==HIGH && sensor[3]==HIGH)
  {
    //Serial.print("6666");
    //Serial.println();
    motor_drive1(0, 0);
   delay(8);
    md();
  }
  else if(sensor[6]==HIGH && sensor[4]==HIGH)
  {
    //Serial.print("7777");
    //Serial.println();
    motor_drive1(0, 0);
   delay(8);
    md1();
  }
  else if(sensor[6]==HIGH && sensor[5]==HIGH && sensor[3]== HIGH)
  {
    //Serial.print("77777");
    //Serial.println();
    motor_drive1(0, 0);
   delay(8);
   digitalWrite(LM1,LOW);
  digitalWrite(LM2, HIGH);
   digitalWrite(RM1, HIGH);
  digitalWrite(RM2,LOW);
  analogWrite(E2,140);
  //delay(150);
  analogWrite(E1,0);
  //delay(150);
  }
  
  
  
  else
  {
    //Serial.print("88888");
    //Serial.println();
  read_ir();
  pid_calc(ir_setvalue,ir_currentvalue);
  calc_turn(); 
motor_drive1(right_motor_speed, left_motor_speed);
// motor_drive1(200,200);
  }
  //read_ir();
  pid_calc(ir_setvalue,ir_currentvalue);
  calc_turn(); 
motor_drive1(right_motor_speed,left_motor_speed);
//motor_drive1(255,255);
//delay(3000);
//delay(8);
}
