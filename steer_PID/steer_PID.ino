#define alpha 0.37
#define max_val 1100//880
#define min_val 70//80
#include <MsTimer2.h>
#define MOTOR3_PWM 8
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10

float old_avg = 0.0; // Xavg(k-1)
float avg     = 0.0; // Xvag(k)
int ad_value = 0;
//int Motor_Pwm = 30; // 
int Steering_Angle = 180;
float Kp = 1.5;
float Ki = 0.1;
float Kd = 3.0; //PID 상수 설정, 실험에 따라 정해야 함 중요!
double error, error_old;
double error_s, error_d;
int pwm_output = 0;
////////////////
#define No_Calibration_Point 11
#define VALUE -2

struct 
{
  double X[No_Calibration_Point];
  double Y[No_Calibration_Point];
}cal_data;

int i;
double y;
////////////////

void Timer_ISR(void){
  ad_value = analogRead(A15); // Pin 지정 540
 // ad_value = 160;
  avg = alpha * old_avg + (1.0 - alpha)*ad_value;
  linear_mapping(avg);
  old_avg = avg;
  PID_Control();
  Protect_steer();
 // ad_value = ad_value + 1;
  
}
void Protect_steer(){
  if(max_val < avg) pwm_output = 0;
  else if(min_val > avg) pwm_output = 0;
  steer_motor_control(pwm_output);
}
void PID_Control()
{
  error = Steering_Angle - avg ;
  error_s += error;
  error_d = error - error_old;
  error_s = (error_s >=  100) ?  100 : error_s;
  error_s = (error_s <= -100) ? -100 : error_s;

  pwm_output = Kp * error + Kd * error_d + Ki * error_s;
  pwm_output = (pwm_output >=  255) ?  255 : pwm_output;
  pwm_output = (pwm_output <= -255) ? -255 : pwm_output;

  if (fabs(error) <= 1.2)
  {
    steer_motor_control(0);
    error_s = 0;
  }
  else          steer_motor_control(pwm_output);
  error_old = error;  
}
void steer_motor_control(int motor_pwm)
{
  if( (avg>= max_val  ) || (avg <= min_val  ) )
  {

    // Serial.println("HH");
     digitalWrite(MOTOR3_ENA, LOW);
     digitalWrite(MOTOR3_ENB, LOW);
     analogWrite(MOTOR3_PWM, 0);
     return;    
  }
  
  if (motor_pwm > 0) // forward
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, HIGH);
    analogWrite(MOTOR3_PWM, motor_pwm);
  }
  else if (motor_pwm < 0) // backward
  {
    digitalWrite(MOTOR3_ENA, HIGH);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, -motor_pwm);
  }
  else // stop
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, 0);
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  MsTimer2::set(10,Timer_ISR);
  MsTimer2::start();
   //Steer
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR3_ENB, OUTPUT);  // L298 motor control PWM
  
  cal_data = {
    {100.0, 180.0, 260.0, 340.0, 420.0, 500.0, 580.0, 660.0, 740.0, 820.0, 900.0},
    {-14.9, -11.3 ,-7.9, -5.4, -2.4, 1.3, 4.3, 7.9, 11.5, 15.0, 18.4} // 15, 18.7
  };
  
}

double linear_mapping(double x)
{
  double x1 = cal_data.X[i];
  double x2 = cal_data.X[i + 1];
  double y1 = cal_data.Y[i];
  double y2 = cal_data.Y[i + 1];

  for (int j = 0; j < No_Calibration_Point-1; j++) {
    if (x < cal_data.X[0])    i = 0;
    else if (x >= cal_data.X[j] && x < cal_data.X[j + 1]) i = j;
    else i = No_Calibration_Point-2;
  }
  return y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1));
}


void loop() {
  // put your main code here, to run repeatedly:
  

  /*avg = alpha * old_avg + (1.0 - alpha)*ad_value;
  old_avg = avg;
*/
  Serial.print("avg = ");Serial.print(avg);Serial.print(" ");
  Serial.print("pwm_output = ");Serial.println(pwm_output);
  Serial.print("Angle = "); Serial.println(y);
}

/*void steering_control( avg ){

 

  y = linear_mapping(VALUE);

  Serial.print("Angle = ");
  Serial.println(y);
  //printf("%lf \n", y);

  return y;
}*/
