// Front Motor Drive Pin 설정

#include <MsTimer2.h>

#define MOTOR1_PWM 2
#define MOTOR1_ENA 3
#define MOTOR1_ENB 4

#define MOTOR3_PWM 5
#define MOTOR3_ENA 6
#define MOTOR3_ENB 7

int f_speed = 0, r_speed = 0;  // 속도

void front_motor_control(int motor1_pwm)
{
  if (motor1_pwm > 0) // forward
  {
    digitalWrite(MOTOR1_ENA, HIGH);   // 방향은 설정에 따라 바꿀 것
    digitalWrite(MOTOR1_ENB, LOW);
    analogWrite(MOTOR1_PWM, motor1_pwm);
  }
  else if (motor1_pwm < 0) // backward
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, HIGH);
    analogWrite(MOTOR1_PWM, -motor1_pwm);
  }
  else
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, LOW);
    analogWrite(MOTOR1_PWM, 0);
  }
}

void rear_motor_control(int motor3_pwm)
{
  if (motor3_pwm > 0) // forward
  {
    digitalWrite(MOTOR3_ENA, HIGH);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, motor3_pwm);
  }
  else if (motor3_pwm < 0) // backward
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, HIGH);
    analogWrite(MOTOR3_PWM, -motor3_pwm);
  }
  else
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, 0);
  }
}

void motor_control(int front_speed, int rear_speed)
{
  front_motor_control(front_speed);
  rear_motor_control(rear_speed);
}

///////////////////// Encoder ///////////////////////////

#include <SPI.h>

#define ENC1_ADD 22
#define ENC2_ADD 23

signed long encoder1count = 0;
signed long encoder2count = 0;

signed long encoder1_error = 0;
signed long encoder2_error = 0;

signed long encoder1_error_d = 0;
signed long encoder2_error_d = 0;

signed long encoder1_target = 348;
signed long encoder2_target = 0;

signed long encoder1_error_old = 0;
signed long encoder2_error_old = 0;
signed long Pos_pid_pwm = 0;

float Pw = 0.4;
float Pw_d = 3.0;

void initEncoders()   //encoder 초기화
{
  // Set slave selects as outputs
  pinMode(ENC1_ADD, OUTPUT);
  pinMode(ENC2_ADD, OUTPUT);

  // Raise select pins
  // Communication begins when you drop the individual select signal
  digitalWrite(ENC1_ADD, HIGH);
  digitalWrite(ENC2_ADD, HIGH);

  SPI.begin();

  // Initialize encoder 1
  // Clock division factor: 0
  // Negative index input
  // free-running count mode
  // x4 quadrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC1_ADD, LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                  // Write to MDR0
  SPI.transfer(0x03);                  // Configure to 4 byte mode
  digitalWrite(ENC1_ADD, HIGH);        // Terminate SPI conversation

  // Initialize encoder 2
  // Clock division factor: 0
  // Negative index input
  // free-running count mode
  // x4 quadrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC2_ADD, LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                  // Write to MDR0
  SPI.transfer(0x03);                  // Configure to 4 byte mode
  digitalWrite(ENC2_ADD, HIGH);        // Terminate SPI conversation
}

long readEncoder(int encoder_no)
{
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;

  digitalWrite(ENC1_ADD + encoder_no - 1, LOW);    // Begin SPI conversation
  SPI.transfer(0x60);                               // Request count
  count_1 = SPI.transfer(0x00);                     // Read highest order byte
  count_2 = SPI.transfer(0x00);
  count_3 = SPI.transfer(0x00);
  count_4 = SPI.transfer(0x00);                     // Read lowest order byte
  digitalWrite(ENC1_ADD + encoder_no - 1, HIGH);   // Terminate SPI conversation

  // Calculate encoder count
  count_value = ((long)count_1 << 24) + ((long)count_2 << 16) + ((long)count_3 << 8) + (long)count_4;

  return count_value;
}

void clearEncoderCount(int encoder_no)
{
  // Set encoder1's data register to 0
  digitalWrite(ENC1_ADD + encoder_no - 1, LOW);    // Begin SPI conversation
  // Write to DTR
  SPI.transfer(0x98);
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC1_ADD + encoder_no - 1, HIGH);   // Terminate SPI conversation

  delayMicroseconds(100);  // provides some breathing room between SPI conversations

  // Set encoder1's current data register to center
  digitalWrite(ENC1_ADD + encoder_no - 1, LOW);    // Begin SPI conversation
  SPI.transfer(0xE0);
  digitalWrite(ENC1_ADD + encoder_no - 1, HIGH);   // Terminate SPI conversation
}


void Pos_pid_control(void)
{
// int Pos_pid_pwm = 0;
 
  Pos_pid_pwm = (Pw * encoder1_error) + (Pw_d * encoder1_error_d) ; 
  Pos_pid_pwm = (Pos_pid_pwm>=255) ? 255 : Pos_pid_pwm;
  Pos_pid_pwm = (Pos_pid_pwm<=-255) ? -255 : Pos_pid_pwm;  
  motor_control(Pos_pid_pwm, Pos_pid_pwm);
}

void control_callback()
{
  encoder1count = readEncoder(1);
  encoder2count = -readEncoder(2);

  encoder1_error = encoder1_target - encoder1count;
  encoder1_error_d = encoder1_error - encoder1_error_old;

  encoder2_error = encoder2_target - encoder2count;
  encoder2_error_d = encoder2_error - encoder2_error_old;

  encoder1_error_old = encoder1_error;
  encoder2_error_old = encoder1_error;
  
  Pos_pid_control();
}

void setup()
{
  Serial.begin(115200);

  // Front Motor Drive Pin Setup
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR1_ENB, OUTPUT);

  // Rear Motor Drive Pin Setup
  pinMode(MOTOR3_PWM, OUTPUT);
  pinMode(MOTOR3_ENA, OUTPUT);  // L298 motor control direction
  pinMode(MOTOR3_ENB, OUTPUT);

  initEncoders();          // initialize encoder
  clearEncoderCount(1);
  clearEncoderCount(2);

  MsTimer2::set(50, control_callback);
  MsTimer2::start();
}

void loop()
{
  Serial.print("readEncoder1,");
  Serial.print(encoder1count);

  Serial.print(",Encoder_error1,");
  Serial.println(encoder1_error);

  
}
