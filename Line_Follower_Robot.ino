// IR Sensors
int sensor1 = A0;      // Left most sensor
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3; 
int sensor5 = A4;    


// Initial Values of Sensors
int sensor[5] = {0,0, 0, 0, 0};

// Motor Variables
int ENA = 11;
int IN1 = 7;
int IN2 = 6;
int IN3 = 5;
int IN4 = 4;
int ENB = 10;



// Initial Speed of Motor
int initial_motor_speed = 150; // Reduced speed for slower movement

// PID Constants
float Kp = 12;  // Adjusted PID values for smoother operation
float Ki = 90;
float Kd = 3.5;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

void setup() {
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(9600);                     // Setting serial monitor at a default baud rate of 9600
  }

void loop() {
  readSensor();
  Serial.println(error);

  if(error == 222|| error == 999) BotStop();
  else {
    PIDControl();}

}

void readSensor(){

  sensor[0] = digitalRead(sensor1);
  sensor[1] = digitalRead(sensor2);
  sensor[2] = digitalRead(sensor3);
  sensor[3] = digitalRead(sensor4);
  sensor[4] = digitalRead(sensor5);

// full đen
   if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) error = 222;

// full trắng
   else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)) error = 999;

// xe không lệch
   else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0)) error = 0;

// lệch trái ít
   else if ((sensor[0] == 0) &&(sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1)&& (sensor[4] == 0)) error = 5;
   else if ((sensor[0] == 0) &&(sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1)&& (sensor[4] == 0)) error = 8;


// lệch trái nhiều
   else if ((sensor[0] == 0) &&(sensor[1] == 0)&&(sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))  error = 15;
   else if ((sensor[0] == 0) &&(sensor[1] == 0)&&(sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)) error = 20;

// lệch phải ít
   else if ((sensor[0] == 0) &&(sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0)&& (sensor[4] == 0)) error = -5;
   else if ((sensor[0] == 0) &&(sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0)&& (sensor[4] == 0)) error = -8;


// lệch trái nhiều
   else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0)&& (sensor[4] == 0))  error = -15;
   else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0)&& (sensor[4] == 0)) error = -20;


}
void PIDControl(){


  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;

  int left_motor_speed = initial_motor_speed - PID_value;
  int right_motor_speed = initial_motor_speed + PID_value;

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  analogWrite(ENB, left_motor_speed); //Left Motor Speed
  analogWrite(ENA, right_motor_speed); //Right Motor Speed
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void BotStop(){

     analogWrite(ENA,0);
     analogWrite(ENB,0);
     digitalWrite(IN1, LOW);
     digitalWrite(IN2, LOW);
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, LOW);
}
