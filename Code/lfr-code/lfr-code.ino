#include <SparkFun_TB6612.h>

// ---------------- Motor pin definitions ----------------
#define AIN1 4
#define BIN1 7
#define AIN2 2
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

// Offset to align motor rotation direction to "forward"
const int offsetA = -1;
const int offsetB = 1;

// Initialize motors
Motor motorL = Motor(AIN1, AIN2, PWMA, offsetA, STBY); // left motor
Motor motorR = Motor(BIN1, BIN2, PWMB, offsetB, STBY); // right motor

// ---------------- Start/Stop Button ----------------
#define BUTTON_PIN 12
bool running = false;
bool lastButtonState = HIGH;

// ---------------- Sensor Pin Definitions ----------------
// Leftmost sensor = A7, rightmost = A0
//const int sensorPins[8] = {A7, A6, A5, A4, A3, A2, A1, A0};
const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// ---------------- PID Variables ----------------
long sensors[8] = {0};
long sensors_average = 0;
int sensors_sum = 0;
int position = 0;
int proportional = 0;
long integral = 0;
int derivative = 0;
int last_proportional = 0;
int motor_speed = 0;
int last_error = 0;

//---------------Motor Variables-----------
int max_speed = 120;
int base_speed = 80;
int right_speed;
int left_speed;
int min_rev_speed = -50;

float Kp = 0.09;
float Ki = 0.0;
float Kd = 0.15;

void setup()
{
  Serial.begin(9600);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop()
{
  handleButton();

  if (!running)
  {
    motorL.brake();
    motorR.brake();
    return;
  }

  read_sensors();
  get_average();
  get_sum();
  if (sensors_sum == 0)
{
  // Set position to center
  position = 3500; // mid point of your range
  // Reset PID control terms
  integral = 0;
  last_error = 0;
  // Optionally, keep motors running at base speed or stop
  // You could also set motor speeds to base_speed here
}
else
{
  get_position();
}
  //get_position();
  get_PID();
  process_control();
  set_motors();
  //printing the postion and motor speed
  Serial.print("Position : ");
  Serial.println(position);
  Serial.print("Motor Speed : ");
  Serial.println(motor_speed);
  Serial.print("Right Speed : ");
  Serial.println(right_speed);
  Serial.print("Left Speed : ");
  Serial.println(left_speed);
  //delay(1000);
}

// ------------------- Button Handler -------------------
void handleButton()
{
  bool currentState = digitalRead(BUTTON_PIN);
  if (currentState == LOW && lastButtonState == HIGH)
  {
    running = !running;
    delay(300); // debounce
  }
  lastButtonState = currentState;
}

// ------------------- Sensor Reading -------------------
void read_sensors()
{
  for (int i = 0; i < 8; i++)
  {
    sensors[i] = analogRead(sensorPins[i]);
    if (sensors[i] < 40) sensors[i] = 0; // noise filter
  }
}

// ------------------- Weighted Average -------------------
void get_average()
{
  sensors_average = 0;
  for (int i = 0; i < 8; i++)
  {
    sensors_average += sensors[i] * (i * 1000);
  }
}

// ------------------- Sum of Sensor Values -------------------
void get_sum()
{
  sensors_sum = 0;
  for (int i = 0; i < 8; i++)
  {
    sensors_sum += sensors[i];
  }
}

// ------------------- Position -------------------
void get_position()
{
  /*if (sensors_sum == 0)
  {
    // No line detected — reset PID terms and stop briefly
    integral = 0;
    derivative = 0;
    motorR.drive(0);
    motorL.drive(0);
    delay(50);
    return;
  }*/
  position = sensors_average / sensors_sum;
}

// ------------------- PID Calculation -------------------
void get_PID()
{
  int error = 3500 - position;  // center of 8 sensors (0–7000)
  proportional = error; 
  integral += error;
  derivative = error - last_error;
  last_error = error;
  //Calculating the motor speed
  motor_speed = int(proportional * Kp + integral * Ki + derivative * Kd);
}

// ------------------- Control Processing -------------------
void process_control()
{
  right_speed = base_speed + motor_speed;
  left_speed  = base_speed - motor_speed;

  if (right_speed > max_speed){
    right_speed = max_speed;
  }
  if (left_speed > max_speed){
    left_speed = max_speed;
  }
  if (right_speed < min_rev_speed){
    right_speed = min_rev_speed;
  }
  if (left_speed < min_rev_speed){
    left_speed = min_rev_speed;
  }
}

// ------------------- Set Motors -------------------
void set_motors()
{
  //right_speed = constrain(right_speed, -100, 100);
  //left_speed  = constrain(left_speed, -100, 100);

  motorR.drive(right_speed);
  motorL.drive(left_speed);
}