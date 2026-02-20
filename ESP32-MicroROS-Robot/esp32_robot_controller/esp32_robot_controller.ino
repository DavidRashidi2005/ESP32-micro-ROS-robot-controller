#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>

#include <ESP32Servo.h>
#include <stdio.h>
#include <string.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/string.h>

// ==================== micro-ROS Objects ====================
rcl_subscription_t LEDs_subscriber;
std_msgs__msg__Int8 LEDs_msg;

rcl_subscription_t servo_subscriber;
std_msgs__msg__Int8 servo_msg;

rcl_subscription_t motor_subscriber;
std_msgs__msg__String motor_msg;

rcl_publisher_t ultrasonic_publisher;
std_msgs__msg__Int16 ultrasonic_msg;

rcl_timer_t ultrasonic_timer;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ==================== Hardware Pins ====================
#define BUZZER_PIN 14
#define Right_LED 32
#define Left_LED 33
#define Front_LED 4

Servo myservo;
int servoPin = 25;

#define ENA 21
#define IN1 19
#define IN2 18
#define ENCODER_A_PIN 22
#define ENCODER_B_PIN 23

#define TRIG_PIN 27
#define ECHO_PIN 26

#define PWM_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define TICKS_PER_REV 1802
#define MIN_PWM 140

// ==================== Position Config ====================
int position_tolerance = 30;
int decel_zone = 300;

// ==================== Ultrasonic ====================
volatile unsigned long echo_start = 0;
volatile unsigned long echo_end = 0;
volatile bool echo_done = false;
bool trig_sent = false;
unsigned long last_trig_time = 0;
int16_t last_distance_cm = -1;

float calibration_factor = 1.0;
float speed_of_sound = 0.0343;

float cm_per_degree = 0.015288;  // 60cm / 3600deg

void IRAM_ATTR echoISR() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    echo_start = micros();
  } else {
    echo_end = micros();
    echo_done = true;
  }
}

void triggerUltrasonic() {
  echo_done = false;
  trig_sent = true;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  last_trig_time = millis();
}

void processUltrasonic() {
  if (!trig_sent) return;

  if (echo_done) {
    unsigned long duration = echo_end - echo_start;

    if (duration > 0 && duration < 25000) {
      float raw = (duration * speed_of_sound) / 2.0;
      raw = raw * calibration_factor;

      if (raw < 2.0) raw = 2.0;
      if (raw > 400.0) raw = 400.0;

      last_distance_cm = (int16_t)raw;
    } else {
      last_distance_cm = -1;
    }
    trig_sent = false;
  }
  else if (millis() - last_trig_time > 30) {
    last_distance_cm = -1;
    trig_sent = false;
  }
}

// ==================== Encoder ====================
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile long encoder_ticks = 0;
volatile int last_A = 0;
volatile int last_B = 0;

long getEncoderTicks() {
  long val;
  portENTER_CRITICAL(&mux);
  val = encoder_ticks;
  portEXIT_CRITICAL(&mux);
  return val;
}

void resetEncoderTicks() {
  portENTER_CRITICAL(&mux);
  encoder_ticks = 0;
  portEXIT_CRITICAL(&mux);
}

void IRAM_ATTR encoderISR_A() {
  portENTER_CRITICAL_ISR(&mux);
  int A = digitalRead(ENCODER_A_PIN);
  int B = digitalRead(ENCODER_B_PIN);
  if (A != last_A) {
    if (A == B) encoder_ticks--;
    else encoder_ticks++;
    last_A = A;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderISR_B() {
  portENTER_CRITICAL_ISR(&mux);
  int A = digitalRead(ENCODER_A_PIN);
  int B = digitalRead(ENCODER_B_PIN);
  if (B != last_B) {
    if (A == B) encoder_ticks++;
    else encoder_ticks--;
    last_B = B;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

// ==================== Motor Variables ====================
long prev_ticks = 0;
unsigned long prev_speed_time = 0;
float current_rpm = 0.0;

int control_mode = 0;

float Kp = 0.7;
float Ki = 4.0;
float Kd = 0.0;
float speed_target = 0.0;
float speed_integral = 0.0;
float speed_prev_error = 0.0;
int speed_output = 0;

long position_target = 0;
long intended_position = 0;
float move_speed = 0.0;
bool move_done = false;

unsigned long pid_last_time = 0;
#define PID_INTERVAL 20

// ==================== Connection State ====================
enum ConnectionState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

ConnectionState state = WAITING_AGENT;

// ==================== String Buffer ====================
#define MOTOR_MSG_BUF_SIZE 64
char motor_msg_buffer[MOTOR_MSG_BUF_SIZE];

// ==================== Error Handling ====================
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ==================== Motor Functions ====================
void setMotorPWM(int pwm) {
  if (pwm > 255) pwm = 255;
  if (pwm < -255) pwm = -255;

  if (pwm == 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM_CHANNEL, 0);
  }
  else if (pwm > 0) {
    int actual = map(pwm, 1, 255, MIN_PWM, 255);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(PWM_CHANNEL, actual);
  }
  else {
    int actual = map(-pwm, 1, 255, MIN_PWM, 255);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(PWM_CHANNEL, actual);
  }
}

void brakeMotor() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  ledcWrite(PWM_CHANNEL, 255);
  delay(50);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(PWM_CHANNEL, 0);
}

void stopAll() {
  control_mode = 0;
  speed_target = 0;
  speed_integral = 0;
  speed_prev_error = 0;
  speed_output = 0;
  move_done = false;
  move_speed = 0;
  setMotorPWM(0);
}

void measureSpeed() {
  unsigned long now = millis();
  if (now - prev_speed_time >= 50) {
    long current = getEncoderTicks();
    long delta = current - prev_ticks;
    float dt = (now - prev_speed_time) / 1000.0;
    current_rpm = (delta / (float)TICKS_PER_REV) / (dt / 60.0);
    prev_ticks = current;
    prev_speed_time = now;
  }
}

void updateSpeedPID() {
  float error = speed_target - current_rpm;

  speed_integral += error * (PID_INTERVAL / 1000.0);
  if (speed_integral > 200) speed_integral = 200;
  if (speed_integral < -200) speed_integral = -200;

  float derivative = (error - speed_prev_error) / (PID_INTERVAL / 1000.0);
  speed_prev_error = error;

  float output = Kp * error + Ki * speed_integral + Kd * derivative;

  if (output > 255) output = 255;
  if (output < -255) output = -255;

  speed_output = (int)output;
  setMotorPWM(speed_output);
}

void updateMoveAtSpeed() {
  long current = getEncoderTicks();
  long error = position_target - current;
  long abs_error = abs(error);

  if (abs_error <= position_tolerance) {
    brakeMotor();
    speed_integral = 0;
    speed_prev_error = 0;
    speed_output = 0;
    speed_target = 0;
    control_mode = 0;
    move_done = true;
    return;
  }

  move_done = false;

  float target_speed;
  if (abs_error < decel_zone) {
    float factor = (float)abs_error / (float)decel_zone;
    if (factor < 0.25) factor = 0.25;
    target_speed = move_speed * factor;
  } else {
    target_speed = move_speed;
  }

  speed_target = (error > 0) ? abs(target_speed) : -abs(target_speed);
  updateSpeedPID();
}

// ==================== Motor API ====================
void setSpeed(float rpm) {
  stopAll();
  if (rpm != 0) {
    control_mode = 1;
    speed_target = rpm;
    prev_ticks = getEncoderTicks();
    prev_speed_time = millis();
    current_rpm = 0;
  }
}

void moveDistance(float degrees, float rpm) {
  stopAll();
  control_mode = 2;
  intended_position += (long)(degrees / 360.0 * TICKS_PER_REV);
  position_target = intended_position;
  move_speed = rpm;
  move_done = false;
  prev_ticks = getEncoderTicks();
  prev_speed_time = millis();
  current_rpm = 0;
}

void stopMotor() {
  stopAll();
}

void resetMotor() {
  stopAll();
  resetEncoderTicks();
  prev_ticks = 0;
  current_rpm = 0;
  intended_position = 0;
  position_target = 0;
}

// ==================== Command Parser ====================
void processMotorCommand(const char* input) {
  if (input == NULL || strlen(input) == 0) return;

  char cmd = input[0];
  float arg1 = 0, arg2 = 0;

  const char* p1 = strchr(input, ' ');
  if (p1 != NULL) {
    arg1 = atof(p1 + 1);
    const char* p2 = strchr(p1 + 1, ' ');
    if (p2 != NULL) {
      arg2 = atof(p2 + 1);
    }
  }

  switch (cmd) {
    case 's':
      // Speed: s 60
      setSpeed(arg1);
      break;

    case 'm':
      // Move degrees: m 360 50
      moveDistance(arg1, (arg2 != 0) ? arg2 : 60);
      break;

    case 'c':
      // Move centimeters: c 50 60
      {
        float cm = arg1;
        float spd = (arg2 != 0) ? arg2 : 60;
        float degrees = cm / cm_per_degree;
        moveDistance(degrees, spd);
      }
      break;

    case 'x':
      // Stop
      stopMotor();
      break;

    case 'r':
      // Reset
      resetMotor();
      break;

    default:
      break;
  }
}
// ==================== micro-ROS Callbacks ====================
void LEDs_subscription_callback(const void* msgin) {
  const std_msgs__msg__Int8* msg = (const std_msgs__msg__Int8*)msgin;
  int8_t value = msg->data;

  switch (value) {
    case 0:
      digitalWrite(Left_LED, LOW);
      digitalWrite(Right_LED, LOW);
      break;
    case 1:
      digitalWrite(Left_LED, HIGH);
      digitalWrite(Right_LED, LOW);
      break;
    case 2:
      digitalWrite(Left_LED, LOW);
      digitalWrite(Right_LED, HIGH);
      break;
    case 3:
      digitalWrite(Left_LED, HIGH);
      digitalWrite(Right_LED, HIGH);
      break;
    case 4:
      digitalWrite(Front_LED, HIGH);
      break;
    case 5:
      digitalWrite(Front_LED, LOW);
      break;
    default:
      break;
  }
}

void servo_callback(const void* msgin) {
  const std_msgs__msg__Int8* msg = (const std_msgs__msg__Int8*)msgin;
  int8_t angle_input = msg->data;
  int8_t constrained_input = constrain(angle_input, -6, 6);
  int servo_offset = constrained_input * 10;
  int servo_position = 90 + servo_offset;
  myservo.write(servo_position);
}

void motor_callback(const void* msgin) {
  const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msgin;
  processMotorCommand(msg->data.data);
}

void ultrasonic_timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    ultrasonic_msg.data = last_distance_cm;
    RCSOFTCHECK(rcl_publish(&ultrasonic_publisher, &ultrasonic_msg, NULL));
  }
}

// ==================== micro-ROS Entity Management ====================
bool createEntities() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(
    &node,
    "ESP32_Robot_Node",
    "",
    &support));

  RCCHECK(rclc_subscription_init_best_effort(
    &LEDs_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "LEDs"));

  RCCHECK(rclc_subscription_init_best_effort(
    &servo_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
    "/servo"));

  RCCHECK(rclc_subscription_init_best_effort(
    &motor_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/motor_cmd"));

  RCCHECK(rclc_publisher_init_best_effort(
    &ultrasonic_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "/ultrasonic"));

  RCCHECK(rclc_timer_init_default(
    &ultrasonic_timer,
    &support,
    RCL_MS_TO_NS(100),
    ultrasonic_timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_set_timeout(&executor, 0));

  RCCHECK(rclc_executor_add_subscription(&executor, &LEDs_subscriber, &LEDs_msg, &LEDs_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &servo_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_subscriber, &motor_msg, &motor_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &ultrasonic_timer));

  return true;
}

void destroyEntities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&LEDs_subscriber, &node);
  rcl_subscription_fini(&servo_subscriber, &node);
  rcl_subscription_fini(&motor_subscriber, &node);
  rcl_publisher_fini(&ultrasonic_publisher, &node);
  rcl_timer_fini(&ultrasonic_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ==================== Ultrasonic Trigger Timer ====================
unsigned long last_ultrasonic_trigger = 0;
#define ULTRASONIC_INTERVAL 100

// ==================== Setup ====================
void setup() {
  set_microros_transports();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(Right_LED, OUTPUT);
  pinMode(Left_LED, OUTPUT);
  pinMode(Front_LED, OUTPUT);

  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 1000, 2000);
  myservo.write(90);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL);
  setMotorPWM(0);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  last_A = digitalRead(ENCODER_A_PIN);
  last_B = digitalRead(ENCODER_B_PIN);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderISR_B, CHANGE);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);

  prev_speed_time = millis();
  pid_last_time = millis();

  motor_msg.data.data = motor_msg_buffer;
  motor_msg.data.size = 0;
  motor_msg.data.capacity = MOTOR_MSG_BUF_SIZE;

  state = WAITING_AGENT;
}

// ==================== Loop ====================
void loop() {
  unsigned long now = millis();

  switch (state) {
    case WAITING_AGENT:
      if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
        state = AGENT_AVAILABLE;
      }
      break;

    case AGENT_AVAILABLE:
      if (createEntities()) {
        state = AGENT_CONNECTED;
      } else {
        state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      if (rclc_executor_spin_some(&executor, 0) != RCL_RET_OK) {
        state = AGENT_DISCONNECTED;
      } else {
        static unsigned long lastPingTime = 0;
        if (now - lastPingTime >= 1000) {
          lastPingTime = now;
          if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) {
            state = AGENT_DISCONNECTED;
          }
        }
      }
      break;

    case AGENT_DISCONNECTED:
      destroyEntities();
      state = WAITING_AGENT;
      stopMotor();
      myservo.write(90);
      break;
  }

  // Ultrasonic trigger
  if (now - last_ultrasonic_trigger >= ULTRASONIC_INTERVAL) {
    if (!trig_sent) {
      triggerUltrasonic();
      last_ultrasonic_trigger = now;
    }
  }

  // Ultrasonic process
  processUltrasonic();

  // Motor speed
  measureSpeed();

  // Motor PID
  if (now - pid_last_time >= PID_INTERVAL) {
    pid_last_time = now;

    if (control_mode == 1) {
      updateSpeedPID();
    }
    else if (control_mode == 2) {
      updateMoveAtSpeed();
    }
  }

  delayMicroseconds(100);
}
