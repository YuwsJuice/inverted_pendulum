#define ENCODER_A1 32
#define ENCODER_B1 33
#define ENCODER_A2 12
#define ENCODER_B2 13
#define PWM_FORWARD 25
#define PWM_REVERSE 26
#define PPR 2400
#define MIN_PWM 1

/*------------------------------------------------------------------*/
// Acknowledgment
// The swing-up control logic is adapted from the work of Jitendra Singh.
// For more details, refer to the documentation provided by Jitendra Singh.
// link: https://github.com/jitendra825/Inverted-Pendulum-Simulink




/*------------------------------------------------------------------*/
// Parameters
const float r = 0.006;           // Radius of motor shaft
const float M = 0.135;           // Mass of cart
const float m = 0.1;             // Mass of pendulum
const float I = 0.0007176;       // Moment of Inertia of pendulum
const float l = 0.2;             // Center of mass of pendulum
const float g = 9.81;            // Gravitational constant
const float b = 0.00007892;      // Viscous damping at pivot
const float c = 0.63;            // Friction coefficient of cart
const float k_swing = 1.2;       // Swing-up gain
const float n = 3;               // Swing-up limit
const float Er = 2 * m * g * l;  // Potential energy of pendulum
/*------------------------------------------------------------------*/


const int potPin = 34;           // ESP32 analog input pin (GPIO 34)
const float scaleFactor = 20.0 / 512.0;  // Scaling factor: (15/512)
const float maxLimit = 20.0;     // Saturation limit
const int potMid = 512;          // Center point of potentiometer reading

const int potPin2 = 35;          // Secondary potentiometer pin
float rawPot = 0;                // Raw potentiometer value
float filterValue = 0;           // Filtered value

const float ENCODER2_CONVERSION = 0.000015707 * 4;

volatile int encoder_value1 = 0, encoder_value2 = 0;
int lastState1 = 0, lastState2 = 0;
int last_encoder_value1 = 0, last_encoder_value2 = 0;
unsigned long last_time = 0;
float velocity1 = 0.0, velocity2 = 0.0;

const float alpha_angle2 = 0.1;  // Filter constant for angle2
const float alpha_position2 = 0.1; // Filter constant for position2
float filtered_angle2 = 0.0;
float filtered_position2 = 0.0;

void setup() {
  setup_encoder_pins();
  delay(10000);
}

void loop() {
  unsigned long current_time = micros();
  static float theta = 0;
  static float theta_dot = 0;
  const float Ts = 0.01;

  //encoder 1
  float angle1 = calculate_angle(encoder_value1);
  float angle2 = fmod(angle1, 2 * PI);
  if (angle2 < 0) angle2 += 2 * PI;
  filtered_angle2 = alpha_angle2 * angle2 + (1.0 - alpha_angle2) * filtered_angle2;

  //encoder 2
  float position2 = calculate_position(encoder_value2);
  filtered_position2 = alpha_position2 * position2 + (1.0 - alpha_position2) * filtered_position2;

  update_velocity(current_time);

  //swing up
  theta += theta_dot * Ts;
  theta_dot += (calculateSwingUpControl(filtered_angle2, velocity1, velocity2, filtered_position2) - m * g * l * sin(filtered_angle2)) * Ts / (I + m * l * l);

  //compute swing-up control
  float u_swing = calculateSwingUpControl(filtered_angle2, velocity1, velocity2, filtered_position2);
  float u_swing_convert = abs(u_swing * (256 / 12) * 2);

  int potValue = analogRead(potPin);
  float scaledValue = (float)(potValue - potMid) * scaleFactor;
  scaledValue = saturate(scaledValue, maxLimit, -maxLimit) / 100.0;

  rawPot = analogRead(potPin2);
  filterValue = rawPot / 1023.0;

  // LQR
  // remove position_target if no manual position control
  //
  float scalar_result = ((-31.6228) * (filtered_position2 + position_target)) + 
                        ((105.8956) * (PI - filtered_angle2)) + 
                        ((-30.9948) * velocity2) + 
                        ((22.9507) * (-velocity1));

  float angle1_degrees = filtered_angle2 * (180.0 / PI);
  int raw_pwm_output = convert_to_pwm(scalar_result);

  // Apply control logic based on pendulum angle and control result
  if (angle1_degrees > 160 && angle1_degrees < 200) {
    if (scalar_result > 0) {
      analogWrite(PWM_FORWARD, 0);
      analogWrite(PWM_REVERSE, constrain(raw_pwm_output, 0, 255));
    } else {
      analogWrite(PWM_FORWARD, constrain(raw_pwm_output, 0, 255));
      analogWrite(PWM_REVERSE, 0);
    }
  } else {
    if (u_swing > 0) {
      analogWrite(PWM_FORWARD, 0);
      analogWrite(PWM_REVERSE, constrain(u_swing_convert, 0, 255));
    } else {
      analogWrite(PWM_FORWARD, constrain(u_swing_convert, 0, 255));
      analogWrite(PWM_REVERSE, 0);
    }
  }
}

/*------------------------------------------------------------------*/

float saturate(float value, float max_val, float min_val) {
  return (value > max_val) ? max_val : (value < min_val ? min_val : value);
}

float calculateSwingUpControl(float angle2, float velocity1, float velocity2, float position2) {
  float E = m * g * l * (1 - cos(angle2)) + 0.5 * (I + m * l * l) * velocity1 * velocity1;

  static bool is_in_position_phase = true;
  static const float target_position = 0.3;

  if (is_in_position_phase) {
    if (abs(position2 - target_position) > 0.01) {
      return (position2 < target_position) ? -1.0 : 1.0;
    } else {
      is_in_position_phase = false;
    }
  }

  float direction = (velocity1 * cos(angle2) > 0) ? 1 : -1;
  float accel = 2 * (E - Er) * direction;
  accel = k_swing * g * saturate(accel, n * g, -n * g);

  float u_swing = (M + m) * accel 
                  - m * l * velocity1 * velocity1 * sin(angle2) 
                  - m * l * cos(angle2) * (b * velocity1 
                  + m * l * accel * cos(angle2) 
                  + m * g * l * sin(angle2)) / (I + m * l * l);

  return u_swing;
}

void setup_encoder_pins() {
  pinMode(PWM_FORWARD, OUTPUT);
  pinMode(PWM_REVERSE, OUTPUT);
  analogWrite(PWM_FORWARD, 0);
  analogWrite(PWM_REVERSE, 0);

  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  pinMode(ENCODER_A2, INPUT_PULLUP);
  pinMode(ENCODER_B2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), encoder_isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), encoder_isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), encoder_isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B2), encoder_isr2, CHANGE);
}

void encoder_isr1() {
  int A = digitalRead(ENCODER_A1);
  int B = digitalRead(ENCODER_B1);
  int encoded = (A << 1) | B;
  int sum = (lastState1 << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder_value1++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder_value1--;

  lastState1 = encoded;
}

void encoder_isr2() {
  int A = digitalRead(ENCODER_A2);
  int B = digitalRead(ENCODER_B2);
  int encoded = (A << 1) | B;
  int sum = (lastState2 << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder_value2++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder_value2--;

  lastState2 = encoded;
}

float calculate_angle(int encoder_value) {
  return (float(encoder_value) * 2.0 * PI) / PPR;
}

float calculate_position(int encoder_value) {
  return encoder_value * ENCODER2_CONVERSION;
}

void update_velocity(unsigned long current_time) {
  float delta_time = (current_time - last_time) / 1000000.0;
  if (delta_time <= 0) return;

  int delta_encoder1 = encoder_value1 - last_encoder_value1;
  int delta_encoder2 = encoder_value2 - last_encoder_value2;

  float delta_angle1 = (float(delta_encoder1) * 2.0 * PI) / PPR;
  float delta_position2 = delta_encoder2 * ENCODER2_CONVERSION;

  velocity1 = (0.9) * velocity1 + (0.1) * (delta_angle1 / delta_time);
  velocity2 = (0.9) * velocity2 + (0.1) * (delta_position2 / delta_time);

  last_time = current_time;
  last_encoder_value1 = encoder_value1;
  last_encoder_value2 = encoder_value2;
}

int convert_to_pwm(float value) {
  return ensure_min_pwm(abs(value) * 64 / 12.0);
}

int ensure_min_pwm(int pwm_value) {
  return pwm_value < MIN_PWM ? MIN_PWM : pwm_value;
}
