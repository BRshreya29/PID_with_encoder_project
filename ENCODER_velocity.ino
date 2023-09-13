// Pin Definitions
#define ENCA_PIN 2
#define ENCB_PIN 3
#define PWM_PIN 5
#define IN1_PIN 6
#define IN2_PIN 7

// Global Variables
long previousTime = 0;
int previousPosition = 0;
volatile int position_i = 0;
volatile float velocity_i = 0;
volatile long previousTime_i = 0;
float filteredVelocity1 = 0;
float previousVelocity1 = 0;
float filteredVelocity2 = 0;
float previousVelocity2 = 0;
float integralError = 0;

void setup() {
  Serial.begin(115200);

  pinMode(ENCA_PIN, INPUT);
  pinMode(ENCB_PIN, INPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA_PIN), readEncoder, RISING);
}

void loop() {
  int position = 0;
  float velocity2 = 0;
  noInterrupts();
  position = position_i;
  velocity2 = velocity_i;
  interrupts();

  long currentTime = micros();
  float deltaTime = ((float) (currentTime - previousTime)) / 1.0e6;
  float velocity1 = (position - previousPosition) / deltaTime;
  previousPosition = position;
  previousTime = currentTime;

  float v1 = velocity1 / 600.0 * 60.0;
  float v2 = velocity2 / 600.0 * 60.0;

  filteredVelocity1 = 0.854 * filteredVelocity1 + 0.0728 * v1 + 0.0728 * previousVelocity1;
  previousVelocity1 = v1;
  filteredVelocity2 = 0.854 * filteredVelocity2 + 0.0728 * v2 + 0.0728 * previousVelocity2;
  previousVelocity2 = v2;

  float targetVelocity = 100 * (sin(currentTime / 1e6) > 0);

  float kp = 5;
  float ki = 10;
  float error = targetVelocity - filteredVelocity1;
  integralError = integralError + error * deltaTime;
  
  float u = kp * error + ki * integralError;

  int direction = 1;
  if (u < 0){
    direction = -1;
  }
  int power = (int) fabs(u);
  if(power > 255){
    power = 255;
  }
  setMotor(direction, power, PWM_PIN, IN1_PIN, IN2_PIN);

  Serial.print(targetVelocity);
  Serial.print(" ");
  Serial.print(filteredVelocity1);
  Serial.println();
  delay(1);
}

void setMotor(int direction, int pwmValue, int pwm, int in1, int in2){
  analogWrite(pwm, pwmValue); 
  if(direction == 1){ 
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if(direction == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);    
  }
}

void readEncoder(){
  int b = digitalRead(ENCB_PIN);
  int increment = (b > 0) ? 1 : -1;
  position_i = position_i + increment;

  long currentTime = micros();
  float deltaTime = ((float) (currentTime - previousTime_i)) / 1.0e6;
  velocity_i = increment / deltaTime;
  previousTime_i = currentTime;
}
