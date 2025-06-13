//Kontroller PID
//Kp untuk mencapai Setpoint, Ki untuk mengurangi steady state error, Kd untuk peredaman

float kp = 6.0;
float ki = 4.2;
float kd = 5.5;

float kp2 = 7.0;
float ki2 = 6.0;
float kd2 = 6.5;

//Setup PIN
const int IN1 = 10;
const int IN2 = 11;
const int ENA = 9;

const int IN3 = 7;
const int IN4 = 8;
const int ENB = 6;

const int encoderPinA = 2;
const int encoderPinB = 4;

const int encoderPinA1 = 3;
const int encoderPinB1 = 5;

//pulse 854 ->  360 degree -> 6.28 radian
// 1 degree -> 854/360 -> 2.37 pulse
long int enc = 0;
int value;
float sudut_motor;
float rpm_motor;

//Setpoint Angle and Speed
float setpoint_angle = 0;
float setpoint_speed = 0;

//Data untuk kamera
float posisi_mentah = 0;        // ganti dengan nilai posisi aktual
float posisi_kurang_matang = 90; // ganti dengan nilai posisi aktual
float posisi_matang = 150;

float errorA,errorS, P1, I1, D1, P2, I2, D2, prev_errorA, prev_errorS, pwm_motorangle, pwm_motorspeed;
int pwm;
int PID1; //PID1 untuk Angle, PID2 untuk Kecepatan
int PID2;

float integral = 0;
float pulsesPerRevolution = 900.0;
float wheelCircumference = 10.05;
float sampling_time = 100; //ms
unsigned long time_prev, rpm_prev_time = 0;

// Variabel untuk filter derivative speed
float D2_filtered = 0;
float alpha_speed = 0.2; // Nilai 0.1-0.5 (semakin kecil semakin halus)
float integral_speed = 0;
const float i_max_speed = 30.0; // Batas integral windup

char mode;
char data_serial;
char data_speed;

volatile long anglePulseCount = 0;
volatile long speedPulseCount = 0;

void setup() {
  Serial.begin(9600);
  //Angle
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  //Speed
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  pinMode(encoderPinA1, INPUT_PULLUP);
  pinMode(encoderPinB1, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), sub, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), sub_speed, RISING);

  time_prev = millis();
  Serial.println("Ready");
}

void loop() {
  if(Serial.available() >= 3){
    data_serial = Serial.read();
    if(data_serial == '0') setpoint_angle = 0;
    if(data_serial == '1') setpoint_angle = 90;
    if(data_serial == '2') setpoint_angle = 150;
    if(data_serial == '3') setpoint_angle = 180;
    
    data_speed = Serial.read();
    if(data_speed == 'a') setpoint_speed = 80.0;
    if(data_speed == 'b') setpoint_speed = 90.0;
    if(data_speed == 'c') setpoint_speed = 100.0;
    if(data_speed == 'd') setpoint_speed = 120.0;
    char data = Serial.read();

    if (data == '0') {
      //Gerakan Motor
      setpoint_angle = posisi_mentah;
      Serial.println("Data Diterima: MENTAH");
    }
    else if (data == '1') {
      setpoint_angle = posisi_kurang_matang;
      Serial.println("Data Diterima : KURANG MATANG");
    }
    else if (data == '2'){
      setpoint_angle = posisi_matang;
      Serial.println("Data Diterima : MATANG");
    }
    else {
      while(Serial.available() > 0) Serial.read(); // Bersihkan buffer
      stopMotor();
      Serial.println("Timeout : Motor Berhenti");
      //return;
    }
    while(Serial.available() > 0) Serial.read();
  }

  if(millis()-time_prev >= sampling_time){
    //Sudut
    
    sudut_motor = (float)anglePulseCount * 360.0/854.0;
    while (sudut_motor > 180) sudut_motor -= 360;
    while (sudut_motor < -180) sudut_motor += 360;

    // Angle PID calculation
    errorA = setpoint_angle - sudut_motor;
    if(errorA > 180){setpoint_angle -= 360;}
    else if(errorA < -180){setpoint_angle += 360;}
    errorA = setpoint_angle - sudut_motor;

    // Deadzone - jika error sangat kecil, anggap sudah sampai
    if(abs(errorA) < 10.0) {
      errorA = 0;
      I1 = 0;  // Reset integral term
      PID1 = 0;
      stopMotor();
      time_prev = millis();
    }
    P1 = kp * errorA;
    // Batasi integral term
    I1 = constrain(I1, -150, 150);
    float alpha = 0.3;  // Faktor filter (0-1)
    float derivative = (errorA - prev_errorA);
    D1 = kd * (alpha * derivative + (1 - alpha) * D1);
    prev_errorA = errorA;

    PID1 = P1 + I1 + D1;
    PID1 = constrain(PID1, -255, 255);

    // Kontrol motor
    if(abs(errorA) > 1.0) {  // Hanya gerakkan motor jika error signifikan
      if(PID1 > 0){
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,HIGH);
      } else {
        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,LOW);
      }
      pwm_motorangle = abs(PID1); 
      pwm_motorangle = constrain(pwm_motorangle, 0, 80); // Batasi PWM maksimum
      analogWrite(ENA, pwm_motorangle);
    } else {
      stopMotor();
      while(Serial.available() > 0) Serial.read();
    }
    //KecepatanMotor
    rpm_motor = (float(speedPulseCount) / pulsesPerRevolution) * wheelCircumference * 10.7;
    speedPulseCount = 0;
    errorS = setpoint_speed - rpm_motor;

    if(abs(errorS) < 10.0) {
    errorS = 0;
    I2 = 0;  // Reset integral term
    PID2 = 0;
    stopMotor();
    time_prev = millis();
    }
    unsigned long now = millis();
    float deltaTime = (now - rpm_prev_time) / 1000.0;
    rpm_prev_time = now;

    integral_speed += errorS * deltaTime;
    integral_speed = constrain(integral_speed, -i_max_speed, i_max_speed);
    
    P2 = kp2 * errorS ;
    I2 += ki2 * integral_speed;

    float derivative_speed = (errorS - prev_errorS) / deltaTime;
    D2_filtered = alpha_speed * derivative_speed + (1- alpha_speed) * D2_filtered;
    D2 = kd2 * D2_filtered;
    prev_errorS = errorS;

    PID2 = (int)P2 + (int)I2 + (int)D2;
    if (PID2 > 255) PID2 = 255;
    else if(PID2 < -255) PID2 = -255;

    //Kontrol Speed Motor
    if(abs(errorS) > 1.0) {  // Hanya gerakkan motor jika error signifikan
        if(PID2 > 0){
          digitalWrite(IN3,LOW);
          digitalWrite(IN4,HIGH);
        } else {
          digitalWrite(IN3,HIGH);
          digitalWrite(IN4,LOW);
        }
        pwm_motorspeed = abs(PID2); 
        pwm_motorspeed = constrain(pwm_motorspeed, 0, 80); // Batasi PWM maksimum
        analogWrite(ENB, pwm_motorspeed);
      } else {
        stopMotor();
        while(Serial.available() > 0) Serial.read();
        //return;
      }
    //Menambahkan Deadzone dan Batasi PWMnya
    pwm_motorspeed = abs(PID2);
    if(pwm_motorspeed < 25) pwm_motorspeed = 0; 
    if(pwm_motorspeed > 80) pwm_motorspeed = 80;
    analogWrite(ENB, pwm_motorspeed);

    // Print Angle Data
    Serial.print("Angle Setpoint: ");
    Serial.print(setpoint_angle);
    Serial.print("\tPWM Angle: ");
    Serial.print(pwm_motorangle);
    Serial.print("\tSudut Motor: ");
    Serial.println(sudut_motor);

    // Print Speed Data
    Serial.print("Speed Setpoint: ");
    Serial.print(setpoint_speed);
    Serial.print("\tPWM Speed: ");
    Serial.print(pwm_motorspeed);
    Serial.print("\tRPM Motor: ");
    Serial.println(rpm_motor);

    Serial.println(); // Adds a blank line for better separation in the Serial Monitor
    delay(1000);      // Add a delay to make the output readable

    time_prev = millis();

  }
}


void sub(){ 
  value = digitalRead(4);
  if(value==1){ anglePulseCount++; }
  else { anglePulseCount--; }
}

void sub_speed(){ 
  value = digitalRead(5);
  if(value==1){ speedPulseCount++; }
  else { speedPulseCount--; }
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  while(Serial.available() >= 3) Serial.read();
}