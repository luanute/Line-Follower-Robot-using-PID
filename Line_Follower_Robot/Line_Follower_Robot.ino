#include <Arduino.h> // Thư viện arduino.h
#define CUSTOM_SETTINGS               // Biến cho việc điều khiển bằng app
#define INCLUDE_GAMEPAD_MODULE        // Biến gamepad
#include <DabbleESP32.h>

#include <QTRSensors.h>

int speedSelect[] = {120, 180, 255}; // Mảng lưu 3 tốc độ

#define IN1 21                        // Chân IN1 của L298N
#define IN2 22                        // Chân IN2 của L298N
#define IN3 16                        // Chân IN3 của L298N
#define IN4 17                        // Chân IN4 của L298N
#define ENA 18                        // Chân điều khiển tốc độ động cơ trái (LPWM)
#define ENB 19                        // Chân điều khiển tốc độ động cơ phải (RPWM)
//qtr.setSensorPins((const uint8_t[]){32/*7*/ , 35/*6*/, 34, 13, 12, 14, 27, 26}, SensorCount);

QTRSensors qtr;

#define BUZZER 2    // chân điều khiển loa
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

#define LED 5

int Speed = 150; //max150

float Kp = 2;
float Ki = 0.0;
float Kd = 5; 
int  P;
int  I;
int  D;
int lastError = 0;

uint8_t multiP = 1;
uint8_t multiI  = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;

const uint8_t maxspeeda = 255;
const uint8_t maxspeedb = 255;
const uint8_t basespeeda = Speed;
const uint8_t basespeedb = Speed;

uint16_t position;
int previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 230;

void setup() {
  pinMode(BUZZER, OUTPUT);
  for (int i = 2000; i < 3500; i += 500) {
    tone(BUZZER, i, 100);
    delay(100);
  }
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Serial.begin(115200);
  Dabble.begin("wavextensa");
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){32/*7*/ , 35/*6*/, 34, 13, 12, 14, 27, 26}, SensorCount);
  pinMode(LED, OUTPUT);
  delay(2000);
  digitalWrite(LED, HIGH);
  Serial.println("Calib");
  for(int i = 0; i < 400; i++){
    qtr.calibrate();
    delay(20);
  }
   for (uint8_t i = 0; i < SensorCount; i++)
  {
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i])/2;
    Serial.print(threshold[i]);
    Serial.print("  ");
  }
  Serial.println();

  for(byte i = 0; i < 10; i++){
    digitalWrite(LED, HIGH);
    delay(200);
    digitalWrite(LED, LOW);
    delay(200);
  }
  Serial.println("Calib Done!");
  for (int i = 2000; i < 3500; i += 500) {
    tone(BUZZER, i, 100);
    delay(100);
  }
  delay(1000);
}


void speed_run(int speedDC_left, int speedDC_right) {
  // Điều khiển động cơ trái
  if (speedDC_left > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speedDC_left); // Điều chỉnh tốc độ
  } else if (speedDC_left < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, abs(speedDC_left)); // Điều chỉnh tốc độ
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0); // Dừng động cơ
  }
  
  // Điều khiển động cơ phải
  if (speedDC_right > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speedDC_right); // Điều chỉnh tốc độ
  } else if (speedDC_right < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, abs(speedDC_right)); // Điều chỉnh tốc độ
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0); // Dừng động cơ
  }
}

int frame = 0;
int speedy = 0;
int mode = 0;

void pid() {
  int16_t position = qtr.readLineWhite(sensorValues);
  Serial.println(position);
  int error = 3500 - position;  
  P = error;
  I = I + error;
  D = error - lastError;
  lastError = error;

  int motorspeed = P*Kp + I*Ki + D*Kd;

  int motorspeeda = basespeeda + motorspeed / 2;
  int motorspeedb = basespeedb - motorspeed / 2;
  
  if (motorspeeda > maxspeeda) {
    motorspeeda = maxspeeda;
  }
  if (motorspeedb > maxspeedb) {
    motorspeedb = maxspeedb;
  }
  if (motorspeeda < 0) {
    motorspeeda = -maxspeedb/2;
  }
  if (motorspeedb < 0) {
    motorspeedb = -maxspeeda/2;
  } 
  speed_run(motorspeeda, motorspeedb);
}

void loop() {
  speedy = speedSelect[frame];
  Dabble.processInput();

  // Xử lý nút Select
  if (GamePad.isSelectPressed()) {
    while (GamePad.isSelectPressed()) {
      Dabble.processInput();
    }
    frame++;
    if (frame > 2) {
      frame = 0;
    }
  }

  // Xử lý nút Start
  if (GamePad.isStartPressed()) {
    while (GamePad.isStartPressed()) {
      Dabble.processInput();
    }
    mode++;
    for (int i = 2000; i < 3500; i += 500) {
      tone(BUZZER, i, 100);
      delay(100);
    }
    if (mode > 1) {
      mode = 0;
    }
  }

  // Hành vi dựa trên mode
  if (mode == 1) {
    while (mode == 1) {
      speedy = speedSelect[frame];
      Dabble.processInput();

      if (GamePad.isSelectPressed()) {
        while (GamePad.isSelectPressed()) {
          Dabble.processInput();
        }
        frame++;
        if (frame > 2) {
          frame = 0;
        }
      }

      pid();
      
      if (GamePad.isStartPressed()) {
        while (GamePad.isStartPressed()) {
          Dabble.processInput();
        }
        mode++;
        for (int i = 2000; i < 3500; i += 500) {
          tone(BUZZER, i, 100);
          delay(100);
      }
        if (mode > 1) {
          mode = 0;
        }
        break;
      }
    }
  }

  // Xử lý các nút điều hướng
  Serial.print("KeyPressed: ");
  if (GamePad.isUpPressed()) {
    Serial.print("UP");
    if (GamePad.isLeftPressed()) {
      speed_run(speedy, -speedy); // Quẹo trái
    } else if (GamePad.isRightPressed()) {
      speed_run(-speedy, speedy); // Quẹo phải
    } else if (GamePad.isSquarePressed()) {
      speed_run(speedy, 0); // Tiến bên trái
    } else if (GamePad.isCirclePressed()) {
      speed_run(0, speedy); // Tiến bên phải
    } else {
      speed_run(speedy, speedy); // Tiến thẳng
    }
  } else if (GamePad.isDownPressed()) {
    Serial.print("DOWN");
    speed_run(-speedy, -speedy); // Lùi
    if (GamePad.isLeftPressed()) {
      speed_run(speedy, -speedy); // Quẹo trái khi lùi
    } else if (GamePad.isRightPressed()) {
      speed_run(-speedy, speedy); // Quẹo phải khi lùi
    } else if (GamePad.isSquarePressed()) {
      speed_run(-speedy, 0); // Lùi bên trái
    } else if (GamePad.isCirclePressed()) {
      speed_run(0, -speedy); // Lùi bên phải
    } else {
      speed_run(-speedy, -speedy); // Lùi thẳng
    }
  } else if (GamePad.isLeftPressed()) {
    Serial.print("Left");
    speed_run(speedy, -speedy); // Quẹo trái
  } else if (GamePad.isRightPressed()) {
    Serial.print("Right");
    speed_run(-speedy, speedy); // Quẹo phải
  } else if (GamePad.isSquarePressed()) {
    Serial.print("Square");
    speed_run(0, speedy); // Tiến bên trái
  } else if (GamePad.isCirclePressed()) {
    Serial.print("Circle");
    speed_run(speedy, 0); // Tiến bên phải
  } else if (GamePad.isCrossPressed()) {
    Serial.print("Cross");
  } else if (GamePad.isTrianglePressed()) {
    Serial.print("Triangle");
  } else if (GamePad.isStartPressed()) {
    Serial.print("Start");
  } else if (GamePad.isSelectPressed()) {
    Serial.print("Select");
  } else {
    speed_run(0, 0); // Dừng
  }

  Serial.print('\t');
}

void robot_control(){
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 4000 (for a white line, use readLineWhite() instead)
  position = qtr.readLineWhite(sensorValues);
  error = 2000 - position;
  while(sensorValues[0]>=980 && sensorValues[1]>=980 && sensorValues[2]>=980 && sensorValues[3]>=980 && sensorValues[4]>=980){ // A case when the line follower leaves the line
    if(previousError>0){       //Turn left if the line was to the left before
      speed_run(-230,230);
    }
    else{
      speed_run(230,-230); // Else turn right
    }
    position = qtr.readLineWhite(sensorValues);
  }
  
  PID_Linefollow(error);
  //PID_Linefollow(error);
}
void PID_Linefollow(int error){
    P = error;
    I = I + error;
    D = error - previousError;
    
    Pvalue = (Kp/pow(10,multiP))*P;
    Ivalue = (Ki/pow(10,multiI))*I;
    Dvalue = (Kd/pow(10,multiD))*D; 

    float PIDvalue = Pvalue + Ivalue + Dvalue;
    previousError = error;

    lsp = lfspeed - PIDvalue;
    rsp = lfspeed + PIDvalue;

    if (lsp > 255) {
      lsp = 255;
    }
    if (lsp < -255) {
      lsp = -255;
    }
    if (rsp > 255) {
      rsp = 255;
    }
    if (rsp < -255) {
      rsp = -255;
    }
    speed_run(lsp,rsp);
}

