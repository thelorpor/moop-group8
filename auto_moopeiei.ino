//ฮาร์ดโค้ดตอนเลี้ยวกันชนT_T
#include <Wire.h>
struct Step {
  char type;
  int  value;
};

Step routeA1[] = {
  {'F', 2}, {'R', 0}, {'F', 1}, {'R', 0}, {'F', 1}, {'L', 0}, {'F', 1}, {'L', 0},
  {'F', 1}, {'R', 0}, {'F', 1}, {'R', 0}, {'F', 1}, {'L', 0}, {'F', 3}
};
const int ROUTE_A1_LEN = sizeof(routeA1) / sizeof(routeA1[0]);

// Route 2: enter parking zone from the top (over the black tape)
Step routeA2[] = {
  {'F', 2}, {'R', 0}, {'F', 1}, {'R', 0}, {'F', 1}, {'L', 0}, {'F', 1}, {'L', 0},
  {'F', 1}, {'R', 0}, {'F', 1}, {'R', 0}, {'F', 1}, {'L', 0}, {'F', 2},
  {'L', 0}, {'F', 2}, {'R', 0}, {'F', 1}, {'R', 0}, {'F', 2}
};
const int ROUTE_A2_LEN = sizeof(routeA2) / sizeof(routeA2[0]);

// ---------------- settings ----------------
const int  ROUTE_CHOICE = 1;       
const bool MIRROR = false;         
const unsigned long START_DELAY_MS = 3000; 
const unsigned long AUTO_LIMIT_MS  = 88000;  

int BASE_SPEED = 150;               
int TURN_SPEED = 150;               
int TURN_SLOW  = 110;              
const unsigned long MS_PER_CELL = 700;   
const int   CELL_CM        = 20;
const int   WALL_STOP_CM   = 6;     
const int   ARRIVE_TOL     = 1;    
const int   SIDE_WALL_CM   = 14;    
const int   SIDE_TARGET_CM = 4;     
const float KP      = 3.0;          
const float KP_SIDE = 1.5;          

// ---------------- pins ----------------
const int ENA = 9, IN1 = 8, IN2 = 7;     
const int ENB = 3, IN3 = 5, IN4 = 4;     
const int TRIG_F = 2,  ECHO_F = 6;       
const int TRIG_L = 11, ECHO_L = 12;      
const int TRIG_R = A0, ECHO_R = A1;      
const int IR_PIN = 10;                   

// ---------------- MPU6050 ----------------
const int MPU_ADDR = 0x68;
float gyroZoffset = 0;
float yaw = 0;                  
float targetHeading = 0;
unsigned long lastMicros = 0;
unsigned long matchStart = 0;

void setMotor(int en, int a, int b, int spd) {
  spd = constrain(spd, -255, 255);
  if (spd > 0)      { digitalWrite(a, HIGH); digitalWrite(b, LOW); }
  else if (spd < 0) { digitalWrite(a, LOW);  digitalWrite(b, HIGH); }
  else              { digitalWrite(a, LOW);  digitalWrite(b, LOW); }
  analogWrite(en, abs(spd));
}

void drive(int l, int r) {
  setMotor(ENB, IN3, IN4, l);
  setMotor(ENA, IN1, IN2, r);
}

void stopMotors() { drive(0, 0); }

//  อย่าเกินเวลาเด้อออ 
void checkTimeUp() {
  if (millis() - matchStart > AUTO_LIMIT_MS) {
    stopMotors();
    while (true) { }
  }
}

long readCm(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long d = pulseIn(echo, HIGH, 15000UL);
  if (d == 0) return 999;
  return d / 58;
}

long readCmMedian(int trig, int echo) {
  long v[3];
  for (int i = 0; i < 3; i++) {
    v[i] = readCm(trig, echo);
    delay(5);
  }
  for (int a = 0; a < 2; a++)
    for (int b = 0; b < 2 - a; b++)
      if (v[b] > v[b + 1]) { long t = v[b]; v[b] = v[b + 1]; v[b + 1] = t; }
  return v[1];
}

int16_t readGyroZraw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2);
  int16_t hi = Wire.read();
  int16_t lo = Wire.read();
  return (hi << 8) | lo;
}

void calibrateGyro() {
  long sum = 0;
  const int N = 500;
  for (int i = 0; i < N; i++) { sum += readGyroZraw(); delay(3); }
  gyroZoffset = (float)sum / N;
}

void updateYaw() {
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;
  float rate = (readGyroZraw() - gyroZoffset) / 131.0;
  if (abs(rate) > 0.3) yaw += rate * dt;
}

void turn90(bool left) {
  targetHeading += left ? 90 : -90;
  stopMotors();
  delay(100);
  lastMicros = micros();
  unsigned long t0 = millis();
  while (millis() - t0 < 2500) {
    checkTimeUp();
    updateYaw();
    float err = targetHeading - yaw;
    if (abs(err) < 2) break;
    int sp = (abs(err) < 25) ? TURN_SLOW : TURN_SPEED;
    if (err > 0) drive(-sp, sp);
    else         drive(sp, -sp);
  }
  stopMotors();
  delay(150);
}

void forwardCells(int n) {
  long d0 = readCmMedian(TRIG_F, ECHO_F);
  long target = d0 - (long)n * CELL_CM;
  if (target < WALL_STOP_CM) target = WALL_STOP_CM;
  unsigned long maxMs = (unsigned long)n * MS_PER_CELL * 2;

  unsigned long t0 = millis();
  lastMicros = micros();
  int loopCount = 0;
  int sideCorr = 0;

  while (millis() - t0 < maxMs) {
    checkTimeUp();
    updateYaw();

    long f = readCm(TRIG_F, ECHO_F);
    if (f <= target + ARRIVE_TOL) break;

    if (++loopCount % 3 == 0) {
      long l = readCm(TRIG_L, ECHO_L);
      long r = readCm(TRIG_R, ECHO_R);
      bool hasL = l < SIDE_WALL_CM;
      bool hasR = r < SIDE_WALL_CM;
      if (hasL && hasR)  sideCorr = -(int)(KP_SIDE * (l - r));
      else if (hasL)     sideCorr = -(int)(KP_SIDE * (l - SIDE_TARGET_CM));
      else if (hasR)     sideCorr =  (int)(KP_SIDE * (r - SIDE_TARGET_CM));
      else               sideCorr = 0;
      sideCorr = constrain(sideCorr, -25, 25);
    }

    float err = yaw - targetHeading;
    int corr = (int)(KP * err) + sideCorr;
    drive(BASE_SPEED + corr, BASE_SPEED - corr);
  }
  stopMotors();
  delay(150);
}

void runRoute(Step *route, int len) {
  for (int i = 0; i < len; i++) {
    Step *s = &route[i];
    char t = s->type;
    if (MIRROR && t == 'L')      t = 'R';
    else if (MIRROR && t == 'R') t = 'L';

    if (t == 'F')      forwardCells(s->value);
    else if (t == 'L') turn90(true);
    else if (t == 'R') turn90(false);
  }
}

void setup() {
  Serial.begin(9600);

  int outs[] = {ENA, IN1, IN2, ENB, IN3, IN4, TRIG_F, TRIG_L, TRIG_R};
  for (int i = 0; i < 9; i++) pinMode(outs[i], OUTPUT);
  pinMode(ECHO_F, INPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(ECHO_R, INPUT);
  pinMode(IR_PIN, INPUT);

  stopMotors();

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);                
  Wire.write(0);
  Wire.endTransmission(true);

  delay(1000);
  calibrateGyro();               
  delay(START_DELAY_MS);

  matchStart = millis();
  lastMicros = micros();
}

void loop() {
  if (ROUTE_CHOICE == 1) runRoute(routeA1, ROUTE_A1_LEN);
  else                   runRoute(routeA2, ROUTE_A2_LEN);

  stopMotors();
  while (true) { }
}
