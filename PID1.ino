#define TRIG_PIN 9
#define ECHO_PIN 10
#define ENA 7
#define IN1 6
#define IN2 5
#define IN3 4
#define IN4 3
#define ENB 2
#define LED_PIN 8
#define DUST_SENSOR_PIN A0

long duration;
int distance;
int setPoint = 20; // Jarak dalam cm untuk menghindari halangan
int error, lastError;
float kp = 2.0, ki = 0.5, kd = 1.0;
float P, I, D;
float PIDValue;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(DUST_SENSOR_PIN, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  // Mengukur jarak
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
  
  // Mengukur konsentrasi debu
  digitalWrite(LED_PIN, LOW); // Mengaktifkan LED IR
  delayMicroseconds(280);
  int dustValue = analogRead(DUST_SENSOR_PIN);
  delayMicroseconds(40);
  digitalWrite(LED_PIN, HIGH); // Mematikan LED IR
  delayMicroseconds(9680);
  
  // PID control
  error = setPoint - distance;
  P = error;
  I += error;
  D = error - lastError;
  PIDValue = kp * P + ki * I + kd * D;
  lastError = error;

  // Logika penghindaran halangan dan deteksi debu
  if (dustValue < 20) { // Ganti 'threshold' dengan nilai yang sesuai untuk deteksi debu
    // Jika tidak ada debu, berhenti
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  } else if (distance <= setPoint) {
    // Belok ke kanan
    analogWrite(ENA, 0);
    analogWrite(ENB, 255);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    // Maju
    analogWrite(ENA, PIDValue);
    analogWrite(ENB, PIDValue);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  delay(100);
}
