// ================================
// Autori: Adin PISICA, Filippo MURU
// Classe: 3AEE
// Data: 04/12/2025
// Descrizione: Ciclo continuo motori
// Avanti, Stop, Avanti Sinistra, Stop,
// Avanti Destra, Stop, Indietro, Stop,
// Indietro Sinistra, Stop, Indietro Destra,
// Stop, Ripete tutto da capo
// ================================

// dichiariamo i pin
int IN1 = 9;
int IN2 = 8;
int IN3 = 10;
int IN4 = 11;

// funzione per accelerazione
void accelera(int pin1, int pin2, int velocitaFinale) {
  digitalWrite(pin2, LOW);
  for (int v = 0; v <= velocitaFinale; v += 5) {
    analogWrite(pin1, v);
    delay(20);
  }
}

void decelera(int pin1) {
  for (int v = 255; v >= 0; v -= 5) {
    analogWrite(pin1, v);
    delay(20);
  }
  digitalWrite(pin1, LOW);
}


void stopMotori() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(700);
}

void avanti() {
  accelera(IN1, IN2, 255);
  accelera(IN3, IN4, 255);
  delay(2000);
  decelera(IN1);
  decelera(IN3);
}

void indietro() {
  accelera(IN2, IN1, 255);
  accelera(IN4, IN3, 255);
  delay(2000);
  decelera(IN2);
  decelera(IN4);
}

void avantiSinistra() {
  accelera(IN1, IN2, 150);
  accelera(IN3, IN4, 255);
  delay(2000);
  decelera(IN1);
  decelera(IN3);
}

void avantiDestra() {
  accelera(IN1, IN2, 255);
  accelera(IN3, IN4, 150);
  delay(2000);
  decelera(IN1);
  decelera(IN3);
}

void dietroSinistra() {
  accelera(IN2, IN1, 150);
  accelera(IN4, IN3, 255);
  delay(2000);
  decelera(IN2);
  decelera(IN4);
}

void dietroDestra() {
  accelera(IN2, IN1, 255);
  accelera(IN4, IN3, 150);
  delay(2000);
  decelera(IN2);
  decelera(IN4);
}


void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}


void loop() {
  avanti();
  stopMotori();

  avantiSinistra();
  stopMotori();

  avantiDestra();
  stopMotori();

  indietro();
  stopMotori();

  dietroSinistra();
  stopMotori();

  dietroDestra();
  stopMotori();
}
