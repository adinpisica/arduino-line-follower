// ================================================================
//  Progetto : Line Follower Robot
//  File     : dev/main_arduino.ino
//  Autori   : Adin PISICA, Filippo MURU
//  Classe   : 3AEE  –  A.S. 2025/2026
//  Istituto : Cigna-Baruffi-Garelli  (plesso Cigna)
//  Docente  : Davide Bertolino
//  Board    : Arduino Uno R3  (ATmega328P)
// ================================================================
//  Disposizione sensori (vista dall'alto, robot verso l'alto)
//
//        [SX 45°]  [CENTRO]  [DX 45°]
//            \        |        /
//             \_______|_______/
//                  (fronte)
//
//  Distanza raccomandata dal pavimento: 8–12 mm
//  Larghezza linea raccomandata: 18–25 mm
// ================================================================
//  Schema pin
//  ──────────────────────────────────────────────────────────────
//  Motori (L298N)
//    IN1 → D9   (PWM ~, motore SX – avanti)
//    IN2 → D8   (DIR,   motore SX – indietro)
//    IN3 → D10  (PWM ~, motore DX – avanti)
//    IN4 → D11  (PWM ~, motore DX – indietro)
//  Sensori HW-870 (uscita AO)
//    SX     → A0
//    CENTRO → A1
//    DX     → A2
//  LED di stato
//    LED    → D13 (built-in)
// ================================================================

// ----------------------------------------------------------------
//  COMPILAZIONE CONDIZIONALE
//  Decommentare per abilitare il debug sul monitor seriale.
//  TENERE COMMENTATO nel deploy finale: Serial occupa RAM preziosa
//  e introduce latenza che riduce la reattività del robot.
// ----------------------------------------------------------------
// #define DEBUG

#ifdef DEBUG
  #define LOG(x)    Serial.print(x)
  #define LOGLN(x)  Serial.println(x)
#else
  #define LOG(x)    ((void)0)
  #define LOGLN(x)  ((void)0)
#endif

// ================================================================
//  CONFIGURAZIONE
//  Modifica solo questa sezione per adattare il robot.
// ================================================================

// --- Pin motori ---
static const uint8_t PIN_IN1 = 9;   // SX avanti  (PWM)
static const uint8_t PIN_IN2 = 8;   // SX indietro
static const uint8_t PIN_IN3 = 10;  // DX avanti  (PWM)
static const uint8_t PIN_IN4 = 11;  // DX indietro (PWM)

// --- Pin sensori (analogici) ---
static const uint8_t PIN_SENS_SX = A0;
static const uint8_t PIN_SENS_C  = A1;
static const uint8_t PIN_SENS_DX = A2;

// --- Pin LED di stato ---
static const uint8_t PIN_LED = 13;

// --- Soglia sensore (0–1023, ADC 10-bit) ---
//  Bianco ≈ 200–450  |  Nero ≈ 600–1023
//  Misura i tuoi valori con test-sensore.ino e imposta il punto
//  a metà tra il valore bianco e il valore nero rilevati.
static const uint16_t SENS_THRESHOLD = 550;

// --- Velocità (0–255) ---
static const uint8_t VEL_DRITTO    = 190; // rettilineo
static const uint8_t VEL_CURVA_EX  = 190; // motore esterno in curva morbida
static const uint8_t VEL_CURVA_IN  =  80; // motore interno in curva morbida
static const uint8_t VEL_SVOLTA    = 160; // rotazione decisa sul posto
static const uint8_t VEL_RICERCA   = 140; // rotazione durante ricerca linea

// --- Timeout ricerca linea (ms) ---
//  Se la linea non viene ritrovata entro questo tempo, il robot
//  si ferma autonomamente per evitare di andare fuori controllo.
static const uint16_t TIMEOUT_RICERCA_MS = 2500;

// --- Ritardo loop principale (ms) ---
//  Valori più bassi aumentano la reattività.
//  Non scendere sotto ~5 ms (limite conversione ADC Uno).
static const uint8_t LOOP_DELAY_MS = 10;

// ================================================================
//  Tipi
// ================================================================

enum Stato : uint8_t {
  ST_FERMO = 0,
  ST_DRITTO,
  ST_CURVA_SX,
  ST_CURVA_DX,
  ST_SVOLTA_SX,
  ST_SVOLTA_DX,
  ST_RICERCA_SX,
  ST_RICERCA_DX
};

struct Sensori {
  bool sx;
  bool centro;
  bool dx;
};

// ================================================================
//  Variabili globali (minimizzate per rispettare i 2 KB di SRAM)
// ================================================================
static Stato    g_stato        = ST_DRITTO;
static Stato    g_ultimaSvolta = ST_SVOLTA_DX;
static uint32_t g_tsRicerca    = 0;

// ================================================================
//  Controllo motori
// ================================================================

static inline void motoreSX(uint8_t vel, bool avanti) {
  if (avanti) {
    analogWrite (PIN_IN1, vel);
    digitalWrite(PIN_IN2, LOW);
  } else {
    digitalWrite(PIN_IN1, LOW);
    analogWrite (PIN_IN2, vel);
  }
}

static inline void motoreDX(uint8_t vel, bool avanti) {
  if (avanti) {
    analogWrite (PIN_IN3, vel);
    digitalWrite(PIN_IN4, LOW);
  } else {
    digitalWrite(PIN_IN3, LOW);
    analogWrite (PIN_IN4, vel);
  }
}

static void fermaMotori() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

// Shorthand manovre
static inline void mDritto()    { motoreSX(VEL_DRITTO,   true);  motoreDX(VEL_DRITTO,   true);  }
static inline void mCurvaSX()   { motoreSX(VEL_CURVA_IN, true);  motoreDX(VEL_CURVA_EX, true);  }
static inline void mCurvaDX()   { motoreSX(VEL_CURVA_EX, true);  motoreDX(VEL_CURVA_IN, true);  }
static inline void mSvoltaSX()  { motoreSX(0,            true);  motoreDX(VEL_SVOLTA,   true);  }
static inline void mSvoltaDX()  { motoreSX(VEL_SVOLTA,   true);  motoreDX(0,            true);  }
static inline void mRicercaSX() { motoreSX(VEL_RICERCA,  false); motoreDX(VEL_RICERCA,  true);  }
static inline void mRicercaDX() { motoreSX(VEL_RICERCA,  true);  motoreDX(VEL_RICERCA,  false); }

// ================================================================
//  Lettura sensori
// ================================================================

static Sensori leggiSensori() {
  // Doppia lettura mediata per ridurre rumore ADC
  uint16_t vSx = (analogRead(PIN_SENS_SX) + analogRead(PIN_SENS_SX)) >> 1;
  uint16_t vC  = (analogRead(PIN_SENS_C)  + analogRead(PIN_SENS_C))  >> 1;
  uint16_t vDx = (analogRead(PIN_SENS_DX) + analogRead(PIN_SENS_DX)) >> 1;

#ifdef DEBUG
  Serial.print(F("[ADC] SX=")); Serial.print(vSx);
  Serial.print(F(" C="));      Serial.print(vC);
  Serial.print(F(" DX="));     Serial.print(vDx);
  Serial.print(F(" THR="));    Serial.println(SENS_THRESHOLD);
#endif

  return { vSx > SENS_THRESHOLD,
           vC  > SENS_THRESHOLD,
           vDx > SENS_THRESHOLD };
}

// ================================================================
//  Macchina a stati – logica line follower
// ================================================================

static void aggiornaSeguiLinea() {
  const Sensori s = leggiSensori();

  // ── CASO 1: tutti i sensori su nero ──────────────────────────
  //    Croce, T-incrocio o fine pista → stop cautela
  if (s.sx && s.centro && s.dx) {
    fermaMotori();
    g_stato = ST_FERMO;
    LOGLN(F("[FSM] Incrocio / fine pista → FERMO"));
    return;
  }

  // ── CASO 2: sensore centrale sulla linea ─────────────────────
  if (s.centro) {
    if (s.sx && !s.dx) {
      // Centro + SX → curva morbida a sinistra
      mCurvaSX();
      g_stato        = ST_CURVA_SX;
      g_ultimaSvolta = ST_SVOLTA_SX;

    } else if (!s.sx && s.dx) {
      // Centro + DX → curva morbida a destra
      mCurvaDX();
      g_stato        = ST_CURVA_DX;
      g_ultimaSvolta = ST_SVOLTA_DX;

    } else {
      // Solo centro → rettilineo
      mDritto();
      g_stato = ST_DRITTO;
    }
    return;
  }

  // ── CASO 3: solo sensore sinistro ────────────────────────────
  if (s.sx && !s.dx) {
    mSvoltaSX();
    g_stato        = ST_SVOLTA_SX;
    g_ultimaSvolta = ST_SVOLTA_SX;
    return;
  }

  // ── CASO 4: solo sensore destro ──────────────────────────────
  if (!s.sx && s.dx) {
    mSvoltaDX();
    g_stato        = ST_SVOLTA_DX;
    g_ultimaSvolta = ST_SVOLTA_DX;
    return;
  }

  // ── CASO 5: nessun sensore vede nero → linea persa ───────────
  if (g_stato != ST_RICERCA_SX && g_stato != ST_RICERCA_DX) {
    g_tsRicerca = millis();
    g_stato     = (g_ultimaSvolta == ST_SVOLTA_SX)
                  ? ST_RICERCA_SX
                  : ST_RICERCA_DX;

    LOGLN(g_stato == ST_RICERCA_SX
          ? F("[FSM] Linea persa → RICERCA_SX")
          : F("[FSM] Linea persa → RICERCA_DX"));
  }

  // Timeout: linea non ritrovata → fermo
  if ((uint32_t)(millis() - g_tsRicerca) > TIMEOUT_RICERCA_MS) {
    fermaMotori();
    g_stato = ST_FERMO;
    LOGLN(F("[FSM] Timeout ricerca → FERMO"));
    return;
  }

  // Rotazione di ricerca nell'ultima direzione nota
  if (g_stato == ST_RICERCA_SX) { mRicercaSX(); }
  else                           { mRicercaDX(); }
}

// ================================================================
//  setup()
// ================================================================
void setup() {
#ifdef DEBUG
  Serial.begin(9600);
  LOGLN(F("================================================"));
  LOGLN(F(" Line Follower  –  Arduino Uno R3  –  DEBUG ON"));
  LOGLN(F("================================================"));
  LOGLN(F(" Motori : IN1=D9 IN2=D8 IN3=D10 IN4=D11"));
  LOGLN(F(" Sensori: SX=A0  C=A1  DX=A2"));
  LOGLN(F(" Avvio fra 3 secondi..."));
  delay(3000);
#endif

  // Pin motori
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  fermaMotori();

  // LED di stato (HIGH = sistema attivo)
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  LOGLN(F("[SYS] Pronto. Line following attivo."));
}

// ================================================================
//  loop()
// ================================================================
void loop() {
  aggiornaSeguiLinea();

  // LED: spento = fermo / ricerca, acceso = segue la linea
  digitalWrite(PIN_LED,
    g_stato != ST_FERMO &&
    g_stato != ST_RICERCA_SX &&
    g_stato != ST_RICERCA_DX);

  delay(LOOP_DELAY_MS);
}
