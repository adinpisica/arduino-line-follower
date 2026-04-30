// ================================================================
//  Progetto : Line Follower Robot  –  Layout 2 Sensori
//  File     : src/main_arduino.ino
//  Autori   : Adin PISICA, Filippo MURU
//  Classe   : 3AEE  –  A.S. 2025/2026
//  Istituto : Cigna-Baruffi-Garelli  (plesso Cigna)
//  Docente  : Davide Bertolino
//  Board    : Arduino Uno R3  (ATmega328P)
// ================================================================
//
//  PRINCIPIO DI FUNZIONAMENTO – 2 SENSORI
//  ──────────────────────────────────────
//  Il nastro adesivo nero passa TRA i due sensori.
//  In condizione normale (robot centrato) entrambi vedono bianco.
//
//  Stato sensori → Azione
//  ──────────────────────
//  SX=bianco  DX=bianco  →  Dritto     (linea al centro, ok)
//  SX=nero    DX=bianco  →  Svolta SX  (robot slittato a destra)
//  SX=bianco  DX=nero    →  Svolta DX  (robot slittato a sinistra)
//  SX=nero    DX=nero    →  Stop       (fine pista / T-incrocio)
//
// ================================================================
//  SCHEMA DI MONTAGGIO
//  ──────────────────────────────────────────────────────────────
//
//  Vista dall'alto (fronte robot in alto):
//
//           ┌─────────────────┐
//           │   [SX]   [DX]   │  ← sensori HW-870, rivolti verso il basso
//           │    |       |    │
//           │    ·  GAP  ·    │  ← nastro nero passa in questo spazio
//           └────────┬────────┘
//                 (fronte)
//
//  Distanza consigliata sensori dal pavimento : 8–12 mm
//  Distanza tra i due sensori (centro PCB)    : 25–35 mm
//  Larghezza nastro nero consigliata           : 18–25 mm
//
//  Il nastro deve passare esattamente al centro tra i due sensori.
//  Se il nastro è più stretto dello spazio tra i sensori entrambi
//  vedranno sempre bianco → il robot andrà sempre dritto.
//  Se il nastro è più largo entrambi vedranno sempre nero → stop.
//  Regola la distanza tra i sensori di conseguenza.
//
// ================================================================
//  SCHEMA PIN
//  ──────────────────────────────────────────────────────────────
//
//  Driver motori L298N
//    IN1 → D9   (PWM ~)  motore SX – avanti
//    IN2 → D8            motore SX – indietro
//    IN3 → D10  (PWM ~)  motore DX – avanti
//    IN4 → D11  (PWM ~)  motore DX – indietro
//
//  Sensori IR HW-870 (uscita AO – analogica)
//    SX  → A0
//    DX  → A1
//
//  LED di stato
//    LED → D13  (LED built-in Uno)
//        HIGH = robot attivo e segue la linea
//        Lampeggio rapido = ricerca linea
//        LOW  = fermo
//
//  Alimentazione
//    Motori  : 7–12 V sul morsetto VM del L298N (separato da Arduino)
//    Arduino : USB o regolatore 5 V
//    Sensori : 3.3 V o 5 V dal pin VCC dei HW-870
//
// ================================================================
//  CALIBRAZIONE SOGLIA SENSORI
//  ──────────────────────────────────────────────────────────────
//  1. Carica test/test-sensore.ino su Arduino (modifica il pin se
//     necessario: usa A0 per il sensore SX, poi A1 per il DX).
//  2. Apri il Monitor Seriale a 9600 baud.
//  3. Annota il valore analogico su BIANCO  (es. 280).
//  4. Annota il valore analogico su NERO    (es. 820).
//  5. Imposta SENS_THRESHOLD a metà: (280+820)/2 = 550.
//  6. Ripeti per entrambi i sensori; usa il valore più conservativo.
// ================================================================

// ----------------------------------------------------------------
//  DEBUG – decommentare la riga seguente per abilitare il monitor
//  seriale. TENERE COMMENTATO nel deploy finale: occupa ~200 B
//  di SRAM aggiuntiva e introduce latenza di ~1 ms per stampa.
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
//  CONFIGURAZIONE  –  modifica solo questa sezione
// ================================================================

// --- Pin motori (L298N) ---
static const uint8_t PIN_IN1 = 9;   // SX avanti  (PWM obbligatorio)
static const uint8_t PIN_IN2 = 8;   // SX indietro
static const uint8_t PIN_IN3 = 10;  // DX avanti  (PWM obbligatorio)
static const uint8_t PIN_IN4 = 11;  // DX indietro (PWM obbligatorio)

// --- Pin sensori (ADC, 10-bit → 0-1023) ---
static const uint8_t PIN_SENS_SX = A0;
static const uint8_t PIN_SENS_DX = A1;

// --- LED di stato ---
static const uint8_t PIN_LED = 13;

// --- Soglia sensore (vedi procedura di calibrazione sopra) ---
//  Sotto la soglia = bianco/chiaro → sensore non vede la linea
//  Sopra la soglia = nero/scuro   → sensore vede la linea
static const uint16_t SENS_THRESHOLD = 550;

// --- Velocità (0–255) ---
static const uint8_t VEL_DRITTO   = 190; // entrambi i motori su rettilineo
static const uint8_t VEL_EXT      = 190; // motore esterno durante svolta
static const uint8_t VEL_INT      =   0; // motore interno durante svolta (0 = pivot)
static const uint8_t VEL_RICERCA  = 150; // velocità rotazione durante ricerca linea

// --- Timeout ricerca linea (ms) ---
//  Se la linea non viene ritrovata entro questo tempo il robot si
//  ferma automaticamente per evitare di uscire dal percorso.
static const uint16_t TIMEOUT_RICERCA_MS = 2500;

// --- Ritardo loop principale (ms) ---
//  Non scendere sotto ~5 ms (limite conversione ADC su Uno).
//  Valori più bassi = maggiore reattività = più correnti di picco.
static const uint8_t LOOP_DELAY_MS = 10;

// ================================================================
//  Tipi
// ================================================================

enum Stato : uint8_t {
  ST_FERMO = 0,
  ST_DRITTO,
  ST_SVOLTA_SX,
  ST_SVOLTA_DX,
  ST_RICERCA_SX,  // rotazione SX alla ricerca della linea
  ST_RICERCA_DX   // rotazione DX alla ricerca della linea
};

struct Sensori {
  bool sx;   // true = vede NERO
  bool dx;   // true = vede NERO
};

// ================================================================
//  Variabili globali  (minimizzate per rispettare i 2 KB di SRAM)
// ================================================================
static Stato    g_stato        = ST_DRITTO;
static Stato    g_ultimaSvolta = ST_SVOLTA_DX; // ultima direzione nota
static uint32_t g_tsRicerca    = 0;             // timestamp inizio ricerca
static uint32_t g_tsLed        = 0;             // timestamp per lampeggio LED
static bool     g_ledState     = false;

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

// Manovre predefinite
static inline void mDritto()    { motoreSX(VEL_DRITTO,  true);  motoreDX(VEL_DRITTO,  true);  }
static inline void mSvoltaSX()  { motoreSX(VEL_INT,     true);  motoreDX(VEL_EXT,     true);  }
static inline void mSvoltaDX()  { motoreSX(VEL_EXT,     true);  motoreDX(VEL_INT,     true);  }
static inline void mRicercaSX() { motoreSX(VEL_RICERCA, false); motoreDX(VEL_RICERCA, true);  }
static inline void mRicercaDX() { motoreSX(VEL_RICERCA, true);  motoreDX(VEL_RICERCA, false); }

// ================================================================
//  Lettura sensori  (doppia lettura mediata per ridurre il rumore)
// ================================================================

static Sensori leggiSensori() {
  uint16_t vSx = (analogRead(PIN_SENS_SX) + analogRead(PIN_SENS_SX)) >> 1;
  uint16_t vDx = (analogRead(PIN_SENS_DX) + analogRead(PIN_SENS_DX)) >> 1;

#ifdef DEBUG
  Serial.print(F("[ADC] SX="));
  if (vSx < 100) Serial.print(F("  "));
  else if (vSx < 1000) Serial.print(F(" "));
  Serial.print(vSx);
  Serial.print(F("  DX="));
  if (vDx < 100) Serial.print(F("  "));
  else if (vDx < 1000) Serial.print(F(" "));
  Serial.print(vDx);
  Serial.print(F("  THR="));
  Serial.print(SENS_THRESHOLD);
  Serial.print(F("  → SX="));
  Serial.print(vSx > SENS_THRESHOLD ? F("NERO ") : F("BLANC"));
  Serial.print(F("  DX="));
  Serial.println(vDx > SENS_THRESHOLD ? F("NERO ") : F("BLANC"));
#endif

  return { vSx > SENS_THRESHOLD, vDx > SENS_THRESHOLD };
}

// ================================================================
//  Aggiornamento LED di stato (non-blocking)
// ================================================================

static void aggiornaLed() {
  switch (g_stato) {
    case ST_DRITTO:
    case ST_SVOLTA_SX:
    case ST_SVOLTA_DX:
      // LED fisso acceso → robot segue la linea
      digitalWrite(PIN_LED, HIGH);
      break;

    case ST_RICERCA_SX:
    case ST_RICERCA_DX: {
      // LED lampeggio rapido (200 ms) → ricerca linea
      uint32_t ora = millis();
      if (ora - g_tsLed >= 200) {
        g_tsLed   = ora;
        g_ledState = !g_ledState;
        digitalWrite(PIN_LED, g_ledState);
      }
      break;
    }

    default:
      // ST_FERMO → LED spento
      digitalWrite(PIN_LED, LOW);
      break;
  }
}

// ================================================================
//  Macchina a stati – logica line follower 2 sensori
// ================================================================

static void aggiornaSeguiLinea() {
  const Sensori s = leggiSensori();

  // ── CASO 1: entrambi NERI → fine pista / incrocio ────────────
  if (s.sx && s.dx) {
    fermaMotori();
    g_stato = ST_FERMO;
    LOGLN(F("[FSM] Fine pista / incrocio → FERMO"));
    return;
  }

  // ── CASO 2: entrambi BIANCHI → linea al centro, dritto ───────
  if (!s.sx && !s.dx) {
    // Siamo già in ricerca? Controlla se è appena terminata.
    if (g_stato == ST_RICERCA_SX || g_stato == ST_RICERCA_DX) {
      LOGLN(F("[FSM] Linea ritrovata → DRITTO"));
    }
    mDritto();
    g_stato = ST_DRITTO;
    return;
  }

  // ── CASO 3: solo SX NERO → robot slittato a destra, gira SX ─
  if (s.sx && !s.dx) {
    mSvoltaSX();
    g_stato        = ST_SVOLTA_SX;
    g_ultimaSvolta = ST_SVOLTA_SX;
    LOGLN(F("[FSM] SX nero → SVOLTA_SX"));
    return;
  }

  // ── CASO 4: solo DX NERO → robot slittato a sinistra, gira DX
  if (!s.sx && s.dx) {
    mSvoltaDX();
    g_stato        = ST_SVOLTA_DX;
    g_ultimaSvolta = ST_SVOLTA_DX;
    LOGLN(F("[FSM] DX nero → SVOLTA_DX"));
    return;
  }

  // ── CASO 5: linea persa (nessun sensore utile raggiunto) ─────
  //   Questo ramo non dovrebbe mai essere raggiunto con 2 sensori
  //   ma viene mantenuto come sicurezza.
  if (g_stato != ST_RICERCA_SX && g_stato != ST_RICERCA_DX) {
    g_tsRicerca = millis();
    g_stato     = (g_ultimaSvolta == ST_SVOLTA_SX)
                  ? ST_RICERCA_SX
                  : ST_RICERCA_DX;
    LOGLN(g_stato == ST_RICERCA_SX
          ? F("[FSM] Linea persa → RICERCA_SX")
          : F("[FSM] Linea persa → RICERCA_DX"));
  }

  if ((uint32_t)(millis() - g_tsRicerca) > TIMEOUT_RICERCA_MS) {
    fermaMotori();
    g_stato = ST_FERMO;
    LOGLN(F("[FSM] Timeout ricerca → FERMO"));
    return;
  }

  if (g_stato == ST_RICERCA_SX) { mRicercaSX(); }
  else                           { mRicercaDX(); }
}

// ================================================================
//  setup()
// ================================================================
void setup() {
#ifdef DEBUG
  Serial.begin(9600);
  while (!Serial) { /* attendi apertura porta */ }
  LOGLN(F("================================================"));
  LOGLN(F(" Line Follower 2-Sensori – Arduino Uno – DEBUG"));
  LOGLN(F("================================================"));
  LOGLN(F(" Motori  : IN1=D9  IN2=D8  IN3=D10 IN4=D11"));
  LOGLN(F(" Sensori : SX=A0   DX=A1"));
  LOGLN(F(" Soglia  : vedere SENS_THRESHOLD nel codice"));
  LOGLN(F(" Avvio fra 3 secondi..."));
  delay(3000);
#endif

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  fermaMotori();

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  LOGLN(F("[SYS] Pronto – line following attivo."));
}

// ================================================================
//  loop()
// ================================================================
void loop() {
  aggiornaSeguiLinea();
  aggiornaLed();
  delay(LOOP_DELAY_MS);
}
