// ================================================================
//  Progetto : Line Follower Robot
//  File     : dev/main_esp32.ino
//  Autori   : Adin PISICA, Filippo MURU
//  Classe   : 3AEE  –  A.S. 2025/2026
//  Istituto : Cigna-Baruffi-Garelli  (plesso Cigna)
//  Docente  : Davide Bertolino
//  Board    : ESP32 (WROOM / DevKit)
// ================================================================
//  Hardware richiesto
//  ──────────────────
//  • Driver motori  : L298N (o compatibile, 4 ingressi PWM)
//  • Sensori IR     : 3× HW-870 (uscita analogica AO)
//  • Alimentazione  : motori 7-12 V separati da ESP32 3.3/5 V
// ================================================================
//  Disposizione sensori (vista dall'alto, robot verso l'alto)
//
//          [SX 45°]  [CENTRO]  [DX 45°]
//              \        |        /
//               \_______|_______/
//                    (fronte)
//
//  I due sensori laterali sono ruotati di 45° verso l'esterno
//  rispetto al sensore centrale, tutti rivolti verso il basso.
// ================================================================

// ----------------------------------------------------------------
//  COMPILAZIONE CONDIZIONALE – decommentare per debug seriale
// ----------------------------------------------------------------
// #define DEBUG

#ifdef DEBUG
  #define LOG(x)     Serial.print(x)
  #define LOGLN(x)   Serial.println(x)
  #define LOGF(...)  Serial.printf(__VA_ARGS__)
#else
  #define LOG(x)     ((void)0)
  #define LOGLN(x)   ((void)0)
  #define LOGF(...)  ((void)0)
#endif

// ----------------------------------------------------------------
//  Librerie
// ----------------------------------------------------------------
#include <WiFi.h>
#include <WebServer.h>

// ================================================================
//  CONFIGURAZIONE – modifica qui senza toccare il resto del codice
// ================================================================

// --- WiFi Access Point ---
static constexpr char    AP_SSID[]     = "LineFollower-AP";
static constexpr char    AP_PASSWORD[] = "robot1234";   // min 8 caratteri
static constexpr uint8_t AP_CHANNEL    = 1;
static constexpr uint8_t AP_MAX_CONN   = 4;

// --- Pin motori (L298N) ---
//   Motore SINISTRO : IN1 avanti (PWM),  IN2 indietro (PWM)
//   Motore DESTRO   : IN3 avanti (PWM),  IN4 indietro (PWM)
static constexpr uint8_t PIN_IN1 = 25;
static constexpr uint8_t PIN_IN2 = 26;
static constexpr uint8_t PIN_IN3 = 27;
static constexpr uint8_t PIN_IN4 = 14;

// --- Pin sensori IR HW-870 (solo uscita analogica AO) ---
//   ATTENZIONE: usare solo pin ADC1 (GPIO 32-39) con WiFi attivo.
//   ADC2 è disabilitato quando il radio WiFi è in uso.
static constexpr uint8_t PIN_SENS_SX = 34;  // sensore sinistro (45° sx)
static constexpr uint8_t PIN_SENS_C  = 35;  // sensore centrale
static constexpr uint8_t PIN_SENS_DX = 32;  // sensore destro   (45° dx)

// --- Soglia sensori (0-4095 su ADC 12-bit ESP32) ---
//   Valori misurati empiricamente: bianco ≈ 300-600, nero ≈ 800-4095
//   Regolare in base alla distanza sensore–pavimento (consigliato 8-12 mm)
static constexpr uint16_t SENS_THRESHOLD = 1800;

// --- Parametri PWM ---
static constexpr uint32_t PWM_FREQ    = 5000;  // Hz
static constexpr uint8_t  PWM_RES     = 8;     // bit → range 0-255

// --- Parametri di guida ---
static constexpr uint8_t  SPEED_MAX    = 200;  // velocità dritta  (0-255)
static constexpr uint8_t  SPEED_CURVE  = 130;  // velocità lato interno in curva
static constexpr uint8_t  SPEED_TURN   = 160;  // velocità rotazione sul posto
static constexpr uint32_t SEARCH_MS    = 2500; // ms di ricerca prima di fermarsi
static constexpr uint32_t LOOP_DELAY   = 15;   // ms tra un ciclo e l'altro (~66 Hz)

// ================================================================
//  Canali LEDC (PWM ESP32 – core v2.x)
// ================================================================
enum LedcCh : uint8_t { CH_IN1 = 0, CH_IN2, CH_IN3, CH_IN4 };

// ================================================================
//  Stato della macchina a stati
// ================================================================
enum RobotState : uint8_t {
  ST_STOP = 0,
  ST_FORWARD,
  ST_CURVE_SX,
  ST_CURVE_DX,
  ST_TURN_SX,    // rotazione decisa sul posto
  ST_TURN_DX,
  ST_SEARCH_SX,  // ricerca linea ruotando a sinistra
  ST_SEARCH_DX   // ricerca linea ruotando a destra
};

// ================================================================
//  Variabili globali
// ================================================================
static volatile bool g_enabled    = false;
static RobotState    g_state      = ST_STOP;
static RobotState    g_lastTurn   = ST_TURN_DX; // ultima svolta nota
static uint32_t      g_searchTs   = 0;          // timestamp inizio ricerca

WebServer httpServer(80);

// ================================================================
//  Gestione motori
// ================================================================

static inline void pwmSet(LedcCh ch, uint8_t val) {
  ledcWrite(static_cast<uint8_t>(ch), val);
}

/**
 * @brief Imposta velocità motore sinistro.
 * @param spd  [-255 .. +255]  positivo=avanti, negativo=indietro, 0=stop
 */
static void motorLeft(int16_t spd) {
  if (spd > 0) {
    pwmSet(CH_IN1, static_cast<uint8_t>(min(spd, (int16_t)255)));
    pwmSet(CH_IN2, 0);
  } else if (spd < 0) {
    pwmSet(CH_IN1, 0);
    pwmSet(CH_IN2, static_cast<uint8_t>(min(-spd, (int16_t)255)));
  } else {
    pwmSet(CH_IN1, 0);
    pwmSet(CH_IN2, 0);
  }
}

/**
 * @brief Imposta velocità motore destro.
 * @param spd  [-255 .. +255]
 */
static void motorRight(int16_t spd) {
  if (spd > 0) {
    pwmSet(CH_IN3, static_cast<uint8_t>(min(spd, (int16_t)255)));
    pwmSet(CH_IN4, 0);
  } else if (spd < 0) {
    pwmSet(CH_IN3, 0);
    pwmSet(CH_IN4, static_cast<uint8_t>(min(-spd, (int16_t)255)));
  } else {
    pwmSet(CH_IN3, 0);
    pwmSet(CH_IN4, 0);
  }
}

static void stopMotors() {
  motorLeft(0);
  motorRight(0);
}

// ================================================================
//  Lettura sensori
// ================================================================

struct Sensors {
  bool sx;     // true = vede nero
  bool center;
  bool dx;
};

/**
 * Legge i tre sensori IR via ADC.
 * Ogni canale viene campionato due volte e mediato per ridurre il rumore.
 */
static Sensors readSensors() {
  // Media di 2 letture per canale per ridurre noise ADC
  uint16_t vSx = (analogRead(PIN_SENS_SX) + analogRead(PIN_SENS_SX)) >> 1;
  uint16_t vC  = (analogRead(PIN_SENS_C)  + analogRead(PIN_SENS_C))  >> 1;
  uint16_t vDx = (analogRead(PIN_SENS_DX) + analogRead(PIN_SENS_DX)) >> 1;

  LOGF("[ADC] SX=%4u C=%4u DX=%4u | THR=%u\n", vSx, vC, vDx, SENS_THRESHOLD);

  return { vSx > SENS_THRESHOLD, vC > SENS_THRESHOLD, vDx > SENS_THRESHOLD };
}

// ================================================================
//  Logica line follower (macchina a stati)
// ================================================================

static void updateFollower() {
  const Sensors s = readSensors();

  // ── Tutti i sensori neri → fine pista o T-incrocio: stop ──────
  if (s.sx && s.center && s.dx) {
    stopMotors();
    g_state = ST_STOP;
    LOGLN("[FSM] Fine pista / T-incrocio → STOP");
    return;
  }

  // ── Sensore centrale sulla linea ─────────────────────────────
  if (s.center) {
    if (s.sx && !s.dx) {
      // Centro + SX → correzione morbida a sinistra
      motorLeft(SPEED_CURVE);
      motorRight(SPEED_MAX);
      g_state    = ST_CURVE_SX;
      g_lastTurn = ST_TURN_SX;
    } else if (!s.sx && s.dx) {
      // Centro + DX → correzione morbida a destra
      motorLeft(SPEED_MAX);
      motorRight(SPEED_CURVE);
      g_state    = ST_CURVE_DX;
      g_lastTurn = ST_TURN_DX;
    } else {
      // Solo centro → dritto
      motorLeft(SPEED_MAX);
      motorRight(SPEED_MAX);
      g_state = ST_FORWARD;
    }
    return;
  }

  // ── Solo sensore sinistro → svolta sinistra decisa ───────────
  if (s.sx && !s.dx) {
    motorLeft(0);
    motorRight(SPEED_TURN);
    g_state    = ST_TURN_SX;
    g_lastTurn = ST_TURN_SX;
    return;
  }

  // ── Solo sensore destro → svolta destra decisa ───────────────
  if (!s.sx && s.dx) {
    motorLeft(SPEED_TURN);
    motorRight(0);
    g_state    = ST_TURN_DX;
    g_lastTurn = ST_TURN_DX;
    return;
  }

  // ── Nessun sensore vede nero → linea persa ───────────────────
  if (g_state != ST_SEARCH_SX && g_state != ST_SEARCH_DX) {
    // Inizia ricerca nell'ultima direzione nota
    g_searchTs = millis();
    g_state    = (g_lastTurn == ST_TURN_SX) ? ST_SEARCH_SX : ST_SEARCH_DX;
    LOGF("[FSM] Linea persa → RICERCA %s\n",
         g_state == ST_SEARCH_SX ? "SX" : "DX");
  }

  // Timeout ricerca → fermati
  if (millis() - g_searchTs > SEARCH_MS) {
    stopMotors();
    g_enabled = false;   // spegne il robot automaticamente
    g_state   = ST_STOP;
    LOGLN("[FSM] Timeout ricerca → STOP forzato");
    return;
  }

  // Ruota sul posto nella direzione di ricerca
  if (g_state == ST_SEARCH_SX) {
    motorLeft(-SPEED_TURN);
    motorRight(SPEED_TURN);
  } else {
    motorLeft(SPEED_TURN);
    motorRight(-SPEED_TURN);
  }
}

// ================================================================
//  Web Server – pagina HTML
// ================================================================

static const char HTML_TEMPLATE[] PROGMEM = R"html(
<!DOCTYPE html>
<html lang="it">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Line Follower</title>
  <style>
    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: 'Segoe UI', system-ui, sans-serif;
      background: #0d0d1a;
      color: #e2e8f0;
      min-height: 100vh;
      display: flex;
      align-items: center;
      justify-content: center;
    }
    .card {
      background: #1a1a2e;
      border: 1px solid #2d2d4e;
      border-radius: 20px;
      padding: 44px 48px;
      box-shadow: 0 20px 60px rgba(0,0,0,.6);
      text-align: center;
      max-width: 380px;
      width: 90%;
    }
    .icon { font-size: 2.8rem; margin-bottom: 12px; }
    h1 { font-size: 1.5rem; font-weight: 700; letter-spacing: .03em; }
    .school { font-size: .8rem; color: #64748b; margin-top: 4px; margin-bottom: 32px; }
    .badge {
      display: inline-flex; align-items: center; gap: 8px;
      padding: 10px 22px; border-radius: 99px;
      font-weight: 700; font-size: .95rem; letter-spacing: .06em;
      margin-bottom: 28px;
    }
    .badge.on  { background: rgba(74,222,128,.15); color: #4ade80;
                 border: 1px solid rgba(74,222,128,.35); }
    .badge.off { background: rgba(248,113,113,.12); color: #f87171;
                 border: 1px solid rgba(248,113,113,.3); }
    .dot { width: 9px; height: 9px; border-radius: 50%; }
    .badge.on  .dot { background: #4ade80; box-shadow: 0 0 6px #4ade80; animation: pulse 1.4s infinite; }
    .badge.off .dot { background: #f87171; }
    @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.4} }
    form { width: 100%; }
    button {
      width: 100%; padding: 15px;
      border: none; border-radius: 12px;
      font-size: 1rem; font-weight: 700; cursor: pointer;
      transition: transform .15s, opacity .15s;
      letter-spacing: .04em;
    }
    button:hover  { opacity: .88; transform: scale(1.025); }
    button:active { transform: scale(.98); }
    .btn-start { background: linear-gradient(135deg,#22c55e,#16a34a); color: #fff; }
    .btn-stop  { background: linear-gradient(135deg,#ef4444,#b91c1c); color: #fff; }
    .footer { margin-top: 28px; font-size: .75rem; color: #475569; }
    .state { margin-top: 10px; font-size: .78rem; color: #64748b; }
  </style>
</head>
<body>
  <div class="card">
    <div class="icon">🤖</div>
    <h1>Line Follower</h1>
    <p class="school">3AEE &mdash; Cigna-Baruffi-Garelli</p>

    <div class="badge %%BADGECLASS%%">
      <span class="dot"></span>%%BADGETEXT%%
    </div>

    <form action="%%ACTION%%" method="POST">
      <button class="%%BTNCLASS%%" type="submit">%%BTNTEXT%%</button>
    </form>

    <p class="state">%%STATETEXT%%</p>
    <p class="footer">Adin Pisica &amp; Filippo Muru</p>
  </div>
</body>
</html>
)html";

static const char* stateLabel(RobotState st) {
  switch (st) {
    case ST_FORWARD:   return "Dritto";
    case ST_CURVE_SX:  return "Curva morbida SX";
    case ST_CURVE_DX:  return "Curva morbida DX";
    case ST_TURN_SX:   return "Svolta SX";
    case ST_TURN_DX:   return "Svolta DX";
    case ST_SEARCH_SX: return "Ricerca linea SX";
    case ST_SEARCH_DX: return "Ricerca linea DX";
    default:           return "Fermo";
  }
}

static String buildPage() {
  String page(HTML_TEMPLATE);
  if (g_enabled) {
    page.replace("%%BADGECLASS%%", "on");
    page.replace("%%BADGETEXT%%",  "ROBOT ATTIVO");
    page.replace("%%ACTION%%",     "/stop");
    page.replace("%%BTNCLASS%%",   "btn-stop");
    page.replace("%%BTNTEXT%%",    "⏹ FERMA ROBOT");
    page.replace("%%STATETEXT%%",  String("Stato: ") + stateLabel(g_state));
  } else {
    page.replace("%%BADGECLASS%%", "off");
    page.replace("%%BADGETEXT%%",  "ROBOT FERMO");
    page.replace("%%ACTION%%",     "/start");
    page.replace("%%BTNCLASS%%",   "btn-start");
    page.replace("%%BTNTEXT%%",    "▶ AVVIA ROBOT");
    page.replace("%%STATETEXT%%",  "In attesa di avvio...");
  }
  return page;
}

// --- Handlers ---

static void onRoot() {
  httpServer.send(200, "text/html", buildPage());
}

static void onStart() {
  g_enabled  = true;
  g_state    = ST_FORWARD;
  g_lastTurn = ST_TURN_DX;
  LOGLN("[WEB] Robot AVVIATO");
  httpServer.sendHeader("Location", "/");
  httpServer.send(303);
}

static void onStop() {
  g_enabled = false;
  stopMotors();
  g_state = ST_STOP;
  LOGLN("[WEB] Robot FERMATO");
  httpServer.sendHeader("Location", "/");
  httpServer.send(303);
}

static void onNotFound() {
  httpServer.send(404, "text/plain", "404 Not Found");
}

// ================================================================
//  Inizializzazione PWM (LEDC ESP32 core v2.x)
// ================================================================
static void initPWM() {
  ledcSetup(CH_IN1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_IN2, PWM_FREQ, PWM_RES);
  ledcSetup(CH_IN3, PWM_FREQ, PWM_RES);
  ledcSetup(CH_IN4, PWM_FREQ, PWM_RES);

  ledcAttachPin(PIN_IN1, CH_IN1);
  ledcAttachPin(PIN_IN2, CH_IN2);
  ledcAttachPin(PIN_IN3, CH_IN3);
  ledcAttachPin(PIN_IN4, CH_IN4);

  stopMotors();
  LOGLN("[PWM] LEDC inizializzato");
}

// ================================================================
//  Inizializzazione WiFi AP
// ================================================================
static void initWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CONN);
  delay(100);
  LOGF("[WiFi] AP: SSID=%s  IP=%s\n",
       AP_SSID, WiFi.softAPIP().toString().c_str());
}

// ================================================================
//  setup()
// ================================================================
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  LOGLN("\n================================================");
  LOGLN(" Line Follower ESP32  –  DEBUG MODE ATTIVO");
  LOGLN("================================================");
#endif

  initPWM();
  initWiFi();

  httpServer.on("/",      HTTP_GET,  onRoot);
  httpServer.on("/start", HTTP_POST, onStart);
  httpServer.on("/stop",  HTTP_POST, onStop);
  httpServer.onNotFound(onNotFound);
  httpServer.begin();

  LOGF("[WEB] Server HTTP avviato – http://%s/\n",
       WiFi.softAPIP().toString().c_str());
  LOGLN("[SYS] Sistema pronto. In attesa di comando via web.");
}

// ================================================================
//  loop()
// ================================================================
void loop() {
  httpServer.handleClient();

  if (g_enabled) {
    updateFollower();
    delay(LOOP_DELAY);
  }
}
