// ================================================================
//  Progetto : Line Follower Robot  –  Layout 2 Sensori
//  File     : src/main_esp32.ino
//  Autori   : Adin PISICA, Filippo MURU
//  Classe   : 3AEE  –  A.S. 2025/2026
//  Istituto : Cigna-Baruffi-Garelli  (plesso Cigna)
//  Docente  : Davide Bertolino
//  Board    : ESP32 (WROOM / DevKit)
// ================================================================
//
//  PRINCIPIO DI FUNZIONAMENTO – 2 SENSORI
//  ──────────────────────────────────────
//  Il nastro adesivo nero passa TRA i due sensori.
//  In condizione normale (robot centrato) entrambi vedono bianco.
//
//  Stato sensori → Azione
//  ──────────────────────
//  SX=bianco  DX=bianco  →  Dritto
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
// ================================================================
//  SCHEMA PIN
//  ──────────────────────────────────────────────────────────────
//
//  Driver motori L298N
//    IN1 → GPIO 25  (PWM)  motore SX – avanti
//    IN2 → GPIO 26  (PWM)  motore SX – indietro
//    IN3 → GPIO 27  (PWM)  motore DX – avanti
//    IN4 → GPIO 14  (PWM)  motore DX – indietro
//
//  Sensori IR HW-870 (uscita AO – analogica)
//    SX  → GPIO 34  (ADC1_CH6)
//    DX  → GPIO 35  (ADC1_CH7)
//
//  ⚠ IMPORTANTE: usare solo pin ADC1 (GPIO 32–39) con WiFi attivo.
//    ADC2 viene disabilitato automaticamente dal driver WiFi.
//
//  Alimentazione
//    Motori   : 7–12 V sul morsetto VM del L298N
//    ESP32    : 5 V via USB o regolatore esterno
//    Sensori  : 3.3 V dal pin 3V3 dell'ESP32
//
// ================================================================
//  ACCESSO AL WEB SERVER
//  ──────────────────────────────────────────────────────────────
//  1. Alimenta l'ESP32.
//  2. Connetti il tuo dispositivo all'access point:
//       SSID     : LineFollower-AP
//       Password : robot1234
//  3. Apri il browser e vai a:  http://192.168.4.1/
//  4. Usa il pulsante "AVVIA ROBOT" / "FERMA ROBOT".
//  5. La pagina mostra in tempo reale lo stato dei sensori
//     e lo stato della macchina a stati.
//
// ================================================================
//  CALIBRAZIONE SOGLIA SENSORI
//  ──────────────────────────────────────────────────────────────
//  1. Abilita #define DEBUG (decommentare sotto).
//  2. Apri il Monitor Seriale a 115200 baud.
//  3. Muovi il sensore su bianco → annota il valore ADC.
//  4. Muovi il sensore su nero   → annota il valore ADC.
//  5. Imposta SENS_THRESHOLD = (bianco + nero) / 2.
//  6. Su ESP32 il ADC è 12-bit (0–4095).
//     Esempio: bianco=800, nero=3200 → soglia=2000.
// ================================================================

// ----------------------------------------------------------------
//  DEBUG – decommentare per abilitare il monitor seriale a 115200.
//  TENERE COMMENTATO nel deploy finale.
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
//  CONFIGURAZIONE  –  modifica solo questa sezione
// ================================================================

// --- WiFi Access Point ---
static constexpr char    AP_SSID[]     = "LineFollower-AP";
static constexpr char    AP_PASSWORD[] = "robot1234";   // min 8 caratteri
static constexpr uint8_t AP_CHANNEL    = 1;
static constexpr uint8_t AP_MAX_CONN   = 4;

// --- Pin motori (L298N) ---
static constexpr uint8_t PIN_IN1 = 25;
static constexpr uint8_t PIN_IN2 = 26;
static constexpr uint8_t PIN_IN3 = 27;
static constexpr uint8_t PIN_IN4 = 14;

// --- Pin sensori IR HW-870 (solo ADC1!) ---
static constexpr uint8_t PIN_SENS_SX = 34;  // ADC1_CH6
static constexpr uint8_t PIN_SENS_DX = 35;  // ADC1_CH7

// --- Soglia sensori (ADC 12-bit ESP32: 0–4095) ---
//  Bianco ≈ 300–900  |  Nero ≈ 2000–4095
//  Valore consigliato di partenza: 1800 (regola con debug)
static constexpr uint16_t SENS_THRESHOLD = 1800;

// --- Parametri PWM (LEDC ESP32) ---
static constexpr uint32_t PWM_FREQ = 5000;  // Hz
static constexpr uint8_t  PWM_RES  = 8;     // bit → range 0–255

// --- Canali LEDC ---
enum LedcCh : uint8_t { CH_IN1 = 0, CH_IN2, CH_IN3, CH_IN4 };

// --- Velocità (0–255) ---
static constexpr uint8_t VEL_DRITTO  = 200; // rettilineo
static constexpr uint8_t VEL_EXT     = 200; // motore esterno in svolta
static constexpr uint8_t VEL_INT     =   0; // motore interno in svolta (pivot)
static constexpr uint8_t VEL_RICERCA = 160; // rotazione di ricerca

// --- Timeout ricerca linea (ms) ---
static constexpr uint32_t TIMEOUT_RICERCA_MS = 2500;

// --- Ritardo loop (ms) ---
static constexpr uint32_t LOOP_DELAY_MS = 15;

// ================================================================
//  Tipi
// ================================================================

enum Stato : uint8_t {
  ST_FERMO = 0,
  ST_DRITTO,
  ST_SVOLTA_SX,
  ST_SVOLTA_DX,
  ST_RICERCA_SX,
  ST_RICERCA_DX
};

struct Sensori {
  bool sx;
  bool dx;
  uint16_t rawSx;  // valore ADC grezzo (utile per debug e pagina web)
  uint16_t rawDx;
};

// ================================================================
//  Variabili globali
// ================================================================
static volatile bool g_enabled    = false;
static Stato         g_stato      = ST_FERMO;
static Stato         g_ultimaSvolta = ST_SVOLTA_DX;
static uint32_t      g_tsRicerca  = 0;
static Sensori       g_lastSens   = {false, false, 0, 0}; // per pagina web

WebServer httpServer(80);

// ================================================================
//  Controllo motori
// ================================================================

static inline void pwmSet(LedcCh ch, uint8_t val) {
  ledcWrite(static_cast<uint8_t>(ch), val);
}

static void motoreSX(int16_t spd) {
  if      (spd > 0) { pwmSet(CH_IN1, (uint8_t)min(spd,(int16_t)255)); pwmSet(CH_IN2, 0); }
  else if (spd < 0) { pwmSet(CH_IN1, 0); pwmSet(CH_IN2, (uint8_t)min(-spd,(int16_t)255)); }
  else              { pwmSet(CH_IN1, 0); pwmSet(CH_IN2, 0); }
}

static void motoreDX(int16_t spd) {
  if      (spd > 0) { pwmSet(CH_IN3, (uint8_t)min(spd,(int16_t)255)); pwmSet(CH_IN4, 0); }
  else if (spd < 0) { pwmSet(CH_IN3, 0); pwmSet(CH_IN4, (uint8_t)min(-spd,(int16_t)255)); }
  else              { pwmSet(CH_IN3, 0); pwmSet(CH_IN4, 0); }
}

static void fermaMotori()   { motoreSX(0);             motoreDX(0);            }
static void mDritto()       { motoreSX(VEL_DRITTO);    motoreDX(VEL_DRITTO);   }
static void mSvoltaSX()     { motoreSX(VEL_INT);       motoreDX(VEL_EXT);      }
static void mSvoltaDX()     { motoreSX(VEL_EXT);       motoreDX(VEL_INT);      }
static void mRicercaSX()    { motoreSX(-VEL_RICERCA);  motoreDX(VEL_RICERCA);  }
static void mRicercaDX()    { motoreSX(VEL_RICERCA);   motoreDX(-VEL_RICERCA); }

// ================================================================
//  Lettura sensori
// ================================================================

static Sensori leggiSensori() {
  uint16_t vSx = (analogRead(PIN_SENS_SX) + analogRead(PIN_SENS_SX)) >> 1;
  uint16_t vDx = (analogRead(PIN_SENS_DX) + analogRead(PIN_SENS_DX)) >> 1;

  LOGF("[ADC] SX=%4u DX=%4u THR=%u | SX=%s DX=%s\n",
       vSx, vDx, SENS_THRESHOLD,
       vSx > SENS_THRESHOLD ? "NERO " : "BLANC",
       vDx > SENS_THRESHOLD ? "NERO " : "BLANC");

  return { vSx > SENS_THRESHOLD, vDx > SENS_THRESHOLD, vSx, vDx };
}

// ================================================================
//  Macchina a stati – logica line follower 2 sensori
// ================================================================

static void aggiornaSeguiLinea() {
  const Sensori s = leggiSensori();
  g_lastSens = s;

  // ── Entrambi NERI → fine pista / incrocio ────────────────────
  if (s.sx && s.dx) {
    fermaMotori();
    g_stato   = ST_FERMO;
    g_enabled = false;   // auto-stop
    LOGLN("[FSM] Fine pista / incrocio → FERMO");
    return;
  }

  // ── Entrambi BIANCHI → linea al centro, dritto ───────────────
  if (!s.sx && !s.dx) {
    mDritto();
    g_stato = ST_DRITTO;
    return;
  }

  // ── Solo SX NERO → robot slittato a destra, correggi a SX ────
  if (s.sx && !s.dx) {
    mSvoltaSX();
    g_stato        = ST_SVOLTA_SX;
    g_ultimaSvolta = ST_SVOLTA_SX;
    LOGLN("[FSM] SX nero → SVOLTA_SX");
    return;
  }

  // ── Solo DX NERO → robot slittato a sinistra, correggi a DX ──
  if (!s.sx && s.dx) {
    mSvoltaDX();
    g_stato        = ST_SVOLTA_DX;
    g_ultimaSvolta = ST_SVOLTA_DX;
    LOGLN("[FSM] DX nero → SVOLTA_DX");
    return;
  }

  // ── Fallback: avvia ricerca nell'ultima direzione nota ────────
  if (g_stato != ST_RICERCA_SX && g_stato != ST_RICERCA_DX) {
    g_tsRicerca = millis();
    g_stato     = (g_ultimaSvolta == ST_SVOLTA_SX)
                  ? ST_RICERCA_SX : ST_RICERCA_DX;
    LOGF("[FSM] Linea persa → RICERCA_%s\n",
         g_stato == ST_RICERCA_SX ? "SX" : "DX");
  }

  if (millis() - g_tsRicerca > TIMEOUT_RICERCA_MS) {
    fermaMotori();
    g_enabled = false;
    g_stato   = ST_FERMO;
    LOGLN("[FSM] Timeout ricerca → FERMO forzato");
    return;
  }

  if (g_stato == ST_RICERCA_SX) { mRicercaSX(); }
  else                           { mRicercaDX(); }
}

// ================================================================
//  Web Server – interfaccia HTML
// ================================================================

static const char HTML_TEMPLATE[] PROGMEM = R"html(
<!DOCTYPE html>
<html lang="it">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta http-equiv="refresh" content="2">
  <title>Line Follower</title>
  <style>
    *,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
    body{
      font-family:'Segoe UI',system-ui,sans-serif;
      background:#0d0d1a;color:#e2e8f0;
      min-height:100vh;display:flex;align-items:center;justify-content:center;
    }
    .card{
      background:#1a1a2e;border:1px solid #2d2d4e;border-radius:20px;
      padding:36px 40px;box-shadow:0 20px 60px rgba(0,0,0,.6);
      text-align:center;max-width:400px;width:92%;
    }
    .icon{font-size:2.6rem;margin-bottom:10px}
    h1{font-size:1.4rem;font-weight:700;letter-spacing:.03em}
    .school{font-size:.78rem;color:#64748b;margin-top:4px;margin-bottom:28px}
    .badge{
      display:inline-flex;align-items:center;gap:8px;
      padding:9px 20px;border-radius:99px;
      font-weight:700;font-size:.9rem;letter-spacing:.06em;margin-bottom:24px;
    }
    .badge.on {background:rgba(74,222,128,.15);color:#4ade80;border:1px solid rgba(74,222,128,.35)}
    .badge.off{background:rgba(248,113,113,.12);color:#f87171;border:1px solid rgba(248,113,113,.3)}
    .dot{width:9px;height:9px;border-radius:50%}
    .badge.on  .dot{background:#4ade80;box-shadow:0 0 6px #4ade80;animation:pulse 1.4s infinite}
    .badge.off .dot{background:#f87171}
    @keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}

    .sensors{
      display:flex;gap:16px;justify-content:center;margin-bottom:24px;
    }
    .sensor-box{
      flex:1;padding:14px 10px;border-radius:12px;border:1px solid #2d2d4e;
    }
    .sensor-box .lbl{font-size:.7rem;color:#64748b;text-transform:uppercase;letter-spacing:.08em;margin-bottom:6px}
    .sensor-box .val{font-size:1.1rem;font-weight:700}
    .sensor-box.nero {background:rgba(248,113,113,.08);border-color:rgba(248,113,113,.3);color:#f87171}
    .sensor-box.blanc{background:rgba(148,163,184,.06);border-color:#2d2d4e;color:#94a3b8}
    .sensor-box .raw{font-size:.7rem;color:#475569;margin-top:4px}

    form{width:100%}
    button{
      width:100%;padding:14px;border:none;border-radius:12px;
      font-size:.95rem;font-weight:700;cursor:pointer;
      transition:transform .15s,opacity .15s;letter-spacing:.04em;
    }
    button:hover {opacity:.88;transform:scale(1.025)}
    button:active{transform:scale(.98)}
    .btn-start{background:linear-gradient(135deg,#22c55e,#16a34a);color:#fff}
    .btn-stop {background:linear-gradient(135deg,#ef4444,#b91c1c);color:#fff}
    .state{margin-top:12px;font-size:.76rem;color:#64748b}
    .footer{margin-top:20px;font-size:.72rem;color:#475569}
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

    <div class="sensors">
      <div class="sensor-box %%SX_CLASS%%">
        <div class="lbl">Sensore SX</div>
        <div class="val">%%SX_STATO%%</div>
        <div class="raw">ADC: %%SX_RAW%%</div>
      </div>
      <div class="sensor-box %%DX_CLASS%%">
        <div class="lbl">Sensore DX</div>
        <div class="val">%%DX_STATO%%</div>
        <div class="raw">ADC: %%DX_RAW%%</div>
      </div>
    </div>

    <form action="%%ACTION%%" method="POST">
      <button class="%%BTNCLASS%%" type="submit">%%BTNTEXT%%</button>
    </form>

    <p class="state">%%STATETEXT%%</p>
    <p class="footer">Adin Pisica &amp; Filippo Muru &mdash; pagina auto-aggiornata ogni 2&nbsp;s</p>
  </div>
</body>
</html>
)html";

static const char* statoLabel(Stato st) {
  switch (st) {
    case ST_DRITTO:     return "Dritto";
    case ST_SVOLTA_SX:  return "Svolta SX";
    case ST_SVOLTA_DX:  return "Svolta DX";
    case ST_RICERCA_SX: return "Ricerca linea SX";
    case ST_RICERCA_DX: return "Ricerca linea DX";
    default:            return "Fermo";
  }
}

static String buildPage() {
  String p(HTML_TEMPLATE);

  // Badge stato robot
  if (g_enabled) {
    p.replace("%%BADGECLASS%%", "on");
    p.replace("%%BADGETEXT%%",  "ROBOT ATTIVO");
    p.replace("%%ACTION%%",     "/stop");
    p.replace("%%BTNCLASS%%",   "btn-stop");
    p.replace("%%BTNTEXT%%",    "&#9209; FERMA ROBOT");
    p.replace("%%STATETEXT%%",  String("Stato: ") + statoLabel(g_stato));
  } else {
    p.replace("%%BADGECLASS%%", "off");
    p.replace("%%BADGETEXT%%",  "ROBOT FERMO");
    p.replace("%%ACTION%%",     "/start");
    p.replace("%%BTNCLASS%%",   "btn-start");
    p.replace("%%BTNTEXT%%",    "&#9654; AVVIA ROBOT");
    p.replace("%%STATETEXT%%",  "In attesa di avvio...");
  }

  // Sensori
  p.replace("%%SX_CLASS%%",  g_lastSens.sx ? "nero" : "blanc");
  p.replace("%%SX_STATO%%",  g_lastSens.sx ? "NERO"  : "BIANCO");
  p.replace("%%SX_RAW%%",    String(g_lastSens.rawSx));
  p.replace("%%DX_CLASS%%",  g_lastSens.dx ? "nero" : "blanc");
  p.replace("%%DX_STATO%%",  g_lastSens.dx ? "NERO"  : "BIANCO");
  p.replace("%%DX_RAW%%",    String(g_lastSens.rawDx));

  return p;
}

// --- Handlers HTTP ---
static void onRoot()     { httpServer.send(200, "text/html", buildPage()); }
static void onNotFound() { httpServer.send(404, "text/plain", "404 Not Found"); }

static void onStart() {
  g_enabled      = true;
  g_stato        = ST_DRITTO;
  g_ultimaSvolta = ST_SVOLTA_DX;
  LOGLN("[WEB] Robot AVVIATO");
  httpServer.sendHeader("Location", "/");
  httpServer.send(303);
}

static void onStop() {
  g_enabled = false;
  fermaMotori();
  g_stato = ST_FERMO;
  LOGLN("[WEB] Robot FERMATO");
  httpServer.sendHeader("Location", "/");
  httpServer.send(303);
}

// ================================================================
//  Inizializzazione PWM LEDC (core v2.x)
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
  fermaMotori();
  LOGLN("[PWM] LEDC inizializzato");
}

// ================================================================
//  Inizializzazione WiFi AP
// ================================================================
static void initWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CONN);
  delay(100);
  LOGF("[WiFi] AP avviato – SSID=%s  IP=%s\n",
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
  LOGLN(" Line Follower 2-Sensori – ESP32 – DEBUG ON");
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
  LOGLN("[SYS] Pronto – connettiti a LineFollower-AP e apri http://192.168.4.1/");
}

// ================================================================
//  loop()
// ================================================================
void loop() {
  httpServer.handleClient();

  if (g_enabled) {
    aggiornaSeguiLinea();
    delay(LOOP_DELAY_MS);
  }
}
