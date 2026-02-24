// --- Pin del Sensore ---
const int pinDigitale = 2; // DO collegato al pin digitale 2
const int pinAnalogico = A0; // AO collegato al pin analogico A0
int ledPin = 5; // pin led

void setup() {
  pinMode(pinDigitale, INPUT);
  pinMode(ledPin, OUTPUT);
  
  // Inizializza il Monitor Seriale a 9600 baud
  Serial.begin(9600);
  Serial.println("--- INIZIO DEBUG SENSORE HW-870 ---");
  Serial.println("Avvicina il sensore al bianco e al nero per vedere i cambiamenti.");
  delay(2000); // Pausa di 2 secondi per farti leggere il messaggio
}

void loop() {
  int valDigitale = 0;
  // Lettura dei sensori
  int valAnalogico = analogRead(pinAnalogico); // Valore da 0 a 1023
  if (valAnalogico > 512) { valDigitale = 1; } else { valDigitale = 0; }

  // Stampa formattata per il Monitor Seriale
  Serial.print("Valore Analogico: ");
  // Aggiungo degli spazi per allineare il testo
  if(valAnalogico < 100) Serial.print(" ");
  if(valAnalogico < 10)  Serial.print(" ");
  Serial.print(valAnalogico);
  
  Serial.print("  |  Valore Digitale: ");
  Serial.print(valDigitale);


  // Aggiungiamo un'etichetta per rendere la lettura immediata
  if (valDigitale == 1) {
    Serial.println("  -> (Rilevato SCURO/NERO)");
    digitalWrite(ledPin, LOW);
  } else {
    Serial.println("  -> (Rilevato CHIARO/BIANCO)");
    digitalWrite(ledPin, HIGH);
  }

  // Pausa di 250ms per non far scorrere i numeri troppo velocemente
  delay(250);
}
