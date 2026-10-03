/*
  TEST BUZZER - CYD ESP32-2432S028(R)
  Buzzer collegato alla presa SPEAK (amplificatore pilotato dal GPIO 26).

  Cosa fa, in ciclo continuo (ogni ciclo ~6 secondi):
    METODO A: 5 toni con il generatore PWM (LEDC), 0,5 s ciascuno:
              1500, 2000, 2400, 2700, 3200 Hz  (lo stesso metodo del firmware)
    pausa 1 s
    METODO B: tono a 2400 Hz per 1,5 s generato "a mano" accendendo e
              spegnendo il pin (non usa il PWM)
    pausa 2 s
  Nel Monitor Seriale (115200) scrive cosa sta suonando in quel momento.

  Come leggere il risultato:
    - suonano A e B  -> collegamento OK
    - suona solo B   -> problema nel PWM con la tua versione della scheda ESP32
    - non suona nulla -> collegamento: saldature, spinotto, presa giusta?
*/

#define PIN_BUZZER 26

void suonaPWM(uint16_t freq, uint16_t durataMs) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_BUZZER, 2000, 8);
  ledcWriteTone(PIN_BUZZER, freq);
  delay(durataMs);
  ledcWriteTone(PIN_BUZZER, 0);
  ledcDetach(PIN_BUZZER);
#else
  ledcSetup(0, 2000, 8);
  ledcAttachPin(PIN_BUZZER, 0);
  ledcWriteTone(0, freq);
  delay(durataMs);
  ledcWriteTone(0, 0);
  ledcDetachPin(PIN_BUZZER);
#endif
}

void suonaAMano(uint16_t freq, uint16_t durataMs) {
  pinMode(PIN_BUZZER, OUTPUT);
  unsigned long mezzoPeriodoUs = 500000UL / freq;
  unsigned long fine = millis() + durataMs;
  while (millis() < fine) {
    digitalWrite(PIN_BUZZER, HIGH);
    delayMicroseconds(mezzoPeriodoUs);
    digitalWrite(PIN_BUZZER, LOW);
    delayMicroseconds(mezzoPeriodoUs);
  }
  digitalWrite(PIN_BUZZER, LOW);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== TEST BUZZER su GPIO 26 (presa SPEAK) ===");
#if defined(ESP_ARDUINO_VERSION_MAJOR)
  Serial.printf("Versione scheda ESP32 (core): %d.%d.%d\n",
                ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
#endif
  // retroilluminazione accesa: cosi' vedi che la scheda e' viva
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);
}

void loop() {
  const uint16_t toni[] = { 1500, 2000, 2400, 2700, 3200 };
  Serial.println("METODO A (PWM):");
  for (uint16_t f : toni) {
    Serial.printf("  %u Hz\n", f);
    suonaPWM(f, 500);
  }
  delay(1000);
  Serial.println("METODO B (a mano): 2400 Hz per 1,5 s");
  suonaAMano(2400, 1500);
  Serial.println("pausa...");
  delay(2000);
}
