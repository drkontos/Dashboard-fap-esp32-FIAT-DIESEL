/*
  DASHBOARD FAP (DPF) - ESP32 CYD + adattatore OBD2 Bluetooth LE
  Istruzioni complete: README.md

  Monitor del filtro antiparticolato per diesel FCA con centralina Bosch
  EDC17C69. Testato su Fiat Tipo 1.6 Multijet (2019).
  Scheda: CYD ESP32-2432S028(R) - Adattatore: ELM327 BLE (testato Vgate iCar Pro)

  SOLO LETTURA / MONITORAGGIO. Nessuna modifica ai parametri della centralina.
  L'unica scrittura possibile e' la cancellazione dei codici errore, su
  richiesta, con conferma e solo a motore spento. Uso a proprio rischio.

  FUNZIONI
    - pagina Guida: temperatura e intasamento FAP in grande (cifre a 7
      segmenti) con barra colorata + 4 riquadri: km dall'ultima rigenerazione,
      pressione differenziale, temperatura motore, numero rigenerazioni
    - barra in alto: verde "Nessuna rigenerazione" oppure rossa lampeggiante
      "RIGENERAZIONE xx%" / "NON SPEGNERE!" con striscia di avanzamento
    - pagina Dettagli: tutti i valori con decimali + tempo medio di lettura
    - pagina Opzioni: attiva/disattiva ogni avviso sonoro e sceglie quanto
      resta acceso lo schermo dopo il collegamento (scelte salvate)
    - pagina Errori: legge i codici errore della centralina motore con stato
      (ATTIVO / MEMORIZZATO / IN ATTESA / STORICO) e li cancella a motore
      spento, con conferma e verifica
    - test guidato del sensore di pressione differenziale FAP (utile con
      P2002/P2453): minimo -> 3000 giri in folle -> motore spento; il test
      sopravvive al riavvio della scheda
    - controllo automatico errori a ogni collegamento (riquadro "N ERRORI")
    - schermo: si spegne dopo 10 s senza tocchi, si riaccende al tocco;
      resta acceso durante rigenerazione, con intasamento >= 98% e nei test
    - avvisi sonori (buzzer passivo sulla presa SPEAK, GPIO 26): avvio,
      inizio rigenerazione, promemoria ogni 5 min, fine rigenerazione,
      intasamento 98%
    - log eventi su memoria interna (LittleFS)

  NAVIGAZIONE
    Guida -> tocco -> Dettagli -> tocco -> Opzioni -> pulsante "Errori"

  RIGENERAZIONE IN CORSO = avanzamento rigenerazione sopra 0,1%

  PID (UDS 22, header 18DA10F1, risposta da 18DAF110, CAN 29 bit 500k):
    22380B Avanzamento rigenerazione  ((A*256)+B)*100/65535     %
    2218DE Temperatura FAP            ((A*256)+B)*0.02 - 40     C
    2218E4 Intasamento FAP            ((A*256)+B)*1000/65535    %
    2218E2 Pressione differenziale    ((A*256)+B) - 32767       mbar
    221003 Temperatura motore         ((A*256)+B)*0.02 - 40     C
    223807 Km dall'ultima rigen.      ((A*65536)+(B*256)+C)*0.1 km
    2218A4 Numero rigenerazioni       (A*256)+B

  COMANDI DAL MONITOR SERIALE (115200 baud):
    DUMP     -> stampa il log degli eventi
    SUONI    -> fa sentire tutti gli avvisi sonori in fila
    CANCELLA -> cancella il log (chiede conferma: scrivi SI)
    STOP     -> mette in pausa la ricerca dell'adattatore (START la riprende)
    DIMENTICA -> dimentica l'adattatore scelto dalla lista
    STORICO  -> stampa lo storico delle rigenerazioni
    AZZERA_STORICO -> cancella lo storico delle rigenerazioni

  Licenza MIT - vedi LICENSE
*/

#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLESecurity.h>
#include <Preferences.h>
#include <LittleFS.h>

// ============================================================================
// TIPI - devono stare qui in cima, prima di qualunque funzione: l'IDE Arduino
// inserisce i prototipi delle funzioni prima della prima funzione dello sketch.
// ============================================================================

struct RisultatoPID {
  bool valido;
  float valore;
};

// Ordine = priorita' (all'avvio vengono letti in quest'ordine)
// P_OLIO e P_BATT sono "di prova": presi dai progetti Alfa Giulia; se la
// centralina non li conosce vengono esclusi da soli.
enum IndicePID { P_AVANZ, P_TFAP, P_INTAS, P_PDIFF, P_TMOT, P_KM, P_NRIG, P_OLIO, P_BATT, NUM_PID };
const int PRIMO_PID_DI_PROVA = P_OLIO;

struct DefinizionePID {
  const char* did;             // 4 cifre esadecimali dopo il "22"
  uint8_t nByte;               // byte di dati nella risposta
  float scala;                 // valore = raw * scala + offset
  float offset;
  unsigned long intervalloMs;  // ogni quanto rileggerlo
  const char* nome;            // per il log
};

struct CodiceErrore {
  char codice[10];             // "P2002-00" (UDS) oppure "P2002" (OBD2)
  uint8_t stato;               // bit di stato: 0x01 attivo, 0x04 in attesa, 0x08 memorizzato
};

enum StatoPaginaErrori { ERR_MAI_LETTI, ERR_LISTA, ERR_NON_RISPONDE, ERR_CONFERMA, ERR_ESITO };

struct Nota {                  // una nota del buzzer: frequenza (0 = pausa) e durata
  uint16_t freq;
  uint16_t durataMs;
};

struct MisuraTest {            // una misura del test sensore FAP
  float somma;
  int campioni;
  float massimo;
  float minimo;
  float giriAlMassimo;
};

struct RecordRigen {           // una rigenerazione nello storico
  uint16_t numero;             // numero dal contatore della centralina (0 = sconosciuto)
  uint16_t kmPrima10;          // km dalla rigenerazione precedente x10 (0xFFFF = sconosciuto)
  uint16_t durataS;            // durata vista dal dispositivo
  int16_t  tMax;               // temperatura FAP massima
  uint8_t  intasInizio;        // % intasamento all'inizio (255 = sconosciuto)
  uint8_t  avanzMax;           // % avanzamento massimo visto
  uint8_t  esito;              // 0 completa, 1 interrotta, 2 in corso
  uint8_t  interruzioni;       // spegnimenti avvenuti durante la rigenerazione
};

struct StatoPID {
  RisultatoPID valore;         // ultimo valore valido (o non valido)
  unsigned long ultimaLettura; // millis() dell'ultima richiesta
  uint8_t falliti;             // letture fallite consecutive
  bool urgente;                // da leggere al prossimo giro
};

// ============================================================================
// CONFIGURAZIONE - MODIFICA QUI
// ============================================================================

// Nome Bluetooth dell'adattatore (Vgate iCar Pro = "IOS-Vlink").
const char* ELM_DEVICE_NAME = "IOS-Vlink";
// MAC dell'adattatore: lascia "" per cercarlo per nome, oppure scrivi il tuo
// (es. "AA:BB:CC:DD:EE:FF") se nelle vicinanze ce ne sono piu' di uno.
// Se non lo trova, compare a schermo la lista dei dispositivi Bluetooth:
// tocca il tuo adattatore e viene ricordato.
const char* ELM_MAC_ADDRESS = "";

#define UUID_SERVICE        "0000FFE0-0000-1000-8000-00805F9B34FB"
#define UUID_CHARACTERISTIC "0000FFE1-0000-1000-8000-00805F9B34FB"

const uint32_t BLE_PIN = 1234;

// Ricerca dell'adattatore: dura AL MASSIMO questi secondi, ma si ferma appena
// lo vede (di solito 1-3 s). Un adattatore appena "svegliato" puo' essere lento.
const int DURATA_RICERCA_S = 15;

//                                    DID     byte scala             offset     ogni(ms) nome
const DefinizionePID DEF_PID[NUM_PID] = {
  /* P_AVANZ */ { "380B", 2, 100.0f / 65535.0f,  0.0f,      500,  "RIGEN_AVANZ" },
  /* P_TFAP  */ { "18DE", 2, 0.02f,             -40.0f,     1000, "TEMP_FAP"    },
  /* P_INTAS */ { "18E4", 2, 1000.0f / 65535.0f, 0.0f,      2000, "INTASAMENTO" },
  /* P_PDIFF */ { "18E2", 2, 1.0f,             -32767.0f,   500,  "PRESS_DIFF"  },
  /* P_TMOT  */ { "1003", 2, 0.02f,             -40.0f,     5000, "TEMP_MOTORE" },
  /* P_KM    */ { "3807", 3, 0.1f,               0.0f,      10000, "KM_DA_RIGEN" },
  /* P_NRIG  */ { "18A4", 2, 1.0f,               0.0f,      30000, "NUM_RIGEN"   },
  /* P_OLIO  */ { "3813", 2, 100.0f / 65535.0f,  0.0f,      60000, "DEGRADO_OLIO" },
  /* P_BATT  */ { "1955", 2, 0.0005f,            0.0f,      10000, "BATTERIA"    },
};

// Soglie colori (modificabili quando avremo visto a che % la Tipo rigenera)
const float SOGLIA_INTAS_ARANCIO = 75.0f;   // % sotto cui la barra e' verde
const float SOGLIA_INTAS_ROSSO   = 95.0f;   // % da cui la barra diventa rossa
const float SOGLIA_TFAP_ARANCIO  = 400.0f;  // C da cui la temperatura e' arancio

// Buzzer sulla presa SPEAK (amplificatore della CYD collegato al GPIO 26)
const bool SUONI_ATTIVI = true;
#define PIN_BUZZER 26
const unsigned long PROMEMORIA_RIGEN_MS = 5UL * 60UL * 1000UL;   // ogni 5 minuti

// Schermo: si spegne dopo questo tempo senza tocchi, e resta sempre acceso
// durante una rigenerazione o con l'intasamento pari o sopra la soglia.
// Spegnimento schermo senza tocchi: si sceglie dalla pagina Opzioni
// (10 s, 30 s, 1 min, 2 min, 5 min oppure Mai = sempre acceso).
const int SCHERMO_TIMEOUT_DEFAULT = 0;   // indice: 0 = 10 secondi
// Appena collegato all'adattatore lo schermo resta acceso per un po':
// la durata si sceglie dalla pagina Opzioni (10 s, 30 s, 1 min, 2 min, 5 min).
const int SCHERMO_DOPO_COLLEGAMENTO_DEFAULT = 2;   // indice: 2 = 1 minuto
unsigned long collegatoDaMs = 0;
const float SOGLIA_INTAS_SCHERMO = 98.0f;
#define PIN_RETROILLUMINAZIONE 21

// Dopo quante letture fallite di fila un valore diventa "--"
const uint8_t FALLIMENTI_PER_NASCONDERE = 3;

// Se non arriva nessun dato valido da tanto, mostra "In attesa dei dati..."
const unsigned long ATTESA_DATI_MS = 8000;

// ============================================================================
// COLORI E POSIZIONI (identici all'anteprima)
// ============================================================================

#define C_BG       0x0000
#define C_TILE     0x2124
#define C_LABEL    0x9CD3
#define C_TEXT     0xFFFF
#define C_GREEN    0x2E8B
#define C_ORANGE   0xFD20
#define C_RED      0xE8E4
#define C_DARKRED  0x7800
#define C_BARBG    0x39E7
#define C_YELLOW   0xFFE0

// colori delle schermate di connessione (come nelle fasi precedenti)
#define COL_BG          TFT_BLACK
#define COL_BOX         0x18E3
#define COL_LABEL       0x4D3F
#define COL_VALUE_OK    TFT_WHITE
#define COL_VALUE_WARN  0xFD20

const int BAR_H = 34;
const int STRIP_Y = 34, STRIP_H = 6;
const int BIG_Y = 44, BIG_H = 106, BIG_W = 154;
const int BIG_X[2] = { 4, 162 };
const int SMALL_Y[2] = { 156, 198 };
const int SMALL_H = 38, SMALL_W = 154;
const int DET_Y0 = 40, DET_PASSO = 26;
const int PULS_Y = 196, PULS_H = 40;   // pulsanti in basso (errori, test, opzioni, ricerca)

// ============================================================================
// OGGETTI GLOBALI
// ============================================================================

#define XPT2046_CS   33
#define XPT2046_IRQ  36

TFT_eSPI tft = TFT_eSPI();
SPIClass touchSPI = SPIClass(VSPI);
XPT2046_Touchscreen touch(XPT2046_CS, XPT2046_IRQ);

BLEClient* bleClient = nullptr;
BLERemoteCharacteristic* bleCaratteristica = nullptr;
String bufferRicevuto = "";
volatile bool promptRicevuto = false;   // arrivato il ">" dell'adattatore

bool elmConnesso = false;
String macScelto = "";
// pausa della ricerca adattatore (pulsanti STOP / START / Scegli o comando dal PC)
bool ricercaInPausa = false;
bool schermataPausaDisegnata = false;
bool sceltaManuale = false;          // "Scegli": cerca e mostra subito la lista
bool eraToccatoRicerca = false;
bool confermaCancellaLog = false;    // CANCELLA dal monitor seriale chiede conferma
Preferences memoriaPermanente;
fs::File fileLog;

StatoPID statoPID[NUM_PID];
bool pidNonSupportato[NUM_PID];   // PID di prova che la centralina non conosce
bool pidProvaRegistrato[NUM_PID]; // primo valore dei PID di prova gia' scritto nel log
int statoSuffisso = -1;          // -1 = da provare, 0 = non supportato, 1 = ok
int tentativiSuffisso = 0;
float msMedioLettura = 0;
unsigned long ultimoDatoValidoMs = 0;

bool rigenInCorso = false;
unsigned long inizioRigenMs = 0;

// ---------- stato dello schermo ----------
int pagina = 0;                  // 0 = guida, 1 = dettagli, 2 = errori, 3 = test FAP, 4 = opzioni, 5 = storico, 6 = avviso

// ---------- impostazioni suoni (salvate in memoria) ----------
enum { S_AVVIO, S_RIGEN_INIZIO, S_PROMEMORIA, S_RIGEN_FINE, S_INTAS_98, NUM_SUONI };
const char* NOMI_SUONI[NUM_SUONI] = {
  "Suono all'accensione", "Inizio rigenerazione", "Promemoria ogni 5 minuti",
  "Fine rigenerazione", "Intasamento al 98%"
};
const char* CHIAVI_SUONI[NUM_SUONI] = { "sAvv", "sRIni", "sProm", "sRFin", "s98" };
bool suonoAbilitato[NUM_SUONI] = { true, true, true, true, true };
// opzione "schermo acceso dopo il collegamento"
const int NUM_OPZ_SCHERMO = 5;
const unsigned long OPZ_SCHERMO_MS[NUM_OPZ_SCHERMO] = { 10000, 30000, 60000, 120000, 300000 };
const char* OPZ_SCHERMO_TESTO[NUM_OPZ_SCHERMO] = { "10 s", "30 s", "1 min", "2 min", "5 min" };
int opzSchermo = SCHERMO_DOPO_COLLEGAMENTO_DEFAULT;
// opzione "spegnimento schermo" (0 = mai, sempre acceso)
const int NUM_OPZ_TIMEOUT = 6;
const unsigned long OPZ_TIMEOUT_MS[NUM_OPZ_TIMEOUT] = { 10000, 30000, 60000, 120000, 300000, 0 };
const char* OPZ_TIMEOUT_TESTO[NUM_OPZ_TIMEOUT] = { "10 s", "30 s", "1 min", "2 min", "5 min", "Mai" };
int opzTimeout = SCHERMO_TIMEOUT_DEFAULT;
Preferences prefSuoni;

// storico rigenerazioni (memoria permanente "dpfstor")
const int MAX_STORICO = 10;
RecordRigen storico[MAX_STORICO];
int numStorico = 0;
RecordRigen rigenAperta;               // rigenerazione in corso (salvata ogni 30 s)
bool rigenApertaValida = false;
bool rigenApertaDaRiavvio = false;     // trovata aperta all'accensione: da capire com'e' finita
bool segmentoRigenAttivo = false;
uint16_t baseDurataS = 0;
unsigned long inizioSegmentoMs = 0;
unsigned long ultimoSalvataggioRigenMs = 0;
int versioneStorico = 0;
int versioneStoricoDisegnata = -1;
String cachePiedeStorico = "";
Preferences prefStorico;
String avvisoRiga1 = "", avvisoRiga2 = "";
bool testDaRiprendere = false;   // riavvio avvenuto durante il test sensore
bool schermoAcceso = true;
unsigned long ultimaAttivitaMs = 0;
bool intasamentoAlto = false;
bool eraForzatoAcceso = false;
bool paginaDaRidisegnare = true;
bool eraToccato = false;
unsigned long ultimoTocco = 0;

const int NUM_SLOT = 14;         // 0-1 grandi, 2-5 piccoli, 6-12 dettagli, 13 tempo
String cacheTesto[NUM_SLOT];
uint16_t cacheColore[NUM_SLOT];
int cacheBarraPieno[2];
uint16_t cacheBarraColore[2];
int cacheStatoBarra = -1;
int cachePercentuale = -1;
int cacheStriscia = -2;
int cacheModoRiquadro0 = -1;

int larghezzaUnitaGrande[2];
int larghezzaUnitaPiccola[4];
int larghezzaEtichettaPiccola[4];
int larghezzaUnitaRiga[NUM_PID];

// ---------- errori centralina ----------
const int MAX_ERRORI = 20;
CodiceErrore errori[MAX_ERRORI];
int numErrori = 0;
bool erroriLetti = false;            // almeno una lettura riuscita
String metodoErrori = "";            // "UDS" oppure "OBD2"
bool controlloAutomaticoFatto = false;
int statoPaginaErrori = ERR_MAI_LETTI;
int paginaListaErrori = 0;
unsigned long confermaDalMs = 0;
String esitoRiga1 = "";
String esitoRiga2 = "";
uint16_t esitoColore = 0xFFFF;

// ---------- suoni ----------
// Il buzzer piccolo suona piu' forte tra 2000 e 2700 Hz circa
const Nota SUONO_AVVIO[]        = { {2400, 150}, {0, 80}, {2700, 150} };
const Nota SUONO_RIGEN_INIZIO[] = { {1800, 150}, {0, 70}, {2200, 150}, {0, 70}, {2700, 350} };
const Nota SUONO_PROMEMORIA[]   = { {2400, 100}, {0, 100}, {2400, 100} };
const Nota SUONO_RIGEN_FINE[]   = { {2700, 150}, {0, 70}, {1800, 350} };
const Nota SUONO_INTAS_ALTO[]   = { {1500, 300}, {0, 150}, {1500, 300} };

const Nota* melodiaCorrente = nullptr;
int lunghezzaMelodia = 0;
int notaCorrente = 0;
unsigned long inizioNotaMs = 0;
unsigned long ultimoPromemoriaMs = 0;
bool eraIntasAltoSuono = false;

#define SUONA(m) suona(m, sizeof(m) / sizeof(m[0]))


// ============================================================================
// BLE: callback e PIN
// ============================================================================

static void callbackNotifica(BLERemoteCharacteristic* characteristic,
                              uint8_t* dati, size_t lunghezza, bool isNotify) {
  for (size_t i = 0; i < lunghezza; i++) {
    bufferRicevuto += (char)dati[i];
    if (dati[i] == '>') promptRicevuto = true;
  }
}

class GestorePinBLE : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() override { return BLE_PIN; }
  void onPassKeyNotify(uint32_t pass_key) override {}
  bool onConfirmPIN(uint32_t pass_key) override { return true; }
  bool onSecurityRequest() override { return true; }
  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) override {}
};

// ============================================================================
// SETUP E LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Avvio Dashboard FAP");

  tft.init();
  tft.setRotation(1);
  pinMode(PIN_RETROILLUMINAZIONE, OUTPUT);
  accendiSchermo();
  avviaBuzzer();
  caricaImpostazioniSuoni();
  if (SUONI_ATTIVI && suonoAbilitato[S_AVVIO]) {
    // Suono d'avvio "bloccante" (subito dopo parte la connessione Bluetooth).
    // Mezzo secondo di attesa: l'amplificatore della CYD appena acceso ha
    // bisogno di un attimo, altrimenti un suono breve si perde.
    delay(500);
    for (const Nota& n : SUONO_AVVIO) {
      impostaTono(n.freq);
      delay(n.durataMs);
    }
    impostaTono(0);
  }
  tft.fillScreen(COL_BG);
  disegnaMessaggio("Avvio in corso...");

  touchSPI.begin(25, 39, 32, XPT2046_CS);
  touch.begin(touchSPI);
  touch.setRotation(1);

  memoriaPermanente.begin("dpf3", false);   // ricorda l'adattatore scelto
  macScelto = memoriaPermanente.getString("mac", "");

  if (!LittleFS.begin(true)) {
    Serial.println("ERRORE: impossibile montare la memoria interna (LittleFS).");
    disegnaMessaggio("ERRORE memoria\ninterna!");
    while (true) delay(1000);
  }
  scriviLog("AVVIO");
  caricaStorico();          // dopo LittleFS: puo' scrivere nel log
  testDaRiprendere = caricaTestInCorso();

  BLEDevice::init("ESP32-DPF");
  BLEDevice::setSecurityCallbacks(new GestorePinBLE());
  BLESecurity* sicurezza = new BLESecurity();
  sicurezza->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
  sicurezza->setCapability(ESP_IO_CAP_OUT);
  sicurezza->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  sicurezza->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  connettiELM();
}

bool setupPIDFatto = false;

void loop() {
  gestisciSeriale();

  if (!elmConnesso) {
    if (ricercaInPausa) {        // ricerca ferma (es. collegato al PC)
      mostraPausaRicerca();
      return;
    }
    connettiELM();
    setupPIDFatto = false;
    if (!elmConnesso && !ricercaInPausa) {
      // in attesa di riprovare: STOP ferma la ricerca, un tocco riaccende lo schermo
      unsigned long inizioAttesa = millis();
      while (millis() - inizioAttesa < 3000) {
        if (gestisciPulsantiRicerca()) break;
        gestisciSeriale();
        if (ricercaInPausa) break;
        gestisciSuono();
        gestisciSchermo();
        delay(30);
      }
    }
    return;
  }

  if (!setupPIDFatto) {
    eseguiSetupPID();
    setupPIDFatto = true;
    paginaDaRidisegnare = true;
    if (testDaRiprendere) {      // riavvio durante il test: torno li'
      testDaRiprendere = false;
      pagina = 3;
      valutaTest();
    }
  }

  gestisciTocco();
  eseguiProssimaLettura();
  aggiornaStatoRigenerazione();
  aggiornaRigenAperta();
  controllaRigenDaRiavvio();
  controlloErroriAutomatico();
  controllaAvvisiSonori();
  gestisciSuono();
  gestisciSchermo();
  aggiornaSchermo();

  if (bleClient == nullptr || !bleClient->isConnected()) {
    elmConnesso = false;
    scriviLog("CONNESSIONE_PERSA");
  }
}

void gestisciSeriale() {
  if (!Serial.available()) return;
  String riga = Serial.readStringUntil('\n');
  riga.trim();
  if (riga.length() == 0) return;
  bool attesaConferma = confermaCancellaLog;
  confermaCancellaLog = false;
  if (riga.equalsIgnoreCase("START")) {
    ricercaInPausa = false;
    schermataPausaDisegnata = false;
    Serial.println("Ricerca adattatore ripresa.");
    return;
  }
  if (riga.equalsIgnoreCase("STOP") || (!elmConnesso && !ricercaInPausa)) {
    // un comando dal PC mentre cerca l'adattatore: sei al banco, la ricerca va in pausa
    if (!elmConnesso && !ricercaInPausa) {
      ricercaInPausa = true;
      schermataPausaDisegnata = false;
      scriviLog("RICERCA_IN_PAUSA,da_seriale");
      Serial.println("Ricerca adattatore in PAUSA (scrivi START per riprendere).");
    }
    if (riga.equalsIgnoreCase("STOP")) return;
  }
  if (riga.equalsIgnoreCase("DUMP")) {
    stampaLogSuSeriale();
  } else if (riga.equalsIgnoreCase("SUONI")) {
    provaTuttiISuoni();
  } else if (riga.equalsIgnoreCase("STORICO")) {
    stampaStoricoSuSeriale();
  } else if (riga.equalsIgnoreCase("AZZERA_STORICO")) {
    numStorico = 0;
    prefStorico.remove("h");
    prefStorico.putUChar("n", 0);
    versioneStorico++;
    scriviLog("STORICO_AZZERATO");
    Serial.println("Storico rigenerazioni azzerato.");
  } else if (riga.equalsIgnoreCase("DIMENTICA")) {
    macScelto = "";
    memoriaPermanente.remove("mac");
    scriviLog("ADATTATORE_DIMENTICATO,da_seriale");
    Serial.println("Adattatore salvato dimenticato: al prossimo collegamento cerca di nuovo per nome/MAC.");
  } else if (riga.equalsIgnoreCase("CANCELLA")) {
    confermaCancellaLog = true;
    Serial.println("Vuoi davvero cancellare il log eventi? Scrivi SI per confermare (altro = annulla).");
  } else if (riga.equalsIgnoreCase("SI") && attesaConferma) {
    LittleFS.remove("/eventi_fap.csv");
    Serial.println("Log cancellato.");
  } else if (attesaConferma) {
    Serial.println("Cancellazione del log annullata.");
  } else {
    Serial.println("Comandi: DUMP, STORICO, SUONI, STOP, START, DIMENTICA, AZZERA_STORICO, CANCELLA");
  }
}

void gestisciTocco() {
  bool toccato = touch.tirqTouched() && touch.touched();
  if (toccato && !eraToccato && !schermoAcceso) {
    accendiSchermo();               // primo tocco: solo riaccensione
    ultimoTocco = millis();
    eraToccato = true;
    return;
  }
  if (toccato && !eraToccato && millis() - ultimoTocco > 400) {
    ultimoTocco = millis();
    ultimaAttivitaMs = millis();
    TS_Point p = touch.getPoint();
    int px = limitaInt(map(p.x, 300, 3800, 0, 320), 0, 319);
    int py = limitaInt(map(p.y, 300, 3800, 0, 240), 0, 239);
    Serial.println("Tocco: x=" + String(px) + " y=" + String(py));
    if (pagina == 2) {
      toccoPaginaErrori(px, py);
    } else if (pagina == 3) {
      toccoPaginaTest(px, py);
    } else if (pagina == 4) {
      toccoPaginaSuoni(px, py);
    } else if (pagina == 6) {          // avviso: un tocco lo chiude
      pagina = 0;
      paginaDaRidisegnare = true;
    } else {
      // Guida -> Dettagli -> Storico -> Opzioni (-> Errori col pulsante)
      pagina = (pagina == 0) ? 1 : (pagina == 1 ? 5 : 4);
      paginaDaRidisegnare = true;
    }
    toccato = touch.tirqTouched() && touch.touched();   // dopo operazioni lunghe
  }
  eraToccato = toccato;
}

// ============================================================================
// CONNESSIONE BLE (identica alle fasi precedenti)
// ============================================================================

int tentativiFalliti = 0;
const int MAX_TENTATIVI_PRIMA_DI_CAMBIARE = 3;

void segnalaTentativoFallito() {
  tentativiFalliti++;
  if (tentativiFalliti >= MAX_TENTATIVI_PRIMA_DI_CAMBIARE && macScelto.length() > 0) {
    macScelto = "";
    memoriaPermanente.remove("mac");
    tentativiFalliti = 0;
  }
}

// e' l'adattatore giusto? (MAC scelto dalla lista, MAC in configurazione o nome)
bool eIlMioAdattatore(BLEAdvertisedDevice& dispositivo) {
  String indirizzo = dispositivo.getAddress().toString().c_str();
  indirizzo.toUpperCase();
  if (macScelto.length() > 0 && indirizzo == macScelto) return true;
  String macAtteso = String(ELM_MAC_ADDRESS);
  macAtteso.toUpperCase();
  if (macAtteso.length() > 0 && indirizzo == macAtteso) return true;
  return dispositivo.haveName() &&
         String(dispositivo.getName().c_str()) == String(ELM_DEVICE_NAME);
}

BLEAdvertisedDevice* adattatoreVisto = nullptr;

// chiamata per ogni dispositivo Bluetooth visto durante la ricerca:
// appena compare il nostro adattatore, la ricerca si ferma
class CallbackRicerca : public BLEAdvertisedDeviceCallbacks {
 public:
  void onResult(BLEAdvertisedDevice dispositivo) override {
    if (adattatoreVisto != nullptr || sceltaManuale) return;
    if (eIlMioAdattatore(dispositivo)) {
      adattatoreVisto = new BLEAdvertisedDevice(dispositivo);
      BLEDevice::getScan()->stop();
    }
  }
};
CallbackRicerca callbackRicerca;

BLEAdvertisedDevice* trovaAdattatore() {
  if (sceltaManuale) {
    disegnaMessaggio("Ricerca dispositivi\nBluetooth...\n(poi tocca il tuo\nadattatore)");
  } else {
    disegnaMessaggio(String("Ricerca ") + ELM_DEVICE_NAME + "...\n(tentativo " +
                     String(tentativiFalliti + 1) + ", max " + String(DURATA_RICERCA_S) + " s)");
    disegnaPulsantiRicerca();
  }

  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(&callbackRicerca);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(99);
  adattatoreVisto = nullptr;
  unsigned long inizioRicerca = millis();
  // a blocchi di 3 s: tra un blocco e l'altro si controllano STOP e comandi dal PC
  BLEScanResults* risultati = nullptr;
  int durata = sceltaManuale ? 6 : DURATA_RICERCA_S;
  for (int t = 0; t < durata && adattatoreVisto == nullptr; t += 3) {
    risultati = scan->start(3, t > 0);
    gestisciSeriale();
    gestisciPulsantiRicerca();
    if (ricercaInPausa) {
      if (adattatoreVisto != nullptr) { delete adattatoreVisto; adattatoreVisto = nullptr; }
      scan->clearResults();
      scriviLog("RICERCA_IN_PAUSA");
      return nullptr;
    }
  }

  if (adattatoreVisto != nullptr) {
    scriviLog("ADATTATORE_TROVATO,dopo_ms=" + String(millis() - inizioRicerca));
    scan->clearResults();
    return adattatoreVisto;
  }

  scriviLog("ADATTATORE_NON_VISTO,dispositivi_BLE=" + String(risultati->getCount()));

  // Non visto: di solito l'adattatore dorme o e' occupato dal telefono, e
  // la lista non servirebbe (bloccherebbe 30 s a ogni giro). La lista per
  // scegliere a mano compare solo al 3o tentativo fallito, poi ogni 5.
  bool mostraLista = sceltaManuale || (tentativiFalliti % 5 == 2);
  sceltaManuale = false;
  if (!mostraLista) {
    scan->clearResults();
    return nullptr;
  }

  BLEAdvertisedDevice* scelto = scegliDaLista(risultati);
  scan->clearResults();

  if (scelto != nullptr) {
    macScelto = scelto->getAddress().toString().c_str();
    macScelto.toUpperCase();
    memoriaPermanente.putString("mac", macScelto);
  }

  return scelto;
}

// ---------- pulsanti durante la ricerca dell'adattatore ----------
// Ricerca in corso: in basso STOP. In pausa: START (riprendi) e Scegli
// (cerca e mostra la lista dei dispositivi). Utile al banco col PC.

void disegnaPulsantiRicerca() {
  tft.fillRect(0, PULS_Y - 4, 320, 240 - (PULS_Y - 4), COL_BG);
  if (!ricercaInPausa) {
    disegnaPulsante(80, PULS_Y, 160, PULS_H, "STOP", C_DARKRED, C_TEXT);
  } else {
    disegnaPulsante(4, PULS_Y, 150, PULS_H, "START", C_GREEN, C_BG);
    disegnaPulsante(166, PULS_Y, 150, PULS_H, "Scegli", C_TILE, C_TEXT);
  }
}

// ritorna true se ha cambiato lo stato (pausa / ripresa / scelta)
bool gestisciPulsantiRicerca() {
  bool toccato = touch.tirqTouched() && touch.touched();
  bool cambiato = false;
  if (toccato && !eraToccatoRicerca) {
    accendiSchermo();
    ultimaAttivitaMs = millis();
    TS_Point p = touch.getPoint();
    int px = limitaInt(map(p.x, 300, 3800, 0, 320), 0, 319);
    int py = limitaInt(map(p.y, 300, 3800, 0, 240), 0, 239);
    if (py >= PULS_Y - 10) {
      if (!ricercaInPausa) {
        ricercaInPausa = true;
        scriviLog("RICERCA_IN_PAUSA,tocco");
      } else if (px < 160) {
        ricercaInPausa = false;
        scriviLog("RICERCA_RIPRESA,tocco");
      } else {
        ricercaInPausa = false;
        sceltaManuale = true;
        scriviLog("RICERCA_SCELTA_MANUALE,tocco");
      }
      schermataPausaDisegnata = false;
      cambiato = true;
    }
  }
  eraToccatoRicerca = toccato;
  return cambiato;
}

void mostraPausaRicerca() {
  if (!schermataPausaDisegnata) {
    disegnaMessaggio("Ricerca adattatore\nIN PAUSA\n\nSTART = riprendi\nScegli = lista Bluetooth");
    disegnaPulsantiRicerca();
    schermataPausaDisegnata = true;
  }
  gestisciPulsantiRicerca();
  gestisciSuono();
  gestisciSchermo();
  delay(30);
}

BLEAdvertisedDevice* scegliDaLista(BLEScanResults* risultati) {
  int numero = risultati->getCount();
  accendiSchermo();   // serve toccare lo schermo per scegliere

  tft.fillScreen(COL_BG);
  paginaDaRidisegnare = true;
  tft.setTextColor(COL_VALUE_WARN, COL_BG);
  tft.setTextSize(1);
  tft.setCursor(5, 3);
  tft.println("Tocca il tuo adattatore:");

  if (numero == 0) {
    tft.setTextColor(COL_LABEL, COL_BG);
    tft.setCursor(5, 20);
    tft.println("Nessun dispositivo BLE visto.");
    delay(3000);
    return nullptr;
  }

  int massimo = numero < 9 ? numero : 9;
  int yStart = 18;
  int altezzaRiga = 24;

  for (int i = 0; i < massimo; i++) {
    BLEAdvertisedDevice d = risultati->getDevice(i);
    String riga = d.getAddress().toString().c_str();
    if (d.haveName()) {
      riga += " ";
      riga += d.getName().c_str();
    }
    int y = yStart + i * altezzaRiga;
    tft.drawRect(2, y, 316, altezzaRiga - 3, COL_BOX);
    tft.setTextColor(COL_VALUE_OK, COL_BG);
    tft.setCursor(6, y + 6);
    tft.println(riga);
  }

  unsigned long inizio = millis();
  while (millis() - inizio < 30000) {
    if (touch.tirqTouched() && touch.touched()) {
      TS_Point p = touch.getPoint();
      int px = map(p.x, 300, 3800, 0, 320);
      int py = map(p.y, 300, 3800, 0, 240);
      (void)px;
      for (int i = 0; i < massimo; i++) {
        int y = yStart + i * altezzaRiga;
        if (py >= y && py <= y + altezzaRiga - 3) {
          delay(300);
          return new BLEAdvertisedDevice(risultati->getDevice(i));
        }
      }
    }
    delay(50);
  }
  return nullptr;
}

void connettiELM() {
  disegnaMessaggio(String("Connessione a\n") + ELM_DEVICE_NAME + "...");

  BLEAdvertisedDevice* dispositivo = trovaAdattatore();
  if (dispositivo == nullptr && ricercaInPausa) {
    elmConnesso = false;
    schermataPausaDisegnata = false;
    return;
  }
  if (dispositivo == nullptr) {
    elmConnesso = false;
    scriviLog("CONNESSIONE_FALLITA,adattatore_non_trovato");
    segnalaTentativoFallito();
    if (tentativiFalliti >= 2) {
      disegnaMessaggio("Adattatore non trovato.\n\n- quadro acceso?\n- telefono collegato\n  all'adattatore?\n  (spegni il suo BT)\nRiprovo...");
    } else {
      disegnaMessaggio("Adattatore non\ntrovato. Riprovo...");
    }
    disegnaPulsantiRicerca();
    return;
  }

  if (bleClient == nullptr) {
    bleClient = BLEDevice::createClient();
  }

  String indirizzoCollegato = dispositivo->getAddress().toString().c_str();
  indirizzoCollegato.toUpperCase();
  bool connesso = bleClient->connect(dispositivo);
  delete dispositivo;

  if (!connesso) {
    elmConnesso = false;
    scriviLog("CONNESSIONE_FALLITA,connessione_bluetooth_rifiutata");
    disegnaMessaggio("Connessione fallita.\nRiprovo...");
    segnalaTentativoFallito();
    return;
  }

  tentativiFalliti = 0;

  bleCaratteristica = nullptr;
  BLERemoteService* servizio = bleClient->getService(UUID_SERVICE);
  if (servizio != nullptr) {
    bleCaratteristica = servizio->getCharacteristic(UUID_CHARACTERISTIC);
  }

  if (bleCaratteristica == nullptr) {
    disegnaMessaggio("Scoperta automatica\nservizi BLE...");
    bleCaratteristica = trovaCaratteristicaAutomatica(bleClient);
  }

  if (bleCaratteristica == nullptr) {
    scriviLog("CONNESSIONE_FALLITA,nessuna_caratteristica_BLE");
    disegnaMessaggio("Nessuna\ncaratteristica\nBLE utilizzabile");
    bleClient->disconnect();
    elmConnesso = false;
    return;
  }

  if (bleCaratteristica->canNotify()) {
    bleCaratteristica->registerForNotify(callbackNotifica);
  } else if (bleCaratteristica->canIndicate()) {
    bleCaratteristica->registerForNotify(callbackNotifica, false);
  }

  elmConnesso = true;
  collegatoDaMs = millis();
  scriviLog("CONNESSO," + indirizzoCollegato);
  if (!inizializzaELM()) {
    // collegato via Bluetooth ma non risponde ai comandi OBD: non e' un
    // adattatore ELM327 (es. scelto per sbaglio dalla lista). Lo dimentico.
    scriviLog("ADATTATORE_NON_RISPONDE,dimenticato," + indirizzoCollegato);
    disegnaMessaggio("Questo dispositivo\nnon risponde come\nadattatore OBD:\nlo dimentico.\n\nRiprovo...");
    if (macScelto.length() > 0) {
      macScelto = "";
      memoriaPermanente.remove("mac");
    }
    bleClient->disconnect();
    elmConnesso = false;
    segnalaTentativoFallito();
    delay(2000);
    return;
  }
}

BLERemoteCharacteristic* trovaCaratteristicaAutomatica(BLEClient* client) {
  std::map<std::string, BLERemoteService*>* servizi = client->getServices();
  BLERemoteCharacteristic* candidata = nullptr;

  for (auto& coppiaServizio : *servizi) {
    BLERemoteService* svc = coppiaServizio.second;
    std::map<std::string, BLERemoteCharacteristic*>* caratteristiche = svc->getCharacteristics();
    for (auto& coppiaCarat : *caratteristiche) {
      BLERemoteCharacteristic* c = coppiaCarat.second;
      bool saScrivere = c->canWrite() || c->canWriteNoResponse();
      bool saNotificare = c->canNotify() || c->canIndicate();
      if (candidata == nullptr && saScrivere && saNotificare) {
        candidata = c;
      }
    }
  }
  return candidata;
}

// ============================================================================
// DIALOGO CON L'ADATTATORE ELM327
// ============================================================================

// restituisce false se dall'altra parte non risponde nessun ELM327
bool inizializzaELM() {
  String rispZ = inviaComandoELM("ATZ", 2000);    delay(1000);
  inviaComandoELM("ATE0", 1000);   delay(100);
  inviaComandoELM("ATL0", 1000);   delay(100);
  inviaComandoELM("ATS0", 1000);   delay(100);   // niente spazi: risposte piu' corte
  // Protocollo 7 (CAN 29 bit 500k); la "A" = se non va, ricerca automatica
  String rispSP = inviaComandoELM("ATSPA7", 1000);
  scriviLog("AT_SETUP,ATSPA7," + pulisci(rispSP));
  rispZ.trim();
  rispSP.trim();
  return rispZ.length() > 0 || rispSP.length() > 0;
}

// Invia un comando e aspetta il ">" finale. Se "atteso" non e' vuoto, scarta
// le risposte "vecchie" arrivate in ritardo (quelle che non contengono la
// risposta attesa ne' un errore) e continua ad aspettare quella giusta.
String inviaComandoAtteso(const String& comando, unsigned long timeoutMs, const String& atteso) {
  if (bleCaratteristica == nullptr) return "";

  bufferRicevuto = "";
  promptRicevuto = false;
  String daInviare = comando + "\r";
  bleCaratteristica->writeValue((uint8_t*)daInviare.c_str(), daInviare.length());

  // risposta negativa per questo servizio: "7F" + primi 2 caratteri del comando
  String negativa = "7F" + comando.substring(0, 2);

  unsigned long inizio = millis();
  while (millis() - inizio < timeoutMs) {
    if (promptRicevuto) {
      if (atteso.length() == 0) break;
      String c = bufferRicevuto;
      c.replace(" ", "");
      if (c.indexOf(atteso) != -1 || c.indexOf(negativa) != -1 || c.indexOf("NODATA") != -1 ||
          c.indexOf("ERROR") != -1 || c.indexOf("STOPPED") != -1 || c.indexOf("UNABLE") != -1 ||
          c.indexOf("?") != -1) {
        break;
      }
      // risposta di una richiesta precedente: la scarto e aspetto ancora
      bufferRicevuto = "";
      promptRicevuto = false;
    }
    gestisciSuono();
    delay(1);
  }

  String risposta = bufferRicevuto;
  risposta.trim();
  return risposta;
}

String inviaComandoELM(const String& comando, unsigned long timeoutMs) {
  return inviaComandoAtteso(comando, timeoutMs, String(""));
}

void eseguiSetupPID() {
  disegnaMessaggio("Configurazione\nindirizzo CAN...");

  String rispSH = inviaComandoELM("ATSH18DA10F1", 1000);
  scriviLog("AT_SETUP,ATSH18DA10F1," + pulisci(rispSH));
  String rispCRA = inviaComandoELM("ATCRA18DAF110", 1000);
  scriviLog("AT_SETUP,ATCRA18DAF110," + pulisci(rispCRA));
  String rispST = inviaComandoELM("ATST32", 1000);   // attesa max 200 ms
  scriviLog("AT_SETUP,ATST32," + pulisci(rispST));

  for (int i = 0; i < NUM_PID; i++) {
    statoPID[i].valore.valido = false;
    statoPID[i].valore.valore = 0;
    statoPID[i].ultimaLettura = 0;
    statoPID[i].falliti = 0;
    statoPID[i].urgente = true;     // leggi tutto subito, in ordine di priorita'
  }
  statoSuffisso = -1;
  tentativiSuffisso = 0;
  controlloAutomaticoFatto = false;
  testaSuffissoRisposte();
}

// Prova "2218A41": se l'adattatore capisce il "1" finale (= aspetta una sola
// risposta) ogni lettura diventa molto piu' veloce.
void testaSuffissoRisposte() {
  tentativiSuffisso++;
  String r = inviaComandoAtteso("2218A41", 1000, "6218A4");
  String c = r;
  c.replace(" ", "");
  if (c.indexOf("6218A4") != -1) {
    statoSuffisso = 1;
  } else if (c.indexOf("?") != -1 || tentativiSuffisso >= 3) {
    statoSuffisso = 0;
  } else {
    statoSuffisso = -1;   // centralina non ancora pronta: riprovo dopo
  }
  scriviLog("TEST_SUFFISSO," + pulisci(r) + ",stato=" + String(statoSuffisso));
}

// ============================================================================
// LETTURA DEI PID
// ============================================================================

// Estrae i byte dati da una risposta Mode 22 tipo "6218DE12C8"
bool estraiDatiRisposta(const String& risposta, const String& did4cifre, uint8_t* out, int nByte) {
  String r = risposta;
  r.replace(" ", "");
  r.replace("\r", "");
  r.replace("\n", "");
  r.replace(">", "");

  String prefisso = "62" + did4cifre;
  int pos = r.indexOf(prefisso);
  if (pos == -1) return false;

  String dati = r.substring(pos + prefisso.length());
  if ((int)dati.length() < nByte * 2) return false;

  for (int i = 0; i < nByte; i++) {
    String coppia = dati.substring(i * 2, i * 2 + 2);
    out[i] = (uint8_t)strtol(coppia.c_str(), NULL, 16);
  }
  return true;
}

RisultatoPID leggiPID(int indice) {
  const DefinizionePID& d = DEF_PID[indice];
  RisultatoPID r = { false, 0 };

  String comando = String("22") + d.did;
  if (statoSuffisso == 1) comando += "1";

  unsigned long t0 = millis();
  String risp = inviaComandoAtteso(comando, 400, String("62") + d.did);
  unsigned long dt = millis() - t0;
  msMedioLettura = (msMedioLettura == 0) ? dt : msMedioLettura * 0.9f + dt * 0.1f;

  uint8_t dati[4];
  int n = d.nByte > 4 ? 4 : d.nByte;
  long raw = 0;
  if (estraiDatiRisposta(risp, String(d.did), dati, n)) {
    for (int i = 0; i < n; i++) raw = (raw << 8) | dati[i];
    r.valore = raw * d.scala + d.offset;
    r.valido = true;
  }
  if (indice >= PRIMO_PID_DI_PROVA) {
    String c = unisciFrame(risp);
    int p = c.indexOf("7F22");
    if (!r.valido && p != -1 && p + 6 <= (int)c.length()) {
      String nrc = c.substring(p + 4, p + 6);
      if (nrc == "31" || nrc == "11" || nrc == "12" || nrc == "33") {
        pidNonSupportato[indice] = true;
        scriviLog(String("PID_NON_SUPPORTATO,") + d.nome + ",nrc=" + nrc);
      }
    }
    if (r.valido) {
      bool plausibile = (indice == P_OLIO) ? (r.valore >= 0 && r.valore <= 100)
                                           : (r.valore >= 6 && r.valore <= 18);
      if (!plausibile) {
        pidNonSupportato[indice] = true;
        r.valido = false;
        scriviLog(String("PID_VALORE_STRANO,") + d.nome + ",raw=" + String(raw));
      } else if (!pidProvaRegistrato[indice]) {
        pidProvaRegistrato[indice] = true;
        scriviLog(String("PID_PROVA,") + d.nome + ",valore=" + String(r.valore, 2) + ",raw=" + String(raw));
      }
    }
  }
  return r;
}

// Sceglie il PID da leggere: prima quelli "urgenti" (in ordine di priorita'),
// poi quello piu' in ritardo rispetto alla sua frequenza. -1 = nessuno.
int scegliProssimoPID() {
  for (int i = 0; i < NUM_PID; i++) {
    if (pidNonSupportato[i]) { statoPID[i].urgente = false; continue; }
    if (statoPID[i].urgente) return i;
  }
  int migliore = -1;
  long ritardoMigliore = -1;
  unsigned long adesso = millis();
  for (int i = 0; i < NUM_PID; i++) {
    if (pidNonSupportato[i]) continue;
    long ritardo = (long)(adesso - statoPID[i].ultimaLettura) - (long)DEF_PID[i].intervalloMs;
    if (ritardo >= 0 && ritardo > ritardoMigliore) {
      ritardoMigliore = ritardo;
      migliore = i;
    }
  }
  return migliore;
}

void eseguiProssimaLettura() {
  if (pagina == 3) {           // test sensore FAP: letture dedicate e piu' veloci
    eseguiLetturaTest();
    return;
  }
  int i = scegliProssimoPID();
  if (i < 0) return;

  RisultatoPID r = leggiPID(i);
  StatoPID& s = statoPID[i];
  s.ultimaLettura = millis();
  s.urgente = false;

  if (r.valido) {
    controllaEventiContatori(i, s.valore, r);
    s.valore = r;
    s.falliti = 0;
    ultimoDatoValidoMs = millis();
    if (statoSuffisso == -1) testaSuffissoRisposte();
  } else {
    if (s.falliti < 255) s.falliti++;
    if (s.falliti >= FALLIMENTI_PER_NASCONDERE) s.valore.valido = false;
  }
}

void forzaLettura(int indice) {
  statoPID[indice].urgente = true;
}

bool datiPresenti() {
  return ultimoDatoValidoMs != 0 && (millis() - ultimoDatoValidoMs) < ATTESA_DATI_MS;
}

// Valore da mostrare: se la centralina non risponde da un po', tutto "--"
RisultatoPID mostrato(int indice) {
  RisultatoPID r = statoPID[indice].valore;
  if (!datiPresenti()) r.valido = false;
  return r;
}

// ============================================================================
// RIGENERAZIONE
// ============================================================================

// Rigenerazione in corso = avanzamento sopra 0,1%. Lo leggiamo ogni mezzo
// secondo, quindi basta questo. (Il vecchio controllo "km appena azzerati"
// e' stato tolto: poteva far diventare rossa la barra DOPO la fine di una
// rigenerazione, quando la centralina azzera il contatore dei km.)
bool fineRigenDaRegistrare = false;
unsigned long durataUltimaRigenS = 0;

void aggiornaStatoRigenerazione() {
  const RisultatoPID& av = statoPID[P_AVANZ].valore;
  // senza una lettura valida lo stato resta com'e' (es. subito dopo una
  // riconnessione): prima chiudeva per sbaglio la rigenerazione
  bool ora = av.valido ? (av.valore > 0.1f) : rigenInCorso;

  if (ora != rigenInCorso) {
    rigenInCorso = ora;
    if (ora) {
      inizioRigenMs = millis();
      ultimoPromemoriaMs = millis();
      if (suonoAbilitato[S_RIGEN_INIZIO]) SUONA(SUONO_RIGEN_INIZIO);
      scriviLog("RIGEN_INIZIO," + fotografiaValori());
      iniziaORiprendiRigen();
    } else {
      if (suonoAbilitato[S_RIGEN_FINE]) SUONA(SUONO_RIGEN_FINE);
      durataUltimaRigenS = (millis() - inizioRigenMs) / 1000;
      fineRigenDaRegistrare = true;   // registro dopo aver riletto km e contatore
      chiudiRigen(0);
    }
    // km e numero di rigenerazioni cambiano proprio adesso: rileggili subito
    forzaLettura(P_KM);
    forzaLettura(P_NRIG);
  }

  if (fineRigenDaRegistrare && !statoPID[P_KM].urgente && !statoPID[P_NRIG].urgente) {
    scriviLog("RIGEN_FINE,durata_s=" + String(durataUltimaRigenS) + "," + fotografiaValori());
    fineRigenDaRegistrare = false;
  }
}

// Registra i cambiamenti "a scatto" di km e contatore: ci diranno se la tua
// centralina azzera i km all'inizio o alla fine della rigenerazione.
void controllaEventiContatori(int indice, const RisultatoPID& precedente, const RisultatoPID& nuovo) {
  if (!precedente.valido || !nuovo.valido) return;
  if (indice == P_KM && nuovo.valore < precedente.valore - 1.0f) {
    scriviLog("KM_AZZERATI,da=" + String(precedente.valore, 1) + ",a=" + String(nuovo.valore, 1) +
              ",rigen_in_corso=" + String(rigenInCorso ? 1 : 0));
  }
  if (indice == P_NRIG && nuovo.valore > precedente.valore + 0.5f) {
    scriviLog("NRIG_AUMENTATO,da=" + String((int)precedente.valore) + ",a=" + String((int)nuovo.valore) +
              ",rigen_in_corso=" + String(rigenInCorso ? 1 : 0));
  }
}

String valoreLog(int indice, int decimali) {
  const RisultatoPID& r = statoPID[indice].valore;
  return r.valido ? String(r.valore, (unsigned int)decimali) : String("ND");
}

String fotografiaValori() {
  return "tfap=" + valoreLog(P_TFAP, 0) + ";intas=" + valoreLog(P_INTAS, 1) +
         ";avanz=" + valoreLog(P_AVANZ, 1) + ";km=" + valoreLog(P_KM, 1) +
         ";nrig=" + valoreLog(P_NRIG, 0) + ";pdiff=" + valoreLog(P_PDIFF, 0) +
         ";tmot=" + valoreLog(P_TMOT, 0);
}

// ============================================================================
// FORMATTAZIONE E COLORI
// ============================================================================

String formatta(const RisultatoPID& r, int decimali) {
  if (!r.valido) return String("--");
  float v = r.valore;
  if (decimali == 0 && v > -0.5f && v < 0.5f) v = 0.0f;   // evita "-0"
  char buf[16];
  snprintf(buf, sizeof(buf), "%.*f", decimali, v);
  return String(buf);
}

String formattaIntasamento(const RisultatoPID& r) {
  if (!r.valido) return String("--");
  return formatta(r, r.valore < 99.95f ? 1 : 0);   // "100.0" non ci sta
}

uint16_t coloreIntasamento(float p) {
  if (p < SOGLIA_INTAS_ARANCIO) return C_GREEN;
  if (p < SOGLIA_INTAS_ROSSO) return C_ORANGE;
  return C_RED;
}

int limitaInt(long v, int minimo, int massimo) {
  if (v < minimo) return minimo;
  if (v > massimo) return massimo;
  return (int)v;
}

float limita01(float f) {
  if (f < 0) return 0;
  if (f > 1) return 1;
  return f;
}

// Scrive l'unita' allineata a destra. "gC" = gradi Celsius con cerchietto.
// Ritorna la larghezza occupata in pixel.
int disegnaUnita(int xDestra, int yCentro, const char* unita, uint16_t colore, uint16_t sfondo, uint8_t font) {
  tft.setTextColor(colore, sfondo);
  tft.setTextPadding(0);
  tft.setTextDatum(MR_DATUM);
  if (strcmp(unita, "gC") == 0) {
    int w = tft.drawString("C", xDestra, yCentro, font);
    int r = (font == 4) ? 3 : 2;
    int yTop = yCentro - tft.fontHeight(font) / 2;
    tft.drawCircle(xDestra - w - r - 1, yTop + r + 3, r, colore);
    return w + 2 * r + 3;
  }
  if (unita[0] == '\0') return 0;
  return tft.drawString(unita, xDestra, yCentro, font);
}

void azzeraCacheSchermo() {
  for (int i = 0; i < NUM_SLOT; i++) {
    cacheTesto[i] = "\x01";   // valore impossibile: forza il ridisegno
    cacheColore[i] = 0;
  }
  for (int i = 0; i < 2; i++) {
    cacheBarraPieno[i] = -1;
    cacheBarraColore[i] = 0;
  }
  cacheStatoBarra = -1;
  cachePercentuale = -1;
  cacheStriscia = -2;
  cacheModoRiquadro0 = -1;
}

// ============================================================================
// SCHERMO: GESTIONE GENERALE
// ============================================================================

void aggiornaSchermo() {
  if (paginaDaRidisegnare) {
    azzeraCacheSchermo();
    paginaDaRidisegnare = false;
    if (pagina == 0) disegnaPaginaGuidaFissa();
    else if (pagina == 1) disegnaPaginaDettagliFissa();
    else if (pagina == 2) disegnaPaginaErrori();
    else if (pagina == 3) disegnaPaginaTestFissa();
    else if (pagina == 5) disegnaPaginaStorico();
    else if (pagina == 6) disegnaPaginaAvviso();
    else disegnaPaginaSuoni();
  }
  if (pagina == 0) aggiornaPaginaGuida();
  else if (pagina == 1) aggiornaPaginaDettagli();
  else if (pagina == 2) aggiornaPaginaErrori();
  else if (pagina == 3) aggiornaPaginaTest();
  else if (pagina == 5) aggiornaPaginaStorico();
}

// ============================================================================
// PAGINA GUIDA
// ============================================================================

// stato: 0 = in attesa dati, 1 = ok, 2 = rigenerazione (frase A), 3 = (frase B)
//        100 + N = ok ma con N errori in centralina
void disegnaBarraStato(int stato) {
  tft.setTextPadding(0);
  if (stato >= 100) {
    int n = stato - 100;
    tft.fillRect(0, 0, 320, BAR_H, C_TILE);
    tft.fillCircle(17, BAR_H / 2, 7, C_GREEN);
    tft.setTextColor(C_GREEN, C_TILE);
    tft.setTextDatum(ML_DATUM);
    tft.drawString("Nessuna rigen.", 34, BAR_H / 2, 4);
    tft.fillRoundRect(222, 6, 92, 22, 6, C_ORANGE);
    tft.setTextColor(C_BG, C_ORANGE);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(String(n) + (n == 1 ? " ERRORE" : " ERRORI"), 268, BAR_H / 2, 2);
  } else if (stato == 0) {
    tft.fillRect(0, 0, 320, BAR_H, C_TILE);
    tft.setTextColor(C_ORANGE, C_TILE);
    tft.setTextDatum(ML_DATUM);
    tft.drawString("In attesa dei dati...", 12, BAR_H / 2, 4);
  } else if (stato == 1) {
    tft.fillRect(0, 0, 320, BAR_H, C_TILE);
    tft.fillCircle(17, BAR_H / 2, 7, C_GREEN);
    tft.setTextColor(C_GREEN, C_TILE);
    tft.setTextDatum(ML_DATUM);
    tft.drawString("Nessuna rigenerazione", 34, BAR_H / 2, 4);
  } else {
    uint16_t sfondo = (stato == 2) ? C_RED : C_DARKRED;
    tft.fillRect(0, 0, 320, BAR_H, sfondo);
    tft.setTextColor(C_TEXT, sfondo);
    if (stato == 2) {
      tft.setTextDatum(ML_DATUM);
      tft.drawString("RIGENERAZIONE", 12, BAR_H / 2, 4);
    } else {
      tft.setTextDatum(MC_DATUM);
      tft.drawString("NON SPEGNERE!", 160, BAR_H / 2, 4);
    }
  }
}

void disegnaPercentualeBarra(int percentuale) {
  tft.setTextColor(C_TEXT, C_RED);
  tft.setTextDatum(MR_DATUM);
  tft.setTextPadding(70);
  tft.drawString(String(percentuale) + "%", 308, BAR_H / 2, 4);
  tft.setTextPadding(0);
}

// pieno = pixel riempiti della striscia (0-320), -1 = striscia nascosta
void disegnaStriscia(int pieno) {
  if (pieno < 0) {
    tft.fillRect(0, STRIP_Y, 320, STRIP_H, C_BG);
    return;
  }
  tft.fillRect(0, STRIP_Y, pieno, STRIP_H, C_TEXT);
  tft.fillRect(pieno, STRIP_Y, 320 - pieno, STRIP_H, C_DARKRED);
}

void disegnaIntestazionePiccola(int indice, const char* etichetta, const char* unita) {
  int x = BIG_X[indice % 2];
  int y = SMALL_Y[indice / 2];
  tft.fillRoundRect(x, y, SMALL_W, SMALL_H, 6, C_TILE);
  tft.setTextColor(C_LABEL, C_TILE);
  tft.setTextDatum(ML_DATUM);
  tft.setTextPadding(0);
  larghezzaEtichettaPiccola[indice] = tft.drawString(etichetta, x + 8, y + SMALL_H / 2, 2);
  larghezzaUnitaPiccola[indice] = disegnaUnita(x + SMALL_W - 8, y + SMALL_H / 2, unita, C_LABEL, C_TILE, 2);
  cacheTesto[2 + indice] = "\x01";   // il valore va riscritto
}

void disegnaPaginaGuidaFissa() {
  tft.fillScreen(C_BG);
  const char* etichette[2] = { "TEMPERATURA FAP", "INTASAMENTO FAP" };
  const char* unita[2] = { "gC", "%" };
  for (int i = 0; i < 2; i++) {
    int x = BIG_X[i];
    tft.fillRoundRect(x, BIG_Y, BIG_W, BIG_H, 8, C_TILE);
    tft.setTextColor(C_LABEL, C_TILE);
    tft.setTextDatum(TL_DATUM);
    tft.setTextPadding(0);
    tft.drawString(etichette[i], x + 10, BIG_Y + 7, 2);
    larghezzaUnitaGrande[i] = disegnaUnita(x + BIG_W - 10, BIG_Y + 42, unita[i], C_LABEL, C_TILE, 4);
    tft.fillRoundRect(x + 10, BIG_Y + 84, BIG_W - 20, 10, 4, C_BARBG);
  }
  // il riquadro 0 dipende dalla rigenerazione: lo disegna aggiornaPaginaGuida()
  disegnaIntestazionePiccola(1, "P. diff.", "mbar");
  disegnaIntestazionePiccola(2, "Motore", "gC");
  disegnaIntestazionePiccola(3, "Rigen. tot.", "");
}

void aggiornaValoreGrande(int indice, const String& testo, uint16_t colore,
                          float frazioneBarra, uint16_t coloreBarra) {
  int x = BIG_X[indice];
  if (testo != cacheTesto[indice] || colore != cacheColore[indice]) {
    int wU = larghezzaUnitaGrande[indice];
    tft.setTextColor(colore, C_TILE);
    tft.setTextDatum(TR_DATUM);
    tft.setTextPadding(BIG_W - 20 - wU - 4);
    tft.drawString(testo, x + BIG_W - 10 - wU - 4, BIG_Y + 30, 7);
    tft.setTextPadding(0);
    cacheTesto[indice] = testo;
    cacheColore[indice] = colore;
  }
  int bx = x + 10;
  int bw = BIG_W - 20;
  int pieno = (int)(bw * limita01(frazioneBarra));
  if (pieno != cacheBarraPieno[indice] || coloreBarra != cacheBarraColore[indice]) {
    tft.fillRoundRect(bx, BIG_Y + 84, bw, 10, 4, C_BARBG);
    if (pieno > 0) tft.fillRoundRect(bx, BIG_Y + 84, pieno < 8 ? 8 : pieno, 10, 4, coloreBarra);
    cacheBarraPieno[indice] = pieno;
    cacheBarraColore[indice] = coloreBarra;
  }
}

void aggiornaValorePiccolo(int indice, const String& testo, uint16_t colore) {
  int slot = 2 + indice;
  if (testo == cacheTesto[slot] && colore == cacheColore[slot]) return;
  int x = BIG_X[indice % 2];
  int y = SMALL_Y[indice / 2];
  int wU = larghezzaUnitaPiccola[indice];
  int xValore = x + SMALL_W - 8 - wU - (wU > 0 ? 4 : 0);
  int spazio = xValore - (x + 8 + larghezzaEtichettaPiccola[indice] + 6);
  tft.setTextColor(colore, C_TILE);
  tft.setTextDatum(MR_DATUM);
  tft.setTextPadding(spazio);
  tft.drawString(testo, xValore, y + SMALL_H / 2, 4);
  tft.setTextPadding(0);
  cacheTesto[slot] = testo;
  cacheColore[slot] = colore;
}

void aggiornaPaginaGuida() {
  RisultatoPID avanz = mostrato(P_AVANZ);

  // barra di stato (lampeggia alternando le due frasi ogni 1,5 s)
  int stato;
  int nErr = erroriLetti ? contaErroriRilevanti() : 0;
  if (nErr > 99) nErr = 99;
  if (!datiPresenti()) stato = 0;
  else if (!rigenInCorso) stato = (nErr > 0) ? 100 + nErr : 1;
  else stato = ((millis() / 1500) % 2 == 0) ? 2 : 3;

  if (stato != cacheStatoBarra) {
    disegnaBarraStato(stato);
    cacheStatoBarra = stato;
    cachePercentuale = -1;
  }
  float percAvanz = avanz.valido ? avanz.valore : 0.0f;
  if (stato == 2) {
    int perc = (int)(percAvanz + 0.5f);
    if (perc != cachePercentuale) {
      disegnaPercentualeBarra(perc);
      cachePercentuale = perc;
    }
  }
  bool rigenVisibile = (stato == 2 || stato == 3);   // (100+N = errori, non rigenerazione)
  int pieno = rigenVisibile ? (int)(320 * limita01(percAvanz / 100.0f)) : -1;
  if (pieno != cacheStriscia) {
    disegnaStriscia(pieno);
    cacheStriscia = pieno;
  }

  // riquadro piccolo 0: km dall'ultima rigenerazione, oppure avanzamento
  int modo0 = rigenInCorso ? 1 : 0;
  if (modo0 != cacheModoRiquadro0) {
    if (rigenInCorso) disegnaIntestazionePiccola(0, "Avanz. rig.", "%");
    else disegnaIntestazionePiccola(0, "Da rigen.", "km");
    cacheModoRiquadro0 = modo0;
  }

  // temperatura FAP
  RisultatoPID tfap = mostrato(P_TFAP);
  uint16_t colT = !tfap.valido ? C_LABEL : (tfap.valore < SOGLIA_TFAP_ARANCIO ? C_TEXT : C_ORANGE);
  uint16_t colBarT = (!tfap.valido || tfap.valore < SOGLIA_TFAP_ARANCIO) ? C_LABEL : C_ORANGE;
  aggiornaValoreGrande(0, formatta(tfap, 0), colT, tfap.valido ? tfap.valore / 700.0f : 0, colBarT);

  // intasamento FAP
  RisultatoPID intas = mostrato(P_INTAS);
  aggiornaValoreGrande(1, formattaIntasamento(intas), intas.valido ? C_TEXT : C_LABEL,
                       intas.valido ? intas.valore / 100.0f : 0,
                       intas.valido ? coloreIntasamento(intas.valore) : C_LABEL);

  // riquadri piccoli
  if (rigenInCorso) {
    aggiornaValorePiccolo(0, formatta(avanz, 0), C_ORANGE);
  } else {
    RisultatoPID km = mostrato(P_KM);
    if (km.valido) km.valore = (float)((long)km.valore);   // km interi
    aggiornaValorePiccolo(0, formatta(km, 0), km.valido ? C_TEXT : C_LABEL);
  }
  RisultatoPID pdiff = mostrato(P_PDIFF);
  RisultatoPID tmot = mostrato(P_TMOT);
  RisultatoPID nrig = mostrato(P_NRIG);
  aggiornaValorePiccolo(1, formatta(pdiff, 0), pdiff.valido ? C_TEXT : C_LABEL);
  aggiornaValorePiccolo(2, formatta(tmot, 0), tmot.valido ? C_TEXT : C_LABEL);
  aggiornaValorePiccolo(3, formatta(nrig, 0), nrig.valido ? C_TEXT : C_LABEL);
}

// ============================================================================
// PAGINA DETTAGLI
// ============================================================================

// ordine delle righe nella pagina dettagli
const int NUM_RIGHE_DET = 7;
const int RIGA_PID[NUM_RIGHE_DET] = { P_TFAP, P_INTAS, P_AVANZ, P_KM, P_NRIG, P_PDIFF, P_TMOT };
const char* RIGA_ETICHETTA[NUM_RIGHE_DET] = {
  "Temperatura FAP", "Intasamento FAP", "Avanzamento rigen.", "Km da ultima rigen.",
  "Rigenerazioni totali", "Pressione differenz.", "Temperatura motore"
};
const char* RIGA_UNITA[NUM_RIGHE_DET] = { "gC", "%", "%", "km", "", "mbar", "gC" };
const int RIGA_DECIMALI[NUM_RIGHE_DET] = { 0, 1, 1, 1, 0, 0, 0 };

void disegnaPaginaDettagliFissa() {
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, 320, BAR_H, C_TILE);
  tft.setTextPadding(0);
  tft.setTextColor(C_TEXT, C_TILE);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("Dettagli", 12, BAR_H / 2, 4);
  tft.setTextColor(C_LABEL, C_TILE);
  tft.setTextDatum(MR_DATUM);
  tft.drawString("tocca per continuare", 308, BAR_H / 2, 2);

  for (int i = 0; i < NUM_RIGHE_DET; i++) {
    int yC = DET_Y0 + i * DET_PASSO + DET_PASSO / 2;
    tft.setTextColor(C_LABEL, C_BG);
    tft.setTextDatum(ML_DATUM);
    tft.setTextPadding(0);
    tft.drawString(RIGA_ETICHETTA[i], 12, yC, 2);
    larghezzaUnitaRiga[i] = disegnaUnita(308, yC, RIGA_UNITA[i], C_LABEL, C_BG, 2);
  }
  tft.setTextColor(C_LABEL, C_BG);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("Tempo medio per lettura:", 12, 231, 2);
}

void aggiornaPaginaDettagli() {
  for (int i = 0; i < NUM_RIGHE_DET; i++) {
    RisultatoPID r = mostrato(RIGA_PID[i]);
    String testo = formatta(r, RIGA_DECIMALI[i]);
    uint16_t colore = r.valido ? C_TEXT : C_LABEL;
    int slot = 6 + i;
    if (testo == cacheTesto[slot] && colore == cacheColore[slot]) continue;
    int wU = larghezzaUnitaRiga[i];
    int yC = DET_Y0 + i * DET_PASSO + DET_PASSO / 2;
    tft.setTextColor(colore, C_BG);
    tft.setTextDatum(MR_DATUM);
    tft.setTextPadding(110);
    tft.drawString(testo, 308 - wU - (wU > 0 ? 4 : 0), yC, 4);
    tft.setTextPadding(0);
    cacheTesto[slot] = testo;
    cacheColore[slot] = colore;
  }

  String tempo = String((int)(msMedioLettura + 0.5f)) + " ms" + (statoSuffisso == 1 ? " (veloce)" : "");
  if (tempo != cacheTesto[13]) {
    tft.setTextColor(C_TEXT, C_BG);
    tft.setTextDatum(ML_DATUM);
    tft.setTextPadding(120);
    tft.drawString(tempo, 186, 231, 2);
    tft.setTextPadding(0);
    cacheTesto[13] = tempo;
  }
}

// ============================================================================
// ERRORI DELLA CENTRALINA MOTORE (lettura e cancellazione)
// ============================================================================
//
// Lettura:  prima UDS "1902FF" (tutti gli errori della centralina, con stato),
//           se la centralina non lo accetta: OBD2 standard modo 03 + modo 07.
// Stato (bit UDS): 0x01 = attivo adesso, 0x04 = in attesa di conferma,
//                  0x08 = memorizzato/confermato, 0x20 = fallito dopo
//                  l'ultima cancellazione (storico).
// Cancellazione: UDS "14FFFFFF", poi OBD2 modo 04 a tutte le centraline
// (indirizzo funzionale), poi modo 04 diretto; alla fine rilegge sempre. Solo con giri
//                motore a zero; dopo la cancellazione rilegge per verificare.

// Descrizioni dei codici standard piu' utili per il 1.6 Multijet.
// I codici specifici Fiat (es. P1xxx) non hanno una descrizione standard.
struct DescrizioneCodice { const char* codice; const char* testo; };
const DescrizioneCodice DESCRIZIONI[] = {
  { "P2002", "FAP: efficienza sotto soglia (vedi Test)" },
  { "P2463", "FAP intasato (accumulo di particolato)" },
  { "P242F", "FAP ostruito da ceneri" },
  { "P244A", "FAP: press. diff. troppo bassa (vedi Test)" },
  { "P244B", "FAP: pressione differenziale troppo alta" },
  { "P2452", "Sensore press. diff. FAP: circuito (Test)" },
  { "P2453", "Sensore press. diff. FAP: prestaz. (Test)" },
  { "P2454", "Sensore pressione diff. FAP: segnale basso" },
  { "P2455", "Sensore pressione diff. FAP: segnale alto" },
  { "P2458", "FAP: rigenerazione troppo lunga" },
  { "P2459", "FAP: rigenerazioni troppo frequenti" },
  { "P0544", "Sensore temp. gas scarico 1: circuito" },
  { "P0545", "Sensore temp. gas scarico 1: segnale basso" },
  { "P0546", "Sensore temp. gas scarico 1: segnale alto" },
  { "P2031", "Sensore temp. gas scarico 2: circuito" },
  { "P2032", "Sensore temp. gas scarico 2: segnale basso" },
  { "P2033", "Sensore temp. gas scarico 2: segnale alto" },
  { "P0470", "Sensore pressione gas scarico" },
  { "P0471", "Sensore pressione scarico: prestazioni" },
  { "P0400", "EGR: flusso anomalo" },
  { "P0401", "EGR: flusso insufficiente" },
  { "P0402", "EGR: flusso eccessivo" },
  { "P0403", "EGR: circuito di comando" },
  { "P0404", "EGR: prestazioni valvola" },
  { "P0405", "EGR: sensore posizione, segnale basso" },
  { "P0406", "EGR: sensore posizione, segnale alto" },
  { "P2413", "EGR: prestazioni del sistema" },
  { "P0087", "Pressione gasolio nel rail troppo bassa" },
  { "P0088", "Pressione gasolio nel rail troppo alta" },
  { "P0089", "Regolatore pressione gasolio: prestazioni" },
  { "P0093", "Perdita nel sistema gasolio (grande)" },
  { "P0190", "Sensore pressione rail: circuito" },
  { "P0191", "Sensore pressione rail: prestazioni" },
  { "P0192", "Sensore pressione rail: segnale basso" },
  { "P0193", "Sensore pressione rail: segnale alto" },
  { "P0201", "Iniettore cilindro 1: circuito" },
  { "P0202", "Iniettore cilindro 2: circuito" },
  { "P0203", "Iniettore cilindro 3: circuito" },
  { "P0204", "Iniettore cilindro 4: circuito" },
  { "P0045", "Turbo: circuito comando pressione" },
  { "P0046", "Turbo: comando pressione, prestazioni" },
  { "P0234", "Turbo: sovralimentazione eccessiva" },
  { "P0299", "Turbo: sovralimentazione insufficiente" },
  { "P2263", "Turbo: prestazioni del sistema" },
  { "P0069", "Correlazione pressione collettore/barometro" },
  { "P2279", "Perdita d'aria in aspirazione" },
  { "P0100", "Debimetro (MAF): circuito" },
  { "P0101", "Debimetro (MAF): prestazioni" },
  { "P0102", "Debimetro (MAF): segnale basso" },
  { "P0103", "Debimetro (MAF): segnale alto" },
  { "P0106", "Sensore pressione collettore: prestazioni" },
  { "P0107", "Sensore pressione collettore: segnale basso" },
  { "P0108", "Sensore pressione collettore: segnale alto" },
  { "P0112", "Sensore temp. aria aspirata: segnale basso" },
  { "P0113", "Sensore temp. aria aspirata: segnale alto" },
  { "P0115", "Sensore temp. motore: circuito" },
  { "P0116", "Sensore temp. motore: prestazioni" },
  { "P0117", "Sensore temp. motore: segnale basso" },
  { "P0118", "Sensore temp. motore: segnale alto" },
  { "P0128", "Termostato: motore non arriva in temp." },
  { "P0335", "Sensore posizione albero motore" },
  { "P0340", "Sensore posizione albero a camme" },
  { "P0380", "Candelette: circuito" },
  { "P0670", "Centralina candelette: circuito" },
  { "P0671", "Candeletta cilindro 1" },
  { "P0672", "Candeletta cilindro 2" },
  { "P0673", "Candeletta cilindro 3" },
  { "P0674", "Candeletta cilindro 4" },
  { "P0480", "Ventola raffreddamento: circuito" },
  { "P0500", "Sensore velocita' veicolo" },
  { "P0560", "Tensione di sistema (batteria)" },
  { "P0562", "Tensione di sistema bassa" },
  { "P0563", "Tensione di sistema alta" },
  { "P0605", "Centralina: errore memoria interna" },
  { "P0606", "Centralina: errore processore" },
  { "P0641", "Alimentazione sensori A: circuito" },
  { "P0651", "Alimentazione sensori B: circuito" },
  { "P2135", "Correlazione sensori farfalla/pedale" },
  { "P2138", "Correlazione sensori pedale acceleratore" },
  { "U0001", "Linea CAN: comunicazione" },
  { "U0100", "Persa comunicazione con centralina motore" },
  { "U0101", "Persa comunicazione con centralina cambio" },
  { "U0121", "Persa comunicazione con ABS" },
  { "U0140", "Persa comunicazione con Body Computer" },
  { "U0155", "Persa comunicazione con quadro strumenti" },
};
const int NUM_DESCRIZIONI = sizeof(DESCRIZIONI) / sizeof(DESCRIZIONI[0]);

const char* descrizioneCodice(const char* codice) {
  for (int i = 0; i < NUM_DESCRIZIONI; i++) {
    if (strncmp(DESCRIZIONI[i].codice, codice, 5) == 0) return DESCRIZIONI[i].testo;
  }
  if (codice[0] == 'P' && (codice[1] == '1' || codice[1] == '3')) return "Codice specifico Fiat: cercalo online";
  return "Descrizione non disponibile";
}

// ---------- decodifica ----------

bool soloHex(const String& s) {
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s.charAt(i);
    if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f'))) return false;
  }
  return s.length() > 0;
}

uint8_t byteHex(const String& d, int posCaratteri) {
  return (uint8_t)strtol(d.substring(posCaratteri, posCaratteri + 2).c_str(), NULL, 16);
}

// Rimette insieme una risposta dell'ELM327, anche su piu' righe:
//   risposta breve:  "5902FF20020009"
//   risposta lunga:  "013" / "0:5902FF200200" / "1:09246300082F00" / ...
// (la prima riga di 3 cifre e' la lunghezza in byte del messaggio completo)
String unisciFrame(const String& risposta) {
  String out = "";
  int lunghezza = -1;
  String r = risposta + "\r";
  int inizio = 0;
  for (unsigned int i = 0; i < r.length(); i++) {
    char ch = r.charAt(i);
    if (ch != '\r' && ch != '\n') continue;
    String riga = r.substring(inizio, i);
    inizio = i + 1;
    riga.replace(" ", "");
    riga.replace(">", "");
    riga.trim();
    if (riga.length() == 0 || riga.indexOf("SEARCHING") != -1) continue;
    int dp = riga.indexOf(':');
    if (dp != -1) {
      out += riga.substring(dp + 1);
    } else if (riga.length() == 3 && soloHex(riga)) {
      lunghezza = (int)strtol(riga.c_str(), NULL, 16);
    } else {
      out += riga;
    }
  }
  if (lunghezza > 0 && (int)out.length() > lunghezza * 2) out = out.substring(0, lunghezza * 2);
  return out;
}

// Vero se una delle righe della risposta inizia con "prefisso"
bool rigaIniziaCon(const String& risposta, const String& prefisso) {
  String r = risposta + "\r";
  int inizio = 0;
  for (unsigned int i = 0; i < r.length(); i++) {
    char ch = r.charAt(i);
    if (ch != '\r' && ch != '\n') continue;
    String riga = r.substring(inizio, i);
    inizio = i + 1;
    riga.replace(" ", "");
    riga.replace(">", "");
    riga.trim();
    if (riga.indexOf(prefisso) == 0) return true;
  }
  return false;
}

// 2 byte OBD -> "P2002"
String codiceDaByte(uint8_t a, uint8_t b) {
  const char lettere[4] = { 'P', 'C', 'B', 'U' };
  char buf[8];
  snprintf(buf, sizeof(buf), "%c%X%X%02X", lettere[(a >> 6) & 3], (a >> 4) & 3, a & 0x0F, b);
  return String(buf);
}

void aggiungiErrore(const String& codice, uint8_t stato) {
  for (int i = 0; i < numErrori; i++) {
    if (codice == String(errori[i].codice)) {   // stesso codice da modo 03 e 07
      errori[i].stato |= stato;
      return;
    }
  }
  if (numErrori >= MAX_ERRORI) return;
  strncpy(errori[numErrori].codice, codice.c_str(), sizeof(errori[numErrori].codice) - 1);
  errori[numErrori].codice[sizeof(errori[numErrori].codice) - 1] = '\0';
  errori[numErrori].stato = stato;
  numErrori++;
}

int prioritaStato(uint8_t s) {
  if (s & 0x01) return 0;   // attivo
  if (s & 0x08) return 1;   // memorizzato
  if (s & 0x04) return 2;   // in attesa
  return 3;                 // storico
}

void ordinaErrori() {
  for (int i = 1; i < numErrori; i++) {
    CodiceErrore x = errori[i];
    int j = i - 1;
    while (j >= 0 && prioritaStato(errori[j].stato) > prioritaStato(x.stato)) {
      errori[j + 1] = errori[j];
      j--;
    }
    errori[j + 1] = x;
  }
}

const char* etichettaStato(uint8_t s) {
  if (s & 0x01) return "ATTIVO";
  if (s & 0x08) return "MEMORIZZATO";
  if (s & 0x04) return "IN ATTESA";
  return "STORICO";
}

uint16_t coloreStato(uint8_t s) {
  if (s & 0x01) return C_RED;
  if (s & 0x08) return C_ORANGE;
  if (s & 0x04) return C_YELLOW;
  return C_LABEL;
}

// errori che contano per l'avviso (attivi, memorizzati, in attesa)
int contaErroriRilevanti() {
  int n = 0;
  for (int i = 0; i < numErrori; i++) {
    if (errori[i].stato & 0x0D) n++;
  }
  return n;
}

String elencoErroriPerLog() {
  if (numErrori == 0) return String("nessuno");
  String s = "";
  for (int i = 0; i < numErrori; i++) {
    char st[4];
    snprintf(st, sizeof(st), "%02X", errori[i].stato);
    if (i > 0) s += ";";
    s += String(errori[i].codice) + ":" + st;
  }
  return s;
}

// ---------- lettura ----------

// Ritorna il numero di errori letti, -1 = nessuna risposta, -2 = rifiutato
int leggiErroriUDS() {
  String risp = inviaComandoAtteso("1902FF", 3000, "5902");
  String d = unisciFrame(risp);
  int p = d.indexOf("5902");
  if (p == -1) {
    if (d.indexOf("7F19") != -1) return -2;
    return -1;
  }
  p += 6;   // "59" "02" + maschera di disponibilita'
  int letti = 0;
  while (p + 8 <= (int)d.length()) {
    uint8_t a = byteHex(d, p), b = byteHex(d, p + 2), c = byteHex(d, p + 4), st = byteHex(d, p + 6);
    p += 8;
    if (a == 0 && b == 0 && c == 0) continue;
    if ((st & 0x2D) == 0) continue;           // test non completato: non e' un guasto
    char ftb[4];
    snprintf(ftb, sizeof(ftb), "%02X", c);
    aggiungiErrore(codiceDaByte(a, b) + "-" + ftb, st);
    letti++;
  }
  return letti;
}

// modo 03 (memorizzati) o 07 (in attesa). -1 = nessuna risposta.
int leggiErroriOBD(const char* modo, const char* risposta, uint8_t stato) {
  String risp = inviaComandoAtteso(modo, 3000, risposta);
  String d = unisciFrame(risp);
  int p = d.indexOf(risposta);
  if (p == -1) return -1;
  p += 2;
  if (p + 2 > (int)d.length()) return 0;
  int quanti = byteHex(d, p);     // su CAN il primo byte e' il numero di codici
  p += 2;
  int letti = 0;
  while (letti < quanti && p + 4 <= (int)d.length()) {
    uint8_t a = byteHex(d, p), b = byteHex(d, p + 2);
    p += 4;
    if (a == 0 && b == 0) continue;
    aggiungiErrore(codiceDaByte(a, b), stato);
    letti++;
  }
  return letti;
}

// Legge tutti gli errori. Ritorna false se la centralina non risponde.
bool leggiErrori() {
  numErrori = 0;
  metodoErrori = "";
  int r = leggiErroriUDS();
  if (r >= 0) {
    metodoErrori = "UDS";
  } else {
    int r3 = leggiErroriOBD("03", "43", 0x08);
    int r7 = leggiErroriOBD("07", "47", 0x04);
    if (r3 < 0 && r7 < 0) {
      erroriLetti = false;
      scriviLog("DTC_LETTURA_FALLITA,uds=" + String(r));
      return false;
    }
    metodoErrori = "OBD2";
  }
  ordinaErrori();
  erroriLetti = true;
  paginaListaErrori = 0;
  scriviLog("DTC_LETTI,metodo=" + metodoErrori + ",n=" + String(numErrori) + "," + elencoErroriPerLog());
  return true;
}

// Giri motore (OBD2 standard 010C). Ritorna false se non risponde.
// Sulla Tipo la centralina NON risponde al 010C sull'indirizzo diretto
// 18DA10F1, ma risponde alla richiesta "a tutti" (18DB33F1). Si prova il
// diretto e, se non va, SUBITO quello "a tutti" nella stessa chiamata; poi si
// ricorda quale funziona. Il filtro ATCRA lascia passare solo la centralina
// motore, quindi con il "1" finale basta la prima risposta (piu' veloce).
bool giriViaFunzionale = false;
int fallimentiGiri = 0;
bool giriMaiLetti = true;

bool estraiGiri(const String& risposta, float& giri) {
  String d = unisciFrame(risposta);
  int p = d.indexOf("410C");
  if (p == -1 || p + 8 > (int)d.length()) return false;
  giri = ((byteHex(d, p + 4) * 256) + byteHex(d, p + 6)) / 4.0f;
  return true;
}

bool leggiGiriDiretto(float& giri) {
  return estraiGiri(inviaComandoAtteso("010C", 600, "410C"), giri);
}

bool leggiGiriFunzionale(float& giri) {
  inviaComandoELM("ATSH18DB33F1", 500);
  String cmd = statoSuffisso == 1 ? "010C1" : "010C";
  bool ok = estraiGiri(inviaComandoAtteso(cmd, 1000, "410C"), giri);
  inviaComandoELM("ATSH18DA10F1", 500);
  return ok;
}

bool leggiGiriMotore(float& giri) {
  bool ok = giriViaFunzionale ? leggiGiriFunzionale(giri) : leggiGiriDiretto(giri);
  if (!ok) {
    // provo subito l'altro modo
    ok = giriViaFunzionale ? leggiGiriDiretto(giri) : leggiGiriFunzionale(giri);
    if (ok) {
      giriViaFunzionale = !giriViaFunzionale;
      scriviLog(giriViaFunzionale ? "GIRI_VIA_INDIRIZZO_FUNZIONALE" : "GIRI_VIA_INDIRIZZO_DIRETTO");
    }
  }
  if (ok) {
    if (giriMaiLetti) {
      giriMaiLetti = false;
      scriviLog("GIRI_LETTI,giri=" + String((int)giri) + ",metodo=" + (giriViaFunzionale ? "funzionale" : "diretto"));
    }
    fallimentiGiri = 0;
    return true;
  }
  if (++fallimentiGiri == 5) scriviLog("GIRI_NON_LEGGIBILI");
  return false;
}

// ---------- cancellazione ----------

void impostaEsito(const String& riga1, const String& riga2, uint16_t colore) {
  esitoRiga1 = riga1;
  esitoRiga2 = riga2;
  esitoColore = colore;
  statoPaginaErrori = ERR_ESITO;
}

void cancellaErrori() {
  float giri = 0;
  bool giriLetti = leggiGiriMotore(giri);
  if (!giriLetti) {
    // giri non leggibili: si procede lo stesso (hai confermato motore spento;
    // se le condizioni non vanno bene la centralina rifiuta da sola)
    scriviLog("DTC_CANCELLAZIONE,giri_non_letti_procedo");
  }
  if (giriLetti && giri > 50) {
    impostaEsito("Motore acceso", "Spegni il motore (quadro acceso) e riprova", C_ORANGE);
    scriviLog("DTC_CANCELLAZIONE,esito=motore_acceso,giri=" + String((int)giri));
    return;
  }

  scriviLog("DTC_PRIMA_DI_CANCELLARE," + elencoErroriPerLog());
  int primaRilevanti = contaErroriRilevanti();

  // attesa piu' lunga: la cancellazione puo' richiedere tempo
  inviaComandoELM("ATAT0", 1000);
  inviaComandoELM("ATSTFF", 1000);

  // 1) UDS "cancella tutto" all'indirizzo della centralina motore
  // 2) se non va: OBD2 standard "04" a TUTTE le centraline (indirizzo
  //    funzionale 18DB33F1). Sulla Tipo i comandi OBD standard (es. i giri)
  //    funzionano SOLO cosi'; e' anche la strada che il gateway di sicurezza
  //    FCA deve lasciare aperta (cancellazione errori legati alle emissioni).
  // 3) ultimo tentativo: "04" all'indirizzo diretto
  String metodo = "";
  String esitoUds = provaCancellazione("UDS", "18DA10F1", "14FFFFFF", "54");
  String esitoObdF = "-", esitoObdD = "-";
  if (esitoUds == "ok") {
    metodo = "UDS";
  } else {
    esitoObdF = provaCancellazione("OBD_FUNZIONALE", "18DB33F1", "04", "44");
    if (esitoObdF == "ok") {
      metodo = "OBD2_funzionale";
    } else {
      esitoObdD = provaCancellazione("OBD_DIRETTO", "18DA10F1", "04", "44");
      if (esitoObdD == "ok") metodo = "OBD2_diretto";
    }
  }
  inviaComandoELM("ATST32", 1000);
  inviaComandoELM("ATAT1", 1000);

  // verifica SEMPRE rileggendo: una risposta persa non vuol dire che non
  // abbia funzionato, e una risposta "ok" non vuol dire che il guasto sia sparito
  delay(1500);
  bool letto = leggiErrori();
  int rimasti = contaErroriRilevanti();
  scriviLog("DTC_CANCELLAZIONE,metodo=" + (metodo.length() ? metodo : String("nessuno")) +
            ",uds=" + esitoUds + ",obd_funz=" + esitoObdF + ",obd_dir=" + esitoObdD +
            ",prima=" + String(primaRilevanti) + ",rimasti=" + (letto ? String(rimasti) : String("ND")));

  if (letto && rimasti == 0) {
    impostaEsito("Errori cancellati", "Nessun errore presente ora", C_GREEN);
  } else if (metodo.length() > 0 && letto) {
    impostaEsito(String(rimasti) + (rimasti == 1 ? " errore e' tornato" : " errori sono tornati"),
                 "Cancellati, ma il guasto e' ancora presente", C_ORANGE);
  } else if (metodo.length() > 0) {
    impostaEsito("Cancellazione inviata", "Rileggi per verificare", C_GREEN);
  } else if (esitoUds == "33" || esitoObdF == "33" || esitoObdD == "33") {
    impostaEsito("Bloccata", "Accesso negato (gateway di sicurezza?)", C_RED);
  } else if (esitoUds == "22" || esitoObdF == "22" || esitoObdD == "22") {
    impostaEsito("Rifiutata dalla centralina", "Condizioni non corrette: motore spento?", C_ORANGE);
  } else if (esitoUds.length() == 2 || esitoObdF.length() == 2 || esitoObdD.length() == 2) {
    impostaEsito("Rifiutata dalla centralina",
                 "Codici: UDS " + testoNrc(esitoUds) + ", OBD " + testoNrc(esitoObdF) + "/" + testoNrc(esitoObdD), C_ORANGE);
  } else {
    impostaEsito("Nessuna risposta", "La cancellazione non e' riuscita", C_ORANGE);
  }
}

String testoNrc(const String& e) {
  return e.length() == 2 ? e : String("-");
}

// Un tentativo di cancellazione. Ritorna "ok", il codice di rifiuto della
// centralina (es. "33" = accesso negato, "22" = condizioni non corrette,
// "11" = servizio non supportato) oppure "" se non risponde nessuno.
// "78" (= "sto lavorando, aspetta") viene ignorato se poi arriva altro.
String provaCancellazione(const String& nome, const String& header, const String& comando, const String& positiva) {
  bool cambiaHeader = (header != "18DA10F1");
  if (cambiaHeader) inviaComandoELM("ATSH" + header, 500);
  String r = inviaComandoAtteso(comando, 6000, positiva);
  if (cambiaHeader) inviaComandoELM("ATSH18DA10F1", 500);
  scriviLog("DTC_CANC_PROVA," + nome + ",risposta=" + pulisci(r));
  if (rigaIniziaCon(r, positiva)) return String("ok");
  String d = unisciFrame(r);
  String negativa = "7F" + comando.substring(0, 2);
  String nrc = "";
  int da = 0;
  while (da < (int)d.length()) {
    int p = d.substring(da).indexOf(negativa);
    if (p == -1) break;
    p += da;
    if (p + 6 <= (int)d.length()) {
      String c = d.substring(p + 4, p + 6);
      if (c != "78" || nrc.length() == 0) nrc = c;
    }
    da = p + 4;
  }
  return nrc;
}

// ---------- controllo automatico (una volta per connessione, senza suono) ----------

void controlloErroriAutomatico() {
  if (controlloAutomaticoFatto || !datiPresenti() || pagina == 3) return;
  for (int i = 0; i < NUM_PID; i++) {
    if (statoPID[i].urgente) return;   // prima faccio un giro completo dei dati FAP
  }
  controlloAutomaticoFatto = true;
  leggiErrori();
  if (pagina == 2 && statoPaginaErrori != ERR_CONFERMA) {
    statoPaginaErrori = erroriLetti ? ERR_LISTA : ERR_NON_RISPONDE;
    paginaDaRidisegnare = true;
  }
}

// ============================================================================
// PAGINA ERRORI
// ============================================================================

const int ERR_LISTA_Y = 38;
const int ERR_RIGA_H = 50;
const int ERR_RIGHE_PER_PAGINA = 3;
const int CONF_Y = 140, CONF_H = 42;     // pulsante CONFERMA (in alto)
const int ANNULLA_Y = 192, ANNULLA_H = 44; // pulsante ANNULLA (in basso)

void disegnaPulsante(int x, int y, int w, int h, const char* testo, uint16_t sfondo, uint16_t colTesto) {
  tft.fillRoundRect(x, y, w, h, 8, sfondo);
  tft.setTextColor(colTesto, sfondo);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.drawString(testo, x + w / 2, y + h / 2, 4);
}

void disegnaTestoCentrato(const String& testo, int y, uint8_t font, uint16_t colore) {
  tft.setTextColor(colore, C_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.drawString(testo, 160, y, font);
}

int pagineListaErrori() {
  return (numErrori + ERR_RIGHE_PER_PAGINA - 1) / ERR_RIGHE_PER_PAGINA;
}

void disegnaIntestazioneErrori() {
  tft.fillRect(0, 0, 320, BAR_H, C_TILE);
  tft.setTextPadding(0);
  tft.setTextColor(C_TEXT, C_TILE);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("Errori centralina", 12, BAR_H / 2, 4);
  if (statoPaginaErrori == ERR_LISTA && pagineListaErrori() > 1) {
    tft.setTextColor(C_LABEL, C_TILE);
    tft.setTextDatum(MR_DATUM);
    tft.drawString(String(paginaListaErrori + 1) + "/" + String(pagineListaErrori()) + " tocca", 308, BAR_H / 2, 2);
  }
}

void disegnaListaErrori() {
  if (numErrori == 0) {
    disegnaTestoCentrato("Nessun errore", 90, 4, C_GREEN);
    disegnaTestoCentrato("nella centralina motore (" + metodoErrori + ")", 122, 2, C_LABEL);
    return;
  }
  int primo = paginaListaErrori * ERR_RIGHE_PER_PAGINA;
  for (int k = 0; k < ERR_RIGHE_PER_PAGINA && primo + k < numErrori; k++) {
    const CodiceErrore& e = errori[primo + k];
    int y = ERR_LISTA_Y + k * ERR_RIGA_H;
    tft.setTextPadding(0);
    tft.setTextColor(C_TEXT, C_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(e.codice, 10, y + 2, 4);
    tft.setTextColor(coloreStato(e.stato), C_BG);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(etichettaStato(e.stato), 310, y + 8, 2);
    tft.setTextColor(C_LABEL, C_BG);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(descrizioneCodice(e.codice), 10, y + 30, 2);
    if (k < ERR_RIGHE_PER_PAGINA - 1 && primo + k + 1 < numErrori) {
      tft.drawFastHLine(10, y + ERR_RIGA_H - 2, 300, C_TILE);
    }
  }
}

void disegnaConfermaCancellazione() {
  tft.fillRect(0, BAR_H, 320, 240 - BAR_H, C_BG);
  int n = numErrori;
  disegnaTestoCentrato("Cancellare " + String(n) + (n == 1 ? " errore?" : " errori?"), 54, 4, C_TEXT);
  disegnaTestoCentrato("Solo a motore SPENTO e quadro acceso.", 84, 2, C_LABEL);
  disegnaTestoCentrato("Se il guasto c'e' ancora, l'errore tornera'.", 102, 2, C_LABEL);
  disegnaTestoCentrato("I codici vengono prima salvati nel log.", 120, 2, C_LABEL);
  disegnaPulsante(4, CONF_Y, 312, CONF_H, "CONFERMA", C_RED, C_TEXT);
  disegnaPulsante(4, ANNULLA_Y, 312, ANNULLA_H, "Annulla", C_TILE, C_TEXT);
}

void disegnaPaginaErrori() {
  tft.fillScreen(C_BG);
  disegnaIntestazioneErrori();
  if (statoPaginaErrori == ERR_CONFERMA) {
    disegnaConfermaCancellazione();
    return;
  }
  switch (statoPaginaErrori) {
    case ERR_MAI_LETTI:
      disegnaTestoCentrato("Tocca LEGGI per controllare", 96, 2, C_LABEL);
      disegnaTestoCentrato("gli errori della centralina motore", 116, 2, C_LABEL);
      break;
    case ERR_NON_RISPONDE:
      disegnaTestoCentrato("Nessuna risposta", 90, 4, C_ORANGE);
      disegnaTestoCentrato("Accendi il quadro e riprova", 122, 2, C_LABEL);
      break;
    case ERR_ESITO:
      disegnaTestoCentrato(esitoRiga1, 90, 4, esitoColore);
      disegnaTestoCentrato(esitoRiga2, 122, 2, C_LABEL);
      break;
    default:
      disegnaListaErrori();
      break;
  }
  disegnaPulsante(4, PULS_Y, 74, PULS_H, "Leggi", C_TILE, C_TEXT);
  disegnaPulsante(82, PULS_Y, 106, PULS_H, "Cancella", C_DARKRED, C_TEXT);
  disegnaPulsante(192, PULS_Y, 60, PULS_H, "Test", C_TILE, C_TEXT);
  disegnaPulsante(256, PULS_Y, 60, PULS_H, "Esci", C_TILE, C_TEXT);
}

// schermata mostrata durante un'operazione che richiede qualche secondo
void mostraOperazioneInCorso(const char* testo) {
  tft.fillRect(0, BAR_H, 320, PULS_Y - BAR_H - 2, C_BG);
  disegnaTestoCentrato(testo, 110, 4, C_TEXT);
}

void eseguiLetturaDaPagina() {
  mostraOperazioneInCorso("Lettura in corso...");
  statoPaginaErrori = leggiErrori() ? ERR_LISTA : ERR_NON_RISPONDE;
  paginaDaRidisegnare = true;
}

void toccoPaginaErrori(int px, int py) {
  if (statoPaginaErrori == ERR_CONFERMA) {
    if (py >= CONF_Y && py < CONF_Y + CONF_H) {
      mostraOperazioneInCorso("Cancellazione...");
      cancellaErrori();
      paginaDaRidisegnare = true;
    } else if (py >= ANNULLA_Y) {
      statoPaginaErrori = ERR_LISTA;
      paginaDaRidisegnare = true;
    }
    return;
  }

  if (py >= PULS_Y - 4) {
    if (px < 80) {                       // Leggi
      eseguiLetturaDaPagina();
    } else if (px < 190) {               // Cancella
      if (!erroriLetti) {
        eseguiLetturaDaPagina();         // prima vediamo cosa c'e'
        if (!erroriLetti) return;
      }
      if (numErrori == 0) {
        impostaEsito("Nessun errore", "Non c'e' niente da cancellare", C_GREEN);
      } else {
        statoPaginaErrori = ERR_CONFERMA;
        confermaDalMs = millis();
      }
      paginaDaRidisegnare = true;
    } else if (px < 254) {               // Test sensore FAP
      azzeraTest();
      salvaTest();
      pagina = 3;
      paginaDaRidisegnare = true;
    } else {                             // Esci
      pagina = 0;
      paginaDaRidisegnare = true;
    }
    return;
  }

  // tocco sulla lista: pagina successiva
  if (statoPaginaErrori == ERR_LISTA && pagineListaErrori() > 1) {
    paginaListaErrori = (paginaListaErrori + 1) % pagineListaErrori();
    paginaDaRidisegnare = true;
  }
}

void aggiornaPaginaErrori() {
  // la conferma scade da sola dopo 15 secondi
  if (statoPaginaErrori == ERR_CONFERMA && millis() - confermaDalMs > 15000) {
    statoPaginaErrori = ERR_LISTA;
    paginaDaRidisegnare = true;
  }
}

// ============================================================================
// TEST SENSORE PRESSIONE DIFFERENZIALE FAP (aiuta a capire il P2002)
// ============================================================================
//
// Un sensore con i tubicini a posto misura ~0 mbar a motore spento e una
// pressione che SALE chiaramente quando aumentano i giri (piu' gas di scarico
// attraverso il filtro). Il test registra da solo tre misure mentre segui i passi:
//   1) motore spento, quadro acceso  -> "zero" del sensore
//   2) motore acceso al minimo        -> pressione di base
//   3) colpi di gas in folle (sopra 1800 giri, anche brevi) -> pressione massima
// e poi dice cosa fa pensare il risultato. Le soglie sono indicative.

const float TEST_OFFSET_MAX = 10.0f;    // mbar: oltre, a motore spento, lo zero e' sbagliato
const float TEST_REAZIONE_MIN = 5.0f;   // mbar: sotto = nessuna reazione ai giri
const float TEST_REAZIONE_BUONA = 12.0f;// mbar: sotto = reazione debole
const int   TEST_CAMPIONI = 12;         // letture per minimo e motore spento
const int   TEST_CAMPIONI_GAS = 4;      // letture sopra TEST_GIRI_GAS (anche non di fila)
const float TEST_GIRI_GAS = 1800.0f;    // giri: da qui conta come "colpo di gas"

MisuraTest misSpento, misMinimo, misGiri;
float giriTest = -1;
unsigned long giriTestMs = 0;
unsigned long ultimaAvanzTestMs = 0;
int turnoTest = 0;
int esitoTest = -1;          // -1 = non ancora, altrimenti TEST_ESITO_*
bool esitoTestRegistrato = false;
int esitoRegistrato = -1;
bool spentoRegistrato = false;

enum { TEST_OK, TEST_DEBOLE, TEST_NESSUNA, TEST_INVERTITI, TEST_ZERO };

void azzeraMisura(MisuraTest& m) {
  m.somma = 0; m.campioni = 0; m.massimo = -100000; m.minimo = 100000; m.giriAlMassimo = 0;
}

void azzeraTest() {
  azzeraMisura(misSpento);
  azzeraMisura(misMinimo);
  azzeraMisura(misGiri);
  esitoTest = -1;
  esitoTestRegistrato = false;
  esitoRegistrato = -1;
  spentoRegistrato = false;
}

// Il test resta "aperto" in memoria finche' non premi Esci: se il dispositivo
// si riavvia (es. presa accendisigari spenta), riparte dalla pagina del test
// con le misure gia' completate.
void salvaTest() {
  memoriaPermanente.putBool("tAtt", true);
  memoriaPermanente.putUChar("tBoot", 0);
  memoriaPermanente.putBytes("tSp", &misSpento, sizeof(MisuraTest));
  memoriaPermanente.putBytes("tMi", &misMinimo, sizeof(MisuraTest));
  memoriaPermanente.putBytes("tGi", &misGiri, sizeof(MisuraTest));
}

void chiudiTest() {
  memoriaPermanente.putBool("tAtt", false);
}

// Chiamata all'avvio: true se c'era un test in corso da riprendere
bool caricaTestInCorso() {
  if (!memoriaPermanente.getBool("tAtt", false)) return false;
  uint8_t avvii = memoriaPermanente.getUChar("tBoot", 0) + 1;
  if (avvii > 3) {           // abbandonato da 3 accensioni: lo chiudo
    chiudiTest();
    return false;
  }
  memoriaPermanente.putUChar("tBoot", avvii);
  azzeraTest();
  if (memoriaPermanente.getBytes("tSp", &misSpento, sizeof(MisuraTest)) != sizeof(MisuraTest)) azzeraMisura(misSpento);
  if (memoriaPermanente.getBytes("tMi", &misMinimo, sizeof(MisuraTest)) != sizeof(MisuraTest)) azzeraMisura(misMinimo);
  if (memoriaPermanente.getBytes("tGi", &misGiri, sizeof(MisuraTest)) != sizeof(MisuraTest)) azzeraMisura(misGiri);
  scriviLog("TEST_FAP_RIPRESO_DOPO_RIAVVIO");
  return true;
}

void aggiungiCampione(MisuraTest& m, float valore, float giri) {
  if (m.campioni < 1000) m.campioni++;
  m.somma += valore;
  if (valore > m.massimo) { m.massimo = valore; m.giriAlMassimo = giri; }
  if (valore < m.minimo) m.minimo = valore;
}

float mediaMisura(const MisuraTest& m) {
  return m.campioni > 0 ? m.somma / m.campioni : 0;
}

bool misuraCompleta(const MisuraTest& m) {
  return m.campioni >= (&m == &misGiri ? TEST_CAMPIONI_GAS : TEST_CAMPIONI);
}

void valutaTest() {
  if (!misuraCompleta(misMinimo) || !misuraCompleta(misGiri)) {
    // lo zero sbagliato si puo' gia' dire con la sola misura a motore spento
    if (misuraCompleta(misSpento) && fabs(mediaMisura(misSpento)) > TEST_OFFSET_MAX) esitoTest = TEST_ZERO;
    return;
  }
  float base = mediaMisura(misMinimo);
  float salita = misGiri.massimo - base;
  if (misuraCompleta(misSpento) && fabs(mediaMisura(misSpento)) > TEST_OFFSET_MAX) esitoTest = TEST_ZERO;
  else if (misGiri.minimo < base - TEST_REAZIONE_MIN && salita < TEST_REAZIONE_MIN) esitoTest = TEST_INVERTITI;
  else if (salita < TEST_REAZIONE_MIN) esitoTest = TEST_NESSUNA;
  else if (salita < TEST_REAZIONE_BUONA) esitoTest = TEST_DEBOLE;
  else esitoTest = TEST_OK;

  // registro nel log la prima volta e ogni volta che il risultato si
  // completa o cambia (es. quando arriva la misura a motore spento)
  bool conSpento = misuraCompleta(misSpento);
  if (!esitoTestRegistrato || esitoTest != esitoRegistrato || conSpento != spentoRegistrato) {
    esitoTestRegistrato = true;
    esitoRegistrato = esitoTest;
    spentoRegistrato = conSpento;
    const char* nomi[] = { "OK", "DEBOLE", "NESSUNA_REAZIONE", "TUBICINI_INVERTITI", "ZERO_SBAGLIATO" };
    scriviLog(String("TEST_FAP,esito=") + nomi[esitoTest] +
              ",spento=" + (misuraCompleta(misSpento) ? String(mediaMisura(misSpento), 1) : String("ND")) +
              ",minimo=" + String(base, 1) + ",max=" + String(misGiri.massimo, 1) +
              ",min=" + String(misGiri.minimo, 1) + ",giri=" + String((int)misGiri.giriAlMassimo));
  }
}

// Durante il test: alterna pressione differenziale e giri (letture veloci),
// ogni 2 s controlla anche l'avanzamento rigenerazione.
void eseguiLetturaTest() {
  if (millis() - ultimaAvanzTestMs > 2000) {
    ultimaAvanzTestMs = millis();
    RisultatoPID r = leggiPID(P_AVANZ);
    if (r.valido) { statoPID[P_AVANZ].valore = r; ultimoDatoValidoMs = millis(); }
    return;
  }
  turnoTest++;
  if (turnoTest % 2 == 0) {
    float g;
    if (leggiGiriMotore(g)) { giriTest = g; giriTestMs = millis(); }
    return;
  }
  RisultatoPID r = leggiPID(P_PDIFF);
  statoPID[P_PDIFF].ultimaLettura = millis();
  if (!r.valido) return;
  statoPID[P_PDIFF].valore = r;
  statoPID[P_PDIFF].falliti = 0;
  ultimoDatoValidoMs = millis();

  // il campione vale solo se i giri sono stati letti da poco
  if (giriTest < 0 || millis() - giriTestMs > 1200) return;
  if (rigenInCorso) return;   // durante una rigenerazione i valori non sono confrontabili
  MisuraTest* m = nullptr;
  if (giriTest < 50) {
    if (misSpento.campioni < TEST_CAMPIONI * 2) m = &misSpento;
  } else if (giriTest >= 550 && giriTest <= 1100) {
    if (misMinimo.campioni < TEST_CAMPIONI * 2) m = &misMinimo;
  } else if (giriTest >= TEST_GIRI_GAS) {
    m = &misGiri;
  }
  if (m != nullptr) {
    aggiungiCampione(*m, r.valore, giriTest);
    // salvo quando una misura si completa, e poi ogni 10 campioni (poche scritture)
    int soglia = (m == &misGiri) ? TEST_CAMPIONI_GAS : TEST_CAMPIONI;
    if (m->campioni == soglia || m->campioni % 10 == 0) salvaTest();
  }
  valutaTest();
}

// ---------- disegno ----------

const int TEST_PASSO_Y = 78, TEST_PASSO_H = 22;
const int TEST_ESITO_Y = 148;

void disegnaPaginaTestFissa() {
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, 320, BAR_H, C_TILE);
  tft.setTextPadding(0);
  tft.setTextColor(C_TEXT, C_TILE);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("Test sensore FAP", 12, BAR_H / 2, 4);
  // riquadri valori in tempo reale
  for (int i = 0; i < 2; i++) {
    int x = BIG_X[i];
    tft.fillRoundRect(x, 38, SMALL_W, 32, 6, C_TILE);
    tft.setTextColor(C_LABEL, C_TILE);
    tft.setTextDatum(ML_DATUM);
    tft.drawString(i == 0 ? "Giri" : "P. diff.", x + 8, 54, 2);
  }
  tft.setTextColor(C_LABEL, C_TILE);
  tft.setTextDatum(MR_DATUM);
  tft.drawString("mbar", BIG_X[1] + SMALL_W - 8, 54, 2);
  disegnaPulsante(4, PULS_Y, 150, PULS_H, "Ricomincia", C_TILE, C_TEXT);
  disegnaPulsante(166, PULS_Y, 150, PULS_H, "Esci", C_TILE, C_TEXT);
}

void scriviValoreTest(int slot, int xDestra, int spazio, const String& testo) {
  if (testo == cacheTesto[slot]) return;
  tft.setTextColor(C_TEXT, C_TILE);
  tft.setTextDatum(MR_DATUM);
  tft.setTextPadding(spazio);
  tft.drawString(testo, xDestra, 54, 4);
  tft.setTextPadding(0);
  cacheTesto[slot] = testo;
}

void disegnaPassoTest(int slot, int indice, const char* testo, const MisuraTest& m, bool attivo, const String& risultato) {
  String chiave = String(indice) + (attivo ? "a" : "") + risultato;
  if (chiave == cacheTesto[slot]) return;
  cacheTesto[slot] = chiave;
  int y = TEST_PASSO_Y + indice * TEST_PASSO_H;
  tft.fillRect(0, y, 320, TEST_PASSO_H, C_BG);
  bool fatto = misuraCompleta(m);
  uint16_t colore = fatto ? C_GREEN : (attivo ? C_TEXT : C_LABEL);
  tft.setTextColor(colore, C_BG);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(testo, 12, y + TEST_PASSO_H / 2, 2);
  tft.setTextDatum(MR_DATUM);
  tft.drawString(risultato, 308, y + TEST_PASSO_H / 2, 2);
}

String risultatoMisura(const MisuraTest& m, bool massimo) {
  if (m.campioni == 0) return String("");
  if (!misuraCompleta(m)) {
    if (&m == &misGiri) return String(m.campioni) + "/" + String(TEST_CAMPIONI_GAS) + " letture";
    return String("in corso...");
  }
  if (massimo) return "max " + String((int)lroundf(m.massimo)) + " mbar";
  return String((int)lroundf(mediaMisura(m))) + " mbar";
}

void disegnaEsitoTest() {
  String chiave = "esito" + String(esitoTest) + String((int)lroundf(misGiri.massimo - mediaMisura(misMinimo))) +
                  String((int)lroundf(mediaMisura(misSpento)));
  if (chiave == cacheTesto[13]) return;
  cacheTesto[13] = chiave;
  tft.fillRect(0, TEST_ESITO_Y - 4, 320, PULS_Y - TEST_ESITO_Y, C_BG);
  String r1, r2, r3;
  uint16_t col = C_LABEL;
  int salita = (int)lroundf(misGiri.massimo - mediaMisura(misMinimo));
  switch (esitoTest) {
    case TEST_OK:
      r1 = "Il sensore reagisce: sale di " + String(salita) + " mbar.";
      r2 = "Sensore e tubicini sembrano a posto.";
      r3 = "Se il P2002 torna: officina (filtro o scarico).";
      col = C_GREEN; break;
    case TEST_DEBOLE:
      r1 = "Reazione debole: sale solo di " + String(salita) + " mbar.";
      r2 = "Controlla i tubicini (crepe, fuliggine).";
      r3 = "Se sono a posto: filtro da far verificare.";
      col = C_ORANGE; break;
    case TEST_NESSUNA:
      r1 = "Nessuna reazione ai giri!";
      r2 = "Tubicini staccati, rotti o intasati,";
      r3 = "oppure sensore guasto.";
      col = C_RED; break;
    case TEST_INVERTITI:
      r1 = "In accelerazione la pressione scende!";
      r2 = "Tubicini probabilmente INVERTITI";
      r3 = "(scambiati tra loro sul sensore).";
      col = C_RED; break;
    case TEST_ZERO:
      r1 = "A motore spento legge " + String((int)lroundf(mediaMisura(misSpento))) + " mbar invece di 0:";
      r2 = "sensore probabilmente guasto.";
      r3 = "";
      col = C_RED; break;
    default:
      r1 = "Motore GIA' acceso, in folle, freno a mano.";
      r2 = "Passo 2: 3-4 colpi di gas fino a ~3000 giri";
      r3 = "(non serve tenerli fissi). Poi spegni.";
      break;
  }
  tft.setTextColor(col, C_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.drawString(r1, 160, TEST_ESITO_Y + 6, 2);
  tft.drawString(r2, 160, TEST_ESITO_Y + 22, 2);
  if (r3.length() > 0) tft.drawString(r3, 160, TEST_ESITO_Y + 38, 2);
}

void aggiornaPaginaTest() {
  bool giriValidi = giriTest >= 0 && millis() - giriTestMs < 2000;
  scriviValoreTest(6, BIG_X[0] + SMALL_W - 8, 80, giriValidi ? String((int)giriTest) : String("--"));
  const RisultatoPID& pd = statoPID[P_PDIFF].valore;
  scriviValoreTest(7, BIG_X[1] + SMALL_W - 8 - 32, 52, pd.valido ? String((int)lroundf(pd.valore)) : String("--"));

  // passo "attivo" = quello che corrisponde ai giri attuali
  int attivo = -1;
  if (giriValidi) {
    if (giriTest < 50) attivo = 2;
    else if (giriTest <= 1100) attivo = 0;
    else attivo = 1;
  }
  // Ordine pensato per NON riavviare il motore durante il test: all'avviamento
  // la presa accendisigari si spegne e il dispositivo si riavvia.
  disegnaPassoTest(8, 0, "1. Motore acceso, al minimo", misMinimo, attivo == 0, risultatoMisura(misMinimo, false));
  disegnaPassoTest(9, 1, "2. Colpi di gas in folle", misGiri, attivo == 1, risultatoMisura(misGiri, true));
  disegnaPassoTest(10, 2, "3. Spegni motore, quadro acceso", misSpento, attivo == 2, risultatoMisura(misSpento, false));
  disegnaEsitoTest();
}

void toccoPaginaTest(int px, int py) {
  if (py < PULS_Y - 4) return;
  if (px < 160) {            // Ricomincia
    azzeraTest();
    salvaTest();
    paginaDaRidisegnare = true;
  } else {                   // Esci
    chiudiTest();
    pagina = 0;
    paginaDaRidisegnare = true;
  }
}

// ============================================================================
// SUONI (non bloccanti: le letture continuano mentre suona)
// ============================================================================


void avviaBuzzer() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_BUZZER, 2000, 8);
#else
  ledcSetup(0, 2000, 8);
  ledcAttachPin(PIN_BUZZER, 0);
#endif
  impostaTono(0);
}

void impostaTono(uint16_t freq) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWriteTone(PIN_BUZZER, freq);
#else
  ledcWriteTone(0, freq);
#endif
}

void suona(const Nota* melodia, int lunghezza) {
  if (!SUONI_ATTIVI) return;
  melodiaCorrente = melodia;
  lunghezzaMelodia = lunghezza;
  notaCorrente = 0;
  inizioNotaMs = millis();
  impostaTono(melodia[0].freq);
}

// da chiamare spesso: passa alla nota successiva quando e' il momento
void gestisciSuono() {
  if (melodiaCorrente == nullptr) return;
  if (millis() - inizioNotaMs < melodiaCorrente[notaCorrente].durataMs) return;
  notaCorrente++;
  if (notaCorrente >= lunghezzaMelodia) {
    impostaTono(0);
    melodiaCorrente = nullptr;
    return;
  }
  inizioNotaMs = millis();
  impostaTono(melodiaCorrente[notaCorrente].freq);
}

// Comando "SUONI" dal Monitor Seriale: fa sentire tutti gli avvisi in fila
void suonaEAttendi(const Nota* m, int n, const char* nome) {
  Serial.println(String("  ") + nome);
  for (int i = 0; i < n; i++) {
    impostaTono(m[i].freq);
    delay(m[i].durataMs);
  }
  impostaTono(0);
  delay(1200);
}

void provaTuttiISuoni() {
  Serial.println("Prova suoni:");
  suonaEAttendi(SUONO_AVVIO, sizeof(SUONO_AVVIO) / sizeof(Nota), "avvio");
  suonaEAttendi(SUONO_RIGEN_INIZIO, sizeof(SUONO_RIGEN_INIZIO) / sizeof(Nota), "inizio rigenerazione");
  suonaEAttendi(SUONO_PROMEMORIA, sizeof(SUONO_PROMEMORIA) / sizeof(Nota), "promemoria (ogni 5 min)");
  suonaEAttendi(SUONO_RIGEN_FINE, sizeof(SUONO_RIGEN_FINE) / sizeof(Nota), "fine rigenerazione");
  suonaEAttendi(SUONO_INTAS_ALTO, sizeof(SUONO_INTAS_ALTO) / sizeof(Nota), "intasamento al 98%");
  Serial.println("Fine prova suoni.");
}

// promemoria durante la rigenerazione e avviso al 98%
void controllaAvvisiSonori() {
  if (rigenInCorso && millis() - ultimoPromemoriaMs > PROMEMORIA_RIGEN_MS) {
    ultimoPromemoriaMs = millis();
    if (suonoAbilitato[S_PROMEMORIA]) SUONA(SUONO_PROMEMORIA);
  }
  const RisultatoPID& intas = statoPID[P_INTAS].valore;
  bool alto = eraIntasAltoSuono;
  if (intas.valido && datiPresenti()) {
    if (intas.valore >= SOGLIA_INTAS_SCHERMO) alto = true;
    else if (intas.valore < SOGLIA_INTAS_SCHERMO - 0.5f) alto = false;
  }
  if (alto && !eraIntasAltoSuono && !rigenInCorso && suonoAbilitato[S_INTAS_98]) SUONA(SUONO_INTAS_ALTO);
  eraIntasAltoSuono = alto;
}

// ============================================================================
// ACCENSIONE / SPEGNIMENTO SCHERMO
// ============================================================================

void accendiSchermo() {
  digitalWrite(PIN_RETROILLUMINAZIONE, HIGH);
  schermoAcceso = true;
  ultimaAttivitaMs = millis();
}

void spegniSchermo() {
  digitalWrite(PIN_RETROILLUMINAZIONE, LOW);
  schermoAcceso = false;
}

// Vero se lo schermo deve restare acceso anche senza tocchi
bool schermoForzatoAcceso() {
  const RisultatoPID& intas = statoPID[P_INTAS].valore;
  if (intas.valido) {
    if (intas.valore >= SOGLIA_INTAS_SCHERMO) intasamentoAlto = true;
    else if (intas.valore < SOGLIA_INTAS_SCHERMO - 0.5f) intasamentoAlto = false;   // isteresi
  }
  if (!datiPresenti()) intasamentoAlto = false;
  return rigenInCorso || intasamentoAlto ||
         pagina == 3 ||                                         // test sensore in corso
         (pagina == 2 && statoPaginaErrori == ERR_CONFERMA);    // conferma cancellazione
}

void gestisciSchermo() {
  // avviso = rigenerazione, intasamento alto, test sensore, conferma cancellazione
  bool avviso = elmConnesso && schermoForzatoAcceso();
  if (avviso && !eraForzatoAcceso && (rigenInCorso || intasamentoAlto) &&
      (pagina == 1 || pagina == 4 || pagina == 5 || pagina == 6 || (pagina == 2 && statoPaginaErrori != ERR_CONFERMA))) {
    pagina = 0;                       // appena parte l'avviso, mostra la pagina principale
    paginaDaRidisegnare = true;
  }
  bool appenaCollegato = elmConnesso && millis() - collegatoDaMs < OPZ_SCHERMO_MS[opzSchermo];
  unsigned long timeout = OPZ_TIMEOUT_MS[opzTimeout];
  bool sempreAcceso = (timeout == 0);
  // acceso anche mentre cerca l'adattatore e subito dopo il collegamento
  bool forzato = avviso || !elmConnesso || appenaCollegato || sempreAcceso;
  if (forzato) {
    if (!schermoAcceso) accendiSchermo();
    ultimaAttivitaMs = millis();
  } else if (schermoAcceso && millis() - ultimaAttivitaMs > timeout) {
    spegniSchermo();
    if (pagina == 1 || pagina == 4 || pagina == 5 || pagina == 6 || (pagina == 2 && statoPaginaErrori != ERR_CONFERMA)) {
      pagina = 0;                     // al risveglio si riparte dalla pagina principale
      paginaDaRidisegnare = true;
    }
  }
  eraForzatoAcceso = avviso;
}

// ============================================================================
// STORICO RIGENERAZIONI E RIGENERAZIONI INTERROTTE
// ============================================================================
//
// Ogni rigenerazione diventa un record: km dalla precedente, durata,
// temperatura massima, esito. Mentre e' in corso il record viene salvato ogni
// 30 s: se il dispositivo si spegne (= hai spento l'auto) alla riaccensione
// si capisce com'e' andata:
//   - avanzamento ancora sopra 0 -> la centralina la sta RIPRENDENDO
//   - km dall'ultima rigenerazione azzerati -> e' finita mentre eri spento
//   - altrimenti -> INTERROTTA: avviso a schermo

void caricaStorico() {
  prefStorico.begin("dpfstor", false);
  numStorico = prefStorico.getUChar("n", 0);
  if (numStorico > MAX_STORICO) numStorico = 0;
  size_t attesi = sizeof(RecordRigen) * numStorico;
  if (numStorico > 0 && prefStorico.getBytes("h", storico, attesi) != attesi) numStorico = 0;
  rigenApertaValida = prefStorico.getBytes("ap", &rigenAperta, sizeof(RecordRigen)) == sizeof(RecordRigen) &&
                      rigenAperta.esito == 2;
  rigenApertaDaRiavvio = rigenApertaValida;
  segmentoRigenAttivo = false;
  baseDurataS = rigenApertaValida ? rigenAperta.durataS : 0;
  if (rigenApertaDaRiavvio) {
    scriviLog("RIGEN_APERTA_ALL_ACCENSIONE,durata_prima_s=" + String(rigenAperta.durataS) +
              ",avanz_max=" + String(rigenAperta.avanzMax));
  }
}

void salvaRigenAperta() {
  prefStorico.putBytes("ap", &rigenAperta, sizeof(RecordRigen));
  ultimoSalvataggioRigenMs = millis();
}

uint16_t durataRigenS() {
  if (!segmentoRigenAttivo) return baseDurataS;
  unsigned long t = baseDurataS + (millis() - inizioSegmentoMs) / 1000;
  return t > 65000 ? 65000 : (uint16_t)t;
}

void iniziaORiprendiRigen() {
  if (rigenApertaValida && rigenApertaDaRiavvio) {
    // era in corso quando il dispositivo si e' spento: la centralina la riprende
    rigenApertaDaRiavvio = false;
    if (rigenAperta.interruzioni < 255) rigenAperta.interruzioni++;
    baseDurataS = rigenAperta.durataS;
    inizioSegmentoMs = millis();
    segmentoRigenAttivo = true;
    salvaRigenAperta();
    scriviLog("RIGEN_RIPRESA,interruzioni=" + String(rigenAperta.interruzioni) +
              ",durata_prima_s=" + String(baseDurataS));
    versioneStorico++;
    return;
  }
  if (rigenApertaValida) return;      // gia' aperta in questa accensione
  const RisultatoPID& nr = statoPID[P_NRIG].valore;
  const RisultatoPID& km = statoPID[P_KM].valore;
  const RisultatoPID& tf = statoPID[P_TFAP].valore;
  const RisultatoPID& it = statoPID[P_INTAS].valore;
  rigenAperta.numero = nr.valido ? (uint16_t)lroundf(nr.valore + 1) : 0;   // il contatore sale alla fine
  rigenAperta.kmPrima10 = km.valido ? (uint16_t)limitaInt(lroundf(km.valore * 10), 0, 65000) : 0xFFFF;
  rigenAperta.durataS = 0;
  rigenAperta.tMax = tf.valido ? (int16_t)lroundf(tf.valore) : -100;
  rigenAperta.intasInizio = it.valido ? (uint8_t)limitaInt(lroundf(it.valore), 0, 254) : 255;
  rigenAperta.avanzMax = 0;
  rigenAperta.esito = 2;
  rigenAperta.interruzioni = 0;
  baseDurataS = 0;
  inizioSegmentoMs = millis();
  segmentoRigenAttivo = true;
  rigenApertaValida = true;
  salvaRigenAperta();
  versioneStorico++;
}

// durante la rigenerazione: temperatura massima, avanzamento, salvataggio ogni 30 s
void aggiornaRigenAperta() {
  if (!rigenApertaValida || !segmentoRigenAttivo) return;
  const RisultatoPID& tf = statoPID[P_TFAP].valore;
  const RisultatoPID& av = statoPID[P_AVANZ].valore;
  if (tf.valido && tf.valore > rigenAperta.tMax) rigenAperta.tMax = (int16_t)lroundf(tf.valore);
  if (av.valido && av.valore > rigenAperta.avanzMax) rigenAperta.avanzMax = (uint8_t)limitaInt(lroundf(av.valore), 0, 100);
  if (millis() - ultimoSalvataggioRigenMs > 30000) {
    rigenAperta.durataS = durataRigenS();
    salvaRigenAperta();
  }
}

void aggiungiAStorico(const RecordRigen& r) {
  if (numStorico >= MAX_STORICO) {
    for (int i = 1; i < MAX_STORICO; i++) storico[i - 1] = storico[i];
    numStorico = MAX_STORICO - 1;
  }
  storico[numStorico++] = r;
  prefStorico.putBytes("h", storico, sizeof(RecordRigen) * numStorico);
  prefStorico.putUChar("n", numStorico);
  versioneStorico++;
}

// esito: 0 = completa, 1 = interrotta
void chiudiRigen(uint8_t esito) {
  if (!rigenApertaValida) return;
  rigenAperta.durataS = durataRigenS();
  rigenAperta.esito = esito;
  segmentoRigenAttivo = false;
  rigenApertaValida = false;
  rigenApertaDaRiavvio = false;
  prefStorico.remove("ap");
  aggiungiAStorico(rigenAperta);
  scriviLog("STORICO,numero=" + String(rigenAperta.numero) + ",esito=" + String(esito) +
            ",durata_s=" + String(rigenAperta.durataS) + ",tmax=" + String(rigenAperta.tMax) +
            ",interruzioni=" + String(rigenAperta.interruzioni));
}

// all'accensione, con una rigenerazione rimasta aperta: com'e' finita?
void controllaRigenDaRiavvio() {
  if (!rigenApertaDaRiavvio || rigenInCorso) return;   // se riparte la gestisce iniziaORiprendiRigen
  if (statoPID[P_AVANZ].urgente || !statoPID[P_AVANZ].valore.valido) return;
  if (statoPID[P_KM].urgente) return;
  const RisultatoPID& km = statoPID[P_KM].valore;
  if (rigenAperta.interruzioni < 255) rigenAperta.interruzioni++;
  bool kmAzzerati = km.valido && rigenAperta.kmPrima10 != 0xFFFF &&
                    km.valore < rigenAperta.kmPrima10 / 10.0f - 1.0f;
  if (kmAzzerati) {
    scriviLog("RIGEN_FINITA_A_DISPOSITIVO_SPENTO,km_ora=" + String(km.valore, 1));
    chiudiRigen(0);
    return;
  }
  scriviLog("RIGEN_INTERROTTA,durata_s=" + String(rigenAperta.durataS) +
            ",avanz_max=" + String(rigenAperta.avanzMax) + ",km_ora=" + (km.valido ? String(km.valore, 1) : String("ND")));
  int durataMin = (rigenAperta.durataS + 30) / 60;
  avvisoRiga1 = "Interrotta al " + String(rigenAperta.avanzMax) + "% dopo " +
                (rigenAperta.durataS < 60 ? String(rigenAperta.durataS) + " s." : String(durataMin) + " min.");
  avvisoRiga2 = "Appena puoi: 15-20 min a velocita' costante.";
  chiudiRigen(1);
  pagina = 6;
  paginaDaRidisegnare = true;
  accendiSchermo();
  ultimaAttivitaMs = millis();
  if (SUONI_ATTIVI && suonoAbilitato[S_PROMEMORIA]) SUONA(SUONO_PROMEMORIA);
}

String testoDurata(uint16_t s) {
  if (s < 60) return String(s) + " s";
  return String((s + 30) / 60) + " min";
}

void disegnaPaginaAvviso() {
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, 320, BAR_H, C_RED);
  tft.setTextPadding(0);
  tft.setTextColor(C_TEXT, C_RED);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("ATTENZIONE", 160, BAR_H / 2, 4);
  tft.setTextColor(C_RED, C_BG);
  tft.drawString("Rigenerazione FAP", 160, 62, 4);
  tft.drawString("INTERROTTA", 160, 92, 4);
  tft.setTextColor(C_TEXT, C_BG);
  tft.drawString(avvisoRiga1, 160, 128, 2);
  tft.drawString(avvisoRiga2, 160, 150, 2);
  tft.setTextColor(C_LABEL, C_BG);
  tft.drawString("(extraurbano, sopra i 2000 giri)", 160, 168, 2);
  tft.drawString("Vedi la pagina Storico.", 160, 194, 2);
  tft.drawString("tocca per chiudere", 160, 225, 2);
}

// colonne della tabella (destra di ogni colonna)
const int ST_X_NUM = 12, ST_X_KM = 120, ST_X_DUR = 172, ST_X_TMAX = 220, ST_X_ESITO = 308;
const int ST_Y_TESTA = 46, ST_Y0 = 64, ST_PASSO = 19, ST_RIGHE = 6;

void disegnaRigaStorico(int y, const RecordRigen& r, bool aperta) {
  tft.setTextPadding(0);
  tft.setTextColor(C_TEXT, C_BG);
  tft.setTextDatum(ML_DATUM);
  tft.drawString(r.numero ? String(r.numero) : String("-"), ST_X_NUM, y, 2);
  tft.setTextDatum(MR_DATUM);
  tft.drawString(r.kmPrima10 == 0xFFFF ? String("-") : String((r.kmPrima10 + 5) / 10), ST_X_KM, y, 2);
  tft.drawString(aperta ? testoDurata(durataRigenS()) : testoDurata(r.durataS), ST_X_DUR, y, 2);
  tft.drawString(r.tMax <= -100 ? String("-") : String(r.tMax), ST_X_TMAX, y, 2);
  String esito; uint16_t col;
  if (aperta)                 { esito = "in corso"; col = C_ORANGE; }
  else if (r.esito == 1)      { esito = "INTERROTTA"; col = C_RED; }
  else if (r.interruzioni > 0){ esito = "ripresa"; col = C_ORANGE; }
  else                        { esito = "OK"; col = C_GREEN; }
  tft.setTextColor(col, C_BG);
  tft.drawString(esito, ST_X_ESITO, y, 2);
}

void disegnaPaginaStorico() {
  versioneStoricoDisegnata = versioneStorico;
  cachePiedeStorico = "";
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, 320, BAR_H, C_TILE);
  tft.setTextPadding(0);
  tft.setTextColor(C_TEXT, C_TILE);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("Storico", 12, BAR_H / 2, 4);
  tft.setTextColor(C_LABEL, C_TILE);
  tft.setTextDatum(MR_DATUM);
  tft.drawString("tocca per continuare", 308, BAR_H / 2, 2);

  bool aperta = rigenApertaValida && !rigenApertaDaRiavvio;
  if (numStorico == 0 && !aperta) {
    disegnaTestoCentrato("Nessuna rigenerazione registrata.", 100, 2, C_TEXT);
    disegnaTestoCentrato("Lo storico si riempie da solo", 124, 2, C_LABEL);
    disegnaTestoCentrato("a ogni rigenerazione.", 142, 2, C_LABEL);
  } else {
    tft.setTextColor(C_LABEL, C_BG);
    tft.setTextDatum(ML_DATUM);
    tft.drawString("N.", ST_X_NUM, ST_Y_TESTA, 2);
    tft.setTextDatum(MR_DATUM);
    tft.drawString("km prima", ST_X_KM, ST_Y_TESTA, 2);
    tft.drawString("durata", ST_X_DUR, ST_Y_TESTA, 2);
    tft.drawString("T max", ST_X_TMAX, ST_Y_TESTA, 2);
    tft.drawString("esito", ST_X_ESITO, ST_Y_TESTA, 2);
    tft.drawFastHLine(12, ST_Y_TESTA + 9, 296, C_TILE);
    int riga = 0;
    if (aperta) disegnaRigaStorico(ST_Y0 + (riga++) * ST_PASSO, rigenAperta, true);
    for (int i = numStorico - 1; i >= 0 && riga < ST_RIGHE; i--) {
      disegnaRigaStorico(ST_Y0 + (riga++) * ST_PASSO, storico[i], false);
    }
  }

  // media km tra rigenerazioni e quante interrotte
  long sommaKm = 0; int nKm = 0, nInterrotte = 0;
  for (int i = 0; i < numStorico; i++) {
    if (storico[i].kmPrima10 != 0xFFFF) { sommaKm += storico[i].kmPrima10; nKm++; }
    if (storico[i].esito == 1 || storico[i].interruzioni > 0) nInterrotte++;
  }
  tft.drawFastHLine(12, 184, 296, C_TILE);
  String riepilogo = nKm > 0 ? "Media " + String((int)lroundf(sommaKm / 10.0f / nKm)) + " km tra rigen."
                             : String("Media km: non ancora disponibile");
  if (numStorico > 0) riepilogo += " - interrotte " + String(nInterrotte) + "/" + String(numStorico);
  disegnaTestoCentrato(riepilogo, 196, 2, C_TEXT);
}

void aggiornaPaginaStorico() {
  if (versioneStorico != versioneStoricoDisegnata) { paginaDaRidisegnare = true; return; }
  // piede: valori di prova (degrado olio, batteria)
  String olio = pidNonSupportato[P_OLIO] ? String("n.d.") : formatta(mostrato(P_OLIO), 0) + "%";
  String batt = pidNonSupportato[P_BATT] ? String("n.d.") : formatta(mostrato(P_BATT), 1) + " V";
  String piede = "Degrado olio " + olio + "    Batteria " + batt;
  if (pidNonSupportato[P_OLIO] && pidNonSupportato[P_BATT]) piede = "Olio e batteria: non letti da questa centralina";
  if (piede == cachePiedeStorico) return;
  cachePiedeStorico = piede;
  tft.setTextColor(C_LABEL, C_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(310);
  tft.drawString(piede, 160, 222, 2);
  tft.setTextPadding(0);
}

void stampaStoricoSuSeriale() {
  Serial.println("=== STORICO RIGENERAZIONI (dalla piu' vecchia) ===");
  Serial.println("numero,km_prima,durata_s,tmax,intas_inizio,avanz_max,esito(0=ok;1=interrotta),interruzioni");
  for (int i = 0; i < numStorico; i++) {
    const RecordRigen& r = storico[i];
    Serial.println(String(r.numero) + "," + (r.kmPrima10 == 0xFFFF ? String("ND") : String(r.kmPrima10 / 10.0f, 1)) + "," +
                   String(r.durataS) + "," + String(r.tMax) + "," + String(r.intasInizio) + "," +
                   String(r.avanzMax) + "," + String(r.esito) + "," + String(r.interruzioni));
  }
  if (rigenApertaValida) Serial.println("(una rigenerazione e' ancora aperta)");
  Serial.println("=== FINE STORICO ===");
}

// ============================================================================
// PAGINA SUONI (impostazioni)
// ============================================================================

const int SUONI_Y0 = 40, SUONI_RIGA_H = 22;
const int RIGA_TIMEOUT = NUM_SUONI;          // spegnimento schermo
const int RIGA_COLLEGAMENTO = NUM_SUONI + 1; // schermo acceso al collegamento
const int NUM_RIGHE_OPZIONI = NUM_SUONI + 2;   // 5 suoni + 2 schermo

void caricaImpostazioniSuoni() {
  prefSuoni.begin("dpfsuoni", false);
  for (int i = 0; i < NUM_SUONI; i++) {
    suonoAbilitato[i] = prefSuoni.getBool(CHIAVI_SUONI[i], true);
  }
  opzSchermo = prefSuoni.getUChar("scrColl", SCHERMO_DOPO_COLLEGAMENTO_DEFAULT);
  opzTimeout = prefSuoni.getUChar("scrTmo", SCHERMO_TIMEOUT_DEFAULT);
  if (opzTimeout < 0 || opzTimeout >= NUM_OPZ_TIMEOUT) opzTimeout = SCHERMO_TIMEOUT_DEFAULT;
  if (opzSchermo < 0 || opzSchermo >= NUM_OPZ_SCHERMO) opzSchermo = SCHERMO_DOPO_COLLEGAMENTO_DEFAULT;
}

void disegnaRigaSuono(int i) {
  int y = SUONI_Y0 + i * SUONI_RIGA_H;
  tft.fillRect(0, y, 320, SUONI_RIGA_H, C_BG);
  tft.setTextPadding(0);
  tft.setTextColor(C_TEXT, C_BG);
  tft.setTextDatum(ML_DATUM);
  if (i == RIGA_TIMEOUT || i == RIGA_COLLEGAMENTO) {
    bool tmo = (i == RIGA_TIMEOUT);
    tft.drawString(tmo ? "Spegni schermo dopo" : "Schermo acceso al collegam.", 12, y + SUONI_RIGA_H / 2, 2);
    bool mai = tmo && OPZ_TIMEOUT_MS[opzTimeout] == 0;
    uint16_t sfondo = mai ? C_ORANGE : C_TILE;
    tft.fillRoundRect(244, y + 2, 64, 18, 9, sfondo);
    tft.setTextColor(mai ? C_BG : C_TEXT, sfondo);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(tmo ? OPZ_TIMEOUT_TESTO[opzTimeout] : OPZ_SCHERMO_TESTO[opzSchermo], 276, y + SUONI_RIGA_H / 2, 2);
    if (tmo) tft.drawFastHLine(12, y + SUONI_RIGA_H - 1, 296, C_TILE);
    return;
  }
  tft.drawString(NOMI_SUONI[i], 12, y + SUONI_RIGA_H / 2, 2);
  bool on = suonoAbilitato[i];
  uint16_t sfondo = on ? C_GREEN : C_BARBG;
  tft.fillRoundRect(252, y + 2, 56, 18, 9, sfondo);
  tft.setTextColor(on ? C_BG : C_TEXT, sfondo);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(on ? "SI" : "NO", 280, y + SUONI_RIGA_H / 2, 2);
  tft.drawFastHLine(12, y + SUONI_RIGA_H - 1, 296, C_TILE);
}

void disegnaPaginaSuoni() {
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, 320, BAR_H, C_TILE);
  tft.setTextPadding(0);
  tft.setTextColor(C_TEXT, C_TILE);
  tft.setTextDatum(ML_DATUM);
  tft.drawString("Opzioni", 12, BAR_H / 2, 4);
  tft.setTextColor(C_LABEL, C_TILE);
  tft.setTextDatum(MR_DATUM);
  tft.drawString("tocca una riga per cambiare", 308, BAR_H / 2, 2);
  for (int i = 0; i < NUM_RIGHE_OPZIONI; i++) disegnaRigaSuono(i);
  disegnaPulsante(4, PULS_Y, 150, PULS_H, "Errori", C_TILE, C_TEXT);
  disegnaPulsante(166, PULS_Y, 150, PULS_H, "Esci", C_TILE, C_TEXT);
}

void toccoPaginaSuoni(int px, int py) {
  if (py >= PULS_Y - 4) {
    if (px < 160) {                    // Errori
      pagina = 2;
      if (statoPaginaErrori != ERR_CONFERMA) statoPaginaErrori = erroriLetti ? ERR_LISTA : ERR_MAI_LETTI;
    } else {                           // Esci
      pagina = 0;
    }
    paginaDaRidisegnare = true;
    return;
  }
  if (py < SUONI_Y0) return;
  int i = (py - SUONI_Y0) / SUONI_RIGA_H;
  if (i < 0 || i >= NUM_RIGHE_OPZIONI) return;
  if (i == RIGA_TIMEOUT) {              // spegnimento schermo: valore successivo
    opzTimeout = (opzTimeout + 1) % NUM_OPZ_TIMEOUT;
    prefSuoni.putUChar("scrTmo", opzTimeout);
    scriviLog(String("IMPOSTAZIONE_SPEGNIMENTO_SCHERMO,") + OPZ_TIMEOUT_TESTO[opzTimeout]);
    ultimaAttivitaMs = millis();
    disegnaRigaSuono(i);
    return;
  }
  if (i == RIGA_COLLEGAMENTO) {         // schermo dopo il collegamento: valore successivo
    opzSchermo = (opzSchermo + 1) % NUM_OPZ_SCHERMO;
    prefSuoni.putUChar("scrColl", opzSchermo);
    scriviLog(String("IMPOSTAZIONE_SCHERMO_COLLEGAMENTO,") + OPZ_SCHERMO_TESTO[opzSchermo]);
    disegnaRigaSuono(i);
    return;
  }
  suonoAbilitato[i] = !suonoAbilitato[i];
  prefSuoni.putBool(CHIAVI_SUONI[i], suonoAbilitato[i]);
  scriviLog(String("IMPOSTAZIONE_SUONO,") + CHIAVI_SUONI[i] + "=" + (suonoAbilitato[i] ? "SI" : "NO"));
  disegnaRigaSuono(i);
  if (suonoAbilitato[i]) {             // anteprima del suono appena attivato
    switch (i) {
      case S_AVVIO:        SUONA(SUONO_AVVIO); break;
      case S_RIGEN_INIZIO: SUONA(SUONO_RIGEN_INIZIO); break;
      case S_PROMEMORIA:   SUONA(SUONO_PROMEMORIA); break;
      case S_RIGEN_FINE:   SUONA(SUONO_RIGEN_FINE); break;
      case S_INTAS_98:     SUONA(SUONO_INTAS_ALTO); break;
    }
  }
}

// ============================================================================
// MESSAGGI A SCHERMO INTERO (connessione, configurazione)
// ============================================================================

void disegnaMessaggio(String msg) {
  tft.fillScreen(COL_BG);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(COL_LABEL, COL_BG);
  tft.setTextSize(2);
  int righe = 1;
  for (unsigned int i = 0; i < msg.length(); i++) if (msg.charAt(i) == '\n') righe++;
  int y = 120 - righe * 9;
  if (y < 8) y = 8;
  tft.setCursor(10, y);
  tft.println(msg);
  tft.setTextSize(1);
  paginaDaRidisegnare = true;   // la dashboard andra' ridisegnata da capo
}

// ============================================================================
// LOG EVENTI
// ============================================================================

void scriviLog(String riga) {
  String rigaConTempo = String(millis()) + "," + riga;
  fileLog = LittleFS.open("/eventi_fap.csv", "a");
  if (fileLog) {
    fileLog.println(rigaConTempo);
    fileLog.close();
  }
  Serial.println(rigaConTempo);
}

String pulisci(String risposta) {
  String r = risposta;
  r.replace(" ", "");
  r.replace("\r", "\\r");
  r.replace("\n", "\\n");
  r.replace(">", "");
  r.replace(",", ";");
  return r;
}

void stampaLogSuSeriale() {
  Serial.println("=== INIZIO DUMP eventi_fap.csv ===");
  fs::File f = LittleFS.open("/eventi_fap.csv", "r");
  if (!f) {
    Serial.println("(nessun file trovato)");
    return;
  }
  while (f.available()) {
    Serial.write(f.read());
  }
  f.close();
  Serial.println("");
  Serial.println("=== FINE DUMP ===");
}
