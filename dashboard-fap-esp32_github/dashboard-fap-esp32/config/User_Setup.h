// User_Setup.h per la libreria TFT_eSPI
// Configurazione specifica per la scheda CYD ESP32-2432S028(R)
//
// ISTRUZIONI: questo file NON va compilato direttamente e non va
// messo nella cartella del progetto insieme allo sketch .ino.
// Va copiato DENTRO la cartella della libreria TFT_eSPI (che si crea
// installando "TFT_eSPI" da Arduino IDE > Strumenti > Gestione librerie),
// sostituendo il file User_Setup.h che trovi gia' li' dentro.
//
// Dove si trova la cartella della libreria:
//   Windows: Documenti\Arduino\libraries\TFT_eSPI\User_Setup.h
//   Mac:     ~/Documents/Arduino/libraries/TFT_eSPI/User_Setup.h
//   Linux:   ~/Arduino/libraries/TFT_eSPI/User_Setup.h

#define ILI9341_DRIVER

#define TFT_MISO 12
#define TFT_MOSI 13
#define TFT_SCLK 14
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST  -1   // il reset non e' collegato su questa scheda
#define TFT_BL   21   // pin della retroilluminazione
#define TFT_BACKLIGHT_ON HIGH

// Touchscreen resistivo XPT2046 (bus SPI separato, VSPI)
#define TOUCH_CS 33

#define SPI_FREQUENCY       40000000
#define SPI_READ_FREQUENCY  20000000
#define SPI_TOUCH_FREQUENCY  2500000

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT
