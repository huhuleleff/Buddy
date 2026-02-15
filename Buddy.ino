#include "frame.h"
#include "IkonaGraf.h"
#include "IkonaGraf1.h"
#include "IkonaZob.h"
#include "IkonaZob1.h"
#include "trigozadje.h"
// ESP32-S3 Reset Debugging
#include "esp_heap_trace.h"
#include "esp_system.h"
#include "esp_debug_helpers.h"

// Reset reason tracking
struct ResetInfo {
  int reset_count;
  int last_reset_reason;
  unsigned long last_reset_time;
  int free_heap_before;
  int min_free_heap;
} reset_info;

// Save reset info to RTC memory
RTC_DATA_ATTR int rtc_reset_count = 0;
RTC_DATA_ATTR int rtc_last_reason = 0;

// Function to get detailed reset reason
String getResetReason() {
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_UNKNOWN:    return "Unknown reset";
    case ESP_RST_POWERON:    return "Power on reset";
    case ESP_RST_EXT:        return "External pin reset";
    case ESP_RST_SW:         return "Software reset";
    case ESP_RST_PANIC:      return "Exception/panic reset";
    case ESP_RST_INT_WDT:    return "Interrupt watchdog reset";
    case ESP_RST_TASK_WDT:   return "Task watchdog reset";
    case ESP_RST_WDT:        return "Other watchdog reset";
    case ESP_RST_DEEPSLEEP:  return "Deep sleep reset";
    case ESP_RST_BROWNOUT:   return "Brownout reset";
    case ESP_RST_SDIO:       return "SDIO reset";
    default:                 return "Unknown";
  }
}

// Function to read ESP32-S3 internal temperature
float getInternalTemperature() {
  // ESP32-S3 internal temperature sensor
  // Note: This is not very accurate, but gives a rough estimate
  return temperatureRead(); // Returns temperature in Celsius
}

// Function to calculate Saturated Vapor Pressure (SVP) using Magnus-Tetens formula
float calculateSVP(float temperatureC) {
  return 0.61078 * exp((17.27 * temperatureC) / (temperatureC + 237.3));
}

// Function to calculate Vapor Pressure Deficit (VPD)
float calculateVPD(float temperatureC, float relativeHumidity) {
  float svp = calculateSVP(temperatureC);
  return svp * (1.0 - (relativeHumidity / 100.0));
}

// Function to get VPD status description
String getVPDStatus(float vpd) {
  if (vpd < 0.4) {
    return "Too Low - Risk of mold/rot";
  } else if (vpd >= 0.4 && vpd < 0.8) {
    return "Low - Reduced transpiration";
  } else if (vpd >= 0.8 && vpd <= 1.2) {
    return "Ideal - Optimal growth";
  } else if (vpd > 1.2 && vpd <= 1.6) {
    return "Good - Acceptable range";
  } else if (vpd > 1.6) {
    return "Too High - Plant stress";
  } else {
    return "Unknown";
  }
}

// Function prototype for OTA update
void doOTA();

// Function prototype for automatic update checking
bool checkForUpdates(String& firmwareUrl);

// Function to log reset info
void logResetInfo() {
  Serial.println("\n=== ESP32-S3 RESET DEBUG INFO ===");
  Serial.printf("Reset Reason: %s\n", getResetReason().c_str());
  Serial.printf("Reset Count: %d\n", rtc_reset_count);
  Serial.printf("Free Heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Min Free Heap: %d bytes\n", ESP.getMinFreeHeap());
  Serial.printf("Largest Free Block: %d bytes\n", ESP.getMaxAllocHeap());
  Serial.printf("Total PSRAM: %d bytes\n", ESP.getPsramSize());
  Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());
  Serial.printf("Flash Size: %d bytes\n", ESP.getFlashChipSize());
  Serial.printf("CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  Serial.printf("SDK Version: %s\n", ESP.getSdkVersion());
  Serial.println("================================\n");
}

// Function to setup heap tracing (optional - for advanced debugging)
void setupHeapTracing() {
  // Enable heap tracing for memory leak detection
  const size_t num_records = 100;
  static heap_trace_record_t trace_record[num_records];
  
  heap_trace_init_standalone(trace_record, num_records);
  heap_trace_start(HEAP_TRACE_LEAKS);
}

// Function to print heap usage
void printHeapUsage(const char* location) {
  Serial.printf("[%s] Heap: %d bytes, Min: %d, Largest: %d\n", 
                location, ESP.getFreeHeap(), ESP.getMinFreeHeap(), ESP.getMaxAllocHeap());
}



#include <SPI.h>
#include <TFT_eSPI.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ADS1115_WE.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "nadzornikprostora.h"
#include <HTTPClient.h>
#include <NTPClient.h>
#include <Update.h>
#include <WiFiUdp.h>
#include <HTTPUpdate.h>
#include "SheetFan.h"
#include "potenciometer.h"
#include "configicon.h"
#include "configicon2.h"
#include "pinoutcfg.h"
#include "ikonacfg.h"
#include "ikonacfgizbrana.h"
#include <iostream>
#include <algorithm>
#include <FS.h>
#include "manager.h"
#include <esp_task_wdt.h>
#include <ESPmDNS.h>
#include <SD.h>
#include <FFat.h>
#include <TJpg_Decoder.h>
#define FS_NO_GLOBALS
#include <AnimatedGIF.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <sys/time.h>  // For RTC functions
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// ============================================================================
// AUTOMATIC FIRMWARE UPDATE SYSTEM
// ============================================================================

// Compile-time flag to enable/disable automatic updates
#define AUTO_UPDATE_ENABLED true

// Configuration constants
const char* versionCheckURL = "https://raw.githubusercontent.com/huhuleleff/Buddy/refs/heads/main/version.json";
const unsigned long UPDATE_CHECK_INTERVAL = 86400000; // 24 hours in milliseconds

// Automatic firmware version calculated at compile time
const String firmwareVersion = String(__DATE__) + " " + String(__TIME__);

// ============================================================================

TaskHandle_t gifTaskHandle = NULL;
using namespace std;

WiFiClientSecure client;
#define BME280_ADDRESS 0x76
Adafruit_BME280 bme; 

// Define a structure to hold date information along with its original index
struct Date {
  int dnevivzorec;
  int mesecvzorec;
  int leto;
  int index;  // original index of the date

  // Default constructor
  Date()
    : dnevivzorec(0), mesecvzorec(0), leto(0), index(0) {}

  // Constructor
  Date(int d, int m, int y, int i)
    : dnevivzorec(d), mesecvzorec(m), leto(y), index(i) {}
};

// Comparison function for sorting the dates
bool compareDates(const Date& date1, const Date& date2) {

  if (date1.leto != date2.leto)
    return date1.leto < date2.leto;
  if (date1.mesecvzorec != date2.mesecvzorec)
    return date1.mesecvzorec < date2.mesecvzorec;
  return date1.dnevivzorec < date2.dnevivzorec;
}
const int size = 366;
int arraysinorder[size];

#define I2C_ADDRESS 0x48
//#define DHTPIN 17
//#define DHTTYPE DHT22  // DHT 22 (AM2302)
ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);
//DHT_Unified dht(DHTPIN, DHTTYPE);
sensors_event_t event;

String discordWebhookUrl = "https://discord.com/api/webhooks/YOUR_WEBHOOK_ID/YOUR_WEBHOOK_TOKEN";
String defaultFirmwareUrl = "https://raw.githubusercontent.com/huhuleleff/Buddy/refs/heads/main/Buddy.ino.bin";
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;     // GMT+1 for Slovenia (adjust for your timezone)
const int daylightOffset_sec = 3600; // Daylight saving time offset (3600 for summer, 0 for winter)
WiFiUDP udp;
NTPClient timeClient(udp, ntpServer, gmtOffset_sec);

// RTC time tracking variables
unsigned long lastNTPSync = 0;
const unsigned long NTP_SYNC_INTERVAL = 86400000; // 24 hours in milliseconds
unsigned long lastSecondCheck = 0;
bool rtcInitialized = false;
bool ntpSyncAttempted = false;

// OTA timing variables
unsigned long wifiConnectionTime = 0;
bool otaTriggered = false;
const unsigned long OTA_DELAY = 15000; // 15 seconds delay

// Additional variables for delayed OTA from web updates
bool otaPendingFromWeb = false;
unsigned long otaWebRequestTime = 0;
const unsigned long OTA_WEB_DELAY = 10000; // 10 seconds delay for web-triggered updates

// Flag to track if update check has been performed after WiFi connection
bool updateCheckDone = false;

// WiFi health check variables
unsigned long lastWiFiHealthCheck = 0;
const unsigned long WIFI_HEALTH_CHECK_INTERVAL = 30000; // 30 seconds
unsigned long lastServerRestart = 0;
const unsigned long SERVER_RESTART_INTERVAL = 3600000; // 1 hour

#define TFT_CS 1
#define TFT_RST -1
#define TFT_DC 2
//definition for comunication with stm32, custom protocol
#define DATA_PIN          15    // GPIO15 - Shared data line
#define DISCOVER_PIN      16    // GPIO16 - Discovery chain control
#define DATA_BAUD         9600  // Bit rate for DATA line

// Protocol Timing (microseconds)
#define BIT_PERIOD        (1000000 / DATA_BAUD)
#define DISCOVERY_TIMEOUT 500000  // 500ms timeout per node
#define ACK_TIMEOUT       100000  // 100ms for acknowledgment
#define INTER_CMD_DELAY   50000   // 50ms between commands
#define DATA_TIMEOUT      100000  // 100ms for data response

// Protocol Commands
#define CMD_DISCOVERY_ASSIGN  0xDA  // Discovery-assignment command
#define CMD_VERIFY_ADDRESS    0x5B  // Verify address command
#define CMD_PING              0x50  // Ping command
#define CMD_READ_DATA         0x52  // Read data command ('R' = 0x52)
#define RESPONSE_ACK          0xA5  // Acknowledgment
#define RESPONSE_NACK         0x5A  // Negative acknowledgment


#define TIPKAGOR_PIN 5
#define TIPKADOL_PIN 4
#define TIPKAPOTRDI_PIN 7
#define TIPKANAZAJ_PIN 6
#define RELE1_PIN 21
#define RELE2_PIN 47
#define PWM1_PIN 11
#define PWM2_PIN 12

// UART communication with STM32
#define ESP_TX_PIN 9    // ESP32 pin 9 -> STM32 pin 10
#define ESP_RX_PIN 10   // ESP32 pin 10 -> STM32 pin 9
#define STM32_UART_BAUD 115200
#define STM32_UART Serial1  // Use UART1 for STM32 communication

// Encoder variables from STM32
int encoderValue = 0;
bool encoderButtonPressed = false;
unsigned long lastEncoderUpdate = 0;

hw_timer_t* timer = NULL;
hw_timer_t* timer2 = NULL;

//void IRAM_ATTR onTimer();
TFT_eSPI tft = TFT_eSPI();
AnimatedGIF gif;

// Forward declarations for global variables
extern uint8_t discovered_nodes;
extern uint8_t node_addresses[];
extern bool discovery_complete;

void drawGraph();
void racunaj();
void izrismreze();
void izrismenugraf();
float bezierPoint(float a, float b, float c, float d, float t);
void tipke();
void osvezigraf();
void vzorec();
void izrisglavnimenu();
void prikaziVlagazemlje();
String napovedVremena();
void meritev();
void drawHorizontalMenu();
void ura();
void grafmoci();
void drawLargeCircle();
void dolocimocssr();
void zcross();
void upravljajmoci();
void nastavitve();
void nalozivse();
void shranivse();
void populateDebugData();
bool hasExistingData();
bool clearAllDataFiles();
void smitt();
void racunajsmit();
void urnik();
void updateSensorDataTask(void* parameter);
void runDiscovery();
void verifyAllAddresses();
void sendByte(uint8_t data);
bool sendDiscoveryCommand(uint8_t address);
bool pingNode(uint8_t address);
bool receiveByte(uint8_t *data, unsigned long timeout_us);
uint8_t calculateChecksum(uint8_t cmd, uint8_t addr);
bool readDataFromNode(uint8_t address, uint16_t *value);
uint8_t calculateDataChecksum(uint8_t data_high, uint8_t data_low);

bool saveStringToSPIFFS(const char* path, const String& value);
bool loadStringFromSPIFFS(const char* path, String& value);
int dayOfWeek();
bool ticktajmer;

// Pointer safety validation functions
bool isValidPointer2D(char (*ptr)[365][25]) {
  return ptr != nullptr;
}

bool isValidPointer1D(char (*ptr)[365]) {
  return ptr != nullptr;
}

bool isValidArrayIndex(int index, int maxIndex) {
  return (index >= 0 && index < maxIndex);
}

bool isValidArrayIndex2D(int index1, int index2, int maxIndex1, int maxIndex2) {
  return (index1 >= 0 && index1 < maxIndex1 && index2 >= 0 && index2 < maxIndex2);
}

bool isValidArraysinorderIndex(int index) {
  return (index >= 0 && index < 366 && arraysinorder[index] != 1001 && arraysinorder[index] >= 0 && arraysinorder[index] < 365);
}


void sendDiscordMessage(String message) {
  HTTPClient http;
  http.begin(discordWebhookUrl);
  http.addHeader("Content-Type", "application/json");
  
  String payload = "{\"content\":\"" + message + "\"}";
  int httpResponseCode = http.POST(payload);
  
  if (httpResponseCode > 0) {
    Serial.printf("Discord message sent successfully, response code: %d\n", httpResponseCode);
  } else {
    Serial.printf("Error sending Discord message, response code: %d\n", httpResponseCode);
  }
  http.end();
}

const unsigned char numDataPoints = 24;
const unsigned char numDataPointsM = 31;
int indeksdnevi;
static char tempPodat[365][25];
static char RH[365][25];
static char senzor1[365][25];
static char senzor2[365][25];
static char senzor3[365][25];
static char senzor4[365][25];

static unsigned char indeksUre;
static char najvisjadnevnaT[365];
static char najnizjadnevnaT[365];
static char povprecnadnevnaT[365];
static char najvisjadnevnaRH[365];
static char najnizjadnevnaRH[365];
static char povprecnadnevnaRH[365];
static char najnizjadnevnasens1[365];
static char najnizjadnevnasens2[365];
static char najnizjadnevnasens3[365];
static char najnizjadnevnasens4[365];

unsigned char x = 0;
unsigned char x2 = 0;
int x3 = 0;
unsigned char x4 = 0;
float temperatura2;
float pritisk2;
float vlaznost2;
float vpd2; // Vapor Pressure Deficit
char  temperatura;
char ZrakkPa;
char pritisk;
char vlaznost;

// Current day real-time max/min tracking variables
float trenutniDnevniMaxTemp = -999.0;
float trenutniDnevniMinTemp = 999.0;
float trenutniDnevniMaxRH = -999.0;
float trenutniDnevniMinRH = 999.0;
float prejsnjaTemperatura = -999.0;
float prejsnjaVlaznost = -999.0;
bool tempRising = false;
bool tempFalling = false;
bool vlaznostRising = false;
bool vlaznostFalling = false;
int trenutniDan = -1; // Track current day to reset variables at midnight


#define MAX_SOIL_SENSORS 99
#define LOGGED_SENSORS_COUNT 4
char vlagazemlje[MAX_SOIL_SENSORS];
int stPrikazanihVlagazemlje = 4;  // Number of soil moisture sensors to display (1-4)
int selectedSensors[LOGGED_SENSORS_COUNT] = {1, 2, 3, 4};  // Which sensors to log (default: sensors 1-4)
float prejsnjaPritiska = 0;  // Previous pressure reading for hourly calculation
float spremembaPritiskaNaUro = 0;  // Hourly pressure change rate
String pot1 = "Vent";
String pot2 = "Lum1";
String pot3 = "Lum2";
String pot4 = "Luc1";
String pot5 = "Luc2";

int temperatureMin = 0;
int temperatureMax = 40;
unsigned long casovnik;
unsigned long casovnik2;
unsigned long casovnik4;
int casovnikflag;
bool flagglavnimeniizrisan;
int previousWiFiStatus = -1;
bool prvo = 1;
bool flagmesecnigraf = 0;
bool flagmenugraf;
bool flagmenuMDt = 0;
bool flagmenuPDt = 0;
bool flagmenuMiDt = 0;
bool flagmenuMRH = 0;
bool flagmenuPDRH = 0;
bool flagmenuMiRH = 0;
bool flagmenuMvl;
bool flagmenuPvl;
bool flagmenuMivl;
int flagmenugsnz;
bool flagmenuMsnz;
bool flagmenuPsnz;
bool flagmenuMinsnz;
bool flagmenuPPsnz;
int navimenugraf = 0;
bool flagzaklenivstop1;
int indeksUrepuscice;
bool flagnarisan;
bool flagzaklenivstop2;
bool flagzaklenivstop3;
bool flagzaklenivstop4;
bool flagmenuTemp = 1;
bool flagmenuRH;
bool flagmenuSNZ1;
bool flagmenuSNZ2;
bool flagmenuSNZ3;
bool flagmenuSNZ4;
bool flagmenusens1;
bool flagmenusens2;
bool flagmenusens3;
bool flagmenusens4;
bool flagdnevnigraf = 0;
bool flagglavnimenu = 1;
int prikazanindeks;
unsigned int mesec;
unsigned int leto;
unsigned int ure;
unsigned int minuta;
unsigned int sekunde;
unsigned int dnevi;

unsigned int buffermesec;
unsigned int bufferleto;
unsigned int bufferure;
unsigned int bufferminuta;
unsigned int buffersekunde;
unsigned int bufferdnevi;

static int mesecvzorec[365];
static int letovzorec[365];
static int dnevivzorec[365];
int tok;
bool flagikonagraf;
bool flagmenuglavnimenu;
bool flagizrisanglavnipodmeni;
bool flagikonanast;
unsigned char indeksglavnipodmeni;
bool flaginfo;
char (*pointerracunaj)[365][25] = nullptr;
char (*pointerracunaj2)[365] = nullptr;
char (*pointerracunaj3)[365] = nullptr;
char (*pointerracunaj4)[365] = nullptr;
char (*pointergraf)[365][25] = nullptr;
char (*pointergrafobdobja)[365] = nullptr;
unsigned char obdobje;

// Initialize all pointers to safe values
void initializePointers() {
  pointerracunaj = nullptr;
  pointerracunaj2 = nullptr;
  pointerracunaj3 = nullptr;
  pointerracunaj4 = nullptr;
  pointergraf = nullptr;
  pointergrafobdobja = nullptr;
}
unsigned char x5 = 28;
unsigned char x6 = 4;
unsigned char x7 = 31;
unsigned char x8 = 0;
float voltage = 0.0;
float voltage2 = 0.0;
float voltage3 = 0.0;
float voltage4 = 0.0;
bool flagposlano[MAX_SOIL_SENSORS];  // Dynamic alert flags for each sensor
bool flagposlano5;
bool flagposlano6;
bool flagposlano7 = 0;  // Flag for startup message
bool flagposlano8 = 0;  // Flag for VPD alerts
bool stanje;
int frekvenca;
bool zaklep;
bool utripaj;
int indeksvzorca;
bool flaggrafmoci;
bool grafmocizrisan;
bool powerGraphStaticDrawn;
unsigned char linijamoci[15][10]= {
  { 10, 20, 30, 40, 60, 80, 90, 100, 100, 100 },
  {}, {}, {}, {},
  { 10, 20, 30, 40, 60, 80, 90, 100, 100, 100 }
};
unsigned char izbirnik;
bool flagvstopizbirnik;
signed char procenti[15];
bool flagupravljajmoc;
signed char pt1;
signed char pt2 = 50;
signed char pt3;
signed char pt4;
signed char pt5;

int psm1 = 124;
int psm2 = 124;
unsigned char izbirnikmoci, izbirniknastavitve;
bool flagizbranpt;
bool flagSensorEditing = false;  // Flag for sensor number editing mode
int zamiktriak = 7000;
unsigned char trajanjepulza = 1;
bool flagpulz;
bool flagizbrancfg;
bool flagikonacfg;
int zeljenmesec = 4;
float gain[5][4] = {
  { 1.00, 1.00, 1.00, 1.00 },
  { 1.00, 1.00, 1.00, 1.00 },
  { 1.00, 1.00, 1.00, 1.00 },
  { 1.00, 1.00, 1.00, 1.00 },
  { 1.00, 1.00, 1.00, 1.00 }
};

float kalibracijamin[MAX_SOIL_SENSORS];  // Dry soil (high voltage, low moisture)
float kalibracijamax[MAX_SOIL_SENSORS];  // Wet soil (low voltage, high moisture)
static unsigned long casovnikwifi;
signed char indekspoz;
signed char indekspoz2;
signed char procent1, procent2, procent3, procent4, procent5, procentoff, nulll = -1;
signed char* mesto[5][5];
String tekstnapis[10]{ "IZKLP", "URNIK", "GRAF0", "GRAF1", "GRAF2", "GRAF3", "SENS0", "SENS1", "SENS2", "SENS3" };
String napis[5][5]= {
  { tekstnapis[0], tekstnapis[1], tekstnapis[2], tekstnapis[3], tekstnapis[4] },
  { tekstnapis[0], tekstnapis[1], tekstnapis[2], tekstnapis[3], tekstnapis[4] },
  { tekstnapis[0], tekstnapis[1], tekstnapis[2], tekstnapis[3], tekstnapis[4] },
  { tekstnapis[0], tekstnapis[1], tekstnapis[6], tekstnapis[7], tekstnapis[8] },
  { tekstnapis[0], tekstnapis[1], tekstnapis[6], tekstnapis[7], tekstnapis[8] }
};

signed char indeksnapisa;
unsigned char indup1, napaka, indup2, incfg, dneviod, startnidan, startnidangraf, maxyobdobje = 40, stevilodni[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 }, flagnastavicas, katerisenzor;
bool flagizbirnikcfg, izbirnikleto, izbirnikmesec, izbirnikdan, flagnenajdemvnosa, flagnastavitve, flagnivecjega = 0;
int obd, prikazanobdobje, prikazanoleto, prikazanoletostart, prikazileto, prikazanoleto2, zelenoleto, zelenmesec, zelendan;
String mesci[24]{ "Jan", "Feb", "Mar", "Apr", "Maj", "Jun", "Jul", "Avg", "Sep", "Okt", "Nov", "Dec", "Jan", "Feb", "Mar", "Apr", "Maj", "Jun", "Jul", "Avg", "Sep", "Okt", "Nov", "Dec" };
bool fileCreated = false;

const char* ssid = "Buddy AP";
const char* password = "12345678";
const char* buildTimestamp = __DATE__ " " __TIME__;
bool incorrectPassword = false;  // Password for the Access Point
String shranjenSSID = "";
String shranjenPassword = "";
String wifiSSID = "micivejperka";
String wifiPassword = "nigeslasploh";
String status;
String growStatus; // New variable for grow environment status
float temperature = 0.0;
float humidity = 0.0;
float pressure = 985.0;
int* izhod;
int mocsignala;
bool flagizbirnikizbrano;
int rele1;
int rele2;
int maksimum;
signed char* potenciometeri;
unsigned char merjenci[3][10]= {
  { 10, 13, 16, 19, 22, 25, 28, 31, 34, 37 },
  { 10, 20, 30, 40, 50, 60, 70, 80, 90, 99 },
  { 10, 20, 16, 19, 22, 25, 28, 31, 34, 37 }
};
bool flagsmit;
uint8_t schmitttemp_l[20];
uint16_t schmitttemp_l_T[2][20];
uint8_t schmitttemp_h[20];
uint16_t schmitttemp_h_T[2][20];
uint8_t schmittRH_l[20];
uint16_t schmittRH_l_T[2][20];
uint8_t schmittRH_h[20];
uint16_t schmittRH_h_T[2][20];
uint8_t schmittzem_l[20];
uint16_t schmittzem_l_T[2][20];
uint8_t schmittzem_h[20];
uint16_t schmittzem_h_T[2][20];
uint8_t schmittco2_l[20];
uint16_t schmittco2_l_T[2][20];
uint8_t schmittco2_h[20];
uint16_t schmittco2_h_T[2][20];
uint16_t testniflag;


uint8_t izbirnikvsmit;
bool flagpotrjenosmit;
bool flagpovecujsmit;
bool flagzmanjsujsmit;
signed char izhodsmit[5][20];
bool prispevaj[5][21];
int rangee[5];
int testniflag2;
int testniflag3;
AsyncWebServer server(80);
uint16_t cassmit;
uint8_t izbransenzor[20];
bool flagurnik;

// SSE (Server-Sent Events) clients management
AsyncWebSocket webSocket("/ws");  // WebSocket for SSE-like updates
bool newDataAvailable = false;

// Function to calculate grow environment status
void calculateGrowStatus() {
  // Optimal ranges for most plants
  float optimalTempMin = 18.0;
  float optimalTempMax = 26.0;
  float optimalHumidityMin = 40.0;
  float optimalHumidityMax = 70.0;
  
  // Calculate temperature score
  float tempScore = 0.0;
  if (temperatura2 >= optimalTempMin && temperatura2 <= optimalTempMax) {
    tempScore = 100.0;
  } else if (temperatura2 < optimalTempMin) {
    tempScore = 100.0 - (optimalTempMin - temperatura2) * 5.0;
  } else {
    tempScore = 100.0 - (temperatura2 - optimalTempMax) * 5.0;
  }
  tempScore = constrain(tempScore, 0.0, 100.0);
  
  // Calculate humidity score
  float humidityScore = 0.0;
  if (vlaznost2 >= optimalHumidityMin && vlaznost2 <= optimalHumidityMax) {
    humidityScore = 100.0;
  } else if (vlaznost2 < optimalHumidityMin) {
    humidityScore = 100.0 - (optimalHumidityMin - vlaznost2) * 2.0;
  } else {
    humidityScore = 100.0 - (vlaznost2 - optimalHumidityMax) * 2.0;
  }
  humidityScore = constrain(humidityScore, 0.0, 100.0);
  
  // Calculate VPD score
  float vpdScore = 0.0;
  if (vpd2 >= 0.8 && vpd2 <= 1.2) {
    vpdScore = 100.0; // Ideal range
  } else if (vpd2 >= 0.4 && vpd2 < 0.8) {
    vpdScore = 80.0; // Low but acceptable
  } else if (vpd2 > 1.2 && vpd2 <= 1.6) {
    vpdScore = 85.0; // Good but not ideal
  } else if (vpd2 < 0.4) {
    vpdScore = 30.0; // Too low - risk of mold
  } else if (vpd2 > 1.6) {
    vpdScore = 40.0; // Too high - plant stress
  }
  vpdScore = constrain(vpdScore, 0.0, 100.0);
  
  // Calculate overall score (40% temperature, 30% humidity, 30% VPD)
  float overallScore = (tempScore * 0.4) + (humidityScore * 0.3) + (vpdScore * 0.3);
  
  // Determine status based on score
  if (overallScore >= 90.0) {
    growStatus = "Odlično";
  } else if (overallScore >= 75.0) {
    growStatus = "Dobro";
  } else if (overallScore >= 60.0) {
    growStatus = "Srednje";
  } else if (overallScore >= 40.0) {
    growStatus = "Slabo";
  } else {
    growStatus = "Slabo";
  }
}

// Function to broadcast sensor data to all connected WebSocket clients
void broadcastSensorData() {
  if (webSocket.count() > 0) {
    String jsonData = "{";
    jsonData += "\"temperature\":" + String(temperatura2, 1) + ",";
    jsonData += "\"humidity\":" + String(vlaznost2, 1) + ",";
    jsonData += "\"vpd\":" + String(vpd2, 2) + ",";
    jsonData += "\"pressure\":" + String(pritisk2, 1) + ",";
    jsonData += "\"tempMax\":" + String(trenutniDnevniMaxTemp, 1) + ",";
    jsonData += "\"tempMin\":" + String(trenutniDnevniMinTemp, 1) + ",";
    jsonData += "\"humidityMax\":" + String(trenutniDnevniMaxRH, 1) + ",";
    jsonData += "\"humidityMin\":" + String(trenutniDnevniMinRH, 1) + ",";
    jsonData += "\"tempRising\":" + String(tempRising ? "true" : "false") + ",";
    jsonData += "\"tempFalling\":" + String(tempFalling ? "true" : "false") + ",";
    jsonData += "\"humidityRising\":" + String(vlaznostRising ? "true" : "false") + ",";
    jsonData += "\"humidityFalling\":" + String(vlaznostFalling ? "true" : "false") + ",";
    jsonData += "\"discoveredSensors\":" + String(discovered_nodes) + ",";
    jsonData += "\"growStatus\":\"" + growStatus + "\",";
    jsonData += "\"vpdStatus\":\"" + getVPDStatus(vpd2) + "\"";
    
    // Add soil moisture data for all discovered sensors
    for (int i = 0; i < discovered_nodes && i < MAX_SOIL_SENSORS; i++) {
      jsonData += ",\"zemlja" + String(i + 1) + "\":";
      jsonData += String((vlagazemlje[i] >= 0 && vlagazemlje[i] <= 100) ? vlagazemlje[i] : 0.0, 1);
    }
    
    jsonData += "}";
    
    webSocket.textAll(jsonData);
  }
}
const char* dan[] = {
  "SOBOTA", "NEDELJA", "PONEDELJEK", "TOREK", "SREDA", "CETRTEK", "PETEK", "X", "X", "X", "X"
};
int stdan;
uint8_t casovnirazpored[7][48];
uint8_t urnikizbranakockaX;
uint8_t urnikizbranakockaY;
unsigned char pozx;
unsigned char pozy;
unsigned char katerdan;
unsigned char kateraura;
unsigned char primerjalnaura;
uint8_t izhodurnik;
uint8_t kolikocasapritisnjena;
uint8_t kolikocasapritisnjena2;
uint8_t kolikocasapritisnjena3;
bool flipizborurnik;
bool pomikajY = 1;  //pomikanje v meniju urnika, izbor dneva
uint8_t napolnilza;
bool urnikizrisan;
bool urnikCacheValid;
uint8_t urnikLastSelectedX;
uint8_t urnikLastSelectedY;
bool poiskusipovezati;
bool sinhronizirajcas;
bool wifiomogocen;
uint8_t casbrezpovezave;
bool flagizrismoc;
float R1 = 1000.0;
float R2 = 2200.0;
bool flagdebag;

// Encoder trigger flags
bool flagEncoderClockwise = false;
bool flagEncoderCounterClockwise = false;

// System State for stm32 communication
uint8_t discovered_nodes = 0;
uint8_t node_addresses[32];  // Max 32 nodes
bool discovery_complete = false;

float readSTM32Channel(uint8_t sensorIndex) {
  uint16_t value = 0;
  float voltage = 0.0;
  
  // Read from STM32 node
  if (sensorIndex < discovered_nodes) {
    uint8_t addr = node_addresses[sensorIndex];
    if (readDataFromNode(addr, &value)) {
      voltage = value;  // Use the actual ADC value
      voltage = voltage /4; // pretvorba iz 1023 na 255.
      voltage = voltage * (3.3 / 255.0); // pretvorba iz 255 v napetost 0-3.3V
    }
  }
  return voltage;
}

//za sd kartico
const int SD_SCK = 42;
const int SD_MISO = 41;
const int SD_MOSI = 40;
const int SD_CS = 39;

const int DISPLAY_WIDTH = 320;
const int DISPLAY_HEIGHT = 240;

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
   if (y >= tft.height()) return 0;
   tft.pushImage(x, y, w, h, bitmap);
   return 1;
}

String listDirJSON(fs::FS &fs, const String& path);
String getContentType(const String& filename);
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
String generateMainPage();

bool removeDirRecursive(fs::FS &fs, const char *path);
String getStorageStats(); // New function for storage stats

void GIFDraw(GIFDRAW *pDraw) {
  uint8_t *s;
  uint16_t *d, *usPalette, usTemp[320];
  int x, y, iWidth;

  iWidth = pDraw->iWidth;
  if (iWidth > DISPLAY_WIDTH)
    iWidth = DISPLAY_WIDTH;
    
  usPalette = pDraw->pPalette;
  y = pDraw->iY + pDraw->y; // current line
  
  s = pDraw->pPixels;
  if (pDraw->ucDisposalMethod == 2) // restore to background color
  {
    for (x=0; x<iWidth; x++)
    {
      if (s[x] == pDraw->ucTransparent)
        s[x] = pDraw->ucBackground;
    }
    pDraw->ucHasTransparency = 0;
  }
  
  // Apply the new pixels to the main image
  if (pDraw->ucHasTransparency) // if transparency used
  {
    uint8_t *pEnd, c, ucTransparent = pDraw->ucTransparent;
    int x, iCount;
    pEnd = s + iWidth;
    x = 0;
    iCount = 0; // count non-transparent pixels
    while(x < iWidth)
    {
      c = ucTransparent-1;
      d = usTemp;
      while (c != ucTransparent && s < pEnd)
      {
        c = *s++;
        if (c == ucTransparent) // done with opaque run
        {
          s--; // back up to treat it like transparent
        }
        else // opaque pixel
        {
          *d++ = usPalette[c];
          iCount++;
        }
      } // while looking for opaque pixels
      if (iCount) // any opaque pixels?
      {
        tft.pushImage(pDraw->iX + x + 0, y + 0, iCount, 1, usTemp);
        x += iCount;
        iCount = 0;
      }
      // no, look for a run of transparent pixels
      c = ucTransparent;
      while (c == ucTransparent && s < pEnd)
      {
        c = *s++;
        if (c == ucTransparent)
          iCount++;
        else
          s--; 
      }
      if (iCount)
      {
        x += iCount; // skip these
        iCount = 0;
      }
    }
  }
  else // no transparency
  {
    s = pDraw->pPixels;
    // Translate 8-bit pixels through the RGB565 palette
    for (x=0; x<iWidth; x++)
      usTemp[x] = usPalette[*s++];
    tft.pushImage(pDraw->iX + 0, y + 0, iWidth, 1, usTemp);
  }
}

// File reading functions for AnimatedGIF library
void * GIFOpenFile(const char *fname, int32_t *pSize) {
  File *f = new File(FFat.open(fname, "r"));
  if (f && *f) {
    *pSize = f->size();
    return (void*)f;
  }
  delete f;
  return NULL;
}

void GIFCloseFile(void *pHandle) {
  File *f = static_cast<File *>(pHandle);
  if (f) {
    f->close();
    delete f;
  }
}

int32_t GIFReadFile(GIFFILE *pFile, uint8_t *pBuf, int32_t iLen) {
  int32_t iBytesRead = iLen;
  File *f = static_cast<File *>(pFile->fHandle);
  if (f && f->available()) {
    iBytesRead = f->read(pBuf, iLen);
  } else {
    iBytesRead = 0;
  }
  pFile->iPos = f->position();
  return iBytesRead;
}

int32_t GIFSeekFile(GIFFILE *pFile, int32_t iPosition) {
  File *f = static_cast<File *>(pFile->fHandle);
  if (f) {
    f->seek(iPosition);
    pFile->iPos = iPosition;
    return iPosition;
  }
  return -1;
}


// ============= RTC TIME FUNCTIONS =============

// Sync time with NTP server and update ESP32 RTC
void syncTimeWithNTP() {
  Serial.println("Syncing time with NTP...");
  ntpSyncAttempted = true;  // Mark that we attempted sync
  
  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  
  // Set system time
  struct timeval tv;
  tv.tv_sec = epochTime;
  tv.tv_usec = 0;
  settimeofday(&tv, NULL);
  
  // Update time variables from RTC
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);
  
  sekunde = timeinfo.tm_sec;
  minuta = timeinfo.tm_min;
  ure = timeinfo.tm_hour;
  dnevi = timeinfo.tm_mday;
  mesec = timeinfo.tm_mon + 1;
  leto = timeinfo.tm_year + 1900;
  
  rtcInitialized = true;
  lastNTPSync = millis();
  
  Serial.printf("Time synced: %02d:%02d:%02d %02d.%02d.%d\n", 
                ure, minuta, sekunde, dnevi, mesec, leto);
}

// Update time variables from ESP32 RTC
void updateTimeFromRTC() {
  time_t now;
  struct tm timeinfo;
  time(&now);
  localtime_r(&now, &timeinfo);
  
  unsigned int old_sekunde = sekunde;
  unsigned int old_minuta = minuta;
  unsigned int old_ure = ure;
  unsigned int old_dnevi = dnevi;
  
  sekunde = timeinfo.tm_sec;
  minuta = timeinfo.tm_min;
  ure = timeinfo.tm_hour;
  dnevi = timeinfo.tm_mday;
  mesec = timeinfo.tm_mon + 1;
  leto = timeinfo.tm_year + 1900;
  
  // Detect transitions for triggering actions
  if (sekunde != old_sekunde) {
    ticktajmer = 1;  // Signal that a second has passed
  }
  
  if (minuta != old_minuta) {
    flagglavnimeniizrisan = 0;
  }
  
  if (ure != old_ure) {
    // Calculate hourly pressure change rate
    if (prejsnjaPritiska > 0) {
      spremembaPritiskaNaUro = pritisk2 - prejsnjaPritiska;
    }
    prejsnjaPritiska = pritisk2;
    vzorec();
  }
  
  if (dnevi != old_dnevi) {
    indeksvzorca++;
    vzorec();
    stdan = dayOfWeek();
    shranivse();
  }
}

// Set RTC time manually (for settings menu)
void setRTCTime(int year, int month, int day, int hour, int minute, int second) {
  struct tm timeinfo;
  timeinfo.tm_year = year - 1900;
  timeinfo.tm_mon = month - 1;
  timeinfo.tm_mday = day;
  timeinfo.tm_hour = hour;
  timeinfo.tm_min = minute;
  timeinfo.tm_sec = second;
  
  time_t t = mktime(&timeinfo);
  struct timeval tv = { .tv_sec = t };
  settimeofday(&tv, NULL);
  
  // Update global variables
  leto = year;
  mesec = month;
  dnevi = day;
  ure = hour;
  minuta = minute;
  sekunde = second;
  
  rtcInitialized = true;
  Serial.printf("RTC time set: %02d:%02d:%02d %02d.%02d.%d\n", 
                hour, minute, second, day, month, year);
}



void setup() {

  // Initialize all pointers to safe values
  initializePointers();

  // ESP32-S3 Reset Debugging - Start logging immediately
  Serial.begin(115200);
  delay(1000);  // Wait for Serial to connect
  
  // Log reset information
  logResetInfo();
  
  // Update reset counter
  rtc_reset_count++;
  rtc_last_reason = (int)esp_reset_reason();
  
  // Print heap usage at key points
  printHeapUsage("Setup Start");
  
  // Optional: Enable heap tracing for advanced debugging
  // setupHeapTracing();

  TJpgDec.setJpgScale(1);
   TJpgDec.setCallback(tft_output);

    SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
   SPI.setFrequency(40000000);
   SPI.setHwCs(true);     
   if (!SD.begin(SD_CS)) {
      Serial.println("SD card mount failed");
      
   } else {
      Serial.println("SD card mounted");
       }

   if (!FFat.begin(false)) {
      Serial.println("FFat mount failed. Attempting to format...");
      if (FFat.format()) {
         Serial.println("FFat formatted successfully");
         if (FFat.begin()) {
            Serial.println("FFat mounted successfully");
            
         } else {
            Serial.println("FFat mount failed after formatting");
           
         }
      } else {
         Serial.println("FFat format failed");
        
      }
   } else {
      Serial.println("FFat mounted successfully");
     
   }

  // Initialize UART communication with STM32 (after SD/FFat to avoid conflicts)
  STM32_UART.begin(STM32_UART_BAUD, SERIAL_8N1, ESP_RX_PIN, ESP_TX_PIN);
  Serial.println("STM32 UART initialized on pins RX:" + String(ESP_RX_PIN) + " TX:" + String(ESP_TX_PIN));

  schmitttemp_l[0] = 20;
  schmitttemp_h[0] = 27;
  
  // Initialize calibration arrays with default values
  for (int i = 0; i < MAX_SOIL_SENSORS; i++) {
    kalibracijamin[i] = 0.6;   // Dry soil (high voltage, low moisture)
    kalibracijamax[i] = 0.15;  // Wet soil (low voltage, high moisture)
    flagposlano[i] = false;    // Initialize alert flags
  }
  
  Serial.begin(115200);

  if (!FFat.begin(true)) {
    Serial.println("FAT initialization failed!");
    return;
  }
   gif.begin(LITTLE_ENDIAN_PIXELS);

  tft.begin();
  tft.init();
  tft.setRotation(1);
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);



//******************************************ANIMACIJA
  if (gif.open("/animation.gif", GIFOpenFile, GIFCloseFile, GIFReadFile, GIFSeekFile, GIFDraw)) {        
    while (gif.playFrame(true, NULL)) {
      // Feed watchdog during animation to prevent timeout
      esp_task_wdt_reset();
      yield();
      //DODAJ DELAJ TUKAJ ZA UPOCASNITEV ANIMACIJE 
    }
    gif.close();
 
  } else {
    Serial.println("Failed to open /animation.gif");
    
  }

//******************************************ANIMACIJA KONEC*/


  WiFi.softAP(ssid, password);
  loadStringFromSPIFFS("/SSID.bin", wifiSSID);
  loadStringFromSPIFFS("/PASSWIFI.bin", wifiPassword);
  loadStringFromSPIFFS("/DISCORD_TOKEN.bin", discordWebhookUrl);
  //loadStringFromSPIFFS("/DEFAULT_URL.bin", defaultFirmwareUrl);
  Serial.println(WiFi.softAPIP());
  
/*
napis[0] = { tekstnapis[0], tekstnapis[1], tekstnapis[2], tekstnapis[3], tekstnapis[4] };
napis[1] = { tekstnapis[0], tekstnapis[1], tekstnapis[2], tekstnapis[3], tekstnapis[4] };
napis[2] = { tekstnapis[0], tekstnapis[1], tekstnapis[2], tekstnapis[3], tekstnapis[4] };
napis[3] = { tekstnapis[0], tekstnapis[1], tekstnapis[6], tekstnapis[7], tekstnapis[8] };
napis[4] = { tekstnapis[0], tekstnapis[1], tekstnapis[6], tekstnapis[7], tekstnapis[8] };


merjenci[0] = { 10, 13, 16, 19, 22, 25, 28, 31, 34, 37 };
merjenci[1] = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 99 };
merjenci[2] = { 10, 20, 16, 19, 22, 25, 28, 31, 34, 37 };


linijamoci[0] = { 10, 20, 30, 40, 60, 80, 90, 100, 100, 100 };
linijamoci[5] = { 10, 20, 30, 40, 60, 80, 90, 100, 100, 100 };
*/

  pinMode(TIPKAGOR_PIN, INPUT_PULLUP);
  pinMode(TIPKADOL_PIN, INPUT_PULLUP);
  pinMode(TIPKAPOTRDI_PIN, INPUT_PULLUP);
  pinMode(TIPKANAZAJ_PIN, INPUT_PULLUP);
  pinMode(RELE1_PIN, OUTPUT);
  pinMode(RELE2_PIN, OUTPUT);
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  //pinMode(SMT32Data_PIN, OUTPUT);
  //pinMode(SMT32Data_PIN, OUTPUT);  
  


 // pinMode(MOC3021, OUTPUT);
  //pinMode(zerocross, INPUT_PULLUP);
 // pinMode(currentpin, INPUT);
  //digitalWrite(BACKLIGHT_PIN, HIGH);  // Turn on backlight
 

  Wire.begin(3, 18);
  //dht.begin();
  adc.init();
  bme.begin(BME280_ADDRESS);

  sensor_t sensor;
  //dht.temperature().getSensor(&sensor);
  adc.setVoltageRange_mV(ADS1115_RANGE_6144);
  adc.setCompareChannels(ADS1115_COMP_0_GND);
  adc.setMeasureMode(ADS1115_CONTINUOUS);
  //ledcAttach(14, 1000, 8);
  //ledcWrite(14, 50);  // Channel 1, 75% duty cycle
  ledcSetup(0, 1000, 8);  // Channel 0
  ledcSetup(1, 1000, 8);
  ledcAttachPin(14, 1);
  ledcWrite(1, 0);  // Channel 1, 75% duty cycle
  if (!FFat.begin()) {

    return;
  }

  meritev();

  casovnikwifi = millis();
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  MDNS.begin("buddy");

  // Serve Chart.js locally from FFat
  server.serveStatic("/chart.min.js", FFat, "/chart.min.js");
  
  // Explicit handler for Chart.js with proper headers
  server.on("/chart.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
    if (FFat.exists("/chart.min.js")) {
      Serial.println("Serving chart.min.js from FFat");
      request->send(FFat, "/chart.min.js", "application/javascript");
    } else {
      Serial.println("chart.min.js not found in FFat");
      request->send(404, "text/plain", "chart.min.js not found");
    }
  });

   server.on("/api/list", HTTP_GET, [](AsyncWebServerRequest *request) {
      String path = "/";
      String storage = "sd";
      if (request->hasParam("path")) {
         path = request->getParam("path")->value();
      }
      if (request->hasParam("storage")) {
         storage = request->getParam("storage")->value();
      }
      fs::FS &fs = (storage == "fatfs") ? static_cast<fs::FS&>(FFat) : static_cast<fs::FS&>(SD);
      String json = listDirJSON(fs, path);
      request->send(200, "application/json", json);
   });

  
 
   // New endpoint for storage stats
   server.on("/api/storageStats", HTTP_GET, [](AsyncWebServerRequest *request) {
      String json = getStorageStats();
      request->send(200, "application/json", json);
   });

 

   server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request) {
      if (!request->hasParam("file") || !request->hasParam("storage")) {
         request->send(400, "text/plain", "Manjkajo parametri");
         return;
      }
      String filepath = request->getParam("file")->value();
      String storage = request->getParam("storage")->value();
      fs::FS &fs = (storage == "fatfs") ? static_cast<fs::FS&>(FFat) : static_cast<fs::FS&>(SD);
      Serial.println("Download requested: " + filepath + " from " + storage);
      
      File file = fs.open(filepath.c_str());
      if (!file) {
         Serial.println("File not found: " + filepath);
         request->send(404, "text/plain", "Datoteka ni najdena: " + filepath);
         return;
      }
      if (file.isDirectory()) {
         file.close();
         request->send(400, "text/plain", "Prenos map ni podprt");
         return;
      }
      file.close();
      
      String contentType = getContentType(filepath);
      AsyncWebServerResponse *response = request->beginResponse(fs, filepath.c_str(), contentType, true);
      response->addHeader("Content-Disposition", "attachment; filename=\"" + filepath.substring(filepath.lastIndexOf('/') + 1) + "\"");
      request->send(response);
   });

  

   server.on("/view", HTTP_GET, [](AsyncWebServerRequest *request) {
      if (!request->hasParam("file") || !request->hasParam("storage")) {
         request->send(400, "text/plain", "Manjkajo parametri");
         return;
      }
      String filepath = request->getParam("file")->value();
      String storage = request->getParam("storage")->value();
      fs::FS &fs = (storage == "fatfs") ? static_cast<fs::FS&>(FFat) : static_cast<fs::FS&>(SD);
      Serial.println("View requested: " + filepath + " from " + storage);
      
      if (!fs.exists(filepath.c_str())) {
         Serial.println("File not found: " + filepath);
         request->send(404, "text/plain", "Datoteka ni najdena: " + filepath);
         return;
      }
      String contentType = getContentType(filepath);
      request->send(fs, filepath.c_str(), contentType);
   });

 

   server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Nalaganje končano");
   }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      static File uploadFile;
      static String targetPath;
      static size_t uploadedSize = 0;
      static bool uploadFailed = false;
      String storage = "sd";
      if (request->hasParam("storage")) {
         storage = request->getParam("storage")->value();
      }
      fs::FS &fs = (storage == "fatfs") ? static_cast<fs::FS&>(FFat) : static_cast<fs::FS&>(SD);
      
      if (index == 0) {
         String uploadDir = "/";
         if (request->hasParam("path")) {
            uploadDir = request->getParam("path")->value();
            if (!uploadDir.startsWith("/")) uploadDir = "/" + uploadDir;
            if (!uploadDir.endsWith("/")) uploadDir += "/";
         }
         targetPath = uploadDir + filename;
         
         uploadedSize = 0;
         uploadFailed = false;
         
         if (fs.exists(targetPath.c_str())) {
            fs.remove(targetPath.c_str());
         }
         uploadFile = fs.open(targetPath.c_str(), FILE_WRITE);
         Serial.printf("Upload started: %s on %s\n", targetPath.c_str(), storage.c_str());
         
         // Send upload start notification via WebSocket
         String progressJson = "{\"type\":\"uploadStart\",\"filename\":\"" + filename + "\",\"size\":0}";
         webSocket.textAll(progressJson);
      }
      
      if (uploadFile) {
         size_t written = uploadFile.write(data, len);
         if (written != len) {
            Serial.printf("Write error: expected %d, wrote %d bytes\n", len, written);
            uploadFailed = true;
         }
         uploadedSize += written;
         
         // Send progress update every 10KB or on final chunk
         if (uploadedSize % 10240 == 0 || final) {
            String progressJson = "{\"type\":\"uploadProgress\",\"filename\":\"" + filename + "\",\"uploaded\":" + String(uploadedSize) + ",\"total\":0,\"progress\":0}";
            webSocket.textAll(progressJson);
         }
      } else {
         uploadFailed = true;
         Serial.printf("Upload file handle is null for: %s\n", targetPath.c_str());
      }
      
      if (final) {
         if (uploadFile) {
            uploadFile.close();
            
            if (uploadFailed) {
               Serial.printf("Upload failed: %s on %s (write error detected)\n", targetPath.c_str(), storage.c_str());
               
               // Auto force delete the corrupted/partial file
               Serial.printf("Auto force deleting failed upload: %s\n", targetPath.c_str());
               bool deleted = false;
               for (int attempt = 0; attempt < 5; attempt++) {
                  if (fs.remove(targetPath.c_str())) {
                     deleted = true;
                     Serial.printf("Auto force delete successful on attempt %d: %s\n", attempt + 1, targetPath.c_str());
                     break;
                  }
                  delay(100);
               }
               
               if (!deleted) {
                  Serial.printf("Auto force delete failed: %s\n", targetPath.c_str());
               }
               
               // Send upload failure notification
               String failJson = "{\"type\":\"uploadFailed\",\"filename\":\"" + filename + "\",\"error\":\"Upload failed - partial file deleted\"}";
               webSocket.textAll(failJson);
            } else {
               Serial.printf("Upload finished: %s on %s (%d bytes)\n", targetPath.c_str(), storage.c_str(), uploadedSize);
               
               // Send upload completion notification via WebSocket
               String progressJson = "{\"type\":\"uploadComplete\",\"filename\":\"" + filename + "\",\"size\":" + String(uploadedSize) + "}";
               webSocket.textAll(progressJson);
            }
         }
      }
   });

   

   server.on("/api/createFolder", HTTP_POST, [](AsyncWebServerRequest *request) {
      if (!request->hasParam("path", true) || !request->hasParam("storage", true)) {
         request->send(400, "text/plain", "Manjkajo parametri");
         return;
      }
      String folderPath = request->getParam("path", true)->value();
      String storage = request->getParam("storage", true)->value();
      fs::FS &fs = (storage == "fatfs") ? static_cast<fs::FS&>(FFat) : static_cast<fs::FS&>(SD);
      if (fs.exists(folderPath.c_str())) {
         request->send(400, "text/plain", "Mapa že obstaja");
         return;
      }
      if (fs.mkdir(folderPath.c_str())) {
         request->send(200, "text/plain", "Mapa uspešno ustvarjena");
      } else {
         request->send(500, "text/plain", "Napaka pri ustvarjanju mape");
      }
   });

  

   server.on("/api/delete", HTTP_POST, [](AsyncWebServerRequest *request) {
      if (!request->hasParam("path", true) || !request->hasParam("storage", true)) {
         request->send(400, "text/plain", "Manjkajo parametri");
         return;
      }
      String path = request->getParam("path", true)->value();
      String storage = request->getParam("storage", true)->value();
      
      fs::FS &fs = (storage == "fatfs") ? static_cast<fs::FS&>(FFat) : static_cast<fs::FS&>(SD);
      
      Serial.printf("Delete request for: %s on %s (using force delete)\n", path.c_str(), storage.c_str());
      
      if (!fs.exists(path.c_str())) {
         request->send(404, "text/plain", "Datoteka ali mapa ni najdena");
         return;
      }
      
      // Try to get file info for better error reporting
      String fileInfo = "";
      File testFile = fs.open(path.c_str());
      if (testFile) {
         if (testFile.isDirectory()) {
            fileInfo = " (mapa)";
         } else {
            size_t fileSize = testFile.size();
            fileInfo = " (datoteka, " + String(fileSize) + " bajtov)";
            if (fileSize == 0) {
               fileInfo += " - PRAZNA DATOTEKA";
            }
         }
         testFile.close();
      } else {
         fileInfo = " (napaka pri odpiranju - morda pokvarjena)";
      }
      
      // Use force delete logic for all deletions
      bool deleted = false;
      for (int attempt = 0; attempt < 5; attempt++) {
         // Try to remove directly first
         if (fs.remove(path.c_str())) {
            deleted = true;
            Serial.printf("Delete successful on attempt %d (direct removal): %s\n", attempt + 1, path.c_str());
            break;
         }
         
         // Try to open and close the file first (might help with some corruption)
         File tryFile = fs.open(path.c_str());
         if (tryFile) {
            tryFile.close();
            delay(50);
            if (fs.remove(path.c_str())) {
               deleted = true;
               Serial.printf("Delete successful on attempt %d (after open/close): %s\n", attempt + 1, path.c_str());
               break;
            }
         }
         
         delay(100); // Delay between attempts
      }
      
      if (deleted) {
         String response = "Uspešno izbrisano: " + path + fileInfo;
         Serial.printf("Delete completed: %s\n", path.c_str());
         request->send(200, "text/plain", response);
      } else {
         String errorMsg = "Napaka pri brisanju: " + path + fileInfo + ". Datoteka je morda zaklenjena ali močno pokvarjena.";
         Serial.printf("Delete failed after all attempts: %s\n", path.c_str());
         request->send(500, "text/plain", errorMsg);
      }
   });

   // Force delete endpoint for corrupted files
   server.on("/api/forceDelete", HTTP_POST, [](AsyncWebServerRequest *request) {
      if (!request->hasParam("path", true) || !request->hasParam("storage", true)) {
         request->send(400, "text/plain", "Manjkajo parametri");
         return;
      }
      String path = request->getParam("path", true)->value();
      String storage = request->getParam("storage", true)->value();
      
      fs::FS &fs = (storage == "fatfs") ? static_cast<fs::FS&>(FFat) : static_cast<fs::FS&>(SD);
      
      Serial.printf("Force delete request for: %s on %s\n", path.c_str(), storage.c_str());
      
      // Try multiple aggressive deletion attempts
      bool deleted = false;
      for (int attempt = 0; attempt < 5; attempt++) {
         // Try to remove directly first
         if (fs.remove(path.c_str())) {
            deleted = true;
            Serial.printf("Force delete successful on attempt %d (direct removal): %s\n", attempt + 1, path.c_str());
            break;
         }
         
         // Try to open and close the file first (might help with some corruption)
         File testFile = fs.open(path);
         if (testFile) {
            testFile.close();
            delay(50);
            if (fs.remove(path.c_str())) {
               deleted = true;
               Serial.printf("Force delete successful on attempt %d (after open/close): %s\n", attempt + 1, path.c_str());
               break;
            }
         }
         
         delay(200); // Longer delay for force attempts
      }
      
      if (deleted) {
         String response = "Force delete uspešen: " + path;
         Serial.printf("Force delete completed: %s\n", path.c_str());
         request->send(200, "text/plain", response);
      } else {
         String errorMsg = "Force delete ni uspel: " + path + ". Datoteka je morda zaklenjena ali močno pokvarjena.";
         Serial.printf("Force delete failed after all attempts: %s\n", path.c_str());
         request->send(500, "text/plain", errorMsg);
      }
   });

   

   server.on("/api/rename", HTTP_POST, [](AsyncWebServerRequest *request) {
      if (!request->hasParam("oldPath", true) || !request->hasParam("newPath", true) || !request->hasParam("storage", true)) {
         request->send(400, "text/plain", "Manjkajo parametri poti");
         return;
      }
      String oldPath = request->getParam("oldPath", true)->value();
      String newPath = request->getParam("newPath", true)->value();
      String storage = request->getParam("storage", true)->value();
      fs::FS &fs = (storage == "fatfs") ? static_cast<fs::FS&>(FFat) : static_cast<fs::FS&>(SD);
      if (!fs.exists(oldPath.c_str())) {
         request->send(404, "text/plain", "Izvorna datoteka ali mapa ni najdena");
         return;
      }
      if (fs.exists(newPath.c_str())) {
         request->send(400, "text/plain", "Ciljna pot že obstaja");
         return;
      }
      if (fs.rename(oldPath.c_str(), newPath.c_str())) {
         request->send(200, "text/plain", "Uspešno preimenovano");
      } else {
         request->send(500, "text/plain", "Napaka pri preimenovanju");
      }
   });

  

   server.on("/api/setBackground", HTTP_POST, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Ozadje uspešno posodobljeno");
   }, [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      static File uploadFile;
      static String targetPath = "/skyozadje.jpg";
      String storage = "sd";
      if (request->hasParam("storage")) {
         storage = request->getParam("storage")->value();
      }
      fs::FS &fs = (storage == "fatfs") ? static_cast<fs::FS&>(FFat) : static_cast<fs::FS&>(SD);
      if (index == 0) {
         if (!filename.endsWith(".jpg") && !filename.endsWith(".jpeg")) {
            Serial.println("Invalid file type for background: " + filename);
            return;
         }
         if (fs.exists(targetPath.c_str())) {
            fs.remove(targetPath.c_str());
         }
         uploadFile = fs.open(targetPath.c_str(), FILE_WRITE);
         Serial.printf("Background upload started: %s on %s\n", targetPath.c_str(), storage.c_str());
      }
      if (uploadFile) {
         uploadFile.write(data, len);
      }
      if (final) {
         if (uploadFile) {
            uploadFile.close();
            Serial.printf("Background upload finished: %s on %s\n", targetPath.c_str(), storage.c_str());
         }
      }
   });

   

  server.on("/nadzornikprostora", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", nadzornikprostora_html);
    response->addHeader("Connection", "close");
    request->send(response);
  });

  server.on("/api/buildTimestamp", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", buildTimestamp);
  });

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    // Serve index.html from FATFS
    if (!FFat.exists("/index.html")) {
      request->send(404, "text/plain", "index.html not found");
      return;
    }
    
    // Use the filesystem object instead of file object to avoid ambiguity
    AsyncWebServerResponse *response = request->beginResponse(FFat, "/index.html", "text/html", false);
    response->addHeader("Connection", "close");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Pragma", "no-cache");
    response->addHeader("Expires", "0");
    request->send(response);
  });

  server.on("/graphs.html", HTTP_GET, [](AsyncWebServerRequest* request) {
    // Serve graphs.html from FATFS
    if (!FFat.exists("/graphs.html")) {
      request->send(404, "text/plain", "graphs.html not found");
      return;
    }
    
    // Use the filesystem object instead of file object to avoid ambiguity
    AsyncWebServerResponse *response = request->beginResponse(FFat, "/graphs.html", "text/html", false);
    response->addHeader("Connection", "close");
    response->addHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    response->addHeader("Pragma", "no-cache");
    request->send(response);
  });

  server.on("/manager", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse *response = request->beginResponse_P(200, "text/html", manager_html);
    response->addHeader("Connection", "close");
    request->send(response);
  });

  server.on("/temperatureData", HTTP_GET, [](AsyncWebServerRequest* request) {
    // Feed watchdog to prevent timeout during large data processing
    vTaskDelay(1);
    
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->print("[");
    for (int i = 0; i < 365; i++) {
      // Feed watchdog every 50 days to prevent timeout
      if (i % 50 == 0) vTaskDelay(1);
      
      if (i > 0) response->print(",");
      response->print("[");
      for (int j = 0; j < 25; j++) {
        if (j > 0) response->print(",");
        response->print(tempPodat[i][j]);
      }
      response->print("]");
    }
    response->print("]");
    request->send(response);
  });

  // Route to handle AJAX request for sensor data
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest* request) {
    // Stream JSON response to avoid heap fragmentation
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->print("{");
    response->print("\"temperature\": ");
    response->print(temperatura2, 1);
    response->print(",\"humidity\": ");
    response->print(vlaznost2, 1);
    response->print(",\"vpd\": ");
    response->print(vpd2, 2);
    response->print(",\"pressure\": ");
    response->print(pritisk2, 1);
    response->print(",\"tempMax\": ");
    response->print(trenutniDnevniMaxTemp, 1);
    response->print(",\"discoveredSensors\": ");
    response->print(discovered_nodes);
    response->print(",\"loggedSensors\": [");
    for (int i = 0; i < LOGGED_SENSORS_COUNT; i++) {
      if (i > 0) response->print(",");
      response->print(selectedSensors[i] + 1);  // +1 to make it 1-based for user
    }
    response->print("],\"zemlja1\": ");
    response->print(vlagazemlje[0], DEC);
    response->print(",\"zemlja2\": ");
    response->print(vlagazemlje[1], DEC);
    response->print(",\"zemlja3\": ");
    response->print(vlagazemlje[2], DEC);
    response->print(",\"zemlja4\": ");
    response->print(vlagazemlje[3], DEC);
    
    // Add temperature and humidity trend data
    response->print(",\"tempRising\": ");
    response->print(tempRising ? "true" : "false");
    response->print(",\"tempFalling\": ");
    response->print(tempFalling ? "true" : "false");
    response->print(",\"tempMax\": ");
    response->print(trenutniDnevniMaxTemp, 1);
    response->print(",\"tempMin\": ");
    response->print(trenutniDnevniMinTemp, 1);
    response->print(",\"humidityRising\": ");
    response->print(vlaznostRising ? "true" : "false");
    response->print(",\"humidityFalling\": ");
    response->print(vlaznostFalling ? "true" : "false");
    response->print(",\"humidityMax\": ");
    response->print(trenutniDnevniMaxRH, 1);
    response->print(",\"humidityMin\": ");
    response->print(trenutniDnevniMinRH, 1);
    
    response->print("}");

    // Send JSON response with sensor data
    request->send(response);
  });

  server.on("/scan", HTTP_GET, [](AsyncWebServerRequest* request) {
    WiFi.scanNetworks(true);  // true for async scan
    request->send(200, "text/plain", "Scanning...");
  });

  // Handle scan result request
  server.on("/scanResults", HTTP_GET, [](AsyncWebServerRequest* request) {
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_RUNNING) {
      request->send(200, "text/plain", "Scan running...");
    } else if (n >= 0) {
      AsyncResponseStream *response = request->beginResponseStream("application/json");
      response->print("[");
      for (int i = 0; i < n; ++i) {
        if (i) response->print(",");
        response->print("{");
        response->print("\"ssid\":\"");
        response->print(WiFi.SSID(i));
        response->print("\",\"rssi\":");
        response->print(WiFi.RSSI(i));
        response->print("}");
      }
      response->print("]");
      request->send(response);
      WiFi.scanDelete();
    } else {
      request->send(500, "text/plain", "Scan failed");
    }
  });

  // Handle connect request
  server.on("/connect", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (request->hasParam("ssid", true) && request->hasParam("password", true)) {
      incorrectPassword = false;
      status = "Povezujem...";
      wifiSSID = request->getParam("ssid", true)->value();
      wifiPassword = request->getParam("password", true)->value();

      request->send(200, "text/plain", "Connecting to " + wifiSSID);

      // Try to connect to the selected network
      WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());

      // Wait for connection
      unsigned long startAttemptTime = millis();

      while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
        esp_task_wdt_reset();
        delay(100);
      }

      if (WiFi.status() == WL_CONNECTED) {
        incorrectPassword = false;
      } else {
        incorrectPassword = true;
      }
    } else {
      request->send(400, "text/plain", "Invalid parameters");
    }
  });

  // Handle status request
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (incorrectPassword) {
      status = "Poizkus neuspesen";

    } else {
      switch (WiFi.status()) {
        case WL_CONNECTED:
          status = "Povezano";
          break;
        case WL_IDLE_STATUS:
          status = "Vmesno stanje";
          break;
        case WL_CONNECTION_LOST:
          status = "Povezava izgubljena";
          break;
        case WL_NO_SSID_AVAIL:
          status = "Ni razpolozljivih omrezij";
          break;
        default:
          status = "Ni  povezave";
      }
    }
    if (status == "Povezano") {
      mocsignala = map(WiFi.RSSI(), -100, -50, 0, 100);
      request->send(200, "text/plain", status + " z " + "\"" + wifiSSID + "\"" + "  Jakost: " + mocsignala + "%");
    } else {
      request->send(200, "text/plain", status);
    }
  });
  // Handle IP address request
  server.on("/ip", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (WiFi.status() == WL_CONNECTED) {
      request->send(200, "text/plain", WiFi.localIP().toString());
    } else {
      request->send(200, "text/plain", "0.0.0.0");
    }
  });

  // Temperature data API for Chart.js - FIXED VERSION
  server.on("/temperature-data", HTTP_GET, [](AsyncWebServerRequest *request){
    int startIndex = 0;
    int endIndex = 50; // Default to limited range to prevent ESP32 reset
    
    // Check for date range parameters
    if (request->hasParam("start") && request->hasParam("end")) {
      startIndex = request->getParam("start")->value().toInt();
      endIndex = request->getParam("end")->value().toInt();
      // Validate range
      if (startIndex < 0) startIndex = 0;
      if (endIndex >= 365) endIndex = 364;
      if (startIndex > endIndex) startIndex = endIndex;
    }
    
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    
    // For initial load (no parameters), only send last 30 days to prevent memory issues
    bool isInitialLoad = !request->hasParam("start") && !request->hasParam("end");
    
    response->print("{");
    response->print("\"dates\":[");
    bool firstDate = true;
    for (int i = startIndex; i <= endIndex; i++) {
      // Skip empty/invalid dates (101 is sentinel value)
      if (dnevivzorec[i] == 101 || mesecvzorec[i] == 101 || letovzorec[i] == 101) {
        continue;
      }
      
      if (!firstDate) response->print(",");
      firstDate = false;
      
      response->print("\"");
      response->print(dnevivzorec[i]);
      response->print(".");
      response->print(mesecvzorec[i]);
      response->print(".");
      response->print(letovzorec[i]);
      response->print("\"");
    }
    response->print("],");
    
    // Store actual indices for JavaScript mapping - limit for memory
    response->print("\"indices\":[");
    bool firstIndex = true;
    int indexStart = isInitialLoad ? max(0, endIndex - 29) : startIndex;
    
    for (int i = indexStart; i <= endIndex; i++) {
      if (dnevivzorec[i] == 101 || mesecvzorec[i] == 101 || letovzorec[i] == 101) {
        continue;
      }
      if (!firstIndex) response->print(",");
      firstIndex = false;
      response->print(i);
    }
    response->print("],");
    response->print("\"hours\":[");
    for (int h = 0; h < 24; h++) {
      response->print("\"");
      response->print(h);
      response->print(":00\"");
      if (h < 23) response->print(",");
    }
    response->print("],");
    
    // Create hourly temperature data - ONLY for valid days, but limit initial load
    response->print("\"datasets\":[");
    bool firstDataset = true;
    
    int datasetStart = isInitialLoad ? max(0, endIndex - 29) : startIndex;
    
    for (int day = datasetStart; day <= endIndex; day++) {
      // CRITICAL FIX: Skip invalid dates here too!
      if (dnevivzorec[day] == 101 || mesecvzorec[day] == 101 || letovzorec[day] == 101) {
        continue;
      }
      
      if (!firstDataset) response->print(",");
      firstDataset = false;
      
      response->print("{");
      response->print("\"label\":\"");
      response->print(dnevivzorec[day]);
      response->print(".");
      response->print(mesecvzorec[day]);
      response->print(".");
      response->print(letovzorec[day]);
      response->print("\",");
      response->print("\"data\":[");
      for (int h = 0; h < 24; h++) {
        response->print(tempPodat[day][h], 1);
        if (h < 23) response->print(",");
      }
      response->print("],");
      response->print("\"borderColor\":\"");
      int colorIndex = day % 6;
      switch(colorIndex) {
        case 0: response->print("#ff6b6b"); break;
        case 1: response->print("#4ecdc4"); break;
        case 2: response->print("#45b7d1"); break;
        case 3: response->print("#f9ca24"); break;
        case 4: response->print("#6c5ce7"); break;
        case 5: response->print("#a29bfe"); break;
        default: response->print("#ff6b6b"); break;
      }
      response->print("\",");
      response->print("\"backgroundColor\":\"rgba(255,255,255,0.1)\",");
      response->print("\"fill\":false,");
      response->print("\"tension\":0.1,");
      response->print("\"pointRadius\":1,");
      response->print("\"pointHoverRadius\":3");
      response->print("}");
    }
    response->print("]");
    response->print("}");
    
    request->send(response);
  });

  // API to get current data index
  server.on("/api/currentIndex", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->print("{\"currentIndex\": ");
    response->print(indeksvzorca);
    response->print("}");
    request->send(response);
  });

  // Single day data API - serves one day at a time
  server.on("/api/dayData", HTTP_GET, [](AsyncWebServerRequest *request){
    // Debug: Log heap usage before processing
    printHeapUsage("API /api/dayData Start");
    
    // Feed watchdog to prevent timeout
    vTaskDelay(1);
    
    if (request->hasParam("index")) {
      int idx = request->getParam("index")->value().toInt();
      
      // Safety check for array bounds (arrays are 365 elements, indices 0-364)
      if (idx >= 0 && idx < 365) {
        // Check for invalid dates (101 sentinel values)
        if (dnevivzorec[idx] == 101 || mesecvzorec[idx] == 101 || letovzorec[idx] == 101) {
          request->send(400, "text/plain", "No Data");
          return;
        }
        
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        response->print("{");
        response->print("\"date\": \"");
        response->print(dnevivzorec[idx]);
        response->print(".");
        response->print(mesecvzorec[idx]);
        response->print(".");
        response->print(letovzorec[idx]);
        response->print("\",");
        response->print("\"hours\": [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24],");
        response->print("\"temps\": [");
        for (int h = 0; h < 25; h++) {
          // Feed watchdog every 8 iterations to prevent timeout
          if (h % 8 == 0) vTaskDelay(1);
          
          float val = tempPodat[idx][h];
          // Check for your 'no-data' flag (usually 101 or 1001 in your code)
          if (val >= 100) response->print("null"); 
          else response->print(val, 1);
          if (h < 24) response->print(",");
        }
        response->print("],");
        response->print("\"humidity\": [");
        for (int h = 0; h < 25; h++) {
          // Feed watchdog every 8 iterations to prevent timeout
          if (h % 8 == 0) vTaskDelay(1);
          
          float val = RH[idx][h];
          // Check for your 'no-data' flag (usually 101 or 1001 in your code)
          if (val >= 100) response->print("null"); 
          else response->print(val, 1);
          if (h < 24) response->print(",");
        }
        response->print("],");
        response->print("\"soilMoisture1\": [");
        for (int h = 0; h < 25; h++) {
          // Feed watchdog every 8 iterations to prevent timeout
          if (h % 8 == 0) vTaskDelay(1);
          
          float val = senzor1[idx][h];
          // Check for your 'no-data' flag (usually 101 or 1001 in your code)
          if (val >= 100) response->print("null"); 
          else response->print(val, 1);
          if (h < 24) response->print(",");
        }
        response->print("],");
        response->print("\"soilMoisture2\": [");
        for (int h = 0; h < 25; h++) {
          // Feed watchdog every 8 iterations to prevent timeout
          if (h % 8 == 0) vTaskDelay(1);
          
          float val = senzor2[idx][h];
          // Check for your 'no-data' flag (usually 101 or 1001 in your code)
          if (val >= 100) response->print("null"); 
          else response->print(val, 1);
          if (h < 24) response->print(",");
        }
        response->print("],");
        response->print("\"soilMoisture3\": [");
        for (int h = 0; h < 25; h++) {
          // Feed watchdog every 8 iterations to prevent timeout
          if (h % 8 == 0) vTaskDelay(1);
          
          float val = senzor3[idx][h];
          // Check for your 'no-data' flag (usually 101 or 1001 in your code)
          if (val >= 100) response->print("null"); 
          else response->print(val, 1);
          if (h < 24) response->print(",");
        }
        response->print("],");
        response->print("\"soilMoisture4\": [");
        for (int h = 0; h < 25; h++) {
          // Feed watchdog every 8 iterations to prevent timeout
          if (h % 8 == 0) vTaskDelay(1);
          
          float val = senzor4[idx][h];
          // Check for your 'no-data' flag (usually 101 or 1001 in your code)
          if (val >= 100) response->print("null"); 
          else response->print(val, 1);
          if (h < 24) response->print(",");
        }
        response->print("]}");
        request->send(response);
        
        // Debug: Log heap usage after processing
        printHeapUsage("API /api/dayData End");
      } else {
        request->send(400, "text/plain", "Invalid Index");
      }
    } else {
      request->send(400, "text/plain", "Index required");
    }
  });

  // Monthly data API - serves daily averages for a month
  server.on("/api/monthlyData", HTTP_GET, [](AsyncWebServerRequest *request){
    // Debug: Log heap usage before processing
    printHeapUsage("API /api/monthlyData Start");
    
    // Feed watchdog to prevent timeout
    vTaskDelay(1);
    
    if (request->hasParam("month") && request->hasParam("year")) {
      int month = request->getParam("month")->value().toInt();
      int year = request->getParam("year")->value().toInt();
      
      // Debug output
      Serial.printf("[MONTHLY API] Requested month: %d, year: %d\n", month, year);
      Serial.printf("[MONTHLY API] indeksvzorca: %d\n", indeksvzorca);
      
      // Validate month (1-12)
      if (month < 1 || month > 12) {
        Serial.printf("[MONTHLY API] Invalid month: %d\n", month);
        request->send(400, "text/plain", "Invalid month");
        return;
      }
      
      // Validate year (reasonable range)
      if (year < 2020 || year > 2030) {
        Serial.printf("[MONTHLY API] Invalid year: %d\n", year);
        request->send(400, "text/plain", "Invalid year");
        return;
      }
      
      // Debug: Print first few entries to see what data we have
      Serial.printf("[MONTHLY API] First 5 data entries:\n");
      for (int i = 0; i < min(5, indeksvzorca); i++) {
        Serial.printf("  Index %d: %d.%d.%d (avgT: %d, avgRH: %d)\n", 
                     i, dnevivzorec[i], mesecvzorec[i], letovzorec[i], 
                     povprecnadnevnaT[i], povprecnadnevnaRH[i]);
      }
      
      AsyncResponseStream *response = request->beginResponseStream("application/json");
      response->print("{");
      response->print("\"month\": ");
      response->print(month);
      response->print(",\"year\": ");
      response->print(year);
      response->print(",\"days\": [");
      
      bool firstDay = true;
      int daysInMonth;
      int foundDays = 0;
      
      // Get days in month
      if (month == 2) {
        // Simple leap year calculation
        daysInMonth = ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) ? 29 : 28;
      } else if (month == 4 || month == 6 || month == 9 || month == 11) {
        daysInMonth = 30;
      } else {
        daysInMonth = 31;
      }
      
      Serial.printf("[MONTHLY API] Days in month: %d\n", daysInMonth);
      
      // Find data for each day of the month
      for (int day = 1; day <= daysInMonth; day++) {
        bool foundData = false;
        int dayIndex = -1;
        
        // Search for this day in the data arrays
        for (int i = 0; i < indeksvzorca && i < 365; i++) {
          if (dnevivzorec[i] == day && mesecvzorec[i] == month && letovzorec[i] == year) {
            if (povprecnadnevnaT[i] != 101 && povprecnadnevnaRH[i] != 101) {
              foundData = true;
              dayIndex = i;
              foundDays++;
              break;
            }
          }
        }
        
        if (!firstDay) response->print(",");
        firstDay = false;
        
        if (foundData && dayIndex >= 0) {
          response->print("{\"day\":");
          response->print(day);
          response->print(",\"avgTemp\":");
          int avgTemp = (int)povprecnadnevnaT[dayIndex];
          if (avgTemp >= -50 && avgTemp <= 60) {  // Valid temperature range
            response->print(avgTemp);
          } else {
            response->print("null");
          }
          response->print(",\"avgHumidity\":");
          int avgHumidity = (int)povprecnadnevnaRH[dayIndex];
          if (avgHumidity >= 0 && avgHumidity <= 100) {  // Valid humidity range
            response->print(avgHumidity);
          } else {
            response->print("null");
          }
          response->print(",\"maxTemp\":");
          int maxTemp = (int)najvisjadnevnaT[dayIndex];
          if (maxTemp >= -50 && maxTemp <= 60) {  // Valid temperature range
            response->print(maxTemp);
          } else {
            response->print("null");
          }
          response->print(",\"minTemp\":");
          int minTemp = (int)najnizjadnevnaT[dayIndex];
          if (minTemp >= -50 && minTemp <= 60) {  // Valid temperature range
            response->print(minTemp);
          } else {
            response->print("null");
          }
          response->print(",\"maxHumidity\":");
          int maxHumidity = (int)najvisjadnevnaRH[dayIndex];
          if (maxHumidity >= 0 && maxHumidity <= 100) {  // Valid humidity range
            response->print(maxHumidity);
          } else {
            response->print("null");
          }
          response->print(",\"minHumidity\":");
          int minHumidity = (int)najnizjadnevnaRH[dayIndex];
          if (minHumidity >= 0 && minHumidity <= 100) {  // Valid humidity range
            response->print(minHumidity);
          } else {
            response->print("null");
          }
          response->print(",\"minSoil1\":");
          int minSoil1 = (int)najnizjadnevnasens1[dayIndex];
          if (minSoil1 >= 0 && minSoil1 <= 100) {  // Valid soil moisture range
            response->print(minSoil1);
          } else {
            response->print("null");
          }
          response->print(",\"minSoil2\":");
          int minSoil2 = (int)najnizjadnevnasens2[dayIndex];
          if (minSoil2 >= 0 && minSoil2 <= 100) {  // Valid soil moisture range
            response->print(minSoil2);
          } else {
            response->print("null");
          }
          response->print(",\"minSoil3\":");
          int minSoil3 = (int)najnizjadnevnasens3[dayIndex];
          if (minSoil3 >= 0 && minSoil3 <= 100) {  // Valid soil moisture range
            response->print(minSoil3);
          } else {
            response->print("null");
          }
          response->print(",\"minSoil4\":");
          int minSoil4 = (int)najnizjadnevnasens4[dayIndex];
          if (minSoil4 >= 0 && minSoil4 <= 100) {  // Valid soil moisture range
            response->print(minSoil4);
          } else {
            response->print("null");
          }
          response->print("}");
        } else {
          response->print("{\"day\":");
          response->print(day);
          response->print(",\"avgTemp\":null,\"avgHumidity\":null,\"maxTemp\":null,\"minTemp\":null,\"maxHumidity\":null,\"minHumidity\":null,\"minSoil1\":null,\"minSoil2\":null,\"minSoil3\":null,\"minSoil4\":null}");
        }
        
        // Feed watchdog every 5 days to prevent timeout
        if (day % 5 == 0) vTaskDelay(1);
      }
      
      response->print("]}");
      Serial.printf("[MONTHLY API] Found %d days with data\n", foundDays);
      request->send(response);
      
      // Debug: Log heap usage after processing
      printHeapUsage("API /api/monthlyData End");
    } else {
      Serial.println("[MONTHLY API] Missing month or year parameter");
      request->send(400, "text/plain", "Month and year parameters required");
    }
  });

  // Bot token endpoints
  server.on("/api/discordToken", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->print("{\"discordWebhookUrl\":\"");
    response->print(discordWebhookUrl);
    response->print("\"}");
    request->send(response);
  });

  server.on("/api/discordToken", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("webhookUrl", true)) {
      String newWebhookUrl = request->getParam("webhookUrl", true)->value();
      newWebhookUrl.trim();

      if (newWebhookUrl.length() == 0) {
        request->send(400, "application/json", "{\"success\":false,\"message\":\"Discord webhook URL cannot be empty\"}");
        return;
      }

      if (!newWebhookUrl.startsWith("https://discord.com/api/webhooks/")) {
        request->send(400, "application/json", "{\"success\":false,\"message\":\"Invalid Discord webhook URL format\"}");
        return;
      }

      String previousWebhookUrl = discordWebhookUrl;
      if (saveStringToSPIFFS("/DISCORD_TOKEN.bin", newWebhookUrl)) {
        discordWebhookUrl = newWebhookUrl;
        request->send(200, "application/json", "{\"success\":true,\"message\":\"Discord webhook URL saved successfully\"}");
      } else {
        discordWebhookUrl = previousWebhookUrl;
        request->send(500, "application/json", "{\"success\":false,\"message\":\"Failed to save Discord webhook URL\"}");
      }
    } else {
      request->send(400, "application/json", "{\"success\":false,\"message\":\"Missing webhookUrl parameter\"}");
    }
  });

  // Debug endpoint for heap monitoring
  server.on("/debug/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    response->print("{");
    response->print("\"reset_reason\":\"");
    response->print(getResetReason());
    response->print("\",");
    response->print("\"reset_count\":");
    response->print(rtc_reset_count);
    response->print(",");
    response->print("\"free_heap\":");
    response->print(ESP.getFreeHeap());
    response->print(",");
    response->print("\"min_free_heap\":");
    response->print(ESP.getMinFreeHeap());
    response->print(",");
    response->print("\"max_alloc_heap\":");
    response->print(ESP.getMaxAllocHeap());
    response->print(",");
    response->print("\"total_pram\":");
    response->print(ESP.getPsramSize());
    response->print(",");
    response->print("\"free_pram\":");
    response->print(ESP.getFreePsram());
    response->print(",");
    response->print("\"cpu_freq\":");
    response->print(ESP.getCpuFreqMHz());
    response->print(",");
    response->print("\"flash_size\":");
    response->print(ESP.getFlashChipSize());
    response->print("}");
    request->send(response);
  });

  // OTA Status endpoint
  server.on("/ota/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{";
    json += "\"update_available\":false,";
    json += "\"current_partition\":\"app\",";
    json += "\"free_sketch_space\":" + String(ESP.getFreeSketchSpace()) + ",";
    json += "\"sketch_size\":" + String(ESP.getSketchSize()) + ",";
    json += "\"free_fatfs\":" + String(FFat.freeBytes()) + ",";
    json += "\"total_fatfs\":" + String(FFat.totalBytes());
    json += "}";
    request->send(200, "application/json", json);
  });

  // OTA Download from URL endpoint
  server.on("/ota/download", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("url", true)) {
      String url = request->getParam("url", true)->value();
      String filename = "firmware_update.bin";
      
      // Validate URL
      if (!url.startsWith("http://") && !url.startsWith("https://")) {
        webSocket.textAll("{\"ota_status\":\"invalid_url\"}");
        request->send(400, "text/plain", "Invalid URL");
        return;
      }
      
      Serial.println("OTA update requested from: " + url);
      webSocket.textAll("{\"ota_status\":\"downloading\"}");
      
      // Send response before OTA
      request->send(200, "text/plain", "OTA update started. Device will reboot if successful.");
      
      // Set flag to trigger OTA in loop
      defaultFirmwareUrl = url;
      otaTriggered = true;
    } else {
      webSocket.textAll("{\"ota_status\":\"missing_url\"}");
      request->send(400, "text/plain", "Missing URL parameter");
    }
  });

  // OTA Update from local file endpoint
  server.on("/ota/update", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("filename", true)) {
      String filename = request->getParam("filename", true)->value();
      
      Serial.println("[OTA] Requested update for file: " + filename);
      
      if (!filename.endsWith(".bin")) {
        request->send(400, "text/plain", "Invalid file type");
        return;
      }
      
      String fullPath = "/" + filename;
      Serial.println("[OTA] Looking for file at: " + fullPath);
      Serial.println("[OTA] File exists: " + String(FFat.exists(fullPath) ? "YES" : "NO"));
      
      // List all files in root for debugging
      File root = FFat.open("/");
      if (root) {
        Serial.println("[OTA] Files in FATFS root:");
        File file = root.openNextFile();
        while (file) {
          String fileName = file.name();
          Serial.println("[OTA] - " + fileName + " (" + String(file.size()) + " bytes)");
          file.close();
          file = root.openNextFile();
        }
        root.close();
      }
      
      if (FFat.exists(fullPath)) {
        File firmwareFile = FFat.open(fullPath, "r");
        if (firmwareFile) {
          size_t firmwareSize = firmwareFile.size();
          Serial.println("[OTA] Firmware file size: " + String(firmwareSize) + " bytes");
          
          webSocket.textAll("{\"ota_status\":\"installing\"}");
          
          if (Update.begin(firmwareSize)) {
            Serial.println("Starting OTA update from: " + filename);
            
            size_t written = 0;
            size_t chunkSize = 1024; // Process in chunks
            uint8_t buffer[chunkSize];
            
            while (firmwareFile.available()) {
              size_t read = firmwareFile.read(buffer, chunkSize);
              written += Update.write(buffer, read);
              
              // Feed watchdog to prevent timeout
              esp_task_wdt_reset();
              yield(); // Allow other tasks to run
              
              // Progress reporting every 100KB
              if (written % 100000 == 0) {
                Serial.printf("[OTA] Progress: %d/%d bytes (%d%%)\n", written, firmwareSize, (written * 100) / firmwareSize);
              }
            }
            
            firmwareFile.close();
            
            Serial.println("[OTA] Written: " + String(written) + " bytes");
            
            if (written == firmwareSize && Update.end(true)) {
              Serial.println("OTA update successful");
              webSocket.textAll("{\"ota_status\":\"success\"}");
              request->send(200, "text/plain", "Update successful. Rebooting...");
              
              delay(1000);
              ESP.restart();
            } else {
              Serial.println("OTA update failed");
              Update.printError(Serial);
              webSocket.textAll("{\"ota_status\":\"failed\"}");
              request->send(500, "text/plain", "Update failed");
            }
          } else {
            Serial.println("[OTA] Not enough space for update");
            webSocket.textAll("{\"ota_status\":\"no_space\"}");
            request->send(500, "text/plain", "Not enough space");
          }
        } else {
          Serial.println("[OTA] Failed to open firmware file");
          request->send(404, "text/plain", "File not found");
        }
      } else {
        Serial.println("[OTA] Firmware file not found");
        request->send(404, "text/plain", "File not found");
      }
    } else {
      request->send(400, "text/plain", "Missing filename");
    }
  });

  // Get default firmware URL endpoint
  server.on("/api/defaultUrl", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "{\"defaultUrl\":\"" + defaultFirmwareUrl + "\"}";
    request->send(200, "application/json", json);
  });

  // Set default firmware URL endpoint
  server.on("/api/defaultUrl", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("url", true)) {
      String newUrl = request->getParam("url", true)->value();
      if (newUrl.length() > 0) {
        defaultFirmwareUrl = newUrl;
        if (saveStringToSPIFFS("/DEFAULT_URL.bin", defaultFirmwareUrl)) {
          request->send(200, "application/json", "{\"success\":true,\"message\":\"Default URL saved successfully\"}");
        } else {
          request->send(500, "application/json", "{\"success\":false,\"message\":\"Failed to save default URL\"}");
        }
      } else {
        request->send(400, "application/json", "{\"success\":false,\"message\":\"URL cannot be empty\"}");
      }
    } else {
      request->send(400, "application/json", "{\"success\":false,\"message\":\"Missing URL parameter\"}");
    }
  });

  // Format FATFS partition endpoint
  server.on("/api/formatFatfs", HTTP_POST, [](AsyncWebServerRequest *request) {
    Serial.println("[FATFS] Format request received");
    
    // Attempt to format FATFS partition
    bool formatSuccess = false;
    String formatMessage = "";
    
    try {
      // Close any open files on FATFS first
      
        FFat.end();
      
      
      // Format the FATFS partition
      Serial.println("[FATFS] Starting format...");
      
      // Use the ESP32 FATFS format function
      formatSuccess = FFat.format();
      
      if (formatSuccess) {
        Serial.println("[FATFS] Format successful");
        formatMessage = "FATFS particija uspešno formatirana. Velikost: " + 
                       String(FFat.totalBytes() / (1024 * 1024)) + " MB";
        
        // Remount the filesystem
        if (FFat.begin()) {
          Serial.println("[FATFS] Filesystem remounted successfully");
          formatMessage += " (Filesystem uspešno ponovno priklopljen)";
        } else {
          Serial.println("[FATFS] Warning: Failed to remount filesystem");
          formatMessage += " (Opozorilo: Filesystem ni uspel ponovno priklopiti)";
        }
        
        request->send(200, "text/plain", formatMessage);
      } else {
        Serial.println("[FATFS] Format failed");
        formatMessage = "Formatiranje FATFS particije ni uspelo. Poskusite ponovno.";
        request->send(500, "text/plain", formatMessage);
      }
      
    } catch (...) {
      Serial.println("[FATFS] Exception during format");
      formatMessage = "Izjema med formatiranjem FATFS. System error.";
      request->send(500, "text/plain", formatMessage);
    }
    
    // Restart LittleFS if it was running
    if (!FFat.begin()) {
      Serial.println("[LFS] Failed to restart LittleFS after FATFS format");
    }
  });

  // WebSocket event handler for real-time updates
  webSocket.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
      case WS_EVT_CONNECT:
        client->text("connected");
        break;
      case WS_EVT_DISCONNECT:
        break;
      case WS_EVT_DATA:
        break;
      case WS_EVT_PONG:
      case WS_EVT_ERROR:
        break;
    }
  });

  // Add WebSocket to server
  server.addHandler(&webSocket);

  server.begin();
  lastServerRestart = millis();  // Initialize server restart timer



  for (int i2 = 0; i2 < 365; ++i2) {
    for (int i = 0; i < 25; ++i) {
      tempPodat[i2][i] = 101;
      mesecvzorec[i2] = 101;
      letovzorec[i2] = 101;
      dnevivzorec[i2] = 101;
      najvisjadnevnaT[i2] = 101;
      najnizjadnevnaT[i2] = 101;
      najnizjadnevnasens1[i2] = 101;
      najnizjadnevnasens2[i2] = 101;
      najnizjadnevnasens3[i2] = 101;
      najnizjadnevnasens4[i2] = 101;
      povprecnadnevnaT[i2] = 101;
      najvisjadnevnaRH[i2] = 101;
      najnizjadnevnaRH[i2] = 101;
      povprecnadnevnaRH[i2] = 101;
      RH[i2][i] = 101;
      senzor1[i2][i] = 101;
      senzor2[i2][i] = 101;
      senzor3[i2][i] = 101;
      senzor4[i2][i] = 101;
    }
  }
  for (int i2 = 0; i2 < 5; ++i2) {
    for (int i = 0; i < 20; ++i) {

      if (prispevaj[i2][i] == 0) {
        izhodsmit[i2][i] = -2;
      }
      if (prispevaj[i2][i] == 1) {
        izhodsmit[i2][i] = -3;
      }
    }
  }

  pointergraf = &tempPodat;

    
  nalozivse();
    
   
  meritev();
  vzorec();
  osvezigraf();
  upravljajmoci();
  stdan = dayOfWeek();
  sinhronizirajcas = 1;
  poiskusipovezati = 1;
  wifiomogocen = 1;

// stm32 communication notes
    Serial.println("\n\n========================================");
  Serial.println("ESP32 Master - Auto-Addressing System");
  Serial.println("========================================\n");

    // Initialize pins for stm32 communication
  pinMode(DATA_PIN, INPUT);
  pinMode(DISCOVER_PIN, OUTPUT);
  digitalWrite(DISCOVER_PIN, LOW);

   Serial.println("[INIT] Pins configured:");
  Serial.printf("  DATA_PIN (GPIO%d): INPUT mode\n", DATA_PIN);
  Serial.printf("  DISCOVER_PIN (GPIO%d): OUTPUT mode, set LOW\n", DISCOVER_PIN);
  Serial.printf("  Bit period: %d us (%d baud)\n\n", BIT_PERIOD, DATA_BAUD);
    
  delay(100);
  
  // Run discovery stm32
  runDiscovery();
  
  if (discovery_complete) {
 // verifyAllAddresses();
  }
  
}

//**************************************************************************************************************************************
void loop() {

  // Check for incoming data from STM32
  if (STM32_UART.available()) {
    String received = STM32_UART.readStringUntil('\n');
    received.trim();
    
    // Parse encoder data
    if (received.startsWith("ENC") && received.indexOf(":+") > 0) {
      int encoderId = received.substring(3, received.indexOf(":")).toInt();
      flagEncoderClockwise = true;  // Set encoder trigger flag
      STM32_UART.println("ENC_ACK"); // Send acknowledgment
    }
    else if (received.startsWith("ENC") && received.indexOf(":-") > 0) {
      int encoderId = received.substring(3, received.indexOf(":")).toInt();
      flagEncoderCounterClockwise = true;  // Set encoder trigger flag
      STM32_UART.println("ENC_ACK"); // Send acknowledgment
    }
  }

 
 digitalWrite(RELE1_PIN, HIGH);
  digitalWrite(RELE2_PIN, HIGH);

   // Check if WiFi status has changed
      int currentWiFiStatus = WiFi.status();
      if (currentWiFiStatus != previousWiFiStatus) {
        flagglavnimeniizrisan = 0;  // Reset flag to redraw screen
        previousWiFiStatus = currentWiFiStatus;
      }

 if ( wifiomogocen) {
  // Track WiFi connection state changes
  static bool lastWiFiStatus = false;
  bool currentWiFiStatus = (WiFi.status() == WL_CONNECTED);
  
  // Detect when WiFi connects (either initial connection or reconnection)
  if (currentWiFiStatus && !lastWiFiStatus) {
    wifiConnectionTime = millis();
    otaTriggered = false;
    updateCheckDone = false; // Reset update check flag on WiFi reconnect
    Serial.println("WiFi connected! Time recorded for OTA trigger");
  }
  
  // ============================================================================
  // AUTOMATIC FIRMWARE UPDATE CHECK (triggered once after WiFi connects)
  // ============================================================================
  #if AUTO_UPDATE_ENABLED
    if (currentWiFiStatus && !updateCheckDone) {
      Serial.println("[UPDATE] Starting automatic firmware update check...");
      
      String newFirmwareUrl;
      bool updateAvailable = checkForUpdates(newFirmwareUrl);
      
      if (updateAvailable) {
        Serial.println("[UPDATE] Update available! Triggering delayed OTA...");
        
        // Set flags for delayed OTA execution
        defaultFirmwareUrl = newFirmwareUrl;
        otaPendingFromWeb = true;
        otaWebRequestTime = millis();
        
        Serial.println("[UPDATE] OTA will be triggered in " + String(OTA_WEB_DELAY / 1000) + " seconds");
      }
      
      updateCheckDone = true; // Mark that update check has been performed
    }
  #endif
  // ============================================================================
  
  // Increment disconnection counter
  if (WiFi.status() != WL_CONNECTED) {
    casbrezpovezave++;
  } else {
    casbrezpovezave = 0;  // Reset counter when connected
  }
  
  if  (casbrezpovezave > 15  && WiFi.status() != WL_CONNECTED){
    poiskusipovezati = 1;
    casbrezpovezave = 0;
 }

 if (WiFi.status() != WL_CONNECTED && poiskusipovezati) {
  Serial.println("Attempting WiFi reconnection...");
  WiFi.disconnect(true, true);  // Disconnect and clear WiFi config
  delay(100);
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  
  // Wait for connection with timeout
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    esp_task_wdt_reset();
    yield(); // Allow other tasks to run
    delay(50); // Reduced delay for better responsiveness
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi reconnected successfully");
    // Restart web server to ensure it's responsive
    server.end();
    delay(100);
    server.begin();
    Serial.println("Web server restarted");
  } else {
    Serial.println("WiFi reconnection failed");
  }
  
  poiskusipovezati = 0;
  }
  
  // Update WiFi status tracking
  lastWiFiStatus = (WiFi.status() == WL_CONNECTED);

  // Update time from RTC every loop iteration
  if (rtcInitialized) {
    updateTimeFromRTC();
  }
  
  // Sync with NTP once connected and once per day
  if (WiFi.status() == WL_CONNECTED && sinhronizirajcas) {
    sinhronizirajcas = 0;
    syncTimeWithNTP();
    startnidan = dnevi;
    startnidangraf = map(startnidan, 0, x7, 15, 145);
    stdan = dayOfWeek();
  }
  
  // Daily NTP sync
  if (WiFi.status() == WL_CONNECTED && rtcInitialized) {
    if (millis() - lastNTPSync >= NTP_SYNC_INTERVAL) {
      Serial.println("24 hours passed, syncing with NTP...");
      syncTimeWithNTP();
      stdan = dayOfWeek();
    }
  }
  
  // OTA update trigger - when user provides URL via browser
  if (otaTriggered && defaultFirmwareUrl.length() > 0) {
    // Reset trigger BEFORE running OTA to avoid retry loop on failed update/no-update
    otaTriggered = false;
    Serial.println("triggering OTA update...");
    Serial.println("Firmware URL: " + defaultFirmwareUrl);
    doOTA();
  } else if (otaTriggered && defaultFirmwareUrl.length() == 0) {
    // Defensive reset for invalid empty URL state
    otaTriggered = false;
    Serial.println("[OTA] Trigger ignored because firmware URL is empty");
  }
  
  // Delayed OTA update trigger - from automatic update check
  if (otaPendingFromWeb && defaultFirmwareUrl.length() > 0) {
    if (millis() - otaWebRequestTime >= OTA_WEB_DELAY) {
      Serial.println("[UPDATE] Triggering delayed OTA update...");
      Serial.println("[UPDATE] Firmware URL: " + defaultFirmwareUrl);
      
      // Reset flags
      otaPendingFromWeb = false;
      
      // Trigger OTA
      doOTA();
    } else {
      // Optional: Show countdown (debug purposes)
           
    }
  }
  
  // Clean up WebSocket connections
  webSocket.cleanupClients();
  
  // WiFi health check and server maintenance
  if (millis() - lastWiFiHealthCheck > WIFI_HEALTH_CHECK_INTERVAL) {
    lastWiFiHealthCheck = millis();
    
    // Check if WiFi is connected but server might be unresponsive
    if (WiFi.status() == WL_CONNECTED) {
      // Test server responsiveness by checking if we can create a simple client connection
      WiFiClient testClient;
      if (testClient.connect(WiFi.localIP(), 80)) {
        testClient.stop();
        Serial.println("Web server health check: OK");
      } else {
        Serial.println("Web server health check: FAILED - Restarting server");
        server.end();
        delay(100);
        server.begin();
        lastServerRestart = millis();
      }
    }
  }
  
  // Periodic server restart to prevent memory leaks
  if (millis() - lastServerRestart > SERVER_RESTART_INTERVAL) {
    Serial.println("Periodic server restart for maintenance");
    server.end();
    delay(100);
    server.begin();
    lastServerRestart = millis();
  }
  
  // OPTIONAL: Add debug output (in loop, for testing)
  static unsigned long lastDebugPrint = 0;
  

  }
 

  if (shranjenSSID != wifiSSID || shranjenPassword != wifiPassword) {

    saveStringToSPIFFS("/SSID.bin", wifiSSID);
    saveStringToSPIFFS("/PASSWIFI.bin", wifiPassword);
    shranjenSSID = wifiSSID;
    shranjenPassword = wifiPassword;
  }

  if (flagupravljajmoc) {
    upravljajmoci();
  }
  if (flagglavnimenu) {
    izrisglavnimenu();
    prikaziVlagazemlje();
  }
  dolocimocssr();
  if (flaggrafmoci) {
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(30, 10);
    tft.print("Temp:");
    tft.print(temperatura);
    tft.print("C ");
    tft.setCursor(30, 20);
    tft.print("Moc Vent:");
    tft.print(procenti[0]);
    tft.print("% ");
  }

  // Check soil moisture levels for all discovered sensors and send alerts if needed
  for (int i = 0; i < discovered_nodes && i < MAX_SOIL_SENSORS; i++) {
    if (vlagazemlje[i] < 10 && !flagposlano[i]) {
      String message = "Vlaga v zemlji " + String(i + 1) + " je samo: " + String(vlagazemlje[i], DEC) + "%";
      sendDiscordMessage(message);
      flagposlano[i] = true;
    }
    
    // Reset alert flag when moisture level is back above threshold
    if (vlagazemlje[i] > 10 && flagposlano[i]) {
      flagposlano[i] = false;
    }
  }

  if (temperatura2 > 29 && flagposlano5 == 0) {
    String message = "Temperatura zraka je +" + String(temperatura2, 1) + "°C !";
    sendDiscordMessage(message);  // Send the message
    flagposlano5 = 1;
  }

  if (temperatura < 28 && flagposlano5 == 1) {
    flagposlano5 = 0;
  }

  if (vlaznost2 > 65 && flagposlano6 == 0) {
    String message = "Vlaga zraka je +" + String(vlaznost2, 1) + "% !";
    sendDiscordMessage(message);  // Send the message
    flagposlano6 = 1;
  }

  if (vlaznost < 63 && flagposlano6 == 1) {
    flagposlano6 = 0;
  }

  // VPD alerts for extreme conditions
  if ((vpd2 < 0.4 || vpd2 > 1.6) && flagposlano8 == 0) {
    String message;
    if (vpd2 < 0.4) {
      message = "VPD je prenizko: " + String(vpd2, 2) + " kPa! Tveganje za plesni in gnilobe.";
    } else {
      message = "VPD je previsoko: " + String(vpd2, 2) + " kPa! Rastline so pod stresom.";
    }
    sendDiscordMessage(message);
    flagposlano8 = 1;
  }

  if (vpd2 >= 0.4 && vpd2 <= 1.6 && flagposlano8 == 1) {
    flagposlano8 = 0;
  }

  // Send startup message once after WiFi is connected and NTP sync was attempted
  if (WiFi.status() == WL_CONNECTED && ntpSyncAttempted && flagposlano7 == 0) {
    String message = "Buddy se je pravkar zagnal.";
    
    // Add reset info only if it's not a power-on reset
    esp_reset_reason_t resetReason = esp_reset_reason();
    if (resetReason != ESP_RST_POWERON && resetReason != ESP_RST_DEEPSLEEP) {
      message += " Razlog za ponovni zagon: " + getResetReason() + " (Št. ponovnih zagonov: " + String(rtc_reset_count) + ")";
      
      // Add additional error details for specific reset types
      if (resetReason == ESP_RST_PANIC) {
        message += " Sistem se je sesul.";
      } else if (resetReason == ESP_RST_INT_WDT || resetReason == ESP_RST_TASK_WDT || resetReason == ESP_RST_WDT) {
        message += " Neodzivnost!";
      } else if (resetReason == ESP_RST_BROWNOUT) {
        message += " Motnje v napajanju!";
      }
    }
    
     if (discovered_nodes > 0) { 
    message +=   String(discovered_nodes) + " najdenih senorjev za zemljo.";
    for (int i = 0; i < discovered_nodes && i < 4; i++) {  // Show first 4 sensors in message
      message += "Senzor" + String(i + 1) + ": " + String(vlagazemlje[i], DEC) + "% ";
    }
    } else {message += " Ni najdenih senorjev za zemljo."; }

    if (rtcInitialized) {
      message += "Temperatura sistema: " + String(getInternalTemperature(), 1) + "°C"
                 + " Jakost WiFi: " + String(mocsignala) + "%"
                 + " Čas sistema: " + String(dnevi, DEC) + "." + String(mesec, DEC) + "." + String(leto, DEC) + " " + (ure < 10 ? "0" : "") + String(ure, DEC) + ":" + (minuta < 10 ? "0" : "") + String(minuta, DEC) + ":" + (sekunde < 10 ? "0" : "") + String(sekunde, DEC)
                 + " Temperatura zraka: " + String(temperatura2, 1) + "°C Vlaznost zraka: " + String(vlaznost2, 1) + "%"
                 + " VPD: " + String(vpd2, 2) + " kPa (" + getVPDStatus(vpd2) + ")"
                 + " Tlak zraka: " + String(pressure, 1) + " hPa";
    } else {
      message += "Temperatura sistema: " + String(getInternalTemperature(), 1) + "°C"
                 + " Jakost WiFi: " + String(mocsignala) + "%"
                 + " Čas ni sinhroniziran"
                 + " Temperatura zraka: " + String(temperatura2, 1) + "°C Vlaznost zraka: " + String(vlaznost2, 1) + "%"
                 + " VPD: " + String(vpd2, 2) + " kPa (" + getVPDStatus(vpd2) + ")"
                 + " Tlak zraka: " + String(pressure, 1) + " hPa";
    }
    sendDiscordMessage(message);
    flagposlano7 = 1;
  }

  if (flagdnevnigraf && flagmesecnigraf) { flagmesecnigraf = 0; }

  if (flagdnevnigraf || flagmesecnigraf) {
    tft.setTextSize(1);
    drawGraph();

    if (flaginfo) {
      if (flagdnevnigraf) {
        tft.setCursor(60, 4);
        tft.setTextSize(1);
        tft.setTextColor(TFT_WHITE, TFT_BLUE);
        // Validate prikazanindeks before array access
        if (isValidArrayIndex(prikazanindeks, 365)) {
          tft.print(dnevivzorec[prikazanindeks]);
          tft.print(".");
          tft.print(mesecvzorec[prikazanindeks]);
          tft.print(".");
          tft.print(letovzorec[prikazanindeks]);
        } else {
          tft.print("--.--.----"); // Default display if index is invalid
        }
        tft.setTextColor(TFT_WHITE);
        if (millis() > (casovnik4 + 3000)) {
          izrismreze();
          osvezigraf();
          flaginfo = 0;
        }
      }

      if (flagmesecnigraf) {
        tft.setCursor(35, 4);
        tft.setTextSize(1);
        tft.setTextColor(TFT_WHITE, TFT_BLUE);

        if (obdobje == 0) {
          tft.setCursor(55, 4);
          tft.print(mesci[zeljenmesec - 1] + " " + prikazanoleto2);
        }
        if (obdobje == 1) { tft.print(mesci[zeljenmesec - 1] + " " + prikazanoleto2 + "-" + mesci[zeljenmesec] + " " + prikazanoleto2); }
        if (obdobje == 2) { tft.print(mesci[zeljenmesec - 1] + " " + prikazanoleto2 + "-" + mesci[zeljenmesec + 1] + " " + prikazanoleto2); }
        if (obdobje == 3) { tft.print(mesci[zeljenmesec - 1] + " " + prikazanoleto2 + "-" + mesci[zeljenmesec + 2] + " " + prikazanoleto2); }
        if (obdobje == 4) { tft.print(mesci[zeljenmesec - 1] + " " + prikazanoleto2 + "-" + mesci[zeljenmesec + 3] + " " + prikazanoleto2); }
        if (obdobje == 5) { tft.print(mesci[zeljenmesec - 1] + " " + prikazanoleto2 + "-" + mesci[zeljenmesec + 4] + " " + prikazanoleto2); }
        tft.setTextColor(TFT_WHITE);
        if (millis() > (casovnik4 + 3000)) {
          izrismreze();
          osvezigraf();
          flaginfo = 0;
        }
      }
    }
  }

  if (flagmenuglavnimenu) {
    drawHorizontalMenu();
  }

  tipke();

  // Feed watchdog before time-consuming operations
  esp_task_wdt_reset();
  yield();

  if (millis() > (casovnik2 + 5000)) {  // merjenje s senzorji vsake toliko časa, flagpulz se navezuje na ssr proženje, merjenje povzroča motnje in noćemo meriti v dol. času. ssr cikla
    casovnik2 = millis();
    meritev();
  }

  if (flaggrafmoci) {
    grafmoci();
  }

  if (flagsmit) {
    smitt();
  }


  urnik();

  // Feed watchdog before calculation functions
  esp_task_wdt_reset();
  yield();

  ura();
  racunaj();
  racunajsmit();
  
  if (!flagdebag) {
    flagdebag = 1;
    
    // Check if data already exists before populating
    if (!hasExistingData()) {
      Serial.println("No existing data found, populating debug data...");
      
      // Clear any existing corrupted files first
      clearAllDataFiles();
      
      populateDebugData();
    } else {
      Serial.println("Existing data found, skipping population");
    }
  }

  // Final watchdog reset before loop repeats
  esp_task_wdt_reset();
  
}

//****************************************************************************************************************************************************************************************************************************************************************************



//STM32 FUNCTIONS 

void runDiscovery() {
  Serial.println("========================================");
  Serial.println("Starting Discovery Phase");
  Serial.println("========================================\n");
  
  discovered_nodes = 0;
  uint8_t current_address = 1;  // Start addressing from 1
  
  // Assert DISCOVER line
  Serial.println("[DISCOVERY] Asserting DISCOVER line (HIGH)");
  digitalWrite(DISCOVER_PIN, HIGH);
  
  // Discovery loop
  while (current_address < 255) {  // Prevent infinite loop
    Serial.printf("\n--- Attempting to assign address %d (0x%02X) ---\n", 
                  current_address, current_address);
    
    // Send discovery-assignment command
    bool ack_received = sendDiscoveryCommand(current_address);
    
    if (ack_received) {
      Serial.printf("[SUCCESS] Node at position %d acknowledged address %d\n", 
                    discovered_nodes + 1, current_address);
      node_addresses[discovered_nodes] = current_address;
      discovered_nodes++;
      current_address++;
      
      // Inter-command delay
      delayMicroseconds(INTER_CMD_DELAY);
    } else {
      Serial.println("[DISCOVERY] No acknowledgment received");
      Serial.println("[DISCOVERY] Assuming no more nodes in chain");
      break;
    }
    
    // Safety check
    if (discovered_nodes >= 32) {
      Serial.println("[WARNING] Maximum node count (32) reached");
      break;
    }
  }
  
  // De-assert DISCOVER line
  Serial.println("\n[DISCOVERY] De-asserting DISCOVER line (LOW)");
  digitalWrite(DISCOVER_PIN, LOW);
  
  // Summary
  Serial.println("\n========================================");
  Serial.println("Discovery Phase Complete");
  Serial.println("========================================");
  Serial.printf("Total nodes discovered: %d\n", discovered_nodes);
  
  if (discovered_nodes > 0) {
    Serial.println("\nDiscovered addresses:");
    for (uint8_t i = 0; i < discovered_nodes; i++) {
      Serial.printf("  Node %d: Address 0x%02X (%d)\n", 
                    i + 1, node_addresses[i], node_addresses[i]);
    }
    discovery_complete = true;
  } else {
    Serial.println("\n[WARNING] No nodes discovered!");
    discovery_complete = false;
  }
  
  Serial.println();
}

/*
 * Send discovery-assignment command
 * Protocol: [CMD][ADDRESS][CHECKSUM]
 */
bool sendDiscoveryCommand(uint8_t address) {
  Serial.printf("[TX] Sending discovery command for address 0x%02X\n", address);
  
  uint8_t checksum = calculateChecksum(CMD_DISCOVERY_ASSIGN, address);
  Serial.printf("[DEBUG] Calculated checksum: 0x%02X (%d)\n", checksum, checksum);
  
  // Send command packet
  sendByte(CMD_DISCOVERY_ASSIGN);
  sendByte(address);
  sendByte(checksum);
  
  Serial.println("[TX] Command sent, waiting for ACK...");
  
  // Wait for acknowledgment
  unsigned long start_time = micros();
  uint8_t response = 0;
  bool received = false;
  
  while ((micros() - start_time) < ACK_TIMEOUT) {
    if (receiveByte(&response, ACK_TIMEOUT - (micros() - start_time))) {
      received = true;
      break;
    }
  }
  
  if (received) {
    if (response == RESPONSE_ACK) {
      Serial.printf("[RX] ACK received (0x%02X)\n", response);
      return true;
    } else {
      Serial.printf("[RX] Unexpected response: 0x%02X\n", response);
      return false;
    }
  } else {
    Serial.println("[RX] Timeout - no response");
    return false;
  }
}

/*
 * Verify all assigned addresses
 */
void verifyAllAddresses() {
  Serial.println("\n========================================");
  Serial.println("Verifying Assigned Addresses");
  Serial.println("========================================\n");
  
  bool all_verified = true;
  
  for (uint8_t i = 0; i < discovered_nodes; i++) {
    uint8_t addr = node_addresses[i];
    Serial.printf("[VERIFY] Pinging node at address 0x%02X...\n", addr);
    
    bool response = pingNode(addr);
    
    if (response) {
      Serial.printf("[OK] Node 0x%02X responded correctly\n", addr);
    } else {
      Serial.printf("[FAIL] Node 0x%02X did not respond!\n", addr);
      all_verified = false;
    }
    
    delayMicroseconds(INTER_CMD_DELAY);
  }
  
  Serial.println("\n========================================");
  if (all_verified) {
    Serial.println("✓ All addresses verified successfully!");
  } else {
    Serial.println("✗ Address verification failed for one or more nodes");
  }
  Serial.println("========================================\n");
}

/*
 * Ping a specific node
 * Protocol: [CMD][ADDRESS][CHECKSUM]
 */
bool pingNode(uint8_t address) {
  uint8_t checksum = calculateChecksum(CMD_PING, address);
  Serial.printf("[DEBUG] Ping checksum: 0x%02X (%d)\n", checksum, checksum);
  
  // Send ping packet
  sendByte(CMD_PING);
  sendByte(address);
  sendByte(checksum);
  
  // Wait for acknowledgment
  uint8_t response = 0;
  if (receiveByte(&response, ACK_TIMEOUT)) {
    return (response == RESPONSE_ACK);
  }
  
  return false;
}

/*
 * Read data from a specific node
 * Protocol: 
 *   TX: [CMD_READ_DATA][ADDRESS][CHECKSUM]
 *   RX: [DATA_HIGH][DATA_LOW][CHECKSUM]
 * 
 * Data is 10-bit (0-1023), sent as two bytes:
 *   DATA_HIGH: bits 9-8 (0-3)
 *   DATA_LOW:  bits 7-0 (0-255)
 */
bool readDataFromNode(uint8_t address, uint16_t *value) {
  uint8_t checksum = calculateChecksum(CMD_READ_DATA, address);
  
  // Send read data command
  sendByte(CMD_READ_DATA);
  sendByte(address);
  sendByte(checksum);
  
  Serial.println("[TX] Read data command sent, waiting for response...");
  
  // Wait for data response: [DATA_HIGH][DATA_LOW][CHECKSUM]
  uint8_t data_high = 0;
  uint8_t data_low = 0;
  uint8_t data_checksum = 0;
  
  // CRITICAL: Disable interrupts during precise timing reception
  noInterrupts();
  
  if (!receiveByte(&data_high, DATA_TIMEOUT)) {
    interrupts();
    Serial.println("[RX] Timeout on DATA_HIGH byte");
    return false;
  }
  
  if (!receiveByte(&data_low, DATA_TIMEOUT)) {
    interrupts();
    Serial.println("[RX] Timeout on DATA_LOW byte");
    return false;
  }
  
  if (!receiveByte(&data_checksum, DATA_TIMEOUT)) {
    interrupts();
    Serial.println("[RX] Timeout on DATA_CHECKSUM byte");
    return false;
  }
  
  // Re-enable interrupts
  interrupts();
  
  Serial.printf("[RX] Received: HIGH=0x%02X, LOW=0x%02X, CHK=0x%02X\n", 
                data_high, data_low, data_checksum);
  
  // Verify checksum
  uint8_t expected_checksum = calculateDataChecksum(data_high, data_low);
  if (data_checksum != expected_checksum) {
    Serial.printf("[ERROR] Data checksum mismatch! Expected 0x%02X, got 0x%02X\n",
                  expected_checksum, data_checksum);
    return false;
  }
  
  // Reconstruct 10-bit value
  *value = ((uint16_t)(data_high & 0x03) << 8) | data_low;
  
  Serial.printf("[RX] Data checksum OK, value decoded: %d\n", *value);
  
  return true;
}






/*
 * Send a single byte on DATA line
 * Protocol: Software UART, 8N1
 */
void sendByte(uint8_t data) {
  Serial.printf("[DEBUG] Sending byte: 0x%02X (%d)\n", data, data);
  
  // Switch to OUTPUT mode
  pinMode(DATA_PIN, OUTPUT);
  
  // Start bit (LOW)
  digitalWrite(DATA_PIN, LOW);
  delayMicroseconds(BIT_PERIOD);
  
  // Data bits (LSB first)
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(DATA_PIN, (data >> i) & 0x01);
    delayMicroseconds(BIT_PERIOD);
  }
  
  // Stop bit (HIGH)
  digitalWrite(DATA_PIN, HIGH);
  delayMicroseconds(BIT_PERIOD);
  
  // Return to INPUT mode (high-impedance)
  pinMode(DATA_PIN, INPUT);
  delayMicroseconds(BIT_PERIOD / 2);  // Half bit period guard time
}

/*
 * Receive a single byte from DATA line
 * Returns: true if byte received, false if timeout
 */
bool receiveByte(uint8_t *data, unsigned long timeout_us) {
  unsigned long start_time = micros();
  
  // Wait for start bit (HIGH to LOW transition)
  while (digitalRead(DATA_PIN) == HIGH) {
    if ((micros() - start_time) > timeout_us) {
      return false;
    }
  }
  
  // Wait to middle of start bit
  delayMicroseconds(BIT_PERIOD / 2);
  
  // Verify start bit is LOW
  if (digitalRead(DATA_PIN) != LOW) {
    return false;
  }
  
  delayMicroseconds(BIT_PERIOD);  // Move to first data bit
  
  // Read data bits (LSB first)
  uint8_t byte_value = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (digitalRead(DATA_PIN) == HIGH) {
      byte_value |= (1 << i);
    }
    delayMicroseconds(BIT_PERIOD);
  }
  
  // Verify stop bit (should be HIGH)
  if (digitalRead(DATA_PIN) != HIGH) {
    return false;
  }
  
  *data = byte_value;
  return true;
}

/*
 * Calculate simple checksum
 */
uint8_t calculateChecksum(uint8_t cmd, uint8_t addr) {
  return (cmd ^ addr) ^ 0x55;  // XOR with constant
}



/*
 * Calculate data checksum
 */
uint8_t calculateDataChecksum(uint8_t data_high, uint8_t data_low) {
  return (data_high ^ data_low) ^ 0xAA;
}



// END OF STM32 FUNCTIONS









int dayOfWeek() {
  int local_mesec = mesec;
  int local_leto = leto;
  if (local_mesec < 3) {
    local_mesec += 12;
    local_leto--;
  }
  int K = local_leto % 100;
  int J = local_leto / 100;

  int h = (dnevi + 13 * (local_mesec + 1) / 5 + K + K / 4 + J / 4 + 5 * J) % 7;

  // h = 0 je sobota, h = 1 je nedelja.
  return h;
}

struct UrnikGridStyle {
  int16_t rowStartY;
  int16_t rowEndY;
  uint8_t rowStep;
  int16_t dayLabelX;
  int16_t dayMarkerX;
  int16_t colStartX;
  int16_t colEndX;
  uint8_t colStep;
  uint8_t cellW;
  uint8_t cellH;
  uint8_t cellInnerW;
  uint8_t cellInnerH;
};

// URNIK GRID SIZE TUNING (automatic follow):
// Change only inner size values below; all dependent sizes/steps/ranges are derived.
const int16_t URNIK_TOP_START_Y = 0;

// Day labels lane (keeps labels clear of bars; avoids overlap with graph).
const int16_t URNIK_DAY_LABEL_X = 3;
const uint8_t URNIK_DAY_LABEL_MAX_CHARS = 10;  // "ponedeljek" length
const uint8_t URNIK_FONT1_CHAR_W = 6;
const uint8_t URNIK_LABEL_TO_GRID_GAP_X = 3;
const int16_t URNIK_COL_START_X = URNIK_DAY_LABEL_X + (URNIK_DAY_LABEL_MAX_CHARS * URNIK_FONT1_CHAR_W) + URNIK_LABEL_TO_GRID_GAP_X;
const int16_t URNIK_DAY_MARKER_X = URNIK_DAY_LABEL_X - URNIK_FONT1_CHAR_W;

// Position offsets for moving each graph independently (left/right/up/down).
const int16_t URNIK_TOP_OFFSET_X = 25;
const int16_t URNIK_TOP_OFFSET_Y = 10;
const int16_t URNIK_BOTTOM_OFFSET_X = 25;
const int16_t URNIK_BOTTOM_OFFSET_Y = 35;

const uint8_t URNIK_CELL_INNER_W = 7;
const uint8_t URNIK_CELL_INNER_H = 12;
const uint8_t URNIK_CELL_BORDER = 1;      // 1 px border around inner fill area
const uint8_t URNIK_CELL_OVERLAP_X = 1;   // keep historical dense look
const uint8_t URNIK_CELL_OVERLAP_Y = 1;
const uint8_t URNIK_HALF_GAP_Y = 16;      // vertical distance between top and bottom halves

const uint8_t URNIK_CELL_W = URNIK_CELL_INNER_W + (2 * URNIK_CELL_BORDER);
const uint8_t URNIK_CELL_H = URNIK_CELL_INNER_H + (2 * URNIK_CELL_BORDER);
const uint8_t URNIK_COL_STEP = URNIK_CELL_W - URNIK_CELL_OVERLAP_X;
const uint8_t URNIK_ROW_STEP = URNIK_CELL_H - URNIK_CELL_OVERLAP_Y;
const int16_t URNIK_TOP_END_Y = URNIK_TOP_START_Y + (URNIK_ROW_STEP * 6);
const int16_t URNIK_BOTTOM_START_Y = URNIK_TOP_END_Y + URNIK_HALF_GAP_Y;
const int16_t URNIK_BOTTOM_END_Y = URNIK_BOTTOM_START_Y + (URNIK_ROW_STEP * 6);
const int16_t URNIK_INFO_TEXT_X = URNIK_DAY_LABEL_X;
const int16_t URNIK_INFO_TEXT_W = 220;
const int16_t URNIK_INFO_TEXT_H = 10;

// URNIK fill-speed tuning (single place for quick adjustment)
const uint8_t URNIK_FILL_REPEAT_DIVIDER = 15;       // baseline update cadence (higher = slower)
const uint16_t URNIK_FILL_ACCEL_HOLD_TICKS = 120;   // hold time before acceleration starts
const uint8_t URNIK_FILL_ACCEL_STEP = 2;            // increment step while accelerated
const uint8_t URNIK_FILL_MAX_PAUSE_TICKS = 35;      // pause duration at 100 before wrap to 0

uint16_t blend565(uint16_t c1, uint16_t c2, uint8_t mix255) {
  const uint8_t inv = 255 - mix255;
  const uint8_t r1 = (c1 >> 11) & 0x1F;
  const uint8_t g1 = (c1 >> 5) & 0x3F;
  const uint8_t b1 = c1 & 0x1F;
  const uint8_t r2 = (c2 >> 11) & 0x1F;
  const uint8_t g2 = (c2 >> 5) & 0x3F;
  const uint8_t b2 = c2 & 0x1F;

  const uint8_t r = ((r1 * inv) + (r2 * mix255)) / 255;
  const uint8_t g = ((g1 * inv) + (g2 * mix255)) / 255;
  const uint8_t b = ((b1 * inv) + (b2 * mix255)) / 255;

  return (r << 11) | (g << 5) | b;
}

uint16_t getUrnikFillColor(uint8_t value) {
  const uint8_t clamped = min((uint8_t)100, value);
  const uint16_t lowColor = tft.color565(40, 255, 170);
  const uint16_t highColor = tft.color565(255, 165, 40);
  return blend565(lowColor, highColor, map(clamped, 0, 100, 0, 255));
}

uint16_t getUrnikMainBgColor(int16_t y) {
  const int16_t clampedY = constrain(y, 0, tft.height() - 1);
  const uint16_t bgTop = tft.color565(10, 22, 52);
  const uint16_t bgBottom = tft.color565(38, 108, 188);
  return blend565(bgTop, bgBottom, map(clampedY, 0, tft.height() - 1, 0, 255));
}

void fillUrnikGradientRect(int16_t x, int16_t y, int16_t w, int16_t h) {
  for (int16_t row = 0; row < h; row++) {
    const uint16_t rowColor = getUrnikMainBgColor(y + row);
    tft.drawFastHLine(x, y + row, w, rowColor);
  }
}

void drawUrnikMainBackground() {
  fillUrnikGradientRect(0, 0, tft.width(), tft.height());
}

void getUrnikCellGeometry(uint8_t dayIndex,
                          uint8_t slotIndex,
                          const UrnikGridStyle& topHalfStyle,
                          const UrnikGridStyle& lowerHalfStyle,
                          int16_t& cellX,
                          int16_t& cellY,
                          uint8_t& scheduleDayIndex) {
  const bool isLowerHalf = dayIndex > 6;
  const UrnikGridStyle& style = isLowerHalf ? lowerHalfStyle : topHalfStyle;
  const uint8_t dayOffset = isLowerHalf ? 7 : 0;
  const uint8_t slotOffset = isLowerHalf ? 24 : 0;

  scheduleDayIndex = dayIndex - dayOffset;
  cellX = style.colStartX + ((slotIndex - slotOffset) * style.colStep);
  cellY = style.rowStartY + (scheduleDayIndex * style.rowStep);
}

void drawUrnikDayLabel(uint8_t dayIndex,
                       uint8_t dayLabelOffset,
                       bool selected,
                       const UrnikGridStyle& style) {
  const int16_t rowY = style.rowStartY + ((dayIndex - dayLabelOffset) * style.rowStep);
  const int16_t labelW = URNIK_DAY_LABEL_MAX_CHARS * URNIK_FONT1_CHAR_W;
  const uint16_t baseBg = getUrnikMainBgColor(rowY + (style.cellH / 2));
  const uint16_t labelBg = selected ? blend565(baseBg, tft.color565(130, 175, 235), 120) : baseBg;
  const uint16_t labelFg = selected ? TFT_WHITE : tft.color565(210, 230, 255);

  tft.fillRect(style.dayLabelX, rowY, labelW, style.cellH, labelBg);
  if (selected) {
    tft.drawRect(style.dayLabelX, rowY, labelW, style.cellH, tft.color565(235, 245, 255));
  }
  tft.setTextColor(labelFg, labelBg);
  tft.drawString(dan[dayIndex - dayLabelOffset], style.dayLabelX, rowY + 3, 1);
}

void drawUrnikInfoLine(int16_t infoBaseX, int16_t infoBaseY) {
  fillUrnikGradientRect(infoBaseX, infoBaseY, URNIK_INFO_TEXT_W, URNIK_INFO_TEXT_H);
  const uint16_t infoBg = getUrnikMainBgColor(infoBaseY + (URNIK_INFO_TEXT_H / 2));
  tft.setTextColor(TFT_YELLOW, infoBg);
  tft.setCursor(infoBaseX, infoBaseY);
  tft.print("IZBRANO ");
  if ((urnikizbranakockaX / 2) < 10) { tft.print("0"); }
  tft.print(urnikizbranakockaX / 2);
  tft.print(":");
  if (urnikizbranakockaX % 2 == 0) {
    tft.print("00");
  } else {
    tft.print("30");
  }

  tft.print("-");
  if (((urnikizbranakockaX + 1) / 2) < 10) { tft.print("0"); }
  tft.print((urnikizbranakockaX + 1) / 2);
  tft.print(":");
  if ((urnikizbranakockaX + 1) % 2 == 0) {
    tft.print("00 ");
  } else {
    tft.print("30 ");
  }

  tft.print("(");
}

void drawUrnikInfoPercentage(int16_t infoBaseX, int16_t infoBaseY) {
  const uint8_t URNIK_INFO_VALUE_OFFSET_CHARS = 21;
  const uint8_t URNIK_INFO_VALUE_CHARS = 6;
  const int16_t valueX = infoBaseX + (URNIK_INFO_VALUE_OFFSET_CHARS * URNIK_FONT1_CHAR_W);

  fillUrnikGradientRect(valueX, infoBaseY, URNIK_INFO_VALUE_CHARS * URNIK_FONT1_CHAR_W, URNIK_INFO_TEXT_H);

  const uint16_t infoBg = getUrnikMainBgColor(infoBaseY + (URNIK_INFO_TEXT_H / 2));
  tft.setTextColor(TFT_YELLOW, infoBg);
  tft.setCursor(valueX, infoBaseY);
  if (urnikizbranakockaY > 6) {
    tft.print(casovnirazpored[urnikizbranakockaY - 7][urnikizbranakockaX]);
  } else {
    tft.print(casovnirazpored[urnikizbranakockaY][urnikizbranakockaX]);
  }
  tft.print("%) ");
}

void drawUrnikCell(int16_t x, int16_t y, uint8_t value, bool selected, const UrnikGridStyle& style) {
  const uint8_t fillLevel = map(value, 0, 100, 0, style.cellInnerH - 1);
  const int16_t innerStartX = x + 1;
  const int16_t innerEndX = x + style.cellInnerW;
  const int16_t innerBottomY = y + style.cellInnerH;
  const uint16_t gradientTop = tft.color565(16, 30, 58);
  const uint16_t gradientBottom = tft.color565(24, 46, 78);
  const uint16_t fillColor = getUrnikFillColor(value);

  tft.drawRect(x, y, style.cellW, style.cellH, tft.color565(180, 80, 60));

  for (int i = 0; i < style.cellInnerH; i++) {
    const uint16_t bgColor = blend565(gradientTop, gradientBottom, map(i, 0, style.cellInnerH - 1, 0, 255));
    tft.drawLine(innerStartX, y + 1 + i, innerEndX, y + 1 + i, bgColor);
  }

  if (value != 0) {
    for (int i = 0; i <= fillLevel; i++) {
      tft.drawLine(innerStartX, innerBottomY - i, innerEndX, innerBottomY - i, fillColor);
    }

    if (value > 0) {
      tft.drawLine(innerStartX, innerBottomY, innerEndX, innerBottomY, fillColor);
    }
  }

  if (selected) {
    tft.drawRect(x + 1, y + 1, style.cellInnerW, style.cellInnerH, TFT_ORANGE);
  }
}

void drawUrnikHalf(uint8_t dayIndexStart,
                   uint8_t dayLabelOffset,
                   uint8_t slotStart,
                   uint8_t slotEnd,
                   const UrnikGridStyle& style) {
  uint8_t localDayIndex = dayIndexStart;

  for (int16_t rowY = style.rowStartY; rowY <= style.rowEndY; rowY += style.rowStep) {
    drawUrnikDayLabel(localDayIndex, dayLabelOffset, localDayIndex == urnikizbranakockaY, style);

    uint8_t localSlot = slotStart;
    for (int16_t colX = style.colStartX; colX <= style.colEndX; colX += style.colStep) {
      const uint8_t value = casovnirazpored[localDayIndex - dayLabelOffset][localSlot];
      const bool selected = (localSlot == urnikizbranakockaX && localDayIndex == urnikizbranakockaY);
      drawUrnikCell(colX, rowY, value, selected, style);

      if (localSlot < slotEnd) { localSlot++; }
    }

    if (localDayIndex < (dayIndexStart + 6)) { localDayIndex++; }
  }
}

void urnik() {

  katerdan = 0;
  kateraura = 0;
  uint8_t povecujza = 1;


  // FLAG FLIP SE POSTAVI S TIPKO KO UPORABNIK ŽELI POVEČATI MOČ V ČASOVNEM OBDOBJU S TEM DA DRŽI TIPKO, VKLJUČENA JE TUDI FUNKCIJA KI POHITRI POVEČEVANJE VNOSA ĆE JE TA DLJE PRITISNENJA,
  //ČE TIPKO SPUSTI SE ZNOVA POVEČUJE POČASI, ZA NATANČNO NASTAVITEV, PRAV TAKO SE POVEČEVANJE ZA KRATEK ČAS USTAVI NA 100- MAKSIMUM, DA IMA UPORABNIK ČAS SPUSTITI TIPKO, ČE ŽELI TO VREDNOST.
  // OB PRVEM VNOSU SE KOPIRA PREJŠNJA VNEŠENA VREDNOST, ČE TIPKO DRŽIMO SE ZAČNE POVEČEVATI AMPAK LE PO DOLOČENEM ČASU, DA SE TAKOJ NE POVEČA ČE IMA UPORABNIK NAMEN LE VNOS KOPIRATI
  // POVEČEVANJE SE POČAKA PRI VREDNOSTI 100 PRED OVERFLOWOM NA 0.
  //POSTAVLJA SE IZHODURNIK. NA VOLJO JE EN TEDEN NASTAVITEV OD PONEDELJEKA DO NEDELJE
  // TODO: iZBOR SPREMINJANJA V PLUS ALI MINUS, PRI ZADNJI KOCKI, NASLEDNJI POMIK NA SPODNI URNIK
  if (flipizborurnik) {
urnikizrisan = 0;

    kolikocasapritisnjena++;
     kolikocasapritisnjena3++;
     if (kolikocasapritisnjena3 >= URNIK_FILL_REPEAT_DIVIDER) {
    kolikocasapritisnjena3 = 0;
    if (kolikocasapritisnjena > URNIK_FILL_ACCEL_HOLD_TICKS) { povecujza = URNIK_FILL_ACCEL_STEP; }
    if (urnikizbranakockaY > 6) {
      casovnirazpored[urnikizbranakockaY - 7][urnikizbranakockaX] += povecujza;
      napolnilza =casovnirazpored[urnikizbranakockaY - 7][urnikizbranakockaX];
      if (casovnirazpored[urnikizbranakockaY - 7][urnikizbranakockaX] > 100) {
        casovnirazpored[urnikizbranakockaY - 7][urnikizbranakockaX] = 100;
        kolikocasapritisnjena2++;
        if (kolikocasapritisnjena2 > URNIK_FILL_MAX_PAUSE_TICKS) {
          kolikocasapritisnjena2 = 0;
          casovnirazpored[urnikizbranakockaY - 7][urnikizbranakockaX] = 0;
        }
      }
    } else  // ce izbiramo drug del grafa, -- izbrana kocka y gre nad 7 -dnevi, zaradi preprostosti kode, da se izbere druga polovica nad 12 uro, vendar dnevi so isti
    {
      casovnirazpored[urnikizbranakockaY][urnikizbranakockaX] += povecujza;
        napolnilza =casovnirazpored[urnikizbranakockaY ][urnikizbranakockaX];
      if (casovnirazpored[urnikizbranakockaY][urnikizbranakockaX] > 100) {
        casovnirazpored[urnikizbranakockaY][urnikizbranakockaX] = 100;
       kolikocasapritisnjena2++;
        if (kolikocasapritisnjena2 > URNIK_FILL_MAX_PAUSE_TICKS) {
          kolikocasapritisnjena2 = 0;
          casovnirazpored[urnikizbranakockaY][urnikizbranakockaX] = 0;
        }
      }
    }
  } 
   if (casovnirazpored[urnikizbranakockaY - 7][urnikizbranakockaX] > 100) {casovnirazpored[urnikizbranakockaY - 7][urnikizbranakockaX] = 100; }
 if (casovnirazpored[urnikizbranakockaY][urnikizbranakockaX] > 100) {casovnirazpored[urnikizbranakockaY ][urnikizbranakockaX] = 100; }
} 

  primerjalnaura = (ure * 2) + (minuta / 30);  // urnik ima resolucijo na pol ure, izracunamo v ketero območje spada trenuten čas.
  izhodurnik = casovnirazpored[stdan][primerjalnaura];

  if (!flagurnik) {
    urnikCacheValid = false;
  }

  if (flagurnik&& !urnikizrisan) {
    urnikizrisan = 1;
    if (urnikizbranakockaY > 6 && urnikizbranakockaX < 24) { urnikizbranakockaX = urnikizbranakockaX + 24; }
    if (urnikizbranakockaY < 7 && urnikizbranakockaX > 23) { urnikizbranakockaX = urnikizbranakockaX - 24; }

    const int16_t topGraphEndY = URNIK_TOP_END_Y + URNIK_TOP_OFFSET_Y;
    const int16_t bottomGraphStartY = URNIK_BOTTOM_START_Y + URNIK_BOTTOM_OFFSET_Y;
    const int16_t infoBaseX = URNIK_INFO_TEXT_X + ((URNIK_TOP_OFFSET_X + URNIK_BOTTOM_OFFSET_X) / 2);
    const int16_t infoBaseY = (topGraphEndY + bottomGraphStartY) / 2;
    const int16_t topColStartX = URNIK_COL_START_X + URNIK_TOP_OFFSET_X;
    const int16_t topColEndX = topColStartX + (URNIK_COL_STEP * 23);
    const UrnikGridStyle topHalfStyle = {
      URNIK_TOP_START_Y + URNIK_TOP_OFFSET_Y,
      URNIK_TOP_END_Y + URNIK_TOP_OFFSET_Y,
      URNIK_ROW_STEP,
      URNIK_DAY_LABEL_X + URNIK_TOP_OFFSET_X,
      URNIK_DAY_MARKER_X + URNIK_TOP_OFFSET_X,
      topColStartX,
      topColEndX,
      URNIK_COL_STEP,
      URNIK_CELL_W,
      URNIK_CELL_H,
      URNIK_CELL_INNER_W,
      URNIK_CELL_INNER_H
    };

    const int16_t bottomColStartX = URNIK_COL_START_X + URNIK_BOTTOM_OFFSET_X;
    const int16_t bottomColEndX = bottomColStartX + (URNIK_COL_STEP * 23);
    const UrnikGridStyle lowerHalfStyle = {
      URNIK_BOTTOM_START_Y + URNIK_BOTTOM_OFFSET_Y,
      URNIK_BOTTOM_END_Y + URNIK_BOTTOM_OFFSET_Y,
      URNIK_ROW_STEP,
      URNIK_DAY_LABEL_X + URNIK_BOTTOM_OFFSET_X,
      URNIK_DAY_MARKER_X + URNIK_BOTTOM_OFFSET_X,
      bottomColStartX,
      bottomColEndX,
      URNIK_COL_STEP,
      URNIK_CELL_W,
      URNIK_CELL_H,
      URNIK_CELL_INNER_W,
      URNIK_CELL_INNER_H
    };

    if (!urnikCacheValid) {
      drawUrnikInfoLine(infoBaseX, infoBaseY);
      drawUrnikInfoPercentage(infoBaseX, infoBaseY);
      drawUrnikHalf(0, 0, 0, 23, topHalfStyle);
      drawUrnikHalf(7, 7, 24, 47, lowerHalfStyle);
    } else {
      const bool sameSelection = (urnikLastSelectedY == urnikizbranakockaY && urnikLastSelectedX == urnikizbranakockaX);
      if (!sameSelection) {
        int16_t prevX = 0;
        int16_t prevY = 0;
        uint8_t prevDay = 0;
        getUrnikCellGeometry(urnikLastSelectedY, urnikLastSelectedX, topHalfStyle, lowerHalfStyle, prevX, prevY, prevDay);
        drawUrnikCell(prevX, prevY, casovnirazpored[prevDay][urnikLastSelectedX], false,
                      (urnikLastSelectedY > 6) ? lowerHalfStyle : topHalfStyle);
      }

      int16_t currX = 0;
      int16_t currY = 0;
      uint8_t currDay = 0;
      getUrnikCellGeometry(urnikizbranakockaY, urnikizbranakockaX, topHalfStyle, lowerHalfStyle, currX, currY, currDay);
      drawUrnikCell(currX, currY, casovnirazpored[currDay][urnikizbranakockaX], true,
                    (urnikizbranakockaY > 6) ? lowerHalfStyle : topHalfStyle);

      if (urnikLastSelectedY != urnikizbranakockaY) {
        if (urnikLastSelectedY > 6) {
          drawUrnikDayLabel(urnikLastSelectedY, 7, false, lowerHalfStyle);
        } else {
          drawUrnikDayLabel(urnikLastSelectedY, 0, false, topHalfStyle);
        }

        if (urnikizbranakockaY > 6) {
          drawUrnikDayLabel(urnikizbranakockaY, 7, true, lowerHalfStyle);
        } else {
          drawUrnikDayLabel(urnikizbranakockaY, 0, true, topHalfStyle);
        }
      }

      if (!sameSelection) {
        drawUrnikInfoLine(infoBaseX, infoBaseY);
        drawUrnikInfoPercentage(infoBaseX, infoBaseY);
      } else if (flipizborurnik) {
        drawUrnikInfoPercentage(infoBaseX, infoBaseY);
      }
    }

    urnikLastSelectedX = urnikizbranakockaX;
    urnikLastSelectedY = urnikizbranakockaY;
    urnikCacheValid = true;
  }
}

bool saveArrayToSPIFFS(const char* path, void* array, size_t elementSize, size_t rows, size_t cols = 0) {
  Serial.printf("[FS] Saving to %s (elemSize: %d, rows: %d, cols: %d)\n", path, elementSize, rows, cols);
  
  // Try to mount FFat if not available, but don't force remount if already working
  if (!FFat.begin(true)) {
    Serial.println("[FS] ERROR: FFat not available or failed to mount!");
    napaka = 4;
    return false;
  }
  
  fs::File file = FFat.open(path, FILE_WRITE);  // Use fs::File instead of File
  if (!file) {
    Serial.printf("[FS] ERROR: Failed to open %s for writing\n", path);
    napaka = 4;  //
    return false;
  }

  size_t bytesWritten = 0;
  if (cols == 0) {
    // Saving 1D int array
    bytesWritten = file.write((uint8_t*)array, rows * elementSize);
    Serial.printf("[FS] Wrote %d bytes (expected %d)\n", bytesWritten, rows * elementSize);
  } else {
    // Saving 2D char array
    for (size_t i = 0; i < rows; i++) {
      size_t rowBytes = file.write((uint8_t*)array + i * cols * elementSize, cols * elementSize);
      bytesWritten += rowBytes;
      if (i < 5) { // Only log first 5 rows to avoid spam
        Serial.printf("[FS] Row %d: wrote %d bytes\n", i, rowBytes);
      }
    }
    Serial.printf("[FS] Total written: %d bytes (expected %d)\n", bytesWritten, rows * cols * elementSize);
  }

  file.flush(); // Vital: ensures data is physically moved from RAM to Flash
  file.close(); // Vital: releases the handle for the next file
  
  // Verify file exists and has correct size
  if (FFat.exists(path)) {
    fs::File verifyFile = FFat.open(path, FILE_READ);
    if (verifyFile) {
      size_t fileSize = verifyFile.size();
      verifyFile.close();
      Serial.printf("[FS] Verification: %s exists, size: %d bytes\n", path, fileSize);
      
      // Only return true if file has actual data
      if (fileSize > 0) {
        return true;
      } else {
        Serial.printf("[FS] ERROR: File %s has 0 bytes - write failed!\n", path);
        return false;
      }
    }
  }
  
  Serial.printf("[FS] ERROR: Verification failed for %s\n", path);
  return false;
}

bool loadArrayFromSPIFFS(const char* path, void* array, size_t elementSize, size_t rows, size_t cols = 0) {
  fs::File file = FFat.open(path, FILE_READ);  // Use fs::File instead of File
  if (!file) {
    napaka = 3;  // ni shranjenih vnosov
    return false;
  }

  if (cols == 0) {
    // Loading 1D int array
    file.read((uint8_t*)array, rows * elementSize);
  } else {
    // Loading 2D char array
    for (size_t i = 0; i < rows; i++) {
      file.read((uint8_t*)array + i * cols * elementSize, cols * elementSize);
    }
  }
  file.close();
  return true;
}

bool loadIntFromSPIFFS(const char* path, int& value) {
  fs::File file = FFat.open(path, FILE_READ);
  if (!file) {
    // ni shranjenih vnosov
    return false;
  }

  if (file.read((uint8_t*)&value, sizeof(value)) != sizeof(value)) {
    // ni shranjenih vnosov
    file.close();
    return false;
  }
  file.close();
  return true;
}

bool saveStringToSPIFFS(const char* path, const String& value) {
  // Ensure FFat is available
  if (!FFat.begin(true)) {
    Serial.printf("[FS] ERROR: FFat not available for %s\n", path);
    napaka = 2;  // ne morem shraniti
    return false;
  }

  fs::File file = FFat.open(path, FILE_WRITE);
  if (!file) {
    napaka = 2;  // ne morem shraniti
    return false;
  }

  size_t expected = value.length();
  size_t bytesWritten = file.write((const uint8_t*)value.c_str(), expected);
  file.flush(); // Vital: ensures data is physically moved from RAM to Flash
  file.close(); // Vital: releases the handle for the next file

  if (bytesWritten != expected) {
    Serial.printf("[FS] ERROR: saveStringToSPIFFS write mismatch for %s (expected %d, wrote %d)\n", path, expected, bytesWritten);
    return false;
  }

  return true;
}

bool loadStringFromSPIFFS(const char* path, String& value) {
  fs::File file = FFat.open(path, FILE_READ);
  if (!file) {
    // ni shranjenih vnosov
    return false;
  }

  size_t size = file.size();
  if (size == 0) {
    // ni shranjenih vnosov
    file.close();
    return false;
  }

  std::unique_ptr<char[]> buf(new char[size + 1]);
  size_t bytesRead = file.readBytes(buf.get(), size);
  file.close();

  if (bytesRead != size) {
    Serial.printf("[FS] ERROR: loadStringFromSPIFFS read mismatch for %s (expected %d, read %d)\n", path, size, bytesRead);
    return false;
  }

  buf[size] = '\0';
  value = String(buf.get());
  return true;
}

bool saveIntToSPIFFS(const char* path, int value) {
  // Ensure FFat is available
  if (!FFat.begin(true)) {
    Serial.printf("[FS] ERROR: FFat not available for %s\n", path);
    napaka = 2;  // ne morem shraniti
    return false;
  }
  
  fs::File file = FFat.open(path, FILE_WRITE);
  if (!file) {
    Serial.printf("[FS] ERROR: Failed to open %s for writing\n", path);
    napaka = 2;  // ne morem shraniti
    return false;
  }

  size_t bytesWritten = file.write((uint8_t*)&value, sizeof(value));
  file.flush(); // Vital: ensures data is physically moved from RAM to Flash
  file.close();
  
  Serial.printf("[FS] saveIntToSPIFFS %s: wrote %d bytes, value=%d\n", path, bytesWritten, value);
  
  // Verify file was created and has correct size
  if (FFat.exists(path)) {
    fs::File verifyFile = FFat.open(path, FILE_READ);
    if (verifyFile) {
      size_t fileSize = verifyFile.size();
      verifyFile.close();
      if (fileSize == sizeof(value)) {
        return true;
      }
    }
  }
  
  Serial.printf("[FS] ERROR: Verification failed for %s\n", path);
  return false;
}

void brisinovejsevnose() {

  for (int i2 = 0; i2 < 366; i2++) {
    if (letovzorec[i2] > leto) {

      for (int i = 0; i < 25; i++) {
        tempPodat[i2][i] = 101;
        RH[i2][i] = 101;
        senzor1[i2][i] = 101;
        senzor2[i2][i] = 101;
        senzor3[i2][i] = 101;
        senzor4[i2][i] = 101;
      }

      mesecvzorec[i2] = 101;
      letovzorec[i2] = 101;
      dnevivzorec[i2] = 101;
      najvisjadnevnaT[i2] = 101;
      najnizjadnevnaT[i2] = 101;
      najnizjadnevnasens1[i2] = 101;
      najnizjadnevnasens2[i2] = 101;
      najnizjadnevnasens3[i2] = 101;
      najnizjadnevnasens4[i2] = 101;
      povprecnadnevnaT[i2] = 101;
      najvisjadnevnaRH[i2] = 101;
      najnizjadnevnaRH[i2] = 101;
      povprecnadnevnaRH[i2] = 101;
    }
  }

  for (int i2 = 0; i2 < 366; i2++) {
    if (letovzorec[i2] >= leto) {
      if (mesecvzorec[i2] > mesec) {
        for (int i = 0; i < 25; i++) {
          tempPodat[i2][i] = 101;
          RH[i2][i] = 101;
          senzor1[i2][i] = 101;
          senzor2[i2][i] = 101;
          senzor3[i2][i] = 101;
          senzor4[i2][i] = 101;
        }
        mesecvzorec[i2] = 101;
        letovzorec[i2] = 101;
        dnevivzorec[i2] = 101;
        najvisjadnevnaT[i2] = 101;
        najnizjadnevnaT[i2] = 101;
        najnizjadnevnasens1[i2] = 101;
        najnizjadnevnasens2[i2] = 101;
        najnizjadnevnasens3[i2] = 101;
        najnizjadnevnasens4[i2] = 101;
        povprecnadnevnaT[i2] = 101;
        najvisjadnevnaRH[i2] = 101;
        najnizjadnevnaRH[i2] = 101;
        povprecnadnevnaRH[i2] = 101;
      }
    }
  }

  for (int i2 = 0; i2 < 366; i2++) {
    if (letovzorec[i2] >= leto) {
      if (mesecvzorec[i2] >= mesec) {
        if (dnevivzorec[i2] > dnevi) {
          for (int i = 0; i < 25; i++) {
            tempPodat[i2][i] = 101;
            RH[i2][i] = 101;
            senzor1[i2][i] = 101;
            senzor2[i2][i] = 101;
            senzor3[i2][i] = 101;
            senzor4[i2][i] = 101;
          }
          mesecvzorec[i2] = 101;
          letovzorec[i2] = 101;
          dnevivzorec[i2] = 101;
          najvisjadnevnaT[i2] = 101;
          najnizjadnevnaT[i2] = 101;
          najnizjadnevnasens1[i2] = 101;
          najnizjadnevnasens2[i2] = 101;
          najnizjadnevnasens3[i2] = 101;
          najnizjadnevnasens4[i2] = 101;
          povprecnadnevnaT[i2] = 101;
          najvisjadnevnaRH[i2] = 101;
          najnizjadnevnaRH[i2] = 101;
          povprecnadnevnaRH[i2] = 101;
        }
      }
    }
  }
}

void nastavitve() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);
  if (izbirniknastavitve == 1) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.setCursor(5, 0);
  tft.print("TrajnoShraniPodatke");
  tft.setTextColor(TFT_WHITE);
  if (izbirniknastavitve == 2) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.setCursor(5, 7);
  tft.print("NaloziPodatke");
  tft.setTextColor(TFT_WHITE);
  if (izbirniknastavitve == 3) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.setCursor(5, 14);
  tft.print("Nastavi cas:");

  tft.setCursor(5, 24);
  tft.setTextColor(TFT_WHITE);
  tft.print(" -");
  if (flagnastavicas == 1) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.print(bufferdnevi);
  tft.setTextColor(TFT_WHITE);
  tft.print(".");
  if (flagnastavicas == 2) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.print(buffermesec);
  tft.setTextColor(TFT_WHITE);
  tft.print(".");
  if (flagnastavicas == 3) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.print(bufferleto);

  tft.setTextColor(TFT_WHITE);
  tft.print(" ");
  if (flagnastavicas == 4) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.print(bufferure);

  tft.setTextColor(TFT_WHITE);
  tft.print(":");
  if (flagnastavicas == 5) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.print(bufferminuta);

  tft.setTextColor(TFT_WHITE);
  tft.print(":");
  if (flagnastavicas == 6) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.print(buffersekunde);

  tft.setTextColor(TFT_WHITE);
  if (izbirniknastavitve == 4) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.setCursor(5, 32);
  tft.print(" -Shrani cas");


  tft.setTextColor(TFT_WHITE);
  if (izbirniknastavitve == 5) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.setCursor(5, 40);
  tft.print("Kalibriraj senzor: ");
  tft.print(katerisenzor);
  tft.setTextColor(TFT_WHITE);
  if (izbirniknastavitve == 6) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.setCursor(5, 48);
  tft.print(" -Minumum");
  tft.setTextColor(TFT_WHITE);
  if (izbirniknastavitve == 7) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.setCursor(5, 56);
  tft.print(" -Maksimum");
  tft.setTextColor(TFT_WHITE);
  if (izbirniknastavitve == 8) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.setCursor(5, 64);
  tft.print("Pobrisi vse podatke");
  tft.setTextColor(TFT_WHITE);
  if (izbirniknastavitve == 9) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.setCursor(5, 72);
  tft.print("Stevilo prikazanih Z: ");
  tft.print(stPrikazanihVlagazemlje);
  tft.setTextColor(TFT_WHITE);
  if (izbirniknastavitve == 10) { tft.setTextColor(TFT_WHITE, TFT_BLUE); }
  tft.setCursor(5, 80);
  tft.print("Izberi senzorje za log");
  
  // Sensor selection lines (positions 11-14)
  for (int i = 0; i < 4; i++) {
    tft.setTextColor(TFT_WHITE);
    if (izbirniknastavitve == 11 + i) { 
      if (flagSensorEditing) {
        tft.setTextColor(TFT_YELLOW, TFT_BLUE);  // Yellow text on blue when editing
      } else {
        tft.setTextColor(TFT_WHITE, TFT_BLUE);    // White on blue when selected
      }
    }
    tft.setCursor(5, 88 + i * 8);
    tft.print("Data log ");
    tft.print(i + 1);
    tft.print("...sensor ");
    tft.print(selectedSensors[i]);
    if (flagSensorEditing && izbirniknastavitve == 11 + i) {
      tft.print(" <");  // Show editing indicator
    }
  }
}

bool loadFloatFromSPIFFS(const char* path, float& value) {
  fs::File file = FFat.open(path, FILE_READ);
  if (!file) {
    // ni shranjenih vnosov
    return false;
  }

  if (file.read((uint8_t*)&value, sizeof(value)) != sizeof(value)) {
    // ni shranjenih vnosov
    file.close(); // <--- ADD THIS before the final return
    return false;
  }
  file.close(); // Vital: releases the handle for the next file
  return true;
}

bool saveFloatToSPIFFS(const char* path, float value) {
  fs::File file = FFat.open(path, FILE_WRITE);
  if (!file) {
    napaka = 2;  // ne morem shraniti
    return false;
  }

  file.write((uint8_t*)&value, sizeof(value));
  file.flush(); // Vital: ensures data is physically moved from RAM to Flash
  file.close(); // Vital: releases the handle for the next file

  return true;
}



/*void IRAM_ATTR onTimer() {
  if (!flagpulz) {
    digitalWrite(MOC3021, HIGH);
    timerRestart(timer);
    timerAlarmWrite(timer, 100, true);
    timerRestart(timer);
    flagpulz = 1;
  } else {
    digitalWrite(MOC3021, LOW);
    timerRestart(timer);
    timerAlarmWrite(timer, 10000, true);
    timerRestart(timer);
    flagpulz = 0;
  }
}

void zcross() {
  zamiktriak = constrain(zamiktriak, 0, 8000);
  timerRestart(timer);
  timerAlarmWrite(timer, zamiktriak, true);
  timerRestart(timer);
  digitalWrite(MOC3021, LOW);
  flagpulz = 0;
}
*/

void upravljajmoci() {

  procentoff = -1;


  for (int in = 0; in < 4; in++) {
    if (napis[0][in] == "POTEN") { mesto[0][in] = &nulll; }
    if (napis[0][in] == "GRAF0") { mesto[0][in] = &procenti[0]; }           //Temp
    if (napis[0][in] == "GRAF1") { mesto[0][in] = &procenti[5]; }           //RH
    if (napis[0][in] == "GRAF2") { mesto[0][in] = &procenti[10]; }          //ZemMoist
    if (napis[0][in] == "SENS0") { mesto[0][in] = &izhodsmit[0][in + 0]; }  //Temp
    if (napis[0][in] == "SENS1") { mesto[0][in] = &izhodsmit[1][in + 0]; }  //RH
    if (napis[0][in] == "SENS2") { mesto[0][in] = &izhodsmit[2][in + 0]; }  //ZemMoist
    if (napis[0][in] == "SENS3") { mesto[0][in] = &izhodsmit[3][in + 0]; }  //ZemMoist
    if (napis[0][in] == "URNIK") { mesto[0][in] = &procentoff; }
    if (napis[0][in] == "IZKLP") { mesto[0][in] = &nulll; }
  }

  for (int in = 0; in < 4; in++) {
    if (napis[1][in] == "POTEN") { mesto[1][in] = &nulll; }
    if (napis[1][in] == "GRAF0") { mesto[1][in] = &procenti[1]; }   //Temp
    if (napis[1][in] == "GRAF1") { mesto[1][in] = &procenti[6]; }   //RH
    if (napis[1][in] == "GRAF2") { mesto[1][in] = &procenti[11]; }  //ZemMoist
    if (napis[1][in] == "SENS0") { mesto[1][in] = &izhodsmit[0][in + 4]; }
    if (napis[1][in] == "SENS1") { mesto[1][in] = &izhodsmit[1][in + 4]; }
    if (napis[1][in] == "SENS2") { mesto[1][in] = &izhodsmit[2][in + 4]; }
    if (napis[1][in] == "SENS3") { mesto[1][in] = &izhodsmit[3][in + 4]; }
    if (napis[1][in] == "URNIK") { mesto[1][in] = &procentoff; }
    if (napis[1][in] == "IZKLP") { mesto[1][in] = &nulll; }
  }

  for (int in = 0; in < 4; in++) {
    if (napis[2][in] == "POTEN") { mesto[2][in] = &nulll; }
    if (napis[2][in] == "GRAF0") { mesto[2][in] = &procenti[2]; }   //Temp
    if (napis[2][in] == "GRAF1") { mesto[2][in] = &procenti[7]; }   //RH
    if (napis[2][in] == "GRAF2") { mesto[2][in] = &procenti[12]; }  //ZemMoist
    if (napis[2][in] == "SENS0") { mesto[2][in] = &izhodsmit[0][in + 8]; }
    if (napis[2][in] == "SENS1") { mesto[2][in] = &izhodsmit[1][in + 8]; }
    if (napis[2][in] == "SENS2") { mesto[2][in] = &izhodsmit[2][in + 8]; }
    if (napis[2][in] == "SENS3") { mesto[2][in] = &izhodsmit[3][in + 8]; }
    if (napis[2][in] == "URNIK") { mesto[2][in] = &procentoff; }
    if (napis[2][in] == "IZKLP") { mesto[2][in] = &nulll; }
  }


  for (int in = 0; in < 4; in++) {
    if (napis[3][in] == "POTEN") { mesto[3][in] = &nulll; }
    if (napis[3][in] == "GRAF0") { mesto[3][in] = &procenti[3]; }   //Temp
    if (napis[3][in] == "GRAF1") { mesto[3][in] = &procenti[8]; }   //RH
    if (napis[3][in] == "GRAF2") { mesto[3][in] = &procenti[13]; }  //ZemMoist
    if (napis[3][in] == "SENS0") { mesto[3][in] = &izhodsmit[0][in + 12]; }
    if (napis[3][in] == "SENS1") { mesto[3][in] = &izhodsmit[1][in + 12]; }
    if (napis[3][in] == "SENS2") { mesto[3][in] = &izhodsmit[2][in + 12]; }
    if (napis[3][in] == "SENS3") { mesto[3][in] = &izhodsmit[3][in + 12]; }
    if (napis[3][in] == "URNIK") { mesto[3][in] = &procentoff; }
    if (napis[3][in] == "IZKLP") { mesto[3][in] = &nulll; }
  }


  for (int in = 0; in < 4; in++) {
    if (napis[4][in] == "POTEN") { mesto[4][in] = &nulll; }
    if (napis[4][in] == "GRAF0") { mesto[4][in] = &procenti[4]; }   //Temp
    if (napis[4][in] == "GRAF1") { mesto[4][in] = &procenti[9]; }   //RH
    if (napis[4][in] == "GRAF2") { mesto[4][in] = &procenti[14]; }  //ZemMoist
    if (napis[4][in] == "SENS0") { mesto[4][in] = &izhodsmit[0][in + 16]; }
    if (napis[4][in] == "SENS1") { mesto[4][in] = &izhodsmit[1][in + 16]; }
    if (napis[4][in] == "SENS2") { mesto[4][in] = &izhodsmit[2][in + 16]; }
    if (napis[4][in] == "SENS3") { mesto[4][in] = &izhodsmit[3][in + 16]; }
    if (napis[4][in] == "URNIK") { mesto[4][in] = &procentoff; }
    if (napis[4][in] == "IZKLP") { mesto[4][in] = &nulll; }
  }

  procent1 = map(pt1, 0, 100, 0, 29);
  procent2 = map(pt2, 0, 100, 0, 29);
  procent3 = map(pt3, 0, 100, 0, 29);
  procent4 = map(pt4, 0, 100, 0, 29);
  procent5 = map(pt5, 0, 100, 0, 29);


  // listamo skozi vse izhode in določamo parametre

  for (int vrstica = 0; vrstica < 5; vrstica++) {
    indekspoz = 0;
    indekspoz2 = 0;
    switch (vrstica) {  // s tem se določi izhod in parametri, vsaka vrstica drug izhod.
      case 0:
        izhod = &zamiktriak;
        maksimum = 8000, potenciometeri = &pt1;
        break;
      case 1:
        izhod = &psm1;
        maksimum = 255, potenciometeri = &pt2;
        break;
      case 2:
        izhod = &psm2;
        maksimum = 255, potenciometeri = &pt3;
        break;
      case 3:
        izhod = &rele1;
        maksimum = 100, potenciometeri = &pt4;
        break;
      case 4:
        izhod = &rele2;
        maksimum = 100, potenciometeri = &pt5;
        break;
    }

    for (int i = 0; i < 4; i++) {  // tu se določa število vplivnikov
      if ((*mesto[vrstica][i]) == -1) { break; }
      if ((*mesto[vrstica][i]) == -2) { break; }  // -2 imajo lahko scmitt prozilniki, če imamo enega samega v vrstici vplivnikov, le ta določa 100 ali nula moč, ki je odvisna od potenciometra, če bi imeli -1, bi ostal polen vpliv potenciometra, medtem ko je scmit 0.
      indekspoz++;
    }

    for (int i = 0; i < 4; i++) {  // tu se določa število vplivnikov
      if ((*mesto[vrstica][i]) == -1) { break; }
      indekspoz2++;
    }

    int maksimum2;

    int kolicina = 0;

    maksimum2 = map((*potenciometeri), 0, 100, maksimum, 0);  // potenciometer ima vpliv na vse vrednosti v vrsti, mapiramo vrednost da ustreza maksimumu za izhod


    // tuki pa preverjamo ali vplivnik prispeva k skupni vrednosti, ali le izklaplja naslednje vplivnike v vrsti in nastavljamo temu primerne parametre
    for (int mestoo = 0; mestoo < indekspoz2; mestoo++) {


      if (*mesto[vrstica][mestoo] == 101 || *mesto[vrstica][mestoo] == -2) {  //ce vplivnik ne prispeva se mora spremeniti rang koncne vrednosti
        kolicina++;
      }
      if (*mesto[vrstica][mestoo] == 101) {  //ce vplivnik ne prispeva se mora spremeniti rang koncne vrednosti
        (*mesto[vrstica][mestoo]) = 0;       // 101 pomeni vplinik vkljucen vendar ne prispeva k skupni vrednosti, torej ga lahko po potrebnih operacijah in pred izračunavanjem postavimo na 0,
      }
    }
    rangee[vrstica] = (indekspoz2 * 100) - (kolicina * 100);



    switch (indekspoz) {
      case 0:
        if ((*mesto[vrstica][0]) != -2) {
          (*izhod) = maksimum2;
          break;
        }  //če ni omogočen noben vplivnik, je izhod direktna vrednost potenciometra.

      case 1:
        if ((napis[vrstica][0] == "SENS0" || napis[vrstica][0] == "SENS1" || napis[vrstica][0] == "SENS2" || napis[vrstica][0] == "SENS3") && (*mesto[vrstica][0]) == 0) { (*mesto[vrstica][0]) = 100; }  // ce imamo samo smitt trigger v vrstici naj ima sam po sebi vrednost 0 ali 100
        (*izhod) = (*mesto[vrstica][0]) * gain[vrstica][0];
        (*izhod) = map((*izhod), 0, 100, maksimum, maksimum2);
        (*izhod) = constrain((*izhod), 0, maksimum);

        break;
      case 2:

        (*izhod) = ((*mesto[vrstica][0]) * gain[vrstica][0]) + ((*mesto[vrstica][1]) * gain[vrstica][1]);
        (*izhod) = map((*izhod), 0, rangee[vrstica], maksimum, maksimum2);
        (*izhod) = constrain((*izhod), 0, maksimum);
        break;
      case 3:
        (*izhod) = ((*mesto[vrstica][0]) * gain[vrstica][0]) + ((*mesto[vrstica][1]) * gain[vrstica][1]) + ((*mesto[vrstica][2]) * gain[vrstica][2]);
        (*izhod) = map((*izhod), 0, rangee[vrstica], maksimum, maksimum2);
        (*izhod) = constrain((*izhod), 0, maksimum);
        break;
      case 4:
        (*izhod) = ((*mesto[vrstica][0]) * gain[vrstica][0]) + ((*mesto[vrstica][1]) * gain[vrstica][1]) + ((*mesto[vrstica][2]) * gain[vrstica][2]) + ((*mesto[vrstica][3]) * gain[vrstica][3]);
        (*izhod) = map((*izhod), 0, rangee[vrstica], maksimum, maksimum2);
        (*izhod) = constrain((*izhod), 0, maksimum);
        break;
    }
    indekspoz = 0;
  }


  ledcWrite(14, psm1);
  if (rele1 == 100) { rele1 = 1; }
  if (rele2 == 100) { rele2 = 1; }

  rele1 = !rele1;
  rele2 = !rele2;



  if (!flaggrafmoci && !flagsmit && !flagurnik && flagizrismoc) {

    if (!flagizbrancfg) {
      tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
      if (izbirnikmoci == 0) { tft.setTextColor(TFT_WHITE, TFT_BLACK); }
      tft.setTextSize(1);
      tft.setCursor(8, 0);
      tft.print(pot1);
      tft.print(" ");
      tft.print(static_cast<int>(pt1));
      if (pt1 != 100) { tft.print(" "); }
      tft.pushImage(5, 10, 50, 47, potenciometer[procent1]);

      tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
      if (izbirnikmoci == 1) { tft.setTextColor(TFT_WHITE, TFT_BLACK); }
      tft.setTextSize(1);
      tft.setCursor(58, 0);
      tft.print(pot2);
      tft.print(" ");
      tft.print(static_cast<int>(pt2));
      if (pt2 != 100) { tft.print(" "); }
      tft.pushImage(55, 10, 50, 47, potenciometer[procent2]);

      tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
      if (izbirnikmoci == 2) { tft.setTextColor(TFT_WHITE, TFT_BLACK); }
      tft.setTextSize(1);
      tft.setCursor(108, 0);
      tft.print(pot3);
      tft.print(" ");
      tft.print(static_cast<int>(pt3));
      tft.print(" ");
      tft.pushImage(105, 10, 50, 47, potenciometer[procent3]);

      tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
      if (izbirnikmoci == 3) { tft.setTextColor(TFT_WHITE, TFT_BLACK); }
      tft.setTextSize(1);
      tft.setCursor(8, 64);
      tft.print(pot4);
      tft.print(" ");
      tft.print(static_cast<int>(pt4));
      if (pt4 != 100) { tft.print(" "); }
      tft.pushImage(5, 75, 50, 47, potenciometer[procent4]);

      tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
      if (izbirnikmoci == 4) { tft.setTextColor(TFT_WHITE, TFT_BLACK); }
      tft.setTextSize(1);
      tft.setCursor(58, 64);
      tft.print(pot5);
      tft.print(" ");
      tft.print(static_cast<int>(pt5));
      if (pt5 != 100) { tft.print(" "); }
      tft.pushImage(55, 75, 50, 47, potenciometer[procent5]);

      if (izbirnikmoci != 5) { tft.pushImage(115, 80, 35, 35, configicon[0]); }
      if (izbirnikmoci == 5) { tft.pushImage(112, 77, 40, 40, configicon2[0]); }
    } else {

      char x = 37, y = 11, i2 = 0, i3 = 0, i4 = 0;

      for (char i = 0; i < 20; i++) {


        tft.setCursor(x, y);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        if (flagizbirnikizbrano && indup2 == i4 && indup1 == i3) {
          tft.setTextColor(TFT_BLACK, TFT_YELLOW);
        }

        tft.print(gain[i4][i3]);
        i2++;
        i3++;
        x += 30;
        if (i2 == 4) {
          i2 = 0;
          i3 = 0;
          x = 37;
          y += 21;
          i4++;
        }
      }

      char pozicijanavpicno = 19;
      for (char vrsta = 0; vrsta < 5; vrsta++) {
        tft.setTextSize(1);
        tft.setCursor(19, pozicijanavpicno);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        if (indup1 == 0 && indup2 == vrsta) { tft.setTextColor(TFT_BLACK, TFT_YELLOW); }
        tft.print(napis[vrsta][0]);
        tft.setTextColor(TFT_BLACK);
        tft.print("-");
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        if (indup1 == 1 && indup2 == vrsta) { tft.setTextColor(TFT_BLACK, TFT_YELLOW); }
        tft.print(napis[vrsta][1]);
        tft.setTextColor(TFT_BLACK);
        tft.print("-");
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        if (indup1 == 2 && indup2 == vrsta) { tft.setTextColor(TFT_BLACK, TFT_YELLOW); }
        tft.print(napis[vrsta][2]);
        tft.setTextColor(TFT_BLACK);
        tft.print("-");
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        if (indup1 == 3 && indup2 == vrsta) { tft.setTextColor(TFT_BLACK, TFT_YELLOW); }
        tft.print(napis[vrsta][3]);
        tft.setTextColor(TFT_BLACK);
        pozicijanavpicno = pozicijanavpicno + 21;
      }
    }
  }
}

void drawLargeCircle(int x0, int y0, uint32_t color, int radius) {
  int x = radius;
  int y = 0;
  int err = 0;

  while (x >= y) {
    // Plot pixels in all octants
    tft.drawPixel(x0 + x, y0 + y, color);
    tft.drawPixel(x0 + y, y0 + x, color);
    tft.drawPixel(x0 - y, y0 + x, color);
    tft.drawPixel(x0 - x, y0 + y, color);
    tft.drawPixel(x0 - x, y0 - y, color);
    tft.drawPixel(x0 - y, y0 - x, color);
    tft.drawPixel(x0 + y, y0 - x, color);
    tft.drawPixel(x0 + x, y0 - y, color);

    if (err <= 0) {
      y += 1;
      err += 2 * y + 1;
    }
    if (err > 0) {
      x -= 1;
      err -= 2 * x + 1;
    }
  }
}

uint16_t getPowerGraphGradientColor(int x) {
  float blend = (float)x / (DISPLAY_WIDTH - 1);
  float smoothBlend = blend * blend * (3.0f - 2.0f * blend);  // smoothstep for softer transition
  uint8_t r = 24 + (uint8_t)(78 * smoothBlend);
  uint8_t g = 24 + (uint8_t)(18 * smoothBlend);
  uint8_t b = 30 + (uint8_t)(16 * smoothBlend);
  return tft.color565(r, g, b);
}

void drawPowerGraphPlotArea() {
  const int graphX = 36;
  const int graphY = 16;
  const int graphW = DISPLAY_WIDTH - 52;
  const int graphH = DISPLAY_HEIGHT - 44;
  const int pointRadius = 5;
  const int clearMargin = pointRadius + 1;

  int clearX = max(0, graphX - clearMargin);
  int clearY = max(0, graphY - clearMargin);
  int clearRight = min(DISPLAY_WIDTH - 1, graphX + graphW + clearMargin);
  int clearBottom = min(DISPLAY_HEIGHT - 1, graphY + graphH + clearMargin);

  for (int x = clearX; x <= clearRight; x++) {
    tft.drawFastVLine(x, clearY, clearBottom - clearY + 1, getPowerGraphGradientColor(x));
  }

  uint16_t gridColor = tft.color565(90, 90, 95);
  uint16_t axisColor = tft.color565(210, 210, 210);

  for (int i = 0; i <= 10; i++) {
    int x = graphX + (graphW * i) / 10;
    tft.drawFastVLine(x, graphY, graphH, gridColor);
  }

  for (int i = 0; i <= 10; i++) {
    int y = graphY + (graphH * i) / 10;
    tft.drawFastHLine(graphX, y, graphW, gridColor);
  }

  tft.drawRect(graphX, graphY, graphW, graphH, axisColor);
}

void drawPowerGraphBackground() {
  for (int x = 0; x < DISPLAY_WIDTH; x++) {
    tft.drawFastVLine(x, 0, DISPLAY_HEIGHT, getPowerGraphGradientColor(x));
  }

  drawPowerGraphPlotArea();

  const int graphX = 36;
  const int graphY = 16;
  const int graphW = DISPLAY_WIDTH - 52;
  const int graphH = DISPLAY_HEIGHT - 44;

  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);

  for (int tempC = 0; tempC <= 40; tempC += 4) {
    int x = map(tempC, 0, 40, graphX, graphX + graphW);
    tft.setCursor(x - (tempC >= 10 ? 6 : 3), graphY + graphH + 9);
    tft.print(tempC);
  }
  tft.setCursor(graphX + graphW - 4, graphY + graphH + 19);
  tft.print("C");

  for (int power = 0; power <= 100; power += 10) {
    int y = map(power, 0, 100, graphY + graphH, graphY);
    tft.setCursor(2, y - 3);
    tft.print(power);
  }
  tft.setCursor(2, graphY - 10);
  tft.print("%");
}

void dolocimocssr() {
  unsigned char indeks2;

  for (unsigned char i = 0; i < 5; i++) {
    indeks2 = 0;
    for (unsigned char indeks = 0; temperatura > merjenci[0][indeks]; indeks++) {
      indeks2++;
    }

    procenti[i] = map(temperatura, merjenci[0][indeks2 - 1], merjenci[0][indeks2], linijamoci[i][indeks2 - 1], linijamoci[i][indeks2]);
  }


  for (unsigned char i = 5; i < 10; i++) {
    indeks2 = 0;
    for (unsigned char indeks = 0; vlaznost > merjenci[1][indeks]; indeks++) {
      indeks2++;
    }

    procenti[i] = map(vlaznost, merjenci[1][indeks2 - 1], merjenci[1][indeks2], linijamoci[i][indeks2 - 1], linijamoci[i][indeks2]);
  }
}

void drawPowerGraphThickLine(int x1, int y1, int x2, int y2, uint16_t color) {
  tft.drawLine(x1, y1, x2, y2, color);
  tft.drawLine(x1, y1 + 1, x2, y2 + 1, color);
}

int getPowerGraphPointY(int row, int pointIndex, int graphY, int graphBottom) {
  int y = map(linijamoci[row][pointIndex], 0, 100, graphBottom, graphY);
  return constrain(y, graphY, graphBottom);
}

void drawPowerGraphSmoothCurveRange(int row, int startSegment, int endSegment, int graphX, int graphRight, int graphY, int graphBottom, uint16_t color) {
  startSegment = max(0, startSegment);
  endSegment = min(8, endSegment);
  if (startSegment > endSegment) {
    return;
  }

  const int samplesPerSegment = 6;

  for (int seg = startSegment; seg <= endSegment; seg++) {
    int p0 = max(0, seg - 1);
    int p1 = seg;
    int p2 = seg + 1;
    int p3 = min(9, seg + 2);

    int x0 = map(p0, 0, 9, graphX, graphRight);
    int x1 = map(p1, 0, 9, graphX, graphRight);
    int x2 = map(p2, 0, 9, graphX, graphRight);
    int x3 = map(p3, 0, 9, graphX, graphRight);

    int y0 = getPowerGraphPointY(row, p0, graphY, graphBottom);
    int y1 = getPowerGraphPointY(row, p1, graphY, graphBottom);
    int y2 = getPowerGraphPointY(row, p2, graphY, graphBottom);
    int y3 = getPowerGraphPointY(row, p3, graphY, graphBottom);

    float prevX = (float)x1;
    float prevY = (float)y1;

    for (int step = 1; step <= samplesPerSegment; step++) {
      float t = (float)step / (float)samplesPerSegment;
      float tt = t * t;
      float ttt = tt * t;

      float currX = 0.5f * ((2.0f * x1)
        + (-x0 + x2) * t
        + (2.0f * x0 - 5.0f * x1 + 4.0f * x2 - x3) * tt
        + (-x0 + 3.0f * x1 - 3.0f * x2 + x3) * ttt);

      float currY = 0.5f * ((2.0f * y1)
        + (-y0 + y2) * t
        + (2.0f * y0 - 5.0f * y1 + 4.0f * y2 - y3) * tt
        + (-y0 + 3.0f * y1 - 3.0f * y2 + y3) * ttt);

      int drawX1 = constrain((int)roundf(prevX), graphX, graphRight);
      int drawY1 = constrain((int)roundf(prevY), graphY, graphBottom);
      int drawX2 = constrain((int)roundf(currX), graphX, graphRight);
      int drawY2 = constrain((int)roundf(currY), graphY, graphBottom);

      drawPowerGraphThickLine(drawX1, drawY1, drawX2, drawY2, color);
      prevX = currX;
      prevY = currY;
    }
  }
}

void redrawPowerGraphPointArea(unsigned char pristej, int pointIndex, int selectedIndex) {
  const int graphX = 36;
  const int graphY = 16;
  const int graphW = DISPLAY_WIDTH - 52;
  const int graphH = DISPLAY_HEIGHT - 44;
  const int graphRight = graphX + graphW;
  const int graphBottom = graphY + graphH;
  const int row = indup2 + pristej;

  int startPoint = max(0, pointIndex - 1);
  int endPoint = min(9, pointIndex + 1);

  int minX = graphRight;
  int maxX = graphX;
  int minY = graphBottom;
  int maxY = graphY;

  for (int i = startPoint; i <= endPoint; i++) {
    int px = map(i, 0, 9, graphX, graphRight);
    int py = map(linijamoci[row][i], 0, 100, graphBottom, graphY);
    px = constrain(px, graphX, graphRight);
    py = constrain(py, graphY, graphBottom);
    minX = min(minX, px);
    maxX = max(maxX, px);
    minY = min(minY, py);
    maxY = max(maxY, py);
  }

  const int redrawMarginX = 10;
  const int redrawMarginTop = 8;
  const int redrawMarginBottom = 8;
  int regionX = max(0, minX - redrawMarginX);
  int regionY = max(0, minY - redrawMarginTop);
  int regionRight = min(DISPLAY_WIDTH - 1, maxX + redrawMarginX);
  int regionBottom = min(graphBottom + 6, maxY + redrawMarginBottom);

  for (int x = regionX; x <= regionRight; x++) {
    tft.drawFastVLine(x, regionY, regionBottom - regionY + 1, getPowerGraphGradientColor(x));
  }

  uint16_t gridColor = tft.color565(90, 90, 95);
  uint16_t axisColor = tft.color565(210, 210, 210);

  for (int i = 0; i <= 10; i++) {
    int x = graphX + (graphW * i) / 10;
    if (x >= regionX && x <= regionRight) {
      int y0 = max(graphY, regionY);
      int y1 = min(graphBottom, regionBottom);
      if (y1 >= y0) {
        tft.drawFastVLine(x, y0, y1 - y0 + 1, gridColor);
      }
    }
  }

  for (int i = 0; i <= 10; i++) {
    int y = graphY + (graphH * i) / 10;
    if (y >= regionY && y <= regionBottom) {
      int x0 = max(graphX, regionX);
      int x1 = min(graphRight, regionRight);
      if (x1 >= x0) {
        tft.drawFastHLine(x0, y, x1 - x0 + 1, gridColor);
      }
    }
  }

  if (graphX >= regionX && graphX <= regionRight) {
    int y0 = max(graphY, regionY);
    int y1 = min(graphBottom, regionBottom);
    if (y1 >= y0) { tft.drawFastVLine(graphX, y0, y1 - y0 + 1, axisColor); }
  }
  if (graphRight >= regionX && graphRight <= regionRight) {
    int y0 = max(graphY, regionY);
    int y1 = min(graphBottom, regionBottom);
    if (y1 >= y0) { tft.drawFastVLine(graphRight, y0, y1 - y0 + 1, axisColor); }
  }
  if (graphY >= regionY && graphY <= regionBottom) {
    int x0 = max(graphX, regionX);
    int x1 = min(graphRight, regionRight);
    if (x1 >= x0) { tft.drawFastHLine(x0, graphY, x1 - x0 + 1, axisColor); }
  }
  if (graphBottom >= regionY && graphBottom <= regionBottom) {
    int x0 = max(graphX, regionX);
    int x1 = min(graphRight, regionRight);
    if (x1 >= x0) { tft.drawFastHLine(x0, graphBottom, x1 - x0 + 1, axisColor); }
  }

  int firstSegment = max(0, startPoint - 1);
  int lastSegment = min(8, endPoint);
  drawPowerGraphSmoothCurveRange(row, firstSegment, lastSegment, graphX, graphRight, graphY, graphBottom, TFT_RED);

  if (selectedIndex >= 0 && selectedIndex <= 9) {
    int selectedX = map(selectedIndex, 0, 9, graphX, graphRight);
    int selectedY = getPowerGraphPointY(row, selectedIndex, graphY, graphBottom);
    selectedX = constrain(selectedX, graphX, graphRight);
    selectedY = constrain(selectedY, graphY, graphBottom);
    if (selectedX >= regionX - 6 && selectedX <= regionRight + 6 && selectedY >= regionY - 6 && selectedY <= regionBottom + 6) {
      drawLargeCircle(selectedX, selectedY, TFT_YELLOW, 5);
    }
  }
}

void grafmoci() {
  if (!grafmocizrisan) {
    grafmocizrisan = 1;
    const int graphX = 36;
    const int graphY = 16;
    const int graphW = DISPLAY_WIDTH - 52;
    const int graphH = DISPLAY_HEIGHT - 44;
    const int graphRight = graphX + graphW;
    const int graphBottom = graphY + graphH;
    if (!powerGraphStaticDrawn) {
      drawPowerGraphBackground();
      powerGraphStaticDrawn = 1;
    } else {
      drawPowerGraphPlotArea();
    }
    tft.setTextColor(TFT_LIGHTGREY);

    unsigned char pristej;   // pristejemo k indeksu, da vemo da nastavljamo drug izhod, to je takrat ko spreminjamo naslednjo vrstico vplivnikov
    if (napis[indup2][indup1] == "GRAF0") {
      pristej = 0;
    }
    if (napis[indup2][indup1] == "GRAF1") {
      pristej = 5;
    }
    if (napis[indup2][indup1] == "GRAF2") {
      pristej = 10;
    }


    drawPowerGraphSmoothCurveRange(indup2 + pristej, 0, 8, graphX, graphRight, graphY, graphBottom, TFT_RED);

    int selectedX = map(izbirnik, 0, 9, graphX, graphRight);
    int selectedY = getPowerGraphPointY(indup2 + pristej, izbirnik, graphY, graphBottom);
    selectedX = constrain(selectedX, graphX, graphRight);
    selectedY = constrain(selectedY, graphY, graphBottom);
    drawLargeCircle(selectedX, selectedY, TFT_YELLOW, 5);
  }
}

void ura() {
  // This function is now mostly handled by updateTimeFromRTC()
  // Keep only the month/year rollover logic as safety backup
  
  if (mesec > 12) {
    mesec = 1;
    leto++;
  }
  
  // The rest is handled automatically by RTC in updateTimeFromRTC()
}

void drawHorizontalMenu() {
  flagmenuglavnimenu = 1;
  if (!flagizrisanglavnipodmeni) {

    int gradientSteps = 10;

    // Calculate menu dimensions for 320x240 display
    // Position menu at bottom with narrower height (10% narrower than original 70px)
    int originalHeight = 70;
    int menuHeight = originalHeight * 0.9;  // 10% narrower height (63px)
    int menuY = DISPLAY_HEIGHT - menuHeight;  // Position at very bottom
    int menuWidth = DISPLAY_WIDTH;  // Use full width
    int menuX = 0;  // Start from left edge

    // Draw the main menu background at bottom with narrower height
    tft.fillRect(menuX, menuY, menuWidth, menuHeight, tft.color565(64, 64, 64));

    // Draw gradient effect above menu (full width)
    int y = menuY - 20;
    for (int i = 0; i < 20; i++) {
      y++;
      uint8_t grayValue = map(i, 0, 20 - 1, 0, 64);
      tft.fillRect(menuX, y, menuWidth, 1, tft.color565(grayValue, grayValue, grayValue));
    }

    flagizrisanglavnipodmeni = 1;

    // Calculate icon positions for centered layout on full width
    // Keep original icon sizes intact
    int iconSpacing = (menuWidth - (20 + 20 + 20)) / 4;  // Even spacing between 3 icons
    int iconY = menuY + 15;  // Position icons lower within menu area

    int icon1X = menuX + iconSpacing;  // First icon (graph)
    int icon2X = menuX + iconSpacing + 20 + iconSpacing;  // Second icon (config)
    int icon3X = menuX + iconSpacing + 20 + iconSpacing + 20 + iconSpacing;  // Third icon (settings)

    // Draw icons with original sizes, positioned lower
    if (!flagikonagraf) { tft.pushImage(icon1X, iconY + 13, 35, 29, IkonaGraf[0]); }
    if (flagikonagraf) { tft.pushImage(icon1X, iconY + 7, 40, 33, IkonaGraf1[0]); }
    if (!flagikonanast) { tft.pushImage(icon3X, iconY + 7, 35, 36, IkonaZob[0]); }
    if (flagikonanast) { tft.pushImage(icon3X, iconY, 40, 41, IkonaZob1[0]); }
    if (!flagikonacfg) { tft.pushImage(icon2X, iconY + 7, 35, 30, ikonacfg[0]); }
    if (flagikonacfg) { tft.pushImage(icon2X, iconY + 2, 40, 35, ikonacfgizbrana[0]); }

    // Draw menu selection text below gradient bar
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);
    
    // Calculate text position - centered below gradient bar
    int textY = menuY - 5;  // Position below gradient bar
    
    if (flagikonagraf) {
      tft.setCursor(menuX + menuWidth/2 - 30, textY);  // Center "GRAF" text
      tft.print("GRAFI");
    } else if (flagikonacfg) {
      tft.setCursor(menuX + menuWidth/2 - 30, textY);  // Center "CONFIG" text
      tft.print("NASTAVITVE MOCI");
    } else if (flagikonanast) {
      tft.setCursor(menuX + menuWidth/2 - 30, textY);  // Center "NASTAVITVE" text
      tft.print("NASTAVITVE");
    }
  }
}

void meritev() {

  tok = random(2047);
  flagglavnimeniizrisan = 0;
  flagizrisanglavnipodmeni = 0;

temperatura2 = bme.readTemperature();
vlaznost2 = bme.readHumidity();
pritisk2 = bme.readPressure() / 100.0F; // convert Pa to hPa
vpd2 = calculateVPD(temperatura2, vlaznost2); // Calculate VPD

  //if (intValue < 99) {
    temperatura = static_cast<char>(temperatura2);
 // } else {

   // digitalWrite(BACKLIGHT_PIN, LOW);
   // delay(500);
  //  digitalWrite(BACKLIGHT_PIN, HIGH);
  //  Wire.begin();
 //   dht.begin();
 //   String message = "Senzor DHT resetiran!";
   // sendDiscordMessage(message);
  //}
  //if (intValue2 < 99) {
    vlaznost = static_cast<char>(vlaznost2);
  //}

  // Current day max/min and trend tracking
  // Check if day changed and reset variables if needed
  if (trenutniDan != dnevi) {
    // New day, reset current day tracking variables
    trenutniDnevniMaxTemp = temperatura2;
    trenutniDnevniMinTemp = temperatura2;
    trenutniDnevniMaxRH = vlaznost2;
    trenutniDnevniMinRH = vlaznost2;
    trenutniDan = dnevi;
    prejsnjaTemperatura = temperatura2;
    prejsnjaVlaznost = vlaznost2;
    tempRising = false;
    tempFalling = false;
    vlaznostRising = false;
    vlaznostFalling = false;
  } else {
    // Same day, update max/min values
    if (temperatura2 > trenutniDnevniMaxTemp) {
      trenutniDnevniMaxTemp = temperatura2;
    }
    if (temperatura2 < trenutniDnevniMinTemp) {
      trenutniDnevniMinTemp = temperatura2;
    }
    if (vlaznost2 > trenutniDnevniMaxRH) {
      trenutniDnevniMaxRH = vlaznost2;
    }
    if (vlaznost2 < trenutniDnevniMinRH) {
      trenutniDnevniMinRH = vlaznost2;
    }
    
    // Update trend indicators if we have previous readings
    if (prejsnjaTemperatura > -900) { // Valid previous reading
      if (temperatura2 > prejsnjaTemperatura + 0.1) {
        tempRising = true;
        tempFalling = false;
      } else if (temperatura2 < prejsnjaTemperatura - 0.1) {
        tempFalling = true;
        tempRising = false;
      }
    }
    
    if (prejsnjaVlaznost > -900) { // Valid previous reading
      if (vlaznost2 > prejsnjaVlaznost + 0.5) {
        vlaznostRising = true;
        vlaznostFalling = false;
      } else if (vlaznost2 < prejsnjaVlaznost - 0.5) {
        vlaznostFalling = true;
        vlaznostRising = false;
      }
    }
    
    prejsnjaTemperatura = temperatura2;
    prejsnjaVlaznost = vlaznost2;
  }


  // Read soil moisture sensors dynamically based on discovered nodes
  for (int i = 0; i < discovered_nodes && i < MAX_SOIL_SENSORS; i++) {
    uint16_t value = 0;
    uint8_t addr = node_addresses[i];
    
    if (readDataFromNode(addr, &value)) {
      float voltage = value;
      voltage = voltage / 4;  // pretvorba iz 1023 na 255.
      voltage = voltage * (3.3 / 255.0); // pretvorba iz 255 v napetost 0-3.3V
      
      // Apply calibration and convert to percentage
      voltage = ((voltage - kalibracijamin[i]) / (kalibracijamax[i] - kalibracijamin[i])) * 100;
      
      // Clamp values to valid range
      if (voltage > 99) { voltage = 99; }
      if (voltage < 0) { voltage = 0; }
      
      vlagazemlje[i] = voltage;
    } else {
      // If reading failed, set to 0 or last known value
      vlagazemlje[i] = 0;
    }
    
    delay(100);  // Small delay between requests
  }
  
  // Calculate grow environment status
  calculateGrowStatus();
  
  // Broadcast updated sensor data to all connected web clients
  broadcastSensorData();
}

void izrisglavnimenu() {
  if (!flagglavnimeniizrisan) {

    flagglavnimeniizrisan = 1;



    if (flagglavnimenu && !flagmenuglavnimenu) {
      //tft.pushImage(0, 0, 320, 240, skyozadje);
       TJpgDec.drawFsJpg(0, 0, "/skyozadje.jpg", FFat);

      // WiFi status display with proper clearing
      tft.setTextSize(1);
      
   
      
      if (WiFi.status() == WL_CONNECTED) {
        tft.setCursor(240, 10);
        tft.setTextColor(TFT_GREEN);
        tft.print("POVEZANO");
      } else if (WiFi.status() == WL_CONNECT_FAILED) {
        tft.setCursor(240, 10);
        tft.setTextColor(TFT_RED);
        tft.print("NAPAKA");
      } else if (WiFi.status() == WL_DISCONNECTED) {
        tft.setCursor(240, 10);
        tft.setTextColor(TFT_RED);
        tft.print("PREKINJENO");
      } else if (WiFi.status() == WL_IDLE_STATUS) {
        tft.setCursor(240, 10);
        tft.setTextColor(TFT_YELLOW);
        tft.print("POVEZUJEM");
      } else {
        tft.setCursor(230, 10);
        tft.setTextColor(TFT_RED);
        tft.print("NI POVEZAVE");
      }
      tft.setCursor(290, 10);
      mocsignala = map(WiFi.RSSI(), -100, -50, 0, 100);
      tft.print(mocsignala); 
       tft.print("%  ");
      
      tft.setTextColor(TFT_WHITE);
      tft.setTextSize(3);
      tft.setCursor(105, 212);
      if (ure < 10) { tft.print("0"); }
      tft.print(ure);
      if (utripaj) {
        tft.print(":");
      } else {
        tft.print(":");
      }
      utripaj = !utripaj;
      if (minuta < 10) { tft.print("0"); }
      tft.print(minuta);
      tft.print(" ");
      tft.setCursor(200, 220);
      tft.setTextSize(2);
      tft.print(dnevi);
      tft.print(".");
      tft.print(mesec);
      tft.print(".");
      tft.print(leto);
      tft.print(" ");
      tft.drawString(dan[stdan], 200, 200);
    }


    tft.setTextSize(3);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(24, 11);
    if (temperatura < 10) { tft.print(" ");tft.setCursor(10, 15); }
    tft.print(static_cast<int>(temperatura2 * 10) / 10.0f, 1);
    tft.setTextSize(1);
    tft.print(" ");
    tft.setTextSize(3);
    tft.print("C");

    tft.setTextSize(3);
    tft.setCursor(24, 43);
    if (vlaznost2 < 10) { tft.print(" "); }
    tft.print(static_cast<int>(vlaznost2 * 10) / 10.0f, 1);  
     tft.setTextSize(1);
     tft.print(" ");
      tft.setTextSize(3);
    tft.print("%");

    // Daily max/min display next to main readings
    
    // Temperature max/min and trend next to main temperature
    tft.setTextSize(1);
    tft.setCursor(130, 8);  // Above temperature, next to C mark
    tft.setTextColor(TFT_RED);
    tft.print(static_cast<int>(trenutniDnevniMaxTemp * 10) / 10.0f, 1);
    
    tft.setCursor(130, 28);  // Below temperature, next to C mark  
    tft.setTextColor(TFT_BLUE);
    tft.print(static_cast<int>(trenutniDnevniMinTemp * 10) / 10.0f, 1);
    
    // Temperature trend indicator (larger and more visible)
    tft.setCursor(130, 18);  // Next to temperature
    if (tempRising) {
      tft.setTextColor(TFT_RED);
      tft.setTextSize(1);
      tft.print("narasca");
    } else if (tempFalling) {
      tft.setTextColor(TFT_BLUE);
      tft.setTextSize(1);
      tft.print("pada");
    } else {
      tft.setTextColor(TFT_WHITE);
      tft.setTextSize(1);
      tft.print("stabilno");
    }
    
    // Humidity max/min and trend next to main humidity
    tft.setTextSize(1);
    tft.setCursor(130, 40);  // Above humidity, next to % mark
    tft.setTextColor(TFT_RED);  // Same color as temp max
    tft.print(static_cast<int>(trenutniDnevniMaxRH * 10) / 10.0f, 1);
    
    tft.setCursor(130, 60);  // Below humidity, next to % mark
    tft.setTextColor(TFT_BLUE);  // Same color as temp min
    tft.print(static_cast<int>(trenutniDnevniMinRH * 10) / 10.0f, 1);
    
    // Humidity trend indicator (larger and more visible)
    tft.setCursor(130, 50);  // Next to humidity
    if (vlaznostRising) {
      tft.setTextColor(TFT_RED);  // Same color as temp rising
      tft.setTextSize(1);
      tft.print("narasca");
    } else if (vlaznostFalling) {
      tft.setTextColor(TFT_BLUE);  // Same color as temp falling
      tft.setTextSize(1);
      tft.print("pada");
    } else {
      tft.setTextColor(TFT_WHITE);
      tft.setTextSize(1);
      tft.print("stabilno");
    }
    
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
   
    tft.setTextSize(2);
    tft.setCursor(14, 73);
     if (pritisk2 < 1000) { tft.print(" "); }
    tft.print(static_cast<int>(pritisk2));
     tft.setTextSize(1);
     tft.print(" ");
      tft.setTextSize(2);
    tft.print("hPa");
    
    // Weather prediction display
    tft.setTextSize(1);
    tft.setCursor(14, 95);
    tft.setTextColor(TFT_YELLOW);
    tft.print("Napoved: ");
    tft.print(napovedVremena());
    tft.setTextColor(TFT_WHITE);
    
  }
}

void prikaziVlagazemlje() {
  // Display soil moisture data horizontally, wrap to next line after 10 items
  int startX = 10;
  int startY = 120;
  int itemSpacing = 70;  // Horizontal spacing between items (increased for text size 2)
  int lineSpacing = 25;  // Vertical spacing between lines (increased for text size 2)
  int itemsPerLine = 10;
  
  // Display actual number of discovered sensors, limited by display setting
  int sensorsToDisplay = min((int)stPrikazanihVlagazemlje, (int)discovered_nodes);
  
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  
  for (int i = 0; i < sensorsToDisplay; i++) {
    int x = startX + (i % itemsPerLine) * itemSpacing;
    int y = startY + (i / itemsPerLine) * lineSpacing;
    
    tft.setCursor(x, y);
    tft.print("Z");
    tft.print(i + 1);
    tft.print(":");
    
    // Ensure we display the value even if it's 0
    int value = (int)vlagazemlje[i];
    if (value < 0) value = 0;  // Ensure non-negative
    if (value > 99) value = 99; // Cap at 99 for display
    
    tft.print(value);
    tft.print("%");
  }
}

String napovedVremena() {
  // Weather prediction based on hourly pressure change rate
  // spremembaPritiskaNaUro is calculated every hour when ure++
  
  if (spremembaPritiskaNaUro > 5.0) {
    return "Hitro jasnenje";  // >5 hPa/hour - rapidly rising
  } else if (spremembaPritiskaNaUro > 2.0) {
    return "Jasni se";     // 2-5 hPa/hour - moderately rising
  } else if (spremembaPritiskaNaUro > -1.0 && spremembaPritiskaNaUro <= 2.0) {
    return "Stabilno";     // -1 to 2 hPa/hour - stable
  } else if (spremembaPritiskaNaUro > -3.0 && spremembaPritiskaNaUro <= -1.0) {
    return "Oblacenje";      // -3 to -1 hPa/hour - slowly falling
  } else if (spremembaPritiskaNaUro <= -3.0) {
    return "Nevihta";      // <-3 hPa/hour - rapidly falling
  } else {
    return "Spremenljivo"; // First reading or no data
  }
}

void vzorec() {

  // Ko zasedemo ves prostor premaknemo vse vnose nazaj, tako da se prvi (najstarejši) vnos zgubi, zadnji pa je dupliran, katerega kasneje prepišemo z novimi podatki.
  if (indeksvzorca > 364) {


    for (int i = 0; i < 365; ++i) {
      mesecvzorec[i] = mesecvzorec[i + 1];
      letovzorec[i] = letovzorec[i + 1];
      dnevivzorec[i] = dnevivzorec[i + 1];
      povprecnadnevnaT[i] = povprecnadnevnaT[i + 1];
      najvisjadnevnaT[i] = najvisjadnevnaT[i + 1];
      najnizjadnevnaT[i] = najnizjadnevnaT[i + 1];
      najnizjadnevnasens1[i] = najnizjadnevnasens1[i + 1];
      najnizjadnevnasens2[i] = najnizjadnevnasens2[i + 1];
      najnizjadnevnasens3[i] = najnizjadnevnasens3[i + 1];
      najnizjadnevnasens4[i] = najnizjadnevnasens4[i + 1];

      povprecnadnevnaRH[i] = povprecnadnevnaRH[i + 1];
      najvisjadnevnaRH[i] = najvisjadnevnaRH[i + 1];
      najnizjadnevnaRH[i] = najnizjadnevnaRH[i + 1];
      for (int i2 = 0; i2 < 25; ++i2) {
        tempPodat[i][i2] = tempPodat[i + 1][i2];
        RH[i][i2] = RH[i + 1][i2];
        senzor1[i][i2] = senzor1[i + 1][i2];
        senzor2[i][i2] = senzor2[i + 1][i2];
        senzor3[i][i2] = senzor3[i + 1][i2];
        senzor4[i][i2] = senzor4[i + 1][i2];
      }
    }
    najnizjadnevnaT[364] = 101;
    povprecnadnevnaRH[364] = 101;
    najvisjadnevnaRH[364] = 101;
    najnizjadnevnaRH[364] = 101;
    povprecnadnevnaT[364] = 101;
    najvisjadnevnaT[364] = 101;
    najnizjadnevnasens1[364] = 101;
    najnizjadnevnasens2[364] = 101;
    najnizjadnevnasens3[364] = 101;
    najnizjadnevnasens4[364] = 101;

    for (int i2 = 0; i2 < 25; ++i2) {
      tempPodat[364][i2] = 101;
      RH[364][i2] = 101;
      senzor1[364][i2] = 101;
      senzor2[364][i2] = 101;
      senzor3[364][i2] = 101;
      senzor4[364][i2] = 101;
    }

    indeksvzorca = 364;

    if (prikazanindeks > 0) {
      prikazanindeks--;
    }
  }


  if (indeksvzorca < 365) {

    mesecvzorec[indeksvzorca] = mesec;
    letovzorec[indeksvzorca] = leto;
    dnevivzorec[indeksvzorca] = dnevi;
    //tempPodat[indeksvzorca][ure] = random(39);
    // RH[indeksvzorca][ure] = random(99);
    tempPodat[indeksvzorca][ure] = temperatura;
    RH[indeksvzorca][ure] = vlaznost;
    
    // Log the selected 4 sensors for data history and graphing
    for (int i = 0; i < LOGGED_SENSORS_COUNT; i++) {
      int sensorIndex = selectedSensors[i] - 1;  // Convert from 1-based to 0-based
      if (sensorIndex >= 0 && sensorIndex < discovered_nodes && sensorIndex < MAX_SOIL_SENSORS) {
        float sensorValue = vlagazemlje[sensorIndex];
        switch (i) {
          case 0: senzor1[indeksvzorca][ure] = sensorValue; break;
          case 1: senzor2[indeksvzorca][ure] = sensorValue; break;
          case 2: senzor3[indeksvzorca][ure] = sensorValue; break;
          case 3: senzor4[indeksvzorca][ure] = sensorValue; break;
        }
        // DEBUG: Print sensor logging info
        Serial.printf("[LOG] Data log %d: Selected sensor %d (array index %d), value: %.1f\n", 
                     i+1, selectedSensors[i], sensorIndex, sensorValue);
      } else {
        // If selected sensor doesn't exist, store 101 (no data flag)
        switch (i) {
          case 0: senzor1[indeksvzorca][ure] = 101; break;
          case 1: senzor2[indeksvzorca][ure] = 101; break;
          case 2: senzor3[indeksvzorca][ure] = 101; break;
          case 3: senzor4[indeksvzorca][ure] = 101; break;
        }
        // DEBUG: Print invalid sensor info
        Serial.printf("[LOG] Data log %d: Selected sensor %d (array index %d) INVALID - storing 101\n", 
                     i+1, selectedSensors[i], sensorIndex);
      }
    }

    //senzor1[indeksvzorca][ure] = random(99);
    //senzor2[indeksvzorca][ure] = random(99);
    //senzor3[indeksvzorca][ure] = random(99);
    //senzor4[indeksvzorca][ure] = random(99);
  }
  Date dates[size];
  for (int i = 0; i < size; ++i) {
    dates[i] = Date(dnevivzorec[i], mesecvzorec[i], letovzorec[i], i);
  }

  // Sort the dates array using the custom comparison function
  sort(dates, dates + size, compareDates);

  // Create an array to store the order of dates

  // Initialize ALL positions to sentinel value FIRST
  for (int i = 0; i < size; ++i) {
    arraysinorder[i] = 1001;
  }

  // Then fill in valid dates
  int j = 0;
  for (int i = 0; i < size; ++i) {
    if (dates[i].leto != 101) {
      arraysinorder[j] = dates[i].index;
      j++;
    }
  }


  pointerracunaj = &tempPodat;
  pointerracunaj2 = &najvisjadnevnaT;
  pointerracunaj3 = &najnizjadnevnaT;
  pointerracunaj4 = &povprecnadnevnaT;
  racunaj();

  pointerracunaj = &RH;
  pointerracunaj2 = &najvisjadnevnaRH;
  pointerracunaj3 = &najnizjadnevnaRH;
  pointerracunaj4 = &povprecnadnevnaRH;
  racunaj();
  pointerracunaj = &senzor1;
  pointerracunaj3 = &najnizjadnevnasens1;
  racunaj();
  pointerracunaj = &senzor2;
  pointerracunaj3 = &najnizjadnevnasens2;
  racunaj();
  pointerracunaj = &senzor3;
  pointerracunaj3 = &najnizjadnevnasens3;
  racunaj();
  pointerracunaj = &senzor4;
  pointerracunaj3 = &najnizjadnevnasens4;
  racunaj();

  if (flagdnevnigraf || flagmesecnigraf) { osvezigraf(); }
}

void osvezigraf() {
  flagnarisan = 0;

  if (flagdnevnigraf && !flagmesecnigraf) {
    if (flagmenuTemp) {
      pointergraf = &tempPodat;
      temperatureMin = 0;
      temperatureMax = 40;
      flagnarisan = 0;
      drawGraph();
    }

    if (flagmenuRH) {
      pointergraf = &RH;
      temperatureMin = 0;
      temperatureMax = 101;
      flagnarisan = 0;
      drawGraph();
    }
    if (flagmenuSNZ1) {
      pointergraf = &senzor1;
      temperatureMin = 0;
      temperatureMax = 101;
      flagnarisan = 0;
      drawGraph();
    }
    if (flagmenuSNZ2) {
      pointergraf = &senzor2;
      temperatureMin = 0;
      temperatureMax = 101;
      flagnarisan = 0;
      drawGraph();
    }
    if (flagmenuSNZ3) {
      pointergraf = &senzor3;
      temperatureMin = 0;
      temperatureMax = 101;
      flagnarisan = 0;
      drawGraph();
    }
    if (flagmenuSNZ4) {
      pointergraf = &senzor4;
      temperatureMin = 0;
      temperatureMax = 101;
      flagnarisan = 0;
      drawGraph();
    }
  }

  if (flagmesecnigraf) {
    if (flagmenuPDt) {
      pointergrafobdobja = &povprecnadnevnaT;
      maxyobdobje = 40;
      flagnarisan = 0;
      drawGraph();
    }
    if (flagmenuMiDt) {
      pointergrafobdobja = &najnizjadnevnaT;
      maxyobdobje = 40;
      flagnarisan = 0;
      drawGraph();
    }

    if (flagmenuMDt) {
      pointergrafobdobja = &najvisjadnevnaT;
      maxyobdobje = 40;
      flagnarisan = 0;
      drawGraph();
    }
    if (flagmenuPDRH) {
      pointergrafobdobja = &povprecnadnevnaRH;
      maxyobdobje = 99;
      flagnarisan = 0;
      drawGraph();
    }
    if (flagmenuMiRH) {
      pointergrafobdobja = &najnizjadnevnaRH;
      maxyobdobje = 99;
      flagnarisan = 0;
      drawGraph();
    }

    if (flagmenuMRH) {
      pointergrafobdobja = &najvisjadnevnaRH;
      maxyobdobje = 99;
      flagnarisan = 0;
      drawGraph();
    }
    if (flagmenusens1) {
      pointergrafobdobja = &najnizjadnevnasens1;
      maxyobdobje = 99;
      flagnarisan = 0;
      drawGraph();
    }
    if (flagmenusens2) {
      pointergrafobdobja = &najnizjadnevnasens2;
      maxyobdobje = 99;
      flagnarisan = 0;
      drawGraph();
    }
    if (flagmenusens3) {
      pointergrafobdobja = &najnizjadnevnasens3;
      maxyobdobje = 99;
      flagnarisan = 0;
      drawGraph();
    }
    if (flagmenusens4) {
      pointergrafobdobja = &najnizjadnevnasens4;
      maxyobdobje = 99;
      flagnarisan = 0;
      drawGraph();
    }
  }


  drawGraph();
}
void shranivse() {
  Serial.println("[SAVE] Starting data save to FFat...");
  
  bool success = true;
  
  success &= saveArrayToSPIFFS("/tempPodat.bin", tempPodat, sizeof(char), 365, 25);
  Serial.printf("[SAVE] tempPodat.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/RH.bin", RH, sizeof(char), 365, 25);
  Serial.printf("[SAVE] RH.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/senzor1.bin", senzor1, sizeof(char), 365, 25);
  Serial.printf("[SAVE] senzor1.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/senzor2.bin", senzor2, sizeof(char), 365, 25);
  Serial.printf("[SAVE] senzor2.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/senzor3.bin", senzor3, sizeof(char), 365, 25);
  Serial.printf("[SAVE] senzor3.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/senzor4.bin", senzor4, sizeof(char), 365, 25);
  Serial.printf("[SAVE] senzor4.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/dnevivzorec.bin", dnevivzorec, sizeof(int), 365);
  Serial.printf("[SAVE] dnevivzorec.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/mesecvzorec.bin", mesecvzorec, sizeof(int), 365);
  Serial.printf("[SAVE] mesecvzorec.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/letovzorec.bin", letovzorec, sizeof(int), 365);
  Serial.printf("[SAVE] letovzorec.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/povprecnadnevnaT.bin", povprecnadnevnaT, sizeof(char), 365);
  Serial.printf("[SAVE] povprecnadnevnaT.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/najvisjadnevnaT.bin", najvisjadnevnaT, sizeof(char), 365);
  Serial.printf("[SAVE] najvisjadnevnaT.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/najnizjadnevnaT.bin", najnizjadnevnaT, sizeof(char), 365);
  Serial.printf("[SAVE] najnizjadnevnaT.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/najnizjadnevnasens1.bin", najnizjadnevnasens1, sizeof(char), 365);
  Serial.printf("[SAVE] najnizjadnevnasens1.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/najnizjadnevnasens2.bin", najnizjadnevnasens2, sizeof(char), 365);
  Serial.printf("[SAVE] najnizjadnevnasens2.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/najnizjadnevnasens3.bin", najnizjadnevnasens3, sizeof(char), 365);
  Serial.printf("[SAVE] najnizjadnevnasens3.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/najnizjadnevnasens4.bin", najnizjadnevnasens4, sizeof(char), 365);
  Serial.printf("[SAVE] najnizjadnevnasens4.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/povprecnadnevnaRH.bin", povprecnadnevnaRH, sizeof(char), 365);
  Serial.printf("[SAVE] povprecnadnevnaRH.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/najvisjadnevnaRH.bin", najvisjadnevnaRH, sizeof(char), 365);
  Serial.printf("[SAVE] najvisjadnevnaRH.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveArrayToSPIFFS("/najnizjadnevnaRH.bin", najnizjadnevnaRH, sizeof(char), 365);
  Serial.printf("[SAVE] najnizjadnevnaRH.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveIntToSPIFFS("/indeksvzorca.bin", indeksvzorca);
  Serial.printf("[SAVE] indeksvzorca.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= saveIntToSPIFFS("/stPrikazanihVlagazemlje.bin", stPrikazanihVlagazemlje);
  Serial.printf("[SAVE] stPrikazanihVlagazemlje.bin: %s\n", success ? "OK" : "FAILED");
  
  // Save selected sensors for logging
  Serial.println("[SAVE] Saving selected sensors configuration:");
  for (int i = 0; i < LOGGED_SENSORS_COUNT; i++) {
    String path = "/selectedSensor" + String(i) + ".bin";
    success &= saveIntToSPIFFS(path.c_str(), selectedSensors[i]);
    Serial.printf("  Data log %d: Sensor %d - %s\n", i+1, selectedSensors[i], success ? "OK" : "FAILED");
  }
  
  Serial.printf("[SAVE] Save operation completed. Overall: %s\n", success ? "SUCCESS" : "FAILED");
}

bool izbrisivnos(const char* path) {
  // Check if the file exists
  if (FFat.exists(path)) {

    // Delete the file
    if (FFat.remove(path)) {

      return true;
    } else {

      return false;
    }
  } else {
    napaka = 1;  // ni shranjenih vnosov
    return false;
  }
}

void izbrisizacasne() {
  for (int i2 = 0; i2 < 365; ++i2) {
    for (int i = 0; i < 25; ++i) {
      tempPodat[i2][i] = 101;
      mesecvzorec[i2] = 101;
      letovzorec[i2] = 101;
      dnevivzorec[i2] = 101;
      najvisjadnevnaT[i2] = 101;
      najnizjadnevnaT[i2] = 101;
      najnizjadnevnasens1[i2] = 101;
      najnizjadnevnasens2[i2] = 101;
      najnizjadnevnasens3[i2] = 101;
      najnizjadnevnasens4[i2] = 101;
      povprecnadnevnaT[i2] = 101;
      najvisjadnevnaRH[i2] = 101;
      najnizjadnevnaRH[i2] = 101;
      povprecnadnevnaRH[i2] = 101;
      RH[i2][i] = 101;
      senzor1[i2][i] = 101;
      senzor2[i2][i] = 101;
      senzor3[i2][i] = 101;
      senzor4[i2][i] = 101;
      indeksvzorca = 0;
    }
  }
}

bool clearAllDataFiles() {
  Serial.println("[CLEAR] Removing all existing data files...");
  
  const char* files[] = {
    "/tempPodat.bin", "/RH.bin", "/senzor1.bin", "/senzor2.bin", "/senzor3.bin", "/senzor4.bin",
    "/dnevivzorec.bin", "/mesecvzorec.bin", "/letovzorec.bin",
    "/povprecnadnevnaT.bin", "/najvisjadnevnaT.bin", "/najnizjadnevnaT.bin",
    "/najnizjadnevnasens1.bin", "/najnizjadnevnasens2.bin", "/najnizjadnevnasens3.bin", "/najnizjadnevnasens4.bin",
    "/povprecnadnevnaRH.bin", "/najvisjadnevnaRH.bin", "/najnizjadnevnaRH.bin",
    "/indeksvzorca.bin", "/stPrikazanihVlagazemlje.bin"
  };
  
  int removed = 0;
  int failed = 0;
  
  for (int i = 0; i < sizeof(files)/sizeof(files[0]); i++) {
    if (FFat.exists(files[i])) {
      if (FFat.remove(files[i])) {
        removed++;
        Serial.printf("[CLEAR] Removed %s\n", files[i]);
      } else {
        failed++;
        Serial.printf("[CLEAR] Failed to remove %s\n", files[i]);
      }
    }
  }
  
  Serial.printf("[CLEAR] Completed: %d files removed, %d failed\n", removed, failed);
  return (failed == 0);
}

void nalozivse() {
  Serial.println("[LOAD] Starting data load from FFat...");
  
  bool success = true;
  
  success &= loadArrayFromSPIFFS("/tempPodat.bin", tempPodat, sizeof(char), 365, 25);
  Serial.printf("[LOAD] tempPodat.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/RH.bin", RH, sizeof(char), 365, 25);
  Serial.printf("[LOAD] RH.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/senzor1.bin", senzor1, sizeof(char), 365, 25);
  Serial.printf("[LOAD] senzor1.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/senzor2.bin", senzor2, sizeof(char), 365, 25);
  Serial.printf("[LOAD] senzor2.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/senzor3.bin", senzor3, sizeof(char), 365, 25);
  Serial.printf("[LOAD] senzor3.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/senzor4.bin", senzor4, sizeof(char), 365, 25);
  Serial.printf("[LOAD] senzor4.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/dnevivzorec.bin", dnevivzorec, sizeof(int), 365);
  Serial.printf("[LOAD] dnevivzorec.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/mesecvzorec.bin", mesecvzorec, sizeof(int), 365);
  Serial.printf("[LOAD] mesecvzorec.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/letovzorec.bin", letovzorec, sizeof(int), 365);
  Serial.printf("[LOAD] letovzorec.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/povprecnadnevnaT.bin", povprecnadnevnaT, sizeof(char), 365);
  Serial.printf("[LOAD] povprecnadnevnaT.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/najvisjadnevnaT.bin", najvisjadnevnaT, sizeof(char), 365);
  Serial.printf("[LOAD] najvisjadnevnaT.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/najnizjadnevnaT.bin", najnizjadnevnaT, sizeof(char), 365);
  Serial.printf("[LOAD] najnizjadnevnaT.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/najnizjadnevnasens1.bin", najnizjadnevnasens1, sizeof(char), 365);
  Serial.printf("[LOAD] najnizjadnevnasens1.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/najnizjadnevnasens2.bin", najnizjadnevnasens2, sizeof(char), 365);
  Serial.printf("[LOAD] najnizjadnevnasens2.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/najnizjadnevnasens3.bin", najnizjadnevnasens3, sizeof(char), 365);
  Serial.printf("[LOAD] najnizjadnevnasens3.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/najnizjadnevnasens4.bin", najnizjadnevnasens4, sizeof(char), 365);
  Serial.printf("[LOAD] najnizjadnevnasens4.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/povprecnadnevnaRH.bin", povprecnadnevnaRH, sizeof(char), 365);
  Serial.printf("[LOAD] povprecnadnevnaRH.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/najvisjadnevnaRH.bin", najvisjadnevnaRH, sizeof(char), 365);
  Serial.printf("[LOAD] najvisjadnevnaRH.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadArrayFromSPIFFS("/najnizjadnevnaRH.bin", najnizjadnevnaRH, sizeof(char), 365);
  Serial.printf("[LOAD] najnizjadnevnaRH.bin: %s\n", success ? "OK" : "FAILED");
  
  success &= loadIntFromSPIFFS("/indeksvzorca.bin", indeksvzorca);
  Serial.printf("[LOAD] indeksvzorca.bin: %s (value: %d)\n", success ? "OK" : "FAILED", indeksvzorca);
  
  success &= loadIntFromSPIFFS("/stPrikazanihVlagazemlje.bin", stPrikazanihVlagazemlje);
  Serial.printf("[LOAD] stPrikazanihVlagazemlje.bin: %s\n", success ? "OK" : "FAILED");
  
  loadFloatFromSPIFFS("/kalibracijamax0.bin", kalibracijamax[0]);
  loadFloatFromSPIFFS("/kalibracijamax1.bin", kalibracijamax[1]);
  loadFloatFromSPIFFS("/kalibracijamax2.bin", kalibracijamax[2]);
  loadFloatFromSPIFFS("/kalibracijamax3.bin", kalibracijamax[3]);
  loadFloatFromSPIFFS("/kalibracijamin0.bin", kalibracijamin[0]);
  loadFloatFromSPIFFS("/kalibracijamin1.bin", kalibracijamin[1]);
  loadFloatFromSPIFFS("/kalibracijamin2.bin", kalibracijamin[2]);
  loadFloatFromSPIFFS("/kalibracijamin3.bin", kalibracijamin[3]);
  
  // Load calibration for additional sensors (4-98) if they exist
  for (int i = 4; i < MAX_SOIL_SENSORS; i++) {
    String minPath = "/kalibracijamin" + String(i) + ".bin";
    String maxPath = "/kalibracijamax" + String(i) + ".bin";
    loadFloatFromSPIFFS(minPath.c_str(), kalibracijamin[i]);
    loadFloatFromSPIFFS(maxPath.c_str(), kalibracijamax[i]);
  }
  
  // Load selected sensors for logging (only if files exist, preserve defaults otherwise)
  Serial.println("[LOAD] Loading selected sensors configuration:");
  for (int i = 0; i < LOGGED_SENSORS_COUNT; i++) {
    String path = "/selectedSensor" + String(i) + ".bin";
    int tempValue;
    if (loadIntFromSPIFFS(path.c_str(), tempValue)) {
      // Validate range before updating (sensors are 1-based)
      if (tempValue >= 1 && tempValue <= MAX_SOIL_SENSORS) {
        selectedSensors[i] = tempValue;  // Only update if file exists and value is valid
        Serial.printf("  Data log %d: Loaded sensor %d (from file)\n", i+1, selectedSensors[i]);
      } else {
        Serial.printf("  Data log %d: Invalid value %d in file, keeping default %d\n", i+1, tempValue, selectedSensors[i]);
      }
    } else {
      Serial.printf("  Data log %d: No file found, using default %d\n", i+1, selectedSensors[i]);
    }
  }

  vzorec();

  for (int i = 0; i < 366; i++) {
    if (arraysinorder[i] == 1001) {
        dnevi = dnevivzorec[arraysinorder[i - 1]];
        mesec = mesecvzorec[arraysinorder[i - 1]];
        leto = letovzorec[arraysinorder[i - 1]];

      break;
    }
    if (i == 365) {
      dnevi = dnevivzorec[arraysinorder[i]];
      mesec = mesecvzorec[arraysinorder[i]];
      leto = letovzorec[arraysinorder[i]];

      break;
    }
  }
}


bool hasExistingData() {
  // Check if we already have meaningful data (not default 101 values)
  int dataPoints = 0;
  
  for (int i = 0; i < 365; i++) {
    // Check if temperature data exists for this day (any hour not equal to 101)
    for (int h = 0; h < 25; h++) {
      if (tempPodat[i][h] != 101) {
        dataPoints++;
        break; // Found data for this day, move to next day
      }
    }
  }
  
  Serial.printf("[CHECK] Found %d days with existing data\n", dataPoints);
  
  // Consider data exists if we have more than 10 days of data
  return (dataPoints > 10);
}

void populateDebugData() {
  Serial.println("Populating 182 days of data FORWARD (July 2025 -> Jan 2026)...");
  
  // Starting date: July 29, 2025 (approx 182 days before Jan 27, 2026)
  int d = 29;
  int m = 7;
  int y = 2025;

  for (int i = 0; i < 182; i++) {
    // 1. Assign Date to current index
    dnevivzorec[i] = d;
    mesecvzorec[i] = m;
    letovzorec[i] = y;

    // Temporary variables for daily stats
    int sumT = 0, maxT = -127, minT = 127;
    int sumRH = 0, maxRH = 0, minRH = 100;
    int minS1 = 100, minS2 = 100, minS3 = 100, minS4 = 100;

    // 2. Generate 24 hours of hourly data (0-23 range)
    for (int h = 0; h < 24; h++) {
      char tVal, rhVal;
      
      // Special debug data for the last day (i = 181)
      if (i == 181) {
        tVal = 40;  // Maximum temperature for all hours
        rhVal = 0;   // Minimum humidity for all hours
        Serial.printf("DEBUG: Day %d - Hour %d: Temp=%dC, RH=%d%% (DEBUG MODE)\n", i, h, tVal, rhVal);
      } else {
        tVal = (char)random(18, 32); 
        rhVal = (char)random(40, 80);
      }
      
      tempPodat[i][h] = tVal;
      RH[i][h] = rhVal;

      sumT += tVal; sumRH += rhVal;
      if (tVal > maxT) maxT = tVal; if (tVal < minT) minT = tVal;
      if (rhVal > maxRH) maxRH = rhVal; if (rhVal < minRH) minRH = rhVal;

      // Sensors 1-4: Range 0 to 99
      char s1 = (char)random(0, 100);
      char s2 = (char)random(0, 100);
      char s3 = (char)random(0, 100);
      char s4 = (char)random(0, 100);

      senzor1[i][h] = s1;
      senzor2[i][h] = s2;
      senzor3[i][h] = s3;
      senzor4[i][h] = s4;

      if (s1 < minS1) minS1 = s1;
      if (s2 < minS2) minS2 = s2;
      if (s3 < minS3) minS3 = s3;
      if (s4 < minS4) minS4 = s4;
    }
    
    // 3. Store calculated stats
    najvisjadnevnaT[i] = (char)maxT;
    najnizjadnevnaT[i] = (char)minT;
    povprecnadnevnaT[i] = (char)(sumT / 24);
    najvisjadnevnaRH[i] = (char)maxRH;
    najnizjadnevnaRH[i] = (char)minRH;
    povprecnadnevnaRH[i] = (char)(sumRH / 24);

    najnizjadnevnasens1[i] = (char)minS1;
    najnizjadnevnasens2[i] = (char)minS2;
    najnizjadnevnasens3[i] = (char)minS3;
    najnizjadnevnasens4[i] = (char)minS4;

    // 4. INCREMENT Date (Forward Logic)
    d++;
    int daysInMonth = 31;
    if (m == 4 || m == 6 || m == 9 || m == 11) daysInMonth = 30;
    else if (m == 2) daysInMonth = (y % 4 == 0) ? 29 : 28; // Leap year check

    if (d > daysInMonth) {
      d = 1;
      m++;
      if (m > 12) {
        m = 1;
        y++;
      }
    }
  }

  // 5. SECOND PASS: Set hour 24 values to next day's hour 0
  Serial.println("Setting hour 24 values to next day's hour 0...");
  for (int i = 0; i < 182; i++) {
    if (i < 181) {
      // Use next day's hour 0
      tempPodat[i][24] = tempPodat[i+1][0];
      RH[i][24] = RH[i+1][0];
      senzor1[i][24] = senzor1[i+1][0];
      senzor2[i][24] = senzor2[i+1][0];
      senzor3[i][24] = senzor3[i+1][0];
      senzor4[i][24] = senzor4[i+1][0];
      Serial.printf("Day %d hour 24 set to Day %d hour 0\n", i, i+1);
    } else {
      // Last day - use first day's hour 0 (wrap around for continuous display)
      tempPodat[i][24] = tempPodat[0][0];
      RH[i][24] = RH[0][0];
      senzor1[i][24] = senzor1[0][0];
      senzor2[i][24] = senzor2[0][0];
      senzor3[i][24] = senzor3[0][0];
      senzor4[i][24] = senzor4[0][0];
      Serial.printf("Day %d hour 24 set to Day 0 hour 0 (wrap around)\n", i);
    }
  }

  // 6. Update system index to reflect we have 182 days of data
  indeksvzorca = 182;
  
  Serial.println("Saving to FFat storage...");
  shranivse(); 
  Serial.println("Data populated correctly from 2025 into 2026.");
  Serial.println("DEBUG: Last day (index 181) has all hours at 40C temp and 0% humidity for testing");
  
  // DEBUG: Verify hour 24 data was set correctly
  Serial.println("DEBUG: Verifying hour 24 data:");
  for (int i = 0; i < 5; i++) {
    Serial.printf("Day %d: Hour 0=%.1fC, Hour 24=%.1fC\n", i, tempPodat[i][0], tempPodat[i][24]);
  }
  Serial.printf("Day 181: Hour 0=%.1fC, Hour 24=%.1fC\n", tempPodat[181][0], tempPodat[181][24]);
}


void smitt() {
  uint8_t katerafunkcija = indup1 + (4 * indup2);
  uint8_t tipikagrafika;

  tft.setTextSize(2);
  tft.setCursor(10, 5);
  tft.setTextColor(TFT_RED, TFT_SKYBLUE);
  if (napis[indup2][indup1] == "SENS0") {
    tipikagrafika = 0;
    tft.print("PROZILNIK NA");
    tft.setCursor(17, 23);
    tft.print("TEMPERATURO");
  }

  tft.setCursor(10, 5);
  if (napis[indup2][indup1] == "SENS1") {
    tipikagrafika = 1;
    tft.print("PROZILNIK NA");
    tft.setCursor(17, 23);
    tft.print("VLAGO ZRAKA");
  }
  if (napis[indup2][indup1] == "SENS2") {
    tipikagrafika = 2;
    tft.print("PROZILNIK NA");
    tft.setCursor(10, 23);
    tft.print("VLAGO ZEMLJE");
  }
  if (napis[indup2][indup1] == "SENS3") {
    tipikagrafika = 3;
    tft.print("PROZILNIK NA");
    tft.setCursor(15, 23);
    tft.print("CO2 V ZRAKU");
  }

  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setCursor(10, 57);

  tft.setTextColor(TFT_RED, TFT_SKYBLUE);
  tft.print("Prispevaj: ");

  tft.setCursor(76, 57);
  if (izbirnikvsmit == 1) {
    tft.setTextColor(TFT_RED, TFT_YELLOW);
    if (flagpotrjenosmit) {
      prispevaj[tipikagrafika][katerafunkcija] = !prispevaj[tipikagrafika][katerafunkcija];
      flagpotrjenosmit = 0;
    }
  }

  if (prispevaj[tipikagrafika][katerafunkcija] == 1) {

    tft.print(" DA");
    if (izhodsmit[tipikagrafika][katerafunkcija] == -2) { izhodsmit[tipikagrafika][katerafunkcija] = -3; }
    if (izhodsmit[tipikagrafika][katerafunkcija] == 101) { izhodsmit[tipikagrafika][katerafunkcija] = 100; }
  }


  if (prispevaj[tipikagrafika][katerafunkcija] == 0) {

    tft.print(" NE");
    if (izhodsmit[tipikagrafika][katerafunkcija] == -3) { izhodsmit[tipikagrafika][katerafunkcija] = -2; }
    if (izhodsmit[tipikagrafika][katerafunkcija] == 100) { izhodsmit[tipikagrafika][katerafunkcija] = 101; }
  }

  tft.setTextColor(TFT_RED, TFT_SKYBLUE);

  if (tipikagrafika == 2) {

    tft.setCursor(10, 42);
    tft.print("Izbran senzor: ");
    if (izbirnikvsmit == 0) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    tft.print(izbransenzor[katerafunkcija]);
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);

    if (flagpovecujsmit && izbirnikvsmit == 0) {
      izbransenzor[katerafunkcija]++;
      flagpovecujsmit = 0;
    }
    if (flagzmanjsujsmit && izbirnikvsmit == 0) {
      izbransenzor[katerafunkcija]--;
      flagzmanjsujsmit = 0;
    }
  }

  tft.setCursor(10, 73);
  tft.print("IZKLJUCI NA: ");
  if (napis[indup2][indup1] == "SENS0") {
    if (flagpovecujsmit && izbirnikvsmit == 2) {
      schmitttemp_h[katerafunkcija]--;
      flagpovecujsmit = 0;
    }
    if (flagzmanjsujsmit && izbirnikvsmit == 2) {
      schmitttemp_h[katerafunkcija]++;
      flagzmanjsujsmit = 0;
    }
    if (izbirnikvsmit == 2) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    tft.print(schmitttemp_h[katerafunkcija]);
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(" °C");
  }
  if (napis[indup2][indup1] == "SENS1") {
    if (flagpovecujsmit && izbirnikvsmit == 2) {
      schmittRH_h[katerafunkcija]--;
      flagpovecujsmit = 0;
    }
    if (flagzmanjsujsmit && izbirnikvsmit == 2) {
      schmittRH_h[katerafunkcija]++;
      flagzmanjsujsmit = 0;
    }
    if (izbirnikvsmit == 2) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    tft.print(schmittRH_h[katerafunkcija]);
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(" RH");
  }
  if (napis[indup2][indup1] == "SENS2") {
    if (flagpovecujsmit && izbirnikvsmit == 2) {
      schmittzem_h[katerafunkcija]--;
      flagpovecujsmit = 0;
    }
    if (flagzmanjsujsmit && izbirnikvsmit == 2) {
      schmittzem_h[katerafunkcija]++;
      flagzmanjsujsmit = 0;
    }
    if (izbirnikvsmit == 2) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    tft.print(schmittzem_h[katerafunkcija]);
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(" %");
  }
  if (napis[indup2][indup1] == "SENS3") {
    if (flagpovecujsmit && izbirnikvsmit == 2) {
      schmittco2_h[katerafunkcija]--;
      flagpovecujsmit = 0;
    }
    if (flagzmanjsujsmit && izbirnikvsmit == 2) {
      schmittco2_h[katerafunkcija]++;
      flagzmanjsujsmit = 0;
    }
    if (izbirnikvsmit == 2) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    tft.print(schmittco2_h[katerafunkcija]);
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(" PPM");
  }
  tft.setCursor(10, 73);

  /*  tft.print("VKLJUCI PO: ");   
    if (napis[indup2][indup1]== "SENS0"){  if (flagpovecujsmit && izbirnikvsmit==1){schmitttemp_h_T[0][katerafunkcija]--;schmitttemp_h_T[1][katerafunkcija]--;flagpovecujsmit = 0;} if (flagzmanjsujsmit && izbirnikvsmit==1){schmitttemp_h_T[0][katerafunkcija]++;flagzmanjsujsmit = 0;}   if(izbirnikvsmit == 1){tft.setTextColor(TFT_RED, TFT_YELLOW);}   tft.print(schmitttemp_h_T[0][katerafunkcija]);  tft.setTextColor(TFT_RED, TFT_SKYBLUE); tft.print(" sek");}
    if (napis[indup2][indup1]== "SENS1"){  if (flagpovecujsmit  && izbirnikvsmit==1){schmittRH_h_T[0][katerafunkcija]--;schmittRH_h_T[1][katerafunkcija]--;flagpovecujsmit = 0;} if (flagzmanjsujsmit && izbirnikvsmit==1){schmittRH_h_T[0][katerafunkcija]++;flagzmanjsujsmit = 0;}    if(izbirnikvsmit == 1){tft.setTextColor(TFT_RED, TFT_YELLOW);}   tft.print(schmittRH_h_T[0][katerafunkcija]);  tft.setTextColor(TFT_RED, TFT_SKYBLUE); tft.print(" sek");}
    if (napis[indup2][indup1]== "SENS2"){  if (flagpovecujsmit && izbirnikvsmit==1){schmittzem_h_T[0][katerafunkcija]--;schmittzem_h_T[1][katerafunkcija]--;flagpovecujsmit = 0;} if (flagzmanjsujsmit && izbirnikvsmit==1){schmittzem_h_T[0][katerafunkcija]++;flagzmanjsujsmit = 0;}   if(izbirnikvsmit == 1){tft.setTextColor(TFT_RED, TFT_YELLOW);}   tft.print(schmittzem_h_T[0][katerafunkcija]);  tft.setTextColor(TFT_RED, TFT_SKYBLUE); tft.print(" sek");}
    if (napis[indup2][indup1]== "SENS3"){  if (flagpovecujsmit && izbirnikvsmit==1){schmittco2_h_T[0][katerafunkcija]--;schmittco2_h_T[1][katerafunkcija]--;flagpovecujsmit = 0;} if (flagzmanjsujsmit && izbirnikvsmit==1){schmittco2_h_T[0][katerafunkcija]++;flagzmanjsujsmit = 0;}   if(izbirnikvsmit == 1){tft.setTextColor(TFT_RED, TFT_YELLOW);}   tft.print(schmittco2_h_T[0][katerafunkcija]);  tft.setTextColor(TFT_RED, TFT_SKYBLUE); tft.print(" sek");}
   */
  tft.setCursor(10, 90);
  tft.print("VKLJUCI NA: ");
  if (napis[indup2][indup1] == "SENS0") {
    if (flagpovecujsmit && izbirnikvsmit == 3) {
      schmitttemp_l[katerafunkcija]--;
      flagpovecujsmit = 0;
    }
    if (flagzmanjsujsmit && izbirnikvsmit == 3) {
      schmitttemp_l[katerafunkcija]++;
      flagzmanjsujsmit = 0;
    }
    if (izbirnikvsmit == 3) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    tft.print(schmitttemp_l[katerafunkcija]);
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(" °C");
  }
  if (napis[indup2][indup1] == "SENS1") {
    if (flagpovecujsmit && izbirnikvsmit == 3) {
      schmittRH_l[katerafunkcija]--;
      flagpovecujsmit = 0;
    }
    if (flagzmanjsujsmit && izbirnikvsmit == 3) {
      schmittRH_l[katerafunkcija]++;
      flagzmanjsujsmit = 0;
    }
    if (izbirnikvsmit == 3) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    tft.print(schmittRH_l[katerafunkcija]);
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(" RH");
  }
  if (napis[indup2][indup1] == "SENS2") {
    if (flagpovecujsmit && izbirnikvsmit == 3) {
      schmittzem_l[katerafunkcija]--;
      flagpovecujsmit = 0;
    }
    if (flagzmanjsujsmit && izbirnikvsmit == 3) {
      schmittzem_l[katerafunkcija]++;
      flagzmanjsujsmit = 0;
    }
    if (izbirnikvsmit == 3) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    tft.print(schmittzem_l[katerafunkcija]);
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(" %");
  }
  if (napis[indup2][indup1] == "SENS3") {
    if (flagpovecujsmit && izbirnikvsmit == 3) {
      schmittco2_l[katerafunkcija]--;
      flagpovecujsmit = 0;
    }
    if (flagzmanjsujsmit && izbirnikvsmit == 3) {
      schmittco2_l[katerafunkcija]++;
      flagzmanjsujsmit = 0;
    }
    if (izbirnikvsmit == 3) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    tft.print(schmittco2_l[katerafunkcija]);
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(" PPM");
  }
  tft.setCursor(10, 108);


  //flagpovecuj flagzmanjsuj smit prozita tipki v plus ali minus, če prištevamo minute prištevamo po 60 sekund, minute se nato izračunavajo glede na št sekund.
  tft.print("IZKLJUCI PO: ");


  if (napis[indup2][indup1] == "SENS0") {
    if (flagpovecujsmit && izbirnikvsmit == 4) {

      if ((schmitttemp_l_T[0][katerafunkcija] / 60) == 1000) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, preostanek stevil, ko se mestno stevilo zmanjsa
      if ((schmitttemp_l_T[0][katerafunkcija] / 60) == 100) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }
      schmitttemp_l_T[0][katerafunkcija] -= 60;
      schmitttemp_l_T[1][katerafunkcija] = schmitttemp_l_T[0][katerafunkcija];
      flagpovecujsmit = 0;
    }

    if (flagzmanjsujsmit && izbirnikvsmit == 4) {
      if ((schmitttemp_l_T[0][katerafunkcija] - ((schmitttemp_l_T[0][katerafunkcija] / 60) * 60)) > 15 && (schmitttemp_l_T[0][katerafunkcija] / 60) == 1091) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, po owerflow v plus maks je 1092 min in 15 sek, ce imamo predhodno nastavljeno na 16 ali vec sek, je owerflov ze na 1091 + 60 sek
      if ((schmitttemp_l_T[0][katerafunkcija] / 60) == 1092) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }                                                                                                  //pocistimo smeti na displeju, po owerflow v plus
      schmitttemp_l_T[0][katerafunkcija] += 60;
      schmitttemp_l_T[1][katerafunkcija] = schmitttemp_l_T[0][katerafunkcija];
      flagzmanjsujsmit = 0;
    }

    if (flagpovecujsmit && izbirnikvsmit == 5) {
      if ((schmitttemp_l_T[0][katerafunkcija] / 60) == 1000 && (schmitttemp_l_T[0][katerafunkcija] - ((schmitttemp_l_T[0][katerafunkcija] / 60) * 60)) == 0) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, preostanek stevil, ko se mestno stevilo zmanjsa
      if ((schmitttemp_l_T[0][katerafunkcija] / 60) == 100 && (schmitttemp_l_T[0][katerafunkcija] - ((schmitttemp_l_T[0][katerafunkcija] / 60) * 60)) == 0) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }

      schmitttemp_l_T[0][katerafunkcija] -= 1;
      schmitttemp_l_T[1][katerafunkcija] = schmitttemp_l_T[0][katerafunkcija];
      flagpovecujsmit = 0;
    }

    if (flagzmanjsujsmit && izbirnikvsmit == 5) {
      if ((schmitttemp_l_T[0][katerafunkcija] / 60) == 1092 && (schmitttemp_l_T[0][katerafunkcija] - ((schmitttemp_l_T[0][katerafunkcija] / 60) * 60)) == 15) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, po owerflow v plus
      schmitttemp_l_T[0][katerafunkcija] += 1;
      schmitttemp_l_T[1][katerafunkcija] = schmitttemp_l_T[0][katerafunkcija];
      flagzmanjsujsmit = 0;
    }

    if (izbirnikvsmit == 4) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    if ((schmitttemp_l_T[0][katerafunkcija] / 60) < 10) { tft.print("0"); }
    tft.print(schmitttemp_l_T[0][katerafunkcija] / 60);
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(":");

    if (izbirnikvsmit == 5) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    if ((schmitttemp_l_T[0][katerafunkcija] - ((schmitttemp_l_T[0][katerafunkcija] / 60) * 60)) < 10) { tft.print("0"); }
    tft.print(schmitttemp_l_T[0][katerafunkcija] - ((schmitttemp_l_T[0][katerafunkcija] / 60) * 60));  //izluscimo preostale sekunde od minut
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(" ");
  }


  if (napis[indup2][indup1] == "SENS1") {
    if (flagpovecujsmit && izbirnikvsmit == 4) {
      if ((schmittRH_l_T[0][katerafunkcija] / 60) == 1000) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, preostanek stevil, ko se mestno stevilo zmanjsa
      if ((schmittRH_l_T[0][katerafunkcija] / 60) == 100) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }
      schmittRH_l_T[0][katerafunkcija] -= 60;
      schmittRH_l_T[1][katerafunkcija] = schmittRH_l_T[0][katerafunkcija];
      flagpovecujsmit = 0;
    }

    if (flagzmanjsujsmit && izbirnikvsmit == 4) {
      if ((schmittRH_l_T[0][katerafunkcija] - ((schmittRH_l_T[0][katerafunkcija] / 60) * 60)) > 15 && (schmittRH_l_T[0][katerafunkcija] / 60) == 1091) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, po owerflow v plus maks je 1092 min in 15 sek, ce imamo predhodno nastavljeno na 16 ali vec sek, je owerflov ze na 1091 + 60 sek
      if ((schmittRH_l_T[0][katerafunkcija] / 60) == 1092) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }                                                                                              //pocistimo smeti na displeju, po owerflow v plus
      schmittRH_l_T[0][katerafunkcija] += 60;
      schmittRH_l_T[1][katerafunkcija] = schmittRH_l_T[0][katerafunkcija];
      flagzmanjsujsmit = 0;
    }


    if (flagpovecujsmit && izbirnikvsmit == 5) {
      if ((schmittRH_l_T[0][katerafunkcija] / 60) == 1000 && (schmittRH_l_T[0][katerafunkcija] - ((schmittRH_l_T[0][katerafunkcija] / 60) * 60)) == 0) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, preostanek stevil, ko se mestno stevilo zmanjsa
      if ((schmittRH_l_T[0][katerafunkcija] / 60) == 100 && (schmittRH_l_T[0][katerafunkcija] - ((schmittRH_l_T[0][katerafunkcija] / 60) * 60)) == 0) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }
      schmittRH_l_T[0][katerafunkcija] -= 1;
      schmittRH_l_T[1][katerafunkcija] = schmittRH_l_T[0][katerafunkcija];
      flagpovecujsmit = 0;
    }

    if (flagzmanjsujsmit && izbirnikvsmit == 5) {
      if ((schmittRH_l_T[0][katerafunkcija] / 60) == 1092 && (schmittRH_l_T[0][katerafunkcija] - ((schmittRH_l_T[0][katerafunkcija] / 60) * 60)) == 15) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, po owerflow v plus
      schmittRH_l_T[0][katerafunkcija] += 1;
      schmittRH_l_T[1][katerafunkcija] = schmittRH_l_T[0][katerafunkcija];
      flagzmanjsujsmit = 0;
    }


    if (izbirnikvsmit == 4) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    if ((schmittRH_l_T[0][katerafunkcija] / 60) < 10) { tft.print("0"); }
    tft.print(schmittRH_l_T[0][katerafunkcija] / 60);
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(":");

    if (izbirnikvsmit == 5) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    if ((schmittRH_l_T[0][katerafunkcija] - ((schmittRH_l_T[0][katerafunkcija] / 60) * 60)) < 10) { tft.print("0"); }
    tft.print(schmittRH_l_T[0][katerafunkcija] - ((schmittRH_l_T[0][katerafunkcija] / 60) * 60));  //izluscimo preostale sekunde od minut
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(" ");
  }

  if (napis[indup2][indup1] == "SENS2") {


    if (flagpovecujsmit && izbirnikvsmit == 4) {
      if ((schmittzem_l_T[0][katerafunkcija] / 60) == 1000) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, preostanek stevil, ko se mestno stevilo zmanjsa
      if ((schmittzem_l_T[0][katerafunkcija] / 60) == 100) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }
      schmittzem_l_T[0][katerafunkcija] -= 60;
      schmittzem_l_T[1][katerafunkcija] = schmittzem_l_T[0][katerafunkcija];
      flagpovecujsmit = 0;
    }

    if (flagzmanjsujsmit && izbirnikvsmit == 4) {
      if ((schmittzem_l_T[0][katerafunkcija] - ((schmittzem_l_T[0][katerafunkcija] / 60) * 60)) > 15 && (schmittzem_l_T[0][katerafunkcija] / 60) == 1091) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, po owerflow v plus maks je 1092 min in 15 sek, ce imamo predhodno nastavljeno na 16 ali vec sek, je owerflov ze na 1091 + 60 sek
      if ((schmittzem_l_T[0][katerafunkcija] / 60) == 1092) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }                                                                                                //pocistimo smeti na displeju, po owerflow v plus
      schmittzem_l_T[0][katerafunkcija] += 60;
      schmittzem_l_T[1][katerafunkcija] = schmittzem_l_T[0][katerafunkcija];
      flagzmanjsujsmit = 0;
    }



    if (flagpovecujsmit && izbirnikvsmit == 5) {
      if ((schmittzem_l_T[0][katerafunkcija] / 60) == 1000 && (schmittzem_l_T[0][katerafunkcija] - ((schmittzem_l_T[0][katerafunkcija] / 60) * 60)) == 0) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, preostanek stevil, ko se mestno stevilo zmanjsa
      if ((schmittzem_l_T[0][katerafunkcija] / 60) == 100 && (schmittzem_l_T[0][katerafunkcija] - ((schmittzem_l_T[0][katerafunkcija] / 60) * 60)) == 0) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }
      schmittzem_l_T[0][katerafunkcija] -= 1;
      schmittzem_l_T[1][katerafunkcija] = schmittzem_l_T[0][katerafunkcija];
      flagpovecujsmit = 0;
    }

    if (flagzmanjsujsmit && izbirnikvsmit == 5) {
      if ((schmittzem_l_T[0][katerafunkcija] / 60) == 1092 && (schmittzem_l_T[0][katerafunkcija] - ((schmittzem_l_T[0][katerafunkcija] / 60) * 60)) == 15) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, po owerflow v plus
      schmittzem_l_T[0][katerafunkcija] += 1;
      schmittzem_l_T[1][katerafunkcija] = schmittzem_l_T[0][katerafunkcija];
      flagzmanjsujsmit = 0;
    }


    if (izbirnikvsmit == 4) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    if ((schmittzem_l_T[0][katerafunkcija] / 60) < 10) { tft.print("0"); }
    tft.print(schmittzem_l_T[0][katerafunkcija] / 60);
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(":");

    if (izbirnikvsmit == 5) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    if ((schmittzem_l_T[0][katerafunkcija] - ((schmittzem_l_T[0][katerafunkcija] / 60) * 60)) < 10) { tft.print("0"); }
    tft.print(schmittzem_l_T[0][katerafunkcija] - ((schmittzem_l_T[0][katerafunkcija] / 60) * 60));  //izluscimo preostale sekunde od minut
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(" ");
  }


  if (napis[indup2][indup1] == "SENS3") {
    if (flagpovecujsmit && izbirnikvsmit == 4) {
      if ((schmittco2_l_T[0][katerafunkcija] / 60) == 1000) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, preostanek stevil, ko se mestno stevilo zmanjsa
      if ((schmittco2_l_T[0][katerafunkcija] / 60) == 100) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }
      schmittco2_l_T[0][katerafunkcija] -= 60;
      schmittco2_l_T[1][katerafunkcija] = schmittco2_l_T[0][katerafunkcija];
      flagpovecujsmit = 0;
    }

    if (flagzmanjsujsmit && izbirnikvsmit == 4) {
      if ((schmittco2_l_T[0][katerafunkcija] - ((schmittco2_l_T[0][katerafunkcija] / 60) * 60)) > 15 && (schmittco2_l_T[0][katerafunkcija] / 60) == 1091) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, po owerflow v plus maks je 1092 min in 15 sek, ce imamo predhodno nastavljeno na 16 ali vec sek, je owerflov ze na 1091 + 60 sek
      if ((schmittco2_l_T[0][katerafunkcija] / 60) == 1092) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }                                                                                                //pocistimo smeti na displeju, po owerflow v plus
      schmittco2_l_T[0][katerafunkcija] += 60;
      schmittco2_l_T[1][katerafunkcija] = schmittco2_l_T[0][katerafunkcija];
      flagzmanjsujsmit = 0;
    }



    if (flagpovecujsmit && izbirnikvsmit == 5) {
      if ((schmittco2_l_T[0][katerafunkcija] / 60) == 1000 && (schmittco2_l_T[0][katerafunkcija] - ((schmittco2_l_T[0][katerafunkcija] / 60) * 60)) == 0) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, preostanek stevil, ko se mestno stevilo zmanjsa
      if ((schmittco2_l_T[0][katerafunkcija] / 60) == 100 && (schmittco2_l_T[0][katerafunkcija] - ((schmittco2_l_T[0][katerafunkcija] / 60) * 60)) == 0) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }
      schmittco2_l_T[0][katerafunkcija] -= 1;
      schmittco2_l_T[1][katerafunkcija] = schmittco2_l_T[0][katerafunkcija];
      flagpovecujsmit = 0;
    }

    if (flagzmanjsujsmit && izbirnikvsmit == 5) {
      if ((schmittco2_l_T[0][katerafunkcija] / 60) == 1092 && (schmittco2_l_T[0][katerafunkcija] - ((schmittco2_l_T[0][katerafunkcija] / 60) * 60)) == 15) { tft.pushImage(0, 0, 160, 128, trigozadje[0]); }  //pocistimo smeti na displeju, po owerflow v plus
      schmittco2_l_T[0][katerafunkcija] += 1;
      schmittco2_l_T[1][katerafunkcija] = schmittco2_l_T[0][katerafunkcija];
      flagzmanjsujsmit = 0;
    }



    if (izbirnikvsmit == 4) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    if ((schmittco2_l_T[0][katerafunkcija] / 60) < 10) { tft.print("0"); }
    tft.print(schmittco2_l_T[0][katerafunkcija] / 60);
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(":");

    if (izbirnikvsmit == 5) { tft.setTextColor(TFT_RED, TFT_YELLOW); }
    if ((schmittco2_l_T[0][katerafunkcija] - ((schmittco2_l_T[0][katerafunkcija] / 60) * 60)) < 10) { tft.print("0"); }
    tft.print(schmittco2_l_T[0][katerafunkcija] - ((schmittco2_l_T[0][katerafunkcija] / 60) * 60));  //izluscimo preostale sekunde od minut
    tft.setTextColor(TFT_RED, TFT_SKYBLUE);
    tft.print(" ");
  }

  tft.setTextFont(1);

  //na L vklapljam, na H Izklapljam, v primeru gretja
  //IDEA ko se izhodu stanje spremeni, to je, ko je bil pogoj za to izpolnjen, ampak ker funkcijo že imam se lahko orientiram po izhodu, tedaj lahko pričnemo tudi odštevati, in ga po tem vržemo v prvotno stanje.
  //Uporabno za kakšno črpalko ki gleda samo na to
  //da se vklopi ko zmeri suho zemljo, in izklopi po času, ne meritvi, ker meritev po tem več ni natančna, saj voda počasi namoči zemljo in se vlaga porazdeli kasneje, nazaćetku pa bo netočna v območju merilnika,
  //določit je treba, kaj se zgodi če je pogoj za odštevanje še vedno izpolnjen ampak je čas že odštel, kot tudi kaj se zgodi ko se druga meja za izklop izpolni, ali izklopimo neglede na timer, ali morata biti oba veljavna?
}


void racunajsmit() {

  unsigned char* schmitt_l[21];
  unsigned char* schmitt_h[21];
  char* merjeno;
  uint16_t* schmit_T[2][21];
  uint8_t katerafunkcija = indup1 + (4 * indup2);

  for (int indeksmit = 0; indeksmit < 20; indeksmit++) {
    for (int bruh = 0; bruh < 4; bruh++) {

      switch (bruh) {
        case 0:
          for (int i = 0; i < 20; i++) {
            schmitt_l[i] = &schmitttemp_l[i];
            schmitt_h[i] = &schmitttemp_h[i];
            merjeno = &temperatura;
            schmit_T[1][i] = &schmitttemp_l_T[1][i];
            schmit_T[0][i] = &schmitttemp_l_T[0][i];
          }
          break;

        case 1:
          for (int i = 0; i < 20; i++) {
            schmitt_l[i] = &schmittRH_l[i];
            schmitt_h[i] = &schmittRH_h[i];
            merjeno = &vlaznost;
            schmit_T[1][i] = &schmittRH_l_T[1][i];
            schmit_T[0][i] = &schmittRH_l_T[0][i];
          }
          break;

        case 2:
          for (int i = 0; i < 20; i++) {
            schmitt_l[i] = &schmittzem_l[i];
            schmitt_h[i] = &schmittzem_h[i];


            merjeno = &vlagazemlje[izbransenzor[indeksmit]];
            schmit_T[1][i] = &schmittzem_l_T[1][i];
            schmit_T[0][i] = &schmittzem_l_T[0][i];
          }
          break;

        case 3:
          for (int i = 0; i < 20; i++) {
            schmitt_l[i] = &schmittco2_l[i];
            schmitt_h[i] = &schmittco2_h[i];
            merjeno = &temperatura;
            schmit_T[1][i] = &schmittco2_l_T[1][i];
            schmit_T[0][i] = &schmittco2_l_T[0][i];
          }
          break;
      }

      if (ticktajmer) {                                                                                                   // ticktajmer se postavlja v timerju ki prozi vsako sekundo
        if ((izhodsmit[bruh][indeksmit] == 101 || izhodsmit[bruh][indeksmit] == 100) && (*schmit_T[1][indeksmit]) > 0) {  // ce je izhod vkljucen, in ce ima uporabnik nastavljen casovni izklop in ce ta ni ze pretekel,
          //testniflag++;
          //if (katerafunkcija != indeksmit) { schmitttemp_l_T[1][indeksmit]--; }  // cas odstevamo le ce ga uporabnik trenutno ne nastavlja
          (*schmit_T[1][indeksmit])--;
          if ((*schmit_T[1][indeksmit]) == 0) {


            izhodsmit[bruh][indeksmit] = -3;




          }  // po pretecenem casu, ko je ta nula, izklpimo izhod, v to funkcijo se ne bomo več vrnili, ker v višjem ifu preverjamo če čas ni nula, v njem se odšteva in pride na nulo v tem ifu.
        }
      }

      //  if (schmitttemp_h[indeksmit] < schmitttemp_l[indeksmit]) { schmitttemp_h[indeksmit] = schmitttemp_l[indeksmit] - 1; }  // temperatura kjer izklapljamo gretje mora zmeraj bit višja, kot kjer gretje vklapljamo
      // ce zelimo funkcijo hlajenja namesto gretja, naj bo temperatura kjer izkapljamo hlajenje preprosto nizja kot temperature kjer vkaplamo

      //će je željena kjer vklapljamo nižja od kjer izklapljamo, primer gretje, zalivanje,...kjer dodajamo
      if ((*schmitt_l[indeksmit]) < (*schmitt_h[indeksmit])) {

        if ((*schmitt_h[indeksmit]) < (*merjeno)) {  // ko je temperatura višja od želene
                                                     //Gretje, izklapljamo ko temperatura naraste nad željeno

          (*schmit_T[1][indeksmit]) = (*schmit_T[0][indeksmit]);  //ko izklopimo, ponastavimo timer, da lahko znova odštevamo, pri vklopu




          izhodsmit[bruh][indeksmit] = -3;
        }

        if ((*schmitt_l[indeksmit]) > (*merjeno)) {  // ko je temperatura manjša od želene

          if ((*schmit_T[1][indeksmit]) == (*schmit_T[0][indeksmit])) {  //će še nismo odštevali, da ne vklopimo izhoda ko smo že odšteli


            izhodsmit[bruh][indeksmit] = 100;
          }
        }
      }

      //će je željena kjer vklapljamo višja od kjer izklapljamo, primer hlajenje
      if ((*schmitt_l[indeksmit]) > (*schmitt_h[indeksmit])) {

        if ((*schmitt_l[indeksmit]) < (*merjeno)) {  // ko je temperatura višja od želene
                                                     //Gretje, izklapljamo ko temperatura naraste nad željeno

          if ((*schmit_T[1][indeksmit]) == (*schmit_T[0][indeksmit])) {



            izhodsmit[bruh][indeksmit] = 100;
          }
        }

        if ((*schmitt_h[indeksmit]) > (*merjeno)) {  // ko je temperatura manjša od želene
          (*schmit_T[1][indeksmit]) = (*schmit_T[0][indeksmit]);


          izhodsmit[bruh][indeksmit] = -3;
        }
      }
      if (izhodsmit[bruh][indeksmit] == 100 && prispevaj[bruh][indeksmit] == 0) { izhodsmit[bruh][indeksmit] = 101; }  // neznana napaka,  BUG izhod se postavi na 100 če je želena pod dejansko kljub prispevaj 0. samo 101 mora.
      if (izhodsmit[bruh][indeksmit] == -3 && prispevaj[bruh][indeksmit] == 0) { izhodsmit[bruh][indeksmit] = -2; }

      if (prispevaj[bruh][indeksmit] != 0) {
        testniflag = 10;
        testniflag2 = bruh;
        testniflag3 = indeksmit;
      }

      if (ticktajmer && indeksmit == 19 && bruh == 3) { ticktajmer = 0; }
    }
  }
}

void tipke() {
  //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- TIPKA POTRDITEV
  if (digitalRead(TIPKAPOTRDI_PIN) == LOW) {

    if (!flagzaklenivstop1) {
      flagzaklenivstop1 = 1;
      delay(60);
      if (digitalRead(TIPKAPOTRDI_PIN) == LOW) {
        bool flagzaklenipodvstop = 0;  //za namen preprečitve dvojne potrditve
        flagnenajdemvnosa = 0;


        if (flagurnik && !pomikajY) {
          flipizborurnik = 1;
          kolikocasapritisnjena2 = 0;
          urnikizrisan = 0;
        }


        if (urnikizbranakockaY > 6 && flagurnik) {
          casovnirazpored[urnikizbranakockaY - 7][urnikizbranakockaX] = napolnilza;
        }
        if (urnikizbranakockaY < 7 && flagurnik) {
          casovnirazpored[urnikizbranakockaY][urnikizbranakockaX] = napolnilza;
        }

        if (flagurnik) {
          pomikajY = 0;
          urnikizrisan = 0;
          kolikocasapritisnjena2 = 0;
        }

        if (flagizbirnikizbrano) {
          if (napis[indup2][indup1] == "URNIK") {
            flagurnik = 1;
            flagizbirnikizbrano = 0;
            urnikizrisan = 0;

            drawUrnikMainBackground();
          }
        }





        if (flagsmit) {
          flagpotrjenosmit = 1;
        }

        if (flaggrafmoci) {
          flagvstopizbirnik = 1;
        }
        if (flagizbirnikizbrano) {
          if (napis[indup2][indup1] == "SENS0" || napis[indup2][indup1] == "SENS1" || napis[indup2][indup1] == "SENS2" || napis[indup2][indup1] == "SENS3") {
            flagsmit = 1;
            flagizbirnikizbrano = 0;
            tft.pushImage(0, 0, 160, 128, trigozadje[0]);
          }
          if (napis[indup2][indup1] == "GRAF0" || napis[indup2][indup1] == "GRAF1" || napis[indup2][indup1] == "GRAF2") {
            flaggrafmoci = 1;
            grafmocizrisan = 0;
            powerGraphStaticDrawn = 0;
            flagizbirnikizbrano = 0;
          }
        }

        if (flagizbirnikcfg && !flaggrafmoci && !flagsmit && !flagurnik) {
          flagizbirnikizbrano = 1;
        }

        if (flagnastavitve == 1) {
          if (izbirniknastavitve == 1) {
            shranivse();
          }
          if (izbirniknastavitve == 2) {
            nalozivse();
          }
          if (izbirniknastavitve == 3) {
            flagnastavicas++;
            if (flagnastavicas > 6) { flagnastavicas = 0; }
            nastavitve();
          }
          if (izbirniknastavitve == 4) {
            dnevi = bufferdnevi;
            mesec = buffermesec;
            leto = bufferleto;
            minuta = bufferminuta;
            sekunde = buffersekunde;
            ure = bufferure;
            setRTCTime(bufferleto, buffermesec, bufferdnevi, bufferure, bufferminuta, buffersekunde);
            nastavitve();
            tft.setTextColor(TFT_GREEN);
            tft.setCursor(90, 32);
            tft.print("OK!");
            tft.setTextColor(TFT_WHITE);
            brisinovejsevnose();
            vzorec();
          }

          if (izbirniknastavitve == 5) {
            katerisenzor++;
            if (katerisenzor > discovered_nodes) { katerisenzor = 1; }
            nastavitve();
          }

          if (izbirniknastavitve == 6) {
            // Calibrate MIN value for selected sensor
            if (katerisenzor >= 1 && katerisenzor <= discovered_nodes && katerisenzor <= MAX_SOIL_SENSORS) {
              int sensorIndex = katerisenzor - 1;  // Convert to 0-based index
              kalibracijamin[sensorIndex] = readSTM32Channel(sensorIndex);
              tft.setCursor(90, 48);
              tft.print("OK! ");
              tft.print(kalibracijamin[sensorIndex]);
              Serial.print("Calibrated MIN for sensor ");
              Serial.print(katerisenzor);
              Serial.print(": ");
              Serial.println(kalibracijamin[sensorIndex]);
              
              String filename = "/kalibracijamin" + String(sensorIndex) + ".bin";
              saveFloatToSPIFFS(filename.c_str(), kalibracijamin[sensorIndex]);
            }
            meritev();
          }

          if (izbirniknastavitve == 7) {
            // Calibrate MAX value for selected sensor
            if (katerisenzor >= 1 && katerisenzor <= discovered_nodes && katerisenzor <= MAX_SOIL_SENSORS) {
              int sensorIndex = katerisenzor - 1;  // Convert to 0-based index
              kalibracijamax[sensorIndex] = readSTM32Channel(sensorIndex);
              tft.setCursor(90, 56);
              tft.print("OK! ");
              tft.print(kalibracijamax[sensorIndex]);
              Serial.print("Calibrated MAX for sensor ");
              Serial.print(katerisenzor);
              Serial.print(": ");
              Serial.println(kalibracijamax[sensorIndex]);
              
              String filename = "/kalibracijamax" + String(sensorIndex) + ".bin";
              saveFloatToSPIFFS(filename.c_str(), kalibracijamax[sensorIndex]);
            }
            meritev();
          }

          if (izbirniknastavitve == 8) {
            izbrisivnos("/RH.bin");
            izbrisivnos("/tempPodat.bin");
            izbrisivnos("/senzor1.bin");
            izbrisivnos("/senzor2.bin");
            izbrisivnos("/senzor3.bin");
            izbrisivnos("/senzor4.bin");
            izbrisivnos("/dnevivzorec.bin");
            izbrisivnos("/mesecvzorec.bin");
            izbrisivnos("/letovzorec.bin");
            izbrisivnos("/povprecnadnevnaT.bin");
            izbrisivnos("/najvisjadnevnaT.bin");
            izbrisivnos("/najnizjadnevnaT.bin");
            izbrisivnos("/najnizjadnevnasens1.bin");
            izbrisivnos("/najnizjadnevnasens2.bin");
            izbrisivnos("/najnizjadnevnasens3.bin");
            izbrisivnos("/najnizjadnevnasens4.bin");
            izbrisivnos("/povprecnadnevnaRH.bin");
            izbrisivnos("/najvisjadnevnaRH.bin");
            izbrisivnos("/najnizjadnevnaRH.bin");
            izbrisivnos("/indeksvzorca.bin");
            izbrisivnos("/stPrikazanihVlagazemlje.bin");
            izbrisizacasne();
            nastavitve();
          }
          
          if (izbirniknastavitve == 9) {
            // Toggle number of displayed soil moisture sensors (1-4)
            stPrikazanihVlagazemlje++;
            if (stPrikazanihVlagazemlje > 4) {
              stPrikazanihVlagazemlje = 1;
            }
            nastavitve();
          }
          
          if (izbirniknastavitve == 10) {
            // Just refresh display when selecting sensor log menu
            nastavitve();
          }
          
          // Sensor selection positions (11-14) - toggle editing mode on potrdi
          if (izbirniknastavitve >= 11 && izbirniknastavitve <= 14) {
            flagSensorEditing = !flagSensorEditing;  // Toggle sensor editing mode
            // DEBUG: Print editing mode change
            int sensorIndex = izbirniknastavitve - 11;
            Serial.printf("[SETTINGS] Data log %d: Editing mode %s\n", sensorIndex + 1, 
                         flagSensorEditing ? "ENTERED" : "EXITED");
            nastavitve();
          }
        }

        if (flagizbrancfg) { flagizbirnikcfg = 1; }
        if (flagupravljajmoc && !flaggrafmoci && !flagsmit && !flagurnik) {
          flagizbranpt = 1;
          if (izbirnikmoci == 5) {
            flagizbrancfg = 1;
            tft.pushImage(0, 0, 160, 128, pinoutcfg[0]);
          }
        }

        if (!flagmenugraf && !flagglavnimenu && !flaggrafmoci && !flagupravljajmoc && !flagnastavitve && !flagsmit && !flagurnik) {
          tft.setTextSize(1);
          izrismreze();
          izrismenugraf();
          osvezigraf();
          flagzaklenipodvstop = 1;
        }

        if (flagmenuglavnimenu && !flagzaklenipodvstop) {  // za vstop v dnevnigraf

          if (flagikonagraf) {
            flagmenugraf = 0;
            flagikonagraf = 0;
            flagdnevnigraf = 1;
            flagmesecnigraf = 0;
            flagglavnimenu = 0;
            flagmenuglavnimenu = 0;
            flaginfo = 1;
            casovnik4 = millis();
            prikazanindeks = indeksvzorca;
            zelenoleto = leto;
            zelenmesec = mesec;
            zelendan = dnevi;
            flagnarisan = 0;
            tft.setTextSize(1);
            izrismreze();
            osvezigraf();
          }

          if (flagikonacfg) {
            flagikonagraf = 0;
            flagdnevnigraf = 0;
            flagmesecnigraf = 0;
            flagglavnimenu = 0;
            flagmenuglavnimenu = 0;
            flaginfo = 0;
            flaggrafmoci = 0;
            flagsmit = 0;
            flagurnik = 0;
            grafmocizrisan = 0;
            flagupravljajmoc = 1;
            flagizrismoc= 1;
            flagmenugraf = 0;
            tft.setTextSize(1);
            tft.fillScreen(TFT_BLACK);
          }

          if (flagikonanast) {
            flagikonagraf = 0;
            flagdnevnigraf = 0;
            flagmesecnigraf = 0;
            flagglavnimenu = 0;
            flagmenuglavnimenu = 0;
            flaginfo = 0;
            flaggrafmoci = 0;
            flagsmit = 0;
            flagurnik = 0;
            grafmocizrisan = 0;
            flagupravljajmoc = 0;
            flagmenugraf = 0;
            flagnastavitve = 1;
            buffermesec = mesec;
            bufferleto = leto;
            bufferdnevi = dnevi;
            bufferminuta = minuta;
            buffersekunde = sekunde;
            bufferure = ure;
            nastavitve();
          }
          flagzaklenipodvstop = 1;
        }
        if (flagglavnimenu) {
          flagmenuglavnimenu = 1;
        }
        if (flagmenugraf && (flagmesecnigraf || flagdnevnigraf) && !flagzaklenipodvstop) {
          flagzaklenipodvstop = 1;
          switch (navimenugraf) {
            case 0:
              if (flagmesecnigraf) { flagmenuMDt = !flagmenuMDt; }
              if (flagdnevnigraf) { flagmenuTemp = !flagmenuTemp; }
              break;
            case 10:
              if (flagmesecnigraf) { flagmenuPDt = !flagmenuPDt; }
              if (flagdnevnigraf) { flagmenuRH = !flagmenuRH; }
              break;
            case 20:
              if (flagmesecnigraf) { flagmenuMiDt = !flagmenuMiDt; }
              if (flagdnevnigraf) { flagmenuSNZ1 = !flagmenuSNZ1; }
              break;
            case 30:
              if (flagmesecnigraf) { flagmenuMRH = !flagmenuMRH; }
              if (flagdnevnigraf) { flagmenuSNZ2 = !flagmenuSNZ2; }
              break;
            case 40:
              if (flagmesecnigraf) { flagmenuPDRH = !flagmenuPDRH; }
              if (flagdnevnigraf) { flagmenuSNZ3 = !flagmenuSNZ3; }
              break;
            case 50:
              if (flagmesecnigraf) { flagmenuMiRH = !flagmenuMiRH; }
              if (flagdnevnigraf) { flagmenuSNZ4 = !flagmenuSNZ4; }
              break;
            case 60:
              if (flagmesecnigraf) { flagmenusens1 = !flagmenusens1; }
              if (flagdnevnigraf) {
                for (int i = 0; i < 365; i++) {
                  if (dnevivzorec[i] == zelendan && mesecvzorec[i] == zelenmesec && letovzorec[i] == zelenoleto) {
                    prikazanindeks = i;
                    flagmenugraf = 0;
                    x = 0;
                    x2 = 0;
                    x3 = 0;
                    x4 = 0;
                    x8 = 0;
                    izrismreze();
                    osvezigraf();
                    flaginfo = 1;
                    casovnik4 = millis();
                    break;
                  }
                }
                if (flagmenugraf == 1) {
                  flagnenajdemvnosa = 1;
                }
              }
              break;
            case 70:
              if (flagmesecnigraf) { flagmenusens2 = !flagmenusens2; }
              if (flagdnevnigraf) { izbirnikdan = !izbirnikdan; }
              break;
            case 80:
              if (flagmesecnigraf) { flagmenusens3 = !flagmenusens3; }
              if (flagdnevnigraf) { izbirnikmesec = !izbirnikmesec; }
              break;
            case 90:
              if (flagmesecnigraf) { flagmenusens4 = !flagmenusens4; }
              if (flagdnevnigraf) { izbirnikleto = !izbirnikleto; }
              break;

            case 100:
            
              if (flagmesecnigraf) {
                obdobje++;
                x5 = x5 + 28;
                x6 = x6 + 4;
                x7 = x7 + 31;

                if (obdobje == 6) {
                  obdobje = 0;
                  x5 = 28;
                  x6 = 4;
                  x7 = 31;
                }
              }
              break;
            case 110:

                  if (flagdnevnigraf) {
                // Switch to monthly graph
                flagdnevnigraf = 0;
                flagmesecnigraf = 1;
                flagnarisan = 0;
                navimenugraf = 0;
              }



              flagnivecjega = 0;
              for (int i = 0; i < 365; i++) {
                if (letovzorec[i] > prikazanoleto2) {
                  prikazanoleto2 = letovzorec[i];
                  flagnivecjega = 0;
                  break;
                }
                flagnivecjega = 1;
              }
              if (flagnivecjega) {
                for (int i = 0; i < 365; i++) {
                  if (letovzorec[i] < prikazanoleto2 && letovzorec[i] != 101) {
                    prikazanoleto2 = letovzorec[i];
                  }
                }
              }
              break;
            /*case 120:
              zeljenmesec = mesec;
              flagdnevnigraf = !flagdnevnigraf;
              flagmesecnigraf = !flagdnevnigraf;
              prikazanoleto2 = letovzorec[indeksvzorca];
              flaginfo = 1;
              break;*/
            default:
              break;
          }
          if (flagmenugraf == 1) {
            izrismreze();
            izrismenugraf();
            osvezigraf();
          }
        }
      }
    }
  }

  if (digitalRead(TIPKAPOTRDI_PIN) == HIGH) {
    flipizborurnik = 0;
    flagzaklenivstop1 = 0;
    kolikocasapritisnjena = 0;
    if (casovnirazpored[urnikizbranakockaY - 7][urnikizbranakockaX] == 100) { kolikocasapritisnjena2 = 7; }
    if (casovnirazpored[urnikizbranakockaY][urnikizbranakockaX] == 100) { kolikocasapritisnjena2 = 7; }
  }
  
  //********************************************************************************************************************************************************************************************************************************
  if (digitalRead(TIPKANAZAJ_PIN) == LOW) {
    if (!flagzaklenivstop4) {
      flagzaklenivstop4 = 1;
      delay(60);
      if (digitalRead(TIPKANAZAJ_PIN) == LOW) {
        flagnenajdemvnosa = 0;



            kolikocasapritisnjena3 = 0;
;
           kolikocasapritisnjena = 0;



        if (!flagizbranpt && flagupravljajmoc && !flagizbirnikcfg && !flagizbrancfg) {
          flagupravljajmoc = 0;
        }

        if (!flaggrafmoci && flagizbrancfg && !flagizbirnikcfg && !flagsmit && !flagurnik) {
          tft.fillScreen(TFT_BLACK);
          flagizbrancfg = 0;
        }


        if (!flaggrafmoci && flagizbirnikcfg && !flagsmit && !flagurnik) {
          flagizbirnikcfg = 0;
        }

        if (flagizbirnikizbrano) {
          flagizbirnikizbrano = 0;
        }

        if (flagizbranpt) {
          flagizbranpt = 0;
        }

        if (flaggrafmoci && flagvstopizbirnik) {
          flagvstopizbirnik = 0;
          grafmocizrisan = 0;
        } else if ((flaggrafmoci || flagsmit || flagurnik) && !flagvstopizbirnik && !flagpotrjenosmit && pomikajY) {
          flaggrafmoci = 0;
          powerGraphStaticDrawn = 0;
          flagsmit = 0;
          flagurnik = 0;
          tft.pushImage(0, 0, animation_width, animation_height, pinoutcfg[0]);
        }
        if (flagurnik) {
          pomikajY = 1;
          urnikizrisan = 0;
        
        }

        flagpotrjenosmit = 0;

        if (flagvstopizbirnik && !flaggrafmoci) {
          flagvstopizbirnik = 0;
        }

        if (flagglavnimenu && flagmenuglavnimenu && !flaggrafmoci && !flagsmit && !flagurnik) {
          flagmenuglavnimenu = 0;
          flagizrisanglavnipodmeni = 0;
          tft.fillScreen(TFT_BLACK);
        }

        if (!flagmenugraf && (!flagmesecnigraf || !flagdnevnigraf) && !flaggrafmoci && !flagupravljajmoc && flagnastavicas == 0 && !flagsmit && !flagurnik) {
          // If in sensor editing mode, exit editing mode first
          if (flagSensorEditing) {
            flagSensorEditing = false;
            Serial.println("[SETTINGS] Back button: Exited sensor editing mode");
            nastavitve();
          } else {
            // Otherwise exit the whole settings menu
            Serial.println("[SETTINGS] Back button: Exiting settings menu");
            flagnastavitve = 0;
            flagglavnimenu = 1;
            flagdnevnigraf = 0;
            flagmesecnigraf = 0;
            flagmenuglavnimenu = 0;
            flagizrisanglavnipodmeni = 0;
            flagglavnimeniizrisan = 0;
            tft.fillScreen(TFT_BLACK);
          }
        }

        if (!flagglavnimenu && flagmenugraf && (flagmesecnigraf || flagdnevnigraf) && !flaggrafmoci && !flagupravljajmoc && !izbirnikleto && !izbirnikmesec && !izbirnikdan && !flagsmit && !flagurnik) {
          flagmenugraf = 0;
          x = 0;
          x2 = 0;
          x3 = 0;
          x4 = 0;
          x8 = 0;
          izrismreze();
          osvezigraf();
        }
        if (izbirnikleto || izbirnikmesec || izbirnikdan) {
          izbirnikleto = 0;
          izbirnikmesec = 0;
          izbirnikdan = 0;
        }

        if (flagnastavicas > 0) {
          flagnastavicas = 0;
          nastavitve();
        }
      }
    }
  }

  if (digitalRead(TIPKANAZAJ_PIN) == HIGH) {
    flagzaklenivstop4 = 0;
  }

  //**************************************************************************************************************************************************************************************************************************************************************
  if (digitalRead(TIPKADOL_PIN) == LOW || flagEncoderCounterClockwise) {
    if (!flagzaklenivstop2) {
      flagzaklenivstop2 = 1;
      delay(30);
      if (digitalRead(TIPKADOL_PIN) == LOW || flagEncoderCounterClockwise) {
        if (flagEncoderCounterClockwise) {
          flagEncoderCounterClockwise = false;  // Reset flag immediately after processing
        }
        flagnenajdemvnosa = 0;
     kolikocasapritisnjena = 0;
     kolikocasapritisnjena3 = 0;

        if (flagurnik && !pomikajY) {
          if (urnikizbranakockaX == 0) { urnikizbranakockaX = 48; }

          urnikizbranakockaX--;
          urnikizrisan = 0;
          
        }

        if (flagurnik && pomikajY) {

          urnikizrisan = 0;
          urnikizbranakockaY++;

          if (urnikizbranakockaY > 13) { urnikizbranakockaY = 0; }
        }

        if (flagsmit) {

          if (!flagpotrjenosmit) {
            izbirnikvsmit++;
            if (izbirnikvsmit > 5) { izbirnikvsmit = 0; }
          }

          if (flagpotrjenosmit) {

            flagpovecujsmit = 1;
          }
        }

        if (flagizbirnikizbrano) {
          gain[indup2][indup1] -= 0.01;
          flagzaklenivstop2 = 0;
        }

        if (flagnastavitve) {

          if (flagnastavicas == 1) {
            bufferdnevi--;
            if (bufferdnevi == 0) { bufferdnevi = 31; }
          }
          if (flagnastavicas == 2) {
            buffermesec--;
            if (buffermesec == 0) { buffermesec = 12; }
          }
          if (flagnastavicas == 3) {
            bufferleto--;
            if (bufferleto == 0) { bufferleto = 9999; }
          }
          if (flagnastavicas == 4) {

            if (bufferure == 0) { bufferure = 24; }
            bufferure--;
          }
          if (flagnastavicas == 5) {

            if (bufferminuta == 0) { bufferminuta = 60; }
            bufferminuta--;
          }
          if (flagnastavicas == 6) {

            if (buffersekunde == 0) { buffersekunde = 60; }
            buffersekunde--;
          }

          // Sensor selection handling for positions 11-14 (decrease sensor number) - ONLY when editing
          if (flagSensorEditing && flagnastavicas == 0 && izbirniknastavitve >= 11 && izbirniknastavitve <= 14) {
            int sensorIndex = izbirniknastavitve - 11;  // 0-3 for the 4 sensor slots
            selectedSensors[sensorIndex]--;
            if (selectedSensors[sensorIndex] < 1) {  // Changed from 0 to 1
              selectedSensors[sensorIndex] = discovered_nodes;
            }
            // DEBUG: Print sensor selection change
            Serial.printf("[SETTINGS] Data log %d: Changed to sensor %d\n", sensorIndex + 1, selectedSensors[sensorIndex]);
          }

          if (flagnastavicas == 0 && !flagSensorEditing) {
            if (izbirniknastavitve < 14) {
              izbirniknastavitve++;
            } else {
              izbirniknastavitve = 0;
            }
            // Exit editing mode if navigating away from sensor lines
            if (izbirniknastavitve < 11 || izbirniknastavitve > 14) {
              flagSensorEditing = false;
            }
          }
          nastavitve();
        }

        if (flagizbirnikcfg && !flaggrafmoci && !flagizbirnikizbrano && !flagsmit && !flagurnik) {

          for (unsigned char indk = 0; napis[indup2][indup1] != tekstnapis[indk]; indk++) {  //
            incfg = indk + 1;
          }


          if (incfg > 0) {
            incfg--;
            if ((indup2 == 4 || indup2 == 3) && incfg == 5) { incfg = 1; }

            napis[indup2][indup1] = tekstnapis[incfg];
            tft.pushImage(0, 0, 160, 128, pinoutcfg[0]);
          }
        }


        if (flagizbrancfg && !flagizbirnikcfg && !flaggrafmoci && !flagizbirnikizbrano && !flagsmit && !flagurnik) {

          if (indup1 == 0) {
            if (indup2 > 0) {
              indup2--;
              indup1 = 4;
            }
          }


          indup1--;
          if (indup1 == 255) { indup1 = 0; }
        }


        if (flagupravljajmoc && !flagizbrancfg && !flaggrafmoci && !flagsmit && !flagurnik) {
          if (flagizbranpt) {
            switch (izbirnikmoci) {
              case 0:
                if (pt1 > 0) {
                  pt1--;
                  flagzaklenivstop2 = 0;
                }
                break;
              case 1:
                if (pt2 > 0) {
                  pt2--;
                  flagzaklenivstop2 = 0;
                }
                break;
              case 2:
                if (pt3 > 0) {
                  pt3--;
                  flagzaklenivstop2 = 0;
                }
                break;
              case 3:
                if (pt4 > 0) {
                  pt4 -= 100;
                  flagzaklenivstop2 = 0;
                }
                break;
              case 4:
                if (pt5 > 0) {
                  pt5 -= 100;
                  flagzaklenivstop2 = 0;
                }
                break;
            }
          }
          if (!flagizbranpt) {

            if (izbirnikmoci == 0) { izbirnikmoci == 5; }
            if (izbirnikmoci > 0) { izbirnikmoci--; }
            if (izbirnikmoci == 4) { tft.fillRect(112, 77, 40, 40, TFT_BLACK); }
          }
        }
        unsigned char pristej;
        if (napis[indup2][indup1] == "GRAF0") { pristej = 0; }
        if (napis[indup2][indup1] == "GRAF1") { pristej = 5; }
        if (napis[indup2][indup1] == "GRAF2") { pristej = 10; }
        if (flaggrafmoci) {
          if (!flagvstopizbirnik) {
            if (izbirnik > 0) {
              int oldIzbirnik = izbirnik;
              izbirnik--;
              redrawPowerGraphPointArea(pristej, oldIzbirnik, izbirnik);
              redrawPowerGraphPointArea(pristej, izbirnik, izbirnik);
            }
          } else if (linijamoci[indup2 + pristej][izbirnik] > 0) {
            linijamoci[indup2 + pristej][izbirnik] -= 2;
            ;
            redrawPowerGraphPointArea(pristej, izbirnik, izbirnik);
            flagzaklenivstop2 = 0;
          }
        }

        if (flagmenuglavnimenu && flagglavnimenu) {

          if (indeksglavnipodmeni == 0) { indeksglavnipodmeni = 3; }
          indeksglavnipodmeni--;
          flagizrisanglavnipodmeni = 0;
        }

        switch (indeksglavnipodmeni) {
          case 0:
            {
              flagikonanast = 0;
              flagikonagraf = 1;
              flagikonacfg = 0;
            }
            break;
          case 1:
            {
              flagikonanast = 0;
              flagikonagraf = 0;
              flagikonacfg = 1;
            }
            break;
          case 2:
            {
              flagikonanast = 1;
              flagikonagraf = 0;
              flagikonacfg = 0;
            }
            break;
          default:
            break;
        }

        if (!izbirnikleto && !izbirnikmesec && !izbirnikdan) {
          if (flagmenugraf && navimenugraf < 120) {
            navimenugraf = navimenugraf + 10;
            if (navimenugraf == 120) { navimenugraf = 0; }
            if (flagdnevnigraf && navimenugraf > 90) { navimenugraf = 110; }
            izrismenugraf();
          } else if (flagmenugraf) {
            navimenugraf = 0;
            izrismenugraf();
          }
        }

        if (izbirnikleto) {
          if (zelenoleto == 1) { zelenoleto = 60001; }
          zelenoleto--;
          izrismenugraf();
        }
        if (izbirnikmesec) {
          if (zelenmesec == 1) { zelenmesec = 13; }
          zelenmesec--;
          izrismenugraf();
        }
        if (izbirnikdan) {
          if (zelendan == 1) { zelendan = 32; }
          zelendan--;
          izrismenugraf();
        }


        if (!flagmenugraf && !flagglavnimenu && flagdnevnigraf) {
          if (prikazanindeks > 0) {
            prikazanindeks--;
            if (prikazanindeks < 0) prikazanindeks = 0;
            izrismreze();
            osvezigraf();
            flaginfo = 1;
            casovnik4 = millis();
          }
        }
        if (!flagmenugraf && !flagglavnimenu && flagmesecnigraf) {
          if (zeljenmesec > 1) {

            zeljenmesec--;
            izrismreze();
            osvezigraf();
          }
          flaginfo = 1;
          casovnik4 = millis();
        }
      }
    }
  }


  if (digitalRead(TIPKADOL_PIN) == HIGH) {
    flagzaklenivstop2 = 0;
    flagpovecujsmit = 0;
  }
//********************************************************************************************************************************************************************************************************************************
  if (digitalRead(TIPKAGOR_PIN) == LOW || flagEncoderClockwise) {
    if (!flagzaklenivstop3) {
      flagzaklenivstop3 = 1;
      delay(30);
      if (digitalRead(TIPKAGOR_PIN) == LOW || flagEncoderClockwise) {
        if (flagEncoderClockwise) {
          flagEncoderClockwise = false;  // Reset flag immediately after processing
        }
          Serial.print("navimenu");
 Serial.println(navimenugraf);
        flagnenajdemvnosa = 0;
         kolikocasapritisnjena = 0;
          kolikocasapritisnjena3 = 0;
        if (flagurnik && !pomikajY) {

          urnikizbranakockaX++;
          urnikizrisan = 0;
          if (urnikizbranakockaX > 47) { urnikizbranakockaX = 0;}
        }
        if (flagurnik && pomikajY) {
          if (urnikizbranakockaY == 0) { urnikizbranakockaY = 14; }

          urnikizbranakockaY--;
          urnikizrisan = 0;
        }

        if (flagsmit) {
          if (izbirnikvsmit == 0 && !flagpotrjenosmit) { izbirnikvsmit = 6; }
          if (!flagpotrjenosmit) {
            izbirnikvsmit--;
          }




          if (flagpotrjenosmit) { flagzmanjsujsmit = 1; }
        }


        if (flagizbirnikizbrano) {
          gain[indup2][indup1] += 0.01;
          flagzaklenivstop3 = 0;
        }


        if (flagnastavitve) {

          if (flagnastavicas == 1) {
            bufferdnevi++;
            if (bufferdnevi > 31) { bufferdnevi = 1; }
          }
          if (flagnastavicas == 2) {
            buffermesec++;
            if (buffermesec > 12) { buffermesec = 1; }
          }
          if (flagnastavicas == 3) {
            bufferleto++;
            if (bufferleto > 9999) { bufferleto = 1; }
          }
          if (flagnastavicas == 4) {
            bufferure++;
            if (bufferure > 23) { bufferure = 0; }
          }
          if (flagnastavicas == 5) {
            bufferminuta++;
            if (bufferminuta > 59) { bufferminuta = 0; }
          }
          if (flagnastavicas == 6) {
            buffersekunde++;
            if (buffersekunde > 59) { buffersekunde = 0; }
          }

          // Sensor selection handling for positions 11-14 (increase sensor number) - ONLY when editing
          if (flagSensorEditing && flagnastavicas == 0 && izbirniknastavitve >= 11 && izbirniknastavitve <= 14) {
            int sensorIndex = izbirniknastavitve - 11;  // 0-3 for the 4 sensor slots
            selectedSensors[sensorIndex]++;
            if (selectedSensors[sensorIndex] > discovered_nodes) {
              selectedSensors[sensorIndex] = 1;
            }
            // DEBUG: Print sensor selection change
            Serial.printf("[SETTINGS] Data log %d: Changed to sensor %d\n", sensorIndex + 1, selectedSensors[sensorIndex]);
          }

          if (flagnastavicas == 0 && !flagSensorEditing) {
            if (izbirniknastavitve > 0) {
              izbirniknastavitve--;
            } else {
              izbirniknastavitve = 14;
            }
            // Exit editing mode if navigating away from sensor lines
            if (izbirniknastavitve < 11 || izbirniknastavitve > 14) {
              flagSensorEditing = false;
            }
          }
          nastavitve();
        }
        if (flagizbirnikcfg && !flaggrafmoci && !flagizbirnikizbrano && !flagsmit && !flagurnik) {
          for (unsigned char indk = 0; napis[indup2][indup1] != tekstnapis[indk]; indk++) {
            incfg = indk + 1;
          }

          if (incfg < 9) {
            incfg++;

            if ((indup2 == 4 || indup2 == 3) && incfg == 2) { incfg = 6; }

            napis[indup2][indup1] = tekstnapis[incfg];
            tft.pushImage(0, 0, 160, 128, pinoutcfg[0]);
          }
        }

        if (flagizbrancfg && !flagizbirnikcfg && !flaggrafmoci && !flagizbirnikizbrano) {
          indup1++;
          if (indup1 == 4) {
            indup2++;
            indup1 = 0;
          }
          if (indup2 == 5) { indup2 = 4; }
        }

        if (flagupravljajmoc && !flagizbirnikizbrano) {
          if (flagizbranpt && !flagizbrancfg && !flaggrafmoci && !flagsmit && !flagurnik) {
            switch (izbirnikmoci) {
              case 0:
                if (pt1 < 100) {
                  pt1++;
                  flagzaklenivstop3 = 0;
                }
                break;
              case 1:
                if (pt2 < 100) {
                  pt2++;
                  flagzaklenivstop3 = 0;
                }
                break;
              case 2:
                if (pt3 < 100) {
                  pt3++;
                  flagzaklenivstop3 = 0;
                }
                break;
              case 3:
                if (pt4 < 100) {
                  pt4 += 100;
                  flagzaklenivstop3 = 0;
                }
                break;
              case 4:
                if (pt5 < 100) {
                  pt5 += 100;
                  flagzaklenivstop3 = 0;
                }
                break;
            }
          }
          if (!flagizbranpt) {
            if (izbirnikmoci < 5) { izbirnikmoci++; }
          }
        }
        unsigned char pristej;
        if (napis[indup2][indup1] == "GRAF0") { pristej = 0; }
        if (napis[indup2][indup1] == "GRAF1") { pristej = 5; }
        if (napis[indup2][indup1] == "GRAF2") { pristej = 10; }
        if (flaggrafmoci) {
          if (!flagvstopizbirnik) {
            if (izbirnik < 9) {
              int oldIzbirnik = izbirnik;
              izbirnik++;
              redrawPowerGraphPointArea(pristej, oldIzbirnik, izbirnik);
              redrawPowerGraphPointArea(pristej, izbirnik, izbirnik);
            }
          } else if (linijamoci[indup2 + pristej][izbirnik] < 99) {
            linijamoci[indup2 + pristej][izbirnik] += 2;
            ;
            redrawPowerGraphPointArea(pristej, izbirnik, izbirnik);
            flagzaklenivstop3 = 0;
          }
        }

        if (flagmenuglavnimenu && flagglavnimenu) {
          indeksglavnipodmeni++;
          if (indeksglavnipodmeni == 3) { indeksglavnipodmeni = 0; }

          flagizrisanglavnipodmeni = 0;
        }

        switch (indeksglavnipodmeni) {
          case 0:
            {
              flagikonanast = 0;
              flagikonagraf = 1;
              flagikonacfg = 0;
            }
            break;
          case 1:
            {
              flagikonanast = 0;
              flagikonagraf = 0;
              flagikonacfg = 1;
            }
            break;
          case 2:
            {
              flagikonanast = 1;
              flagikonagraf = 0;
              flagikonacfg = 0;
            }
            break;
          default:
            break;
        }

        if (!izbirnikleto && !izbirnikmesec && !izbirnikdan) {
           if (flagdnevnigraf && navimenugraf == 110) { navimenugraf = 100; }
          if (flagmenugraf && navimenugraf > 0) {
            navimenugraf = navimenugraf - 10;
           
            izrismenugraf();
          } else if (flagmenugraf) {
            navimenugraf = 110;
            izrismenugraf();
          }
        }

        if (izbirnikleto) {
          zelenoleto++;
          if (zelenoleto == 6000) { zelenoleto = 1; }
          izrismenugraf();
        }
        if (izbirnikmesec) {
          zelenmesec++;
          if (zelenmesec == 13) { zelenmesec = 1; }
          izrismenugraf();
        }
        if (izbirnikdan) {
          zelendan++;
          if (zelendan == 32) { zelendan = 1; }
          izrismenugraf();
        }

        if (!flagmenugraf && !flagglavnimenu && flagdnevnigraf) {
          if (prikazanindeks < 365 && prikazanindeks < indeksvzorca) {
            prikazanindeks++;
            // Ensure prikazanindeks stays within bounds
            if (prikazanindeks >= 365) prikazanindeks = 364;
            if (prikazanindeks > indeksvzorca) prikazanindeks = indeksvzorca;
            izrismreze();
            osvezigraf();
            flaginfo = 1;
            casovnik4 = millis();
          }
        }

        if (!flagmenugraf && !flagglavnimenu && flagmesecnigraf) {
          if (zeljenmesec < 12) {
            zeljenmesec++;
            izrismreze();
            osvezigraf();
          }
          flaginfo = 1;
          casovnik4 = millis();
        }
      }
    }
  }

  if (digitalRead(TIPKAGOR_PIN) == HIGH) {
    flagzaklenivstop3 = 0;
  }
}


void izrismreze() {

  if (!flaggrafmoci && !flagsmit && !flagurnik) {
    powerGraphStaticDrawn = 0;
    TJpgDec.drawFsJpg(0, 0, "/mreza.jpg", FFat);
  }
}

void racunaj() {
  // Validate pointers before use
  if (!isValidPointer2D(pointerracunaj) || !isValidPointer1D(pointerracunaj2) || 
      !isValidPointer1D(pointerracunaj3) || !isValidPointer1D(pointerracunaj4)) {
    return; // Exit if any pointer is invalid
  }

  for (int i2 = 0; i2 < 365; ++i2) {
    // Validate array bounds for 2D access
    if (!isValidArrayIndex2D(i2, 23, 365, 25)) {
      continue; // Skip if indices are out of bounds
    }

    if ((*pointerracunaj)[i2][23] != 101 && (*pointerracunaj2)[i2] == 101) {
      (*pointerracunaj2)[i2] = 0;
      for (int i = 0; i < 24; ++i) {
        if (!isValidArrayIndex2D(i2, i, 365, 25)) {
          continue; // Skip if indices are out of bounds
        }
        if ((*pointerracunaj)[i2][i] != 101) {
          if ((*pointerracunaj2)[i2] < (*pointerracunaj)[i2][i]) {
            (*pointerracunaj2)[i2] = (*pointerracunaj)[i2][i];
          }
        }
      }
    }

    if ((*pointerracunaj)[i2][23] != 101 && (*pointerracunaj3)[i2] == 101) {  //če je zadnja temperatura v dnevu zabeležena ter če še nismo izračunavali najnizje dnevne za ta dan
      for (int i = 0; i < 24; ++i) {
        if (!isValidArrayIndex2D(i2, i, 365, 25)) {
          continue; // Skip if indices are out of bounds
        }
        if ((*pointerracunaj3)[i2] > (*pointerracunaj)[i2][i]) {
          (*pointerracunaj3)[i2] = (*pointerracunaj)[i2][i];
        }
      }
    }

    if ((*pointerracunaj)[i2][23] != 101 && (*pointerracunaj4)[i2] == 101) {  //če je zadnja temperatura v dnevu zabeležena ter če še nismo izračunavali povp dnevne za ta dan
      int vmesna;
      unsigned char nultidnevi;
      vmesna = 0;
      nultidnevi = 0; // Initialize to prevent undefined behavior
      for (int i = 0; i < 24; ++i) {
        if (!isValidArrayIndex2D(i2, i, 365, 25)) {
          continue; // Skip if indices are out of bounds
        }
        if ((*pointerracunaj)[i2][i] != 101) {
          vmesna += (*pointerracunaj)[i2][i];
        } else {
          nultidnevi++;
        }
      }
      if (nultidnevi == 0) {
        vmesna = vmesna / 24;
      } else {
        if (24 - nultidnevi > 0) { // Prevent division by zero
          vmesna = vmesna / (24 - nultidnevi);
        } else {
          vmesna = 0; // Default value if all entries are 101
        }
      }

      (*pointerracunaj4)[i2] = vmesna;
    }
  }
}

void drawGraph() {
  if (!flagglavnimenu) {
    if (!flagnarisan) {
      if (flagmesecnigraf == 0 && flagdnevnigraf == 1) {


        for (int i = x; i < numDataPoints; i++) {
          // Validate pointer and array bounds before access
          if (!isValidPointer2D(pointergraf) || !isValidArrayIndex(prikazanindeks, 365)) {
            break; // Exit if pointer is invalid or index out of bounds
          }
          
          // Validate bounds for array access
          if (!isValidArrayIndex2D(prikazanindeks, i, 365, 25) || 
              !isValidArrayIndex2D(prikazanindeks, i + 1, 365, 25)) {
            continue; // Skip if indices are out of bounds
          }
          
          if ((*pointergraf)[prikazanindeks][i] != 101 && (*pointergraf)[prikazanindeks][i + 1] != 101) {  //ce so v prikazanem datumu sploh podatki, pointergraf določa le kater graf želimo izrisovati, temperaturo, vlago....
            int x1 = map(i, 0, 24, 10, 310);
            int y1 = map((*pointergraf)[prikazanindeks][i], temperatureMin, temperatureMax, 225, 15); //255 je 0, 15 je 100 procentov na grafu.
            int x2 = map(i + 1, 0, 24, 10, 310);
            int y2 = map((*pointergraf)[prikazanindeks][i + 1], temperatureMin, temperatureMax, 225, 15); //255 je 0, 15 je 100 procentov na grafu.

            int cx0 = x1 + (x2 - x1) / 3;
            int cy0 = y1;
            int cx1 = x1 + 2 * (x2 - x1) / 3;
            int cy1 = y2;

            for (float t = 0.0; t <= 1.0; t += 0.01) {
              // Feed watchdog every 10 iterations to prevent timeout during sparse data processing
            
              
              int x = int(bezierPoint(x1, cx0, cx1, x2, t));
              int y = int(bezierPoint(y1, cy0, cy1, y2, t));
              if (y1 != y2) {
                if (pointergraf == &tempPodat) { tft.drawPixel(x, y, TFT_RED); }
                if (pointergraf == &RH) { tft.drawPixel(x, y, TFT_BLUE); }
                if (pointergraf == &senzor1) { tft.drawPixel(x, y, TFT_GREEN); }
                if (pointergraf == &senzor2) { tft.drawPixel(x, y, TFT_ORANGE); }
                if (pointergraf == &senzor3) { tft.drawPixel(x, y, TFT_VIOLET); }
                if (pointergraf == &senzor4) { tft.drawPixel(x, y, TFT_BROWN); }
              } else {
                if (pointergraf == &tempPodat) { tft.drawLine(x1, y1, x2, y2, TFT_RED); }
                if (pointergraf == &RH) { tft.drawLine(x1, y1, x2, y2, TFT_BLUE); }
                if (pointergraf == &senzor1) { tft.drawLine(x1, y1, x2, y2, TFT_GREEN); }
                if (pointergraf == &senzor2) { tft.drawLine(x1, y1, x2, y2, TFT_ORANGE); }
                if (pointergraf == &senzor3) { tft.drawLine(x1, y1, x2, y2, TFT_VIOLET); }
                if (pointergraf == &senzor4) { tft.drawLine(x1, y1, x2, y2, TFT_BROWN); }
              }
            }

          } else {
            i = numDataPoints;
          }
        }


        for (int i = x2; i <= 24; i += 2) { //izris ure
          int x = map(i, 0, 24, 15, 300);
          tft.setTextSize(1);
          tft.setTextColor(TFT_WHITE);
          tft.setCursor(x - 4, 225);
          tft.print(i);
        }

        for (int i = 0; i <= 90; i += 10) {
          int y = map(i, 0, 100, 230, 12);
          tft.setTextSize(1);
          tft.setTextColor(TFT_WHITE);
          tft.setCursor(305, y - 10);
          tft.print(i);
        }
        tft.setCursor(305, 3);
        tft.print("99");

        if (flagmenugraf == 0) {
          for (int i = 0; i <= 40; i += 10) {
            int y = map(i, 0, 40, 220, 12);
            tft.setTextSize(1);
            tft.setTextColor(TFT_WHITE);
            tft.setCursor(1, y - 10);
            tft.print(i);
          }
          tft.setCursor(14, 2);
          tft.print("C");
        }

        tft.setCursor(305, 220);
        tft.print('h');
      }

      if (flagmesecnigraf == 1 && flagdnevnigraf == 0) {


        tft.setCursor(60, 10);
        int zamikdnevimax = 0;
        int prikazanmesec[6];
        int zamikdnevi2 = 0;

        if (obdobje == 0) {
          prikazanmesec[0] = zeljenmesec;
          prikazanmesec[1] = zeljenmesec;
          prikazanmesec[2] = zeljenmesec;
          prikazanmesec[3] = zeljenmesec;
          prikazanmesec[4] = zeljenmesec;
          prikazanmesec[5] = zeljenmesec;

          zamikdnevimax = 0;
        }
        if (obdobje == 1) {

          prikazanmesec[0] = zeljenmesec;
          prikazanmesec[1] = zeljenmesec + 1;
          prikazanmesec[2] = zeljenmesec + 1;
          prikazanmesec[3] = zeljenmesec + 1;
          prikazanmesec[4] = zeljenmesec + 1;
          prikazanmesec[5] = zeljenmesec + 1;
          if (prikazanmesec[1] == 13) {
            prikazanmesec[1] = 1;
            prikazanmesec[2] = 1;
            prikazanmesec[3] = 1;
            prikazanmesec[4] = 1;
            prikazanmesec[5] = 1;
          }
          zamikdnevimax = 31;
        }
        if (obdobje == 2) {
          prikazanmesec[0] = zeljenmesec;
          prikazanmesec[1] = zeljenmesec + 1;
          prikazanmesec[2] = zeljenmesec + 2;
          prikazanmesec[3] = zeljenmesec + 2;
          prikazanmesec[4] = zeljenmesec + 2;
          prikazanmesec[5] = zeljenmesec + 2;

          zamikdnevimax = 62;

          if (prikazanmesec[1] == 13) {
            prikazanmesec[1] = 1;
            prikazanmesec[2] = 2;
            prikazanmesec[3] = 2;
            prikazanmesec[4] = 2;
            prikazanmesec[5] = 2;
          }
          if (prikazanmesec[2] == 13) {
            prikazanmesec[2] = 1;
            prikazanmesec[3] = 1;
            prikazanmesec[4] = 1;
            prikazanmesec[5] = 1;
          }
        }

        if (obdobje == 3) {
          prikazanmesec[0] = zeljenmesec;
          prikazanmesec[1] = zeljenmesec + 1;
          prikazanmesec[2] = zeljenmesec + 2;
          prikazanmesec[3] = zeljenmesec + 3;
          prikazanmesec[4] = zeljenmesec + 3;
          prikazanmesec[5] = zeljenmesec + 3;
          zamikdnevimax = 93;
          if (prikazanmesec[1] == 13) {
            prikazanmesec[1] = 1;
            prikazanmesec[2] = 2;
            prikazanmesec[3] = 3;
            prikazanmesec[4] = 3;
            prikazanmesec[5] = 3;
          }
          if (prikazanmesec[2] == 13) {
            prikazanmesec[2] = 1;
            prikazanmesec[3] = 2;
            prikazanmesec[4] = 2;
            prikazanmesec[5] = 2;
          }
          if (prikazanmesec[3] == 13) {
            prikazanmesec[3] = 1;
            prikazanmesec[4] = 1;
            prikazanmesec[5] = 1;
          }
        }

        if (obdobje == 4) {
          zamikdnevimax = 124;
          prikazanmesec[0] = zeljenmesec;
          prikazanmesec[1] = zeljenmesec + 1;
          prikazanmesec[2] = zeljenmesec + 2;
          prikazanmesec[3] = zeljenmesec + 3;
          prikazanmesec[4] = zeljenmesec + 4;
          prikazanmesec[5] = zeljenmesec + 4;
          if (prikazanmesec[1] == 13) {
            prikazanmesec[1] = 1;
            prikazanmesec[2] = 2;
            prikazanmesec[3] = 3;
            prikazanmesec[4] = 4;
            prikazanmesec[5] = 4;
          }
          if (prikazanmesec[2] == 13) {
            prikazanmesec[2] = 1;
            prikazanmesec[3] = 2;
            prikazanmesec[4] = 3;
            prikazanmesec[5] = 3;
          }
          if (prikazanmesec[3] == 13) {
            prikazanmesec[3] = 1;
            prikazanmesec[4] = 2;
            prikazanmesec[5] = 2;
          }
          if (prikazanmesec[4] == 13) {
            prikazanmesec[4] = 1;
            prikazanmesec[5] = 1;
          }
        }


        if (obdobje == 5) {
          prikazanmesec[0] = zeljenmesec;
          prikazanmesec[1] = zeljenmesec + 1;
          prikazanmesec[2] = zeljenmesec + 2;
          prikazanmesec[3] = zeljenmesec + 3;
          prikazanmesec[4] = zeljenmesec + 4;
          prikazanmesec[5] = zeljenmesec + 5;
          zamikdnevimax = 155;
          if (prikazanmesec[1] == 13) {
            prikazanmesec[1] = 1;
            prikazanmesec[2] = 2;
            prikazanmesec[3] = 3;
            prikazanmesec[4] = 4;
            prikazanmesec[5] = 5;
          }
          if (prikazanmesec[2] == 13) {
            prikazanmesec[2] = 1;
            prikazanmesec[3] = 2;
            prikazanmesec[4] = 3;
            prikazanmesec[5] = 4;
          }
          if (prikazanmesec[3] == 13) {
            prikazanmesec[3] = 1;
            prikazanmesec[4] = 2;
            prikazanmesec[5] = 3;
          }
          if (prikazanmesec[4] == 13) {
            prikazanmesec[4] = 1;
            prikazanmesec[5] = 2;
          }
          if (prikazanmesec[5] == 13) { prikazanmesec[5] = 1; }
        }

        /*tft.setCursor(15,30);
tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.print("0-"); 
 tft.print( prikazanmesec[0]); 
  tft.print(" 1-"); 
  tft.print( prikazanmesec[1]); 
  tft.print(" 2-"); 
  tft.print( prikazanmesec[2]); 
  tft.setCursor(15,45);
  tft.print(" 3-"); 
  tft.print( prikazanmesec[3]); 
   tft.print(" 4-"); 
  tft.print( prikazanmesec[4]); 
  tft.print(" 5-"); 
  tft.print( prikazanmesec[5]); */


        //vsebina zanke se izvede če le ta zadane index kateri kaže na vnos na datum, oz mesec, ki je zaželen za prikaz,
        // nato uporabimo ta indeks s katero je for zanka poiskala vnos, saj so datumi vnešeni po naslednjem formatu aa.aa.aaaa a=a
        // torej lahko izluščimo podatke v določenem datumu in obdoboju, za vsak dan imamo vnose za zaporedno št. dneva v mescu, meseca, in leto.
        // ter zabležen podatek oz meritev za vsako uro v dnevu, npr temperatura(100)(24).
        //torej vnose za 100 ti zaporedni dan najdemo na dnevi(100), meseci(100), leto(100) povprečnatemperatura(100)-npr. 29 12 2024 25C, za 101 dan pa dnevi(101), meseci(101), leto(101) itn.
        // Graf se riše od točke do točke, če naslednja glede na prejšnjo ni več v istem mescu, do tja ne rišemo linije.


        for (int i2 = 0; i2 < 365; i2++) {
          // Feed watchdog every 20 iterations to prevent timeout during sparse data processing
          
          
          // Validate arraysinorder bounds before access
          if (!isValidArrayIndex(i2, 365) || !isValidArrayIndex(i2 + 1, 365)) {
            continue; // Skip if indices are out of bounds
          }
          
          // Validate arraysinorder values
          if (!isValidArraysinorderIndex(i2) || !isValidArraysinorderIndex(i2 + 1)) {
            continue; // Skip if arraysinorder values are invalid
          }
          
          // Validate pointergrafobdobja before use
          if (!isValidPointer1D(pointergrafobdobja)) {
            break; // Exit if pointer is invalid
          }
          
          if ((mesecvzorec[arraysinorder[i2]] == prikazanmesec[0] && mesecvzorec[arraysinorder[i2 + 1]] == prikazanmesec[0])
              || (mesecvzorec[arraysinorder[i2]] == prikazanmesec[1] && mesecvzorec[arraysinorder[i2 + 1]] == prikazanmesec[1])
              || (mesecvzorec[arraysinorder[i2]] == prikazanmesec[2] && mesecvzorec[arraysinorder[i2 + 1]] == prikazanmesec[2])
              || (mesecvzorec[arraysinorder[i2]] == prikazanmesec[3] && mesecvzorec[arraysinorder[i2 + 1]] == prikazanmesec[3])
              || (mesecvzorec[arraysinorder[i2]] == prikazanmesec[4] && mesecvzorec[arraysinorder[i2 + 1]] == prikazanmesec[4])
              || (mesecvzorec[arraysinorder[i2]] == prikazanmesec[5] && mesecvzorec[arraysinorder[i2 + 1]] == prikazanmesec[5])) {

            if (letovzorec[arraysinorder[i2]] == prikazanoleto2 || prikazanoleto2 == -1) {

              if ((*pointergrafobdobja)[arraysinorder[i2]] != 101 && (*pointergrafobdobja)[arraysinorder[i2 + 1]] != 101 && arraysinorder[i2] != 1001 && arraysinorder[i2 + 1] != 1001) {

                /*  tft.print(i2); 
                tft.print("="); 
                tft.print(dnevivzorec[arraysinorder[i2]]); 
                 tft.print(".");
                 tft.print(mesecvzorec[arraysinorder[i2]]);
                 tft.print(" ");*/
                if (obdobje == 0) {
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[0]) {
                    zamikdnevi2 = 0;
                    prikazanoleto = letovzorec[arraysinorder[i2]];
                  }
                }
                if (obdobje == 1) {
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[1]) {
                    zamikdnevi2 = 31;
                    prikazanoleto = letovzorec[arraysinorder[i2]];
                  }
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[0]) {
                    zamikdnevi2 = 0;
                    prikazanoletostart = letovzorec[arraysinorder[i2]];
                  }
                }
                if (obdobje == 2) {
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[2]) {
                    zamikdnevi2 = 62;
                    prikazanoleto = letovzorec[arraysinorder[i2]];
                  }
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[1]) { zamikdnevi2 = 31; }
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[0]) {
                    zamikdnevi2 = 0;
                    prikazanoletostart = letovzorec[arraysinorder[i2]];
                  }
                }
                if (obdobje == 3) {
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[3]) {
                    zamikdnevi2 = 93;
                    prikazanoleto = letovzorec[arraysinorder[i2]];
                  }
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[2]) { zamikdnevi2 = 62; }
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[1]) { zamikdnevi2 = 31; }
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[0]) {
                    zamikdnevi2 = 0;
                    prikazanoletostart = letovzorec[arraysinorder[i2]];
                  }
                }
                if (obdobje == 4) {
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[4]) {
                    zamikdnevi2 = 124;
                    prikazanoleto = letovzorec[arraysinorder[i2]];
                  }
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[3]) { zamikdnevi2 = 93; }
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[2]) { zamikdnevi2 = 62; }
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[1]) { zamikdnevi2 = 31; }
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[0]) {
                    zamikdnevi2 = 0;
                    prikazanoletostart = letovzorec[arraysinorder[i2]];
                  }
                }
                if (obdobje == 5) {
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[5]) {
                    zamikdnevi2 = 155;
                    prikazanoleto = letovzorec[arraysinorder[i2]];
                  }
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[4]) { zamikdnevi2 = 124; }
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[3]) { zamikdnevi2 = 93; }
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[2]) { zamikdnevi2 = 62; }
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[1]) { zamikdnevi2 = 31; }
                  if (mesecvzorec[arraysinorder[i2]] == prikazanmesec[0]) {
                    zamikdnevi2 = 0;
                    prikazanoletostart = letovzorec[arraysinorder[i2]];
                  }
                }

                int x1 = map(dnevivzorec[arraysinorder[i2]] + zamikdnevi2, 1, 31 + zamikdnevimax, 10, 310);  // arraysinorder nosi indeks številko katera kaže na vnose v dnevivzorec, ki vsebuje datume,ki niso nujno v zaporedju, arraysinorder vsebuje indekse ki kažejo na datume v zaporedju.
                int y1 = map((*pointergrafobdobja)[arraysinorder[i2]], 0, maxyobdobje, 225, 15);
                int x2 = map(dnevivzorec[arraysinorder[i2 + 1]] + zamikdnevi2, 1, 31 + zamikdnevimax, 10, 310);
                int y2 = map((*pointergrafobdobja)[arraysinorder[i2 + 1]], 0, maxyobdobje, 225, 15);
                int cx0 = x1 + (x2 - x1) / 3;
                int cy0 = y1;
                int cx1 = x1 + 2 * (x2 - x1) / 3;
                int cy1 = y2;

                for (float t = 0.0; t <= 1.0; t += 0.01) {
                  // Feed watchdog every 10 iterations to prevent timeout during sparse data processing
                  
                  
                  int x = int(bezierPoint(x1, cx0, cx1, x2, t));
                  int y = int(bezierPoint(y1, cy0, cy1, y2, t));
                  
                  if (y1 != y2) {
                    if (pointergrafobdobja == &najvisjadnevnaT) { tft.drawPixel(x, y, TFT_ORANGE); }
                    if (pointergrafobdobja == &najnizjadnevnaT) { tft.drawPixel(x, y, TFT_YELLOW); }
                    if (pointergrafobdobja == &povprecnadnevnaT) { tft.drawPixel(x, y, TFT_GREEN); }
                    if (pointergrafobdobja == &najvisjadnevnaRH) { tft.drawPixel(x, y, TFT_RED); }
                    if (pointergrafobdobja == &najnizjadnevnaRH) { tft.drawPixel(x, y, TFT_VIOLET); }
                    if (pointergrafobdobja == &povprecnadnevnaRH) { tft.drawPixel(x, y, TFT_LIGHTGREY); }
                    if (pointergrafobdobja == &najnizjadnevnasens1) { tft.drawPixel(x, y, TFT_PINK); }
                    if (pointergrafobdobja == &najnizjadnevnasens2) { tft.drawPixel(x, y, TFT_BLACK); }
                    if (pointergrafobdobja == &najnizjadnevnasens3) { tft.drawPixel(x, y, TFT_BROWN); }
                    if (pointergrafobdobja == &najnizjadnevnasens4) { tft.drawPixel(x, y, TFT_OLIVE); }
                  } else {
                    if (pointergrafobdobja == &najvisjadnevnaT) { tft.drawLine(x1, y1, x2, y2, TFT_ORANGE); }
                    if (pointergrafobdobja == &najnizjadnevnaT) { tft.drawLine(x1, y1, x2, y2, TFT_YELLOW); }
                    if (pointergrafobdobja == &povprecnadnevnaT) { tft.drawLine(x1, y1, x2, y2, TFT_GREEN); }
                    if (pointergrafobdobja == &najvisjadnevnaRH) { tft.drawLine(x1, y1, x2, y2, TFT_RED); }
                    if (pointergrafobdobja == &najnizjadnevnaRH) { tft.drawLine(x1, y1, x2, y2, TFT_VIOLET); }
                    if (pointergrafobdobja == &povprecnadnevnaRH) { tft.drawLine(x1, y1, x2, y2, TFT_LIGHTGREY); }
                    if (pointergrafobdobja == &najnizjadnevnasens1) { tft.drawLine(x1, y1, x2, y2, TFT_PINK); }
                    if (pointergrafobdobja == &najnizjadnevnasens2) { tft.drawLine(x1, y1, x2, y2, TFT_BLACK); }
                    if (pointergrafobdobja == &najnizjadnevnasens3) { tft.drawLine(x1, y1, x2, y2, TFT_BROWN); }
                    if (pointergrafobdobja == &najnizjadnevnasens4) { tft.drawLine(x1, y1, x2, y2, TFT_OLIVE); }
                  }
                }
              } else {
                tft.setCursor(100, 100);
                tft.setTextColor(TFT_WHITE, TFT_BLUE);
                //i == numDataPointsM;
              }
            }
          }
        }
        for (int i = x4; i <= x5; i += x6) {
          int x = map(i, 0, x7, 15, 300);
          tft.setTextSize(1);
          tft.setTextColor(TFT_WHITE);
          tft.setCursor(x - 4, 225);
          if (x8 <= i) {
            if (i == 0) {
              tft.print("1");
            } else {


              if (obdobje == 0) { tft.print(i); }
              if (obdobje == 1) { tft.print(i); }
              if (obdobje == 2) { tft.print(i); }
              if (obdobje == 3 && i != 16 && i != 48 && i != 80 && i != 112) { tft.print(i); }
              if (obdobje == 4 && i != 120 && i != 140 && i != 40 && i != 80) { tft.print(i); }
              if (obdobje == 5 && i != 24 && i != 168 && i != 72 && i != 120 && i != 96 && i != 1 && i != 48) { tft.print(i); }
            }
          }
        }
        if (flagmenugraf == 0) {
          for (int i = 0; i <= 40; i += 10) {
            int y = map(i, 0, 40, 230, 12);
            tft.setTextSize(1);
            tft.setTextColor(TFT_WHITE);
            tft.setCursor(1, y - 10);
            tft.print(i);
          }
          tft.setCursor(14, 2);
          tft.print("C");
        }

        tft.setCursor(305, 225);
        tft.print(x7);
      }
    }
  }
}


float bezierPoint(float a, float b, float c, float d, float t) {
  float u = 1.0 - t;
  float tt = t * t;
  float uu = u * u;
  float uuu = uu * u;
  float ttt = tt * t;

  float p = uuu * a;
  p += 3 * uu * t * b;
  p += 3 * u * tt * c;
  p += ttt * d;
  return p;
}

void izrismenugraf() {
  if (!flagvstopizbirnik && !flagglavnimenu) {
    flaginfo = 0;
    flagmenugraf = 1;
    Serial.print("navimenu");
 Serial.println(navimenugraf);
    x = 13;   //Graf zamik dnevni
    x2 = 16;  //Napis zamik dnevni
    x3 = 84;  //Graf zamik mesečni
    if (obdobje == 0) {
      x8 = 20;
      x4 = 0;
    }
    if (obdobje == 1) {
      x8 = 36;
      x4 = 0;
    }
    if (obdobje == 2) {
      x8 = 53;
      x4 = 0;  //Napis zamik
    }
    if (obdobje == 3) {
      x8 = 66;
      x4 = 0;
    }

    if (obdobje == 4) {
      x8 = 85;
      x4 = 0;
    }

    // Draw smooth gradient background
    for (int y = 0; y < 240; y++) {
      // Create gradient from dark blue (top) to lighter blue (bottom)
      uint8_t r = 0 + (y * 20 / 240);    // Red component: 0 to ~20
      uint8_t g = 0 + (y * 80 / 240);    // Green component: 0 to ~80  
      uint8_t b = 139 + (y * 50 / 240);  // Blue component: 139 to ~189
      tft.drawFastHLine(0, y, 178, tft.color565(r, g, b));
    }

    if (flagmesecnigraf) {
      // Draw selection highlight first (before text)
      tft.fillRoundRect(2, navimenugraf * 2 - 2 + 3, 174, 18, 3, tft.color565(100, 150, 255));
      tft.drawRoundRect(2, navimenugraf * 2 - 2 + 3, 174, 18, 3, tft.color565(150, 200, 255));
      
      tft.setTextSize(2);
      tft.setTextColor(TFT_WHITE);
      tft.setCursor(6, 3);
      tft.print("MaksTemp");
      tft.setCursor(6, 23);
      tft.print("PovpTemp");
      tft.setCursor(6, 43);
      tft.print("MinTemp");
      tft.setCursor(6, 63);
      tft.print("MaksVlaga");
      tft.setCursor(6, 83);
      tft.print("PovpVlaga");
      tft.setCursor(6, 103);
      tft.print("MinVlaga");
      tft.setCursor(6, 123);
      tft.print("Senzor 1");
      tft.setCursor(6, 143);
      tft.print("Senzor 2");
      tft.setCursor(6, 163);
      tft.print("Senzor 3");
      tft.setCursor(6, 183);
      tft.print("Senzor 4");
      tft.setCursor(6, 203);
      tft.print("Obdobje");
      tft.setCursor(6, 223);
      tft.print("Leto");
      tft.setCursor(6, 243);
      tft.print("DnevniGraf");

      tft.setCursor(145, 0);
      tft.setTextColor(TFT_ORANGE);
      // Modern pill-style toggle switch
      if (flagmenuMDt) {
        // ON state - orange pill
        tft.drawRoundRect(145, 3, 30, 15, 7, tft.color565(100, 50, 0)); // Outline
        tft.fillRoundRect(146, 4, 28, 13, 6, TFT_ORANGE); // Track
        tft.fillCircle(167, 10, 5, TFT_WHITE); // Thumb
        // Drop shadow
        tft.fillCircle(167, 11, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 10, 5, TFT_WHITE); // Redraw thumb over shadow
      } else {
        // OFF state - gray pill
        tft.drawRoundRect(145, 3, 30, 15, 7, tft.color565(100, 100, 100)); // Outline
        tft.fillRoundRect(146, 4, 28, 13, 6, tft.color565(150, 150, 150)); // Track
        tft.fillCircle(153, 10, 5, TFT_WHITE); // Thumb
        // Drop shadow
        tft.fillCircle(153, 11, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 10, 5, TFT_WHITE); // Redraw thumb over shadow
      }
      tft.setCursor(145, 20);
      tft.setTextColor(TFT_GREEN);
      // Modern pill-style toggle switch
      if (flagmenuPDt) {
        tft.drawRoundRect(145, 23, 30, 15, 7, tft.color565(0, 100, 0));
        tft.fillRoundRect(146, 24, 28, 13, 6, TFT_GREEN);
        tft.fillCircle(167, 30, 5, TFT_WHITE);
        tft.fillCircle(167, 31, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 30, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 23, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 24, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 30, 5, TFT_WHITE);
        tft.fillCircle(153, 31, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 30, 5, TFT_WHITE);
      }
      tft.setCursor(145, 40);
      tft.setTextColor(TFT_YELLOW);
      // Modern pill-style toggle switch
      if (flagmenuMiDt) {
        tft.drawRoundRect(145, 43, 30, 15, 7, tft.color565(150, 100, 0));
        tft.fillRoundRect(146, 44, 28, 13, 6, TFT_YELLOW);
        tft.fillCircle(167, 50, 5, TFT_WHITE);
        tft.fillCircle(167, 51, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 50, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 43, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 44, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 50, 5, TFT_WHITE);
        tft.fillCircle(153, 51, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 50, 5, TFT_WHITE);
      }
      tft.setCursor(145, 60);
      tft.setTextColor(TFT_RED);
      // Modern pill-style toggle switch
      if (flagmenuMRH) {
        tft.drawRoundRect(145, 63, 30, 15, 7, tft.color565(100, 0, 0));
        tft.fillRoundRect(146, 64, 28, 13, 6, TFT_RED);
        tft.fillCircle(167, 70, 5, TFT_WHITE);
        tft.fillCircle(167, 71, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 70, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 63, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 64, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 70, 5, TFT_WHITE);
        tft.fillCircle(153, 71, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 70, 5, TFT_WHITE);
      }
      tft.setCursor(145, 80);
      tft.setTextColor(TFT_LIGHTGREY);
      // Modern pill-style toggle switch
      if (flagmenuPDRH) {
        tft.drawRoundRect(145, 83, 30, 15, 7, tft.color565(80, 80, 80));
        tft.fillRoundRect(146, 84, 28, 13, 6, TFT_LIGHTGREY);
        tft.fillCircle(167, 90, 5, TFT_WHITE);
        tft.fillCircle(167, 91, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 90, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 83, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 84, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 90, 5, TFT_WHITE);
        tft.fillCircle(153, 91, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 90, 5, TFT_WHITE);
      }
      tft.setCursor(145, 100);
      tft.setTextColor(TFT_VIOLET);
      // Modern pill-style toggle switch
      if (flagmenuMiRH) {
        tft.drawRoundRect(145, 103, 30, 15, 7, tft.color565(50, 0, 100));
        tft.fillRoundRect(146, 104, 28, 13, 6, TFT_VIOLET);
        tft.fillCircle(167, 110, 5, TFT_WHITE);
        tft.fillCircle(167, 111, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 110, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 103, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 104, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 110, 5, TFT_WHITE);
        tft.fillCircle(153, 111, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 110, 5, TFT_WHITE);
      }
      tft.setCursor(145, 120);
      tft.setTextColor(TFT_PINK);
      // Modern pill-style toggle switch
      if (flagmenusens1) {
        tft.drawRoundRect(145, 123, 30, 15, 7, tft.color565(150, 50, 100));
        tft.fillRoundRect(146, 124, 28, 13, 6, TFT_PINK);
        tft.fillCircle(167, 130, 5, TFT_WHITE);
        tft.fillCircle(167, 131, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 130, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 123, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 124, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 130, 5, TFT_WHITE);
        tft.fillCircle(153, 131, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 130, 5, TFT_WHITE);
      }
      tft.setCursor(155, 143);
      tft.setTextColor(TFT_BLACK);
      // Modern pill-style toggle switch
      if (flagmenusens2) {
        tft.drawRoundRect(145, 143, 30, 15, 7, tft.color565(30, 30, 30));
        tft.fillRoundRect(146, 144, 28, 13, 6, TFT_BLACK);
        tft.fillCircle(167, 150, 5, TFT_WHITE);
        tft.fillCircle(167, 151, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 150, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 143, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 144, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 150, 5, TFT_WHITE);
        tft.fillCircle(153, 151, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 150, 5, TFT_WHITE);
      }
      tft.setCursor(155, 163);
      tft.setTextColor(TFT_BROWN);
      // Modern pill-style toggle switch
      if (flagmenusens3) {
        tft.drawRoundRect(145, 163, 30, 15, 7, tft.color565(50, 25, 0));
        tft.fillRoundRect(146, 164, 28, 13, 6, TFT_BROWN);
        tft.fillCircle(167, 170, 5, TFT_WHITE);
        tft.fillCircle(167, 171, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 170, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 163, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 164, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 170, 5, TFT_WHITE);
        tft.fillCircle(153, 171, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 170, 5, TFT_WHITE);
      }
      tft.setCursor(155, 183);
      tft.setTextColor(TFT_OLIVE);
      // Modern pill-style toggle switch
      if (flagmenusens4) {
        tft.drawRoundRect(145, 183, 30, 15, 7, tft.color565(25, 25, 0));
        tft.fillRoundRect(146, 184, 28, 13, 6, TFT_OLIVE);
        tft.fillCircle(167, 190, 5, TFT_WHITE);
        tft.fillCircle(167, 191, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 190, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 183, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 184, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 190, 5, TFT_WHITE);
        tft.fillCircle(153, 191, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 190, 5, TFT_WHITE);
      }
      tft.setCursor(145, 203);
      tft.setTextColor(TFT_WHITE);
      tft.print((obdobje + 1));
      tft.setCursor(85, 223);
      if (prikazanoleto2 == -1) {
        tft.print((letovzorec[indeksvzorca] - 1));
        tft.print("-");
        tft.print(letovzorec[indeksvzorca]);
      } else {
        tft.print(prikazanoleto2);
      }


          }
    if (flagdnevnigraf) {
      // Draw selection highlight first (before text)
      if (navimenugraf != 60 && navimenugraf != 70 && navimenugraf != 80 && navimenugraf != 90) {
        tft.fillRoundRect(2, navimenugraf * 2 - 2 + 3, 174, 18, 3, tft.color565(100, 150, 255));
        tft.drawRoundRect(2, navimenugraf * 2 - 2 + 3, 174, 18, 3, tft.color565(150, 200, 255));
      }

      tft.setTextSize(2);
      tft.setTextColor(TFT_WHITE);
      tft.setCursor(6, 3);
      tft.print("Temperatura");
      tft.setCursor(155, 3);
      tft.setTextColor(TFT_WHITE);
      // Modern pill-style toggle switch
      if (flagmenuTemp) {
        tft.drawRoundRect(145, 3, 30, 15, 7, tft.color565(150, 0, 0));
        tft.fillRoundRect(146, 4, 28, 13, 6, TFT_RED);
        tft.fillCircle(167, 10, 5, TFT_WHITE);
        tft.fillCircle(167, 11, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 10, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 3, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 4, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 10, 5, TFT_WHITE);
        tft.fillCircle(153, 11, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 10, 5, TFT_WHITE);
      }
      tft.setCursor(6, 23);
      tft.print("Vlaga");
      tft.setCursor(155, 23);
      tft.setTextColor(TFT_WHITE);
      // Modern pill-style toggle switch
      if (flagmenuRH) {
        tft.drawRoundRect(145, 23, 30, 15, 7, tft.color565(0, 0, 150));
        tft.fillRoundRect(146, 24, 28, 13, 6, TFT_BLUE);
        tft.fillCircle(167, 30, 5, TFT_WHITE);
        tft.fillCircle(167, 31, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 30, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 23, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 24, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 30, 5, TFT_WHITE);
        tft.fillCircle(153, 31, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 30, 5, TFT_WHITE);
      }
      tft.setCursor(6, 43);
      tft.print("Senzor 1");
      tft.setCursor(6, 63);
      tft.print("Senzor 2");
      tft.setCursor(155, 63);
      tft.setTextColor(TFT_WHITE);
      // Modern pill-style toggle switch
      if (flagmenuSNZ2) {
        tft.drawRoundRect(145, 63, 30, 15, 7, tft.color565(150, 100, 0));
        tft.fillRoundRect(146, 64, 28, 13, 6, TFT_ORANGE);
        tft.fillCircle(167, 70, 5, TFT_WHITE);
        tft.fillCircle(167, 71, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 70, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 63, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 64, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 70, 5, TFT_WHITE);
        tft.fillCircle(153, 71, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 70, 5, TFT_WHITE);
      }
      tft.setCursor(6, 83);
      tft.print("Senzor 3");
      tft.setCursor(6, 103);
      tft.print("Senzor 4");
      tft.setCursor(155, 103);
      tft.setTextColor(TFT_WHITE);
      // Modern pill-style toggle switch
      if (flagmenuSNZ4) {
        tft.drawRoundRect(145, 103, 30, 15, 7, tft.color565(50, 25, 0));
        tft.fillRoundRect(146, 104, 28, 13, 6, TFT_BROWN);
        tft.fillCircle(167, 110, 5, TFT_WHITE);
        tft.fillCircle(167, 111, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 110, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 103, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 104, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 110, 5, TFT_WHITE);
        tft.fillCircle(153, 111, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 110, 5, TFT_WHITE);
      }
      tft.setCursor(6, 123);
      if (navimenugraf == 60) {
        tft.fillRoundRect(4, 121, 70, 18, 3, tft.color565(100, 150, 255));
        tft.drawRoundRect(4, 121, 70, 18, 3, tft.color565(150, 200, 255));
      }
      tft.print("Pojdi na:");
      tft.setCursor(6, 143);
      if (navimenugraf == 70) {
        tft.fillRoundRect(4, 141, 30, 18, 3, tft.color565(100, 150, 255));
        tft.drawRoundRect(4, 141, 30, 18, 3, tft.color565(150, 200, 255));
      }
      tft.print(zelendan);
      tft.print(".");
      if (navimenugraf == 80) {
        tft.fillRoundRect(32, 141, 30, 18, 3, tft.color565(100, 150, 255));
        tft.drawRoundRect(32, 141, 30, 18, 3, tft.color565(150, 200, 255));
      }
      tft.print(zelenmesec);
      tft.print(".");
      if (navimenugraf == 90) {
        tft.fillRoundRect(66, 141, 50, 18, 3, tft.color565(100, 150, 255));
        tft.drawRoundRect(66, 141, 50, 18, 3, tft.color565(150, 200, 255));
      }
      tft.print(zelenoleto);
      if (flagnenajdemvnosa == 1) {
        tft.setCursor(2, 160);
        tft.print("NI PODATKOV!");
      }
      /*  tft.setCursor(2, 80);
      tft.print("SprSnz3");
      tft.setCursor(2, 90);
      tft.print("SprSnz4");*/
      tft.setCursor(2, 223);
      tft.print("MesecniGraf");

      tft.setCursor(145, 0);
      tft.setTextColor(TFT_WHITE);
      // Modern pill-style toggle switch
      if (flagmenuTemp) {
        tft.drawRoundRect(145, 3, 30, 15, 7, tft.color565(150, 0, 0));
        tft.fillRoundRect(146, 4, 28, 13, 6, TFT_RED);
        tft.fillCircle(167, 10, 5, TFT_WHITE);
        tft.fillCircle(167, 11, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 10, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 3, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 4, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 10, 5, TFT_WHITE);
        tft.fillCircle(153, 11, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 10, 5, TFT_WHITE);
      }
      tft.setCursor(145, 20);
      tft.setTextColor(TFT_WHITE);
      // Modern pill-style toggle switch
      if (flagmenuRH) {
        tft.drawRoundRect(145, 23, 30, 15, 7, tft.color565(0, 0, 150));
        tft.fillRoundRect(146, 24, 28, 13, 6, TFT_BLUE);
        tft.fillCircle(167, 30, 5, TFT_WHITE);
        tft.fillCircle(167, 31, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 30, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 23, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 24, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 30, 5, TFT_WHITE);
        tft.fillCircle(153, 31, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 30, 5, TFT_WHITE);
      }
      tft.setCursor(145, 40);
      tft.setTextColor(TFT_GREEN);
      // Modern pill-style toggle switch
      if (flagmenuSNZ1) {
        tft.drawRoundRect(145, 43, 30, 15, 7, tft.color565(0, 100, 0));
        tft.fillRoundRect(146, 44, 28, 13, 6, TFT_GREEN);
        tft.fillCircle(167, 50, 5, TFT_WHITE);
        tft.fillCircle(167, 51, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 50, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 43, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 44, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 50, 5, TFT_WHITE);
        tft.fillCircle(153, 51, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 50, 5, TFT_WHITE);
      }
      tft.setCursor(145, 60);
      tft.setTextColor(TFT_WHITE);
      // Modern pill-style toggle switch
      if (flagmenuSNZ2) {
        tft.drawRoundRect(145, 63, 30, 15, 7, tft.color565(150, 100, 0));
        tft.fillRoundRect(146, 64, 28, 13, 6, TFT_ORANGE);
        tft.fillCircle(167, 70, 5, TFT_WHITE);
        tft.fillCircle(167, 71, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 70, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 63, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 64, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 70, 5, TFT_WHITE);
        tft.fillCircle(153, 71, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 70, 5, TFT_WHITE);
      }
      tft.setCursor(145, 80);
      tft.setTextColor(TFT_VIOLET);
      // Modern pill-style toggle switch
      if (flagmenuSNZ3) {
        tft.drawRoundRect(145, 83, 30, 15, 7, tft.color565(50, 0, 100));
        tft.fillRoundRect(146, 84, 28, 13, 6, TFT_VIOLET);
        tft.fillCircle(167, 90, 5, TFT_WHITE);
        tft.fillCircle(167, 91, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 90, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 83, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 84, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 90, 5, TFT_WHITE);
        tft.fillCircle(153, 91, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 90, 5, TFT_WHITE);
      }
      tft.setCursor(145, 100);
      tft.setTextColor(TFT_WHITE);
      // Modern pill-style toggle switch
      if (flagmenuSNZ4) {
        tft.drawRoundRect(145, 103, 30, 15, 7, tft.color565(50, 25, 0));
        tft.fillRoundRect(146, 104, 28, 13, 6, TFT_BROWN);
        tft.fillCircle(167, 110, 5, TFT_WHITE);
        tft.fillCircle(167, 111, 5, tft.color565(100, 100, 100));
        tft.fillCircle(167, 110, 5, TFT_WHITE);
      } else {
        tft.drawRoundRect(145, 103, 30, 15, 7, tft.color565(100, 100, 100));
        tft.fillRoundRect(146, 104, 28, 13, 6, tft.color565(150, 150, 150));
        tft.fillCircle(153, 110, 5, TFT_WHITE);
        tft.fillCircle(153, 111, 5, tft.color565(100, 100, 100));
        tft.fillCircle(153, 110, 5, TFT_WHITE);
      }
      tft.setCursor(145, 120);
      tft.setTextColor(TFT_WHITE);


          }
  }
}
bool removeDirRecursive(fs::FS &fs, const char *path) {
   Serial.printf("Attempting to delete: %s\n", path);
   
   // First, try to handle it as a file
   if (!fs.exists(path)) {
      Serial.printf("Path does not exist: %s\n", path);
      return false;
   }
   
   File root = fs.open(path);
   if (!root) {
      Serial.printf("Cannot open path, trying direct removal: %s\n", path);
      // If we can't open it, try to remove it directly (might be corrupted)
      bool removed = fs.remove(path);
      if (removed) {
         Serial.printf("Successfully removed corrupted file: %s\n", path);
         return true;
      } else {
         Serial.printf("Failed to remove corrupted file: %s\n", path);
         return false;
      }
   }
   
   // Check if it's a directory or file
   if (!root.isDirectory()) {
      root.close();
      // It's a file, try multiple removal attempts for corrupted files
      for (int attempt = 0; attempt < 3; attempt++) {
         bool removed = fs.remove(path);
         if (removed) {
            Serial.printf("Successfully removed file on attempt %d: %s\n", attempt + 1, path);
            return true;
         }
         Serial.printf("Failed to remove file on attempt %d: %s\n", attempt + 1, path);
         delay(100); // Small delay between attempts
      }
      Serial.printf("Failed to remove file after 3 attempts: %s\n", path);
      return false;
   }

   // It's a directory, process contents recursively
   File file = root.openNextFile();
   while (file) {
      String filePath = String(path) + "/" + file.name();
      bool isDirectory = file.isDirectory();
      file.close();

      if (isDirectory) {
         if (!removeDirRecursive(fs, filePath.c_str())) {
            Serial.printf("Failed to remove subdirectory: %s\n", filePath.c_str());
            root.close();
            return false;
         }
      } else {
         // Try multiple attempts for each file in directory
         bool fileRemoved = false;
         for (int attempt = 0; attempt < 3; attempt++) {
            if (fs.remove(filePath.c_str())) {
               Serial.printf("Successfully removed file on attempt %d: %s\n", attempt + 1, filePath.c_str());
               fileRemoved = true;
               break;
            }
            Serial.printf("Failed to remove file on attempt %d: %s\n", attempt + 1, filePath.c_str());
            delay(50);
         }
         
         if (!fileRemoved) {
            Serial.printf("Failed to remove file after 3 attempts: %s\n", filePath.c_str());
            root.close();
            return false;
         }
      }
      file = root.openNextFile();
   }
   root.close();
   
   // Try to remove the directory itself
   for (int attempt = 0; attempt < 3; attempt++) {
      if (fs.rmdir(path)) {
         Serial.printf("Successfully removed directory on attempt %d: %s\n", attempt + 1, path);
         return true;
      }
      Serial.printf("Failed to remove directory on attempt %d: %s\n", attempt + 1, path);
      delay(100);
   }
   
   Serial.printf("Failed to remove directory after 3 attempts: %s\n", path);
   return false;
}

// New function to get storage statistics
String getStorageStats() {
   String json = "{";
   // SD card stats
   uint64_t totalSD = SD.totalBytes();
   uint64_t usedSD = SD.usedBytes();
   json += "\"sd\":{";
   json += "\"total\":" + String(totalSD) + ",";
   json += "\"used\":" + String(usedSD) + ",";
   json += "\"free\":" + String(totalSD - usedSD);
   json += "},";
   // FATFS stats
   uint64_t totalFat = FFat.totalBytes();
   uint64_t usedFat = FFat.usedBytes();
   json += "\"fatfs\":{";
   json += "\"total\":" + String(totalFat) + ",";
   json += "\"used\":" + String(usedFat) + ",";
   json += "\"free\":" + String(totalFat - usedFat);
   json += "}";
   json += "}";
   return json;
}

String listDirJSON(fs::FS &fs, const String& path) {
   File root = fs.open(path.c_str());
   if (!root) {
      return "[]";
   }

   if (!root.isDirectory()) {
      root.close();
      return "[]";
   }

   String json = "[";
   File file = root.openNextFile();
   bool first = true;

   while (file) {
      if (!first) json += ",";
      first = false;

      String fileName = file.name();
      String fullPath;
      
      if (path == "/") {
         fullPath = "/" + fileName;
      } else {
         fullPath = path + "/" + fileName;
      }
      
      String displayName = fileName;
      if (fileName.startsWith(path) && path != "/") {
         displayName = fileName.substring(path.length());
         if (displayName.startsWith("/")) {
            displayName = displayName.substring(1);
         }
      }
      
      if (displayName.isEmpty() || displayName == path) {
         int lastSlash = fileName.lastIndexOf('/');
         if (lastSlash >= 0) {
            displayName = fileName.substring(lastSlash + 1);
         } else {
            displayName = fileName;
         }
      }

      json += "{";
      json += "\"name\":\"" + displayName + "\",";
      json += "\"path\":\"" + fullPath + "\",";
      json += "\"size\":" + String(file.size()) + ",";
      json += "\"type\":\"" + String(file.isDirectory() ? "directory" : "file") + "\"";
      json += "}";

      file.close();
      file = root.openNextFile();
   }
   root.close();
   json += "]";
   
   Serial.println("JSON for path " + path + ": " + json);
   return json;
}

String getContentType(const String& filename) {
   if (filename.endsWith(".htm")) return "text/html";
   else if (filename.endsWith(".html")) return "text/html";
   else if (filename.endsWith(".css")) return "text/css";
   else if (filename.endsWith(".js")) return "application/javascript";
   else if (filename.endsWith(".png")) return "image/png";
   else if (filename.endsWith(".gif")) return "image/gif";
   else if (filename.endsWith(".jpg")) return "image/jpeg";
   else if (filename.endsWith(".jpeg")) return "image/jpeg";
   else if (filename.endsWith(".ico")) return "image/x-icon";
   else if (filename.endsWith(".xml")) return "text/xml";
   else if (filename.endsWith(".pdf")) return "application/pdf";
   else if (filename.endsWith(".zip")) return "application/zip";
   else if (filename.endsWith(".rar")) return "application/x-rar-compressed";
   else if (filename.endsWith(".txt")) return "text/plain";
   return "application/octet-stream";
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
   Serial.printf("Listing directory: %s\n", dirname);

   File root = fs.open(dirname);
   if (!root) {
      Serial.println("Failed to open directory");
      return;
   }
   if (!root.isDirectory()) {
      Serial.println("Not a directory");
      root.close();
      return;
   }

   File file = root.openNextFile();
   while (file) {
      if (file.isDirectory()) {
         Serial.print("DIR : ");
         Serial.println(file.name());
         if (levels) {
            listDir(fs, file.name(), levels - 1);
         }
      } else {
         Serial.print("FILE: ");
         Serial.print(file.name());
         Serial.print("\tSIZE: ");
         Serial.println(file.size());
      }
      file.close();
      file = root.openNextFile();
   }
   root.close();
}
// ============================================================================
// AUTOMATIC FIRMWARE UPDATE FUNCTIONS
// ============================================================================

// Check for firmware updates from GitHub
// Returns true if update is available, false otherwise
// firmwareUrl parameter will contain the URL of the new firmware if update is available
bool checkForUpdates(String& firmwareUrl) {
    Serial.println("[UPDATE] Checking for firmware updates...");
    Serial.println("[UPDATE] Current firmware version: " + String(firmwareVersion));
    
    // Check rate limiting using Preferences
    Preferences preferences;
    preferences.begin("update_check", false);
    
    unsigned long lastCheck = preferences.getULong("last_check", 0);
    unsigned long currentTime = millis();
    
    // Handle millis() overflow
    if (currentTime < lastCheck) {
        lastCheck = 0;
    }
    
    // Skip check if not enough time has passed (unless it's first boot)
    if (lastCheck > 0 && (currentTime - lastCheck) < UPDATE_CHECK_INTERVAL) {
        Serial.println("[UPDATE] Rate limited - skipping check (last check: " + String((currentTime - lastCheck) / 1000 / 60) + " minutes ago)");
        preferences.end();
        return false;
    }
    
    // Save current time as last check time (regardless of success/failure)
    preferences.putULong("last_check", currentTime);
    preferences.end();
    
    // Download version.json
    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure(); // Allow HTTPS without certificate validation
    
    if (!http.begin(client, versionCheckURL)) {
        Serial.println("[UPDATE] Failed to begin HTTP connection");
        return false;
    }
    
    http.setTimeout(10000); // 10 second timeout
    
    int httpCode = http.GET();
    
    if (httpCode != HTTP_CODE_OK) {
        Serial.println("[UPDATE] HTTP request failed with code: " + String(httpCode));
        http.end();
        return false;
    }
    
    String payload = http.getString();
    http.end();
    
    // Parse JSON using ArduinoJson v7
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);
    
    if (error) {
        Serial.println("[UPDATE] JSON parse failed: " + String(error.c_str()));
        return false;
    }
    
    // Validate required fields
    if (!doc.containsKey("version") || !doc.containsKey("firmware_url")) {
        Serial.println("[UPDATE] Missing required fields in version.json");
        return false;
    }
    
    // Extract version and URL
    String remoteVersion = doc["version"].as<String>();
    String remoteUrl = doc["firmware_url"].as<String>();
    
    // Validate firmware URL format
    if (!remoteUrl.startsWith("https://")) {
        Serial.println("[UPDATE] Invalid firmware URL format - must start with https://");
        return false;
    }
    
    Serial.println("[UPDATE] Remote version: " + String(remoteVersion));
    Serial.println("[UPDATE] Remote URL: " + remoteUrl);
    
    // Compare versions
    if (remoteVersion > firmwareVersion) {
        Serial.println("[UPDATE] Update available! " + firmwareVersion + " → " + remoteVersion);
        
        // Check available space
        if (ESP.getFreeSketchSpace() <= 0) {
            Serial.println("[UPDATE] Not enough space for update");
            return false;
        }
        
        firmwareUrl = remoteUrl;
        return true;
    } else {
        Serial.println("[UPDATE] Already running latest version " + firmwareVersion);
        return false;
    }
}

// OTA update function
void doOTA() {
  Serial.println("Starting OTA from: " + String(defaultFirmwareUrl));
  
  webSocket.closeAll();
  server.end();
  delay(100);
  
  client.setInsecure();
  t_httpUpdate_return ret = httpUpdate.update(client, defaultFirmwareUrl);
  
  switch(ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("FAILED. Error: %s\n", httpUpdate.getLastErrorString().c_str());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("No update");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("Updated! Rebooting...");
      break;
  }
}
