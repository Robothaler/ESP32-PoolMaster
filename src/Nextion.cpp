/*
  NEXTION TFT related code, based on EasyNextion library by Seithan / Athanasios Seitanis (https://github.com/Seithan/EasyNextionLibrary)
  The trigger(s) functions at the end are called by the Nextion library on event (buttons, page change).

  (c) Loic74 <loic74650@gmail.com> 2018-2020

  Modified to implement display sleep mode.
  Remove every usages of String in order to avoid duplication, fragmentation and random crashes.
  Note usage of snprintf_P which uses a fmt string that resides in program memory.
*/

#include <Arduino.h>
#include "Config.h"
#include "PoolMaster.h"
#include "EasyNextionLibrary.h"
#ifdef MATTER_ENABLED
#include <esp_wifi.h>
#endif

/** DS18 NVS addresses mirrored in `storage` after loadConfig — do not read NVS here (race with prefs). */
static const uint8_t* dsAddrSlotW(int idx)
{
    switch (idx) {
        case 0: return storage.address_W_0;
        case 1: return storage.address_W_1;
        case 2: return storage.address_W_2;
        case 3: return storage.address_W_3;
        case 4: return storage.address_W_4;
        default: return storage.address_W_0;
    }
}

static const uint8_t* dsAddrSlotA(int idx)
{
    switch (idx) {
        case 0: return storage.address_A_0;
        case 1: return storage.address_A_1;
        case 2: return storage.address_A_2;
        case 3: return storage.address_A_3;
        case 4: return storage.address_A_4;
        default: return storage.address_A_0;
    }
}

static volatile int CurrentPage = 0;
static volatile bool TFT_ON = true;           // display status
static volatile bool refresh = false;         // flag to force display refresh

static char buf[48];                          // reusable stack buffer for Nextion writes — no heap
static unsigned long LastAction = 0;          // Last action time done on TFT. Go to sleep after TFT_SLEEP
static char HourBuffer[9];
uint32_t LastTFTUpdate = 0U;
uint32_t LastTFTTouch = 0U;
bool Sleep = false;

// One UpdateTFT cycle ≈ 1 s. After a Nextion touch trigger we set debounceX=1
// to give ProcessCommand (period 500 ms) time to actually mutate the firmware
// state. With debounceCount=1 the display realigns to the firmware truth on
// the second UpdateTFT pass at the latest (~1.5 s worst case) — including
// reverting the button visual when PoolServer rejects the command.
static uint8_t debounceCount = 1;
static uint8_t debounceM     = 0;
static uint8_t debounceB     = 0;
static uint8_t debounceSM    = 0;
static uint8_t debounceSolM  = 0;
static uint8_t debounceSolP  = 0;
static uint8_t debounceSolLE = 0;
static uint8_t debounceVM    = 0;
static uint8_t debounceCM    = 0;
static uint8_t debounceVS    = 0;
static uint8_t debounceSC    = 0;
static uint8_t debounceSP    = 0;
static uint8_t debounceF     = 0;
static uint8_t debouncepH    = 0;
static uint8_t debounceChl   = 0;
static uint8_t debounceH     = 0;
static uint8_t debounceHP    = 0;
static uint8_t debounceR0    = 0;
static uint8_t debounceR1    = 0;
static uint8_t debounceR2    = 0;
static uint8_t debouncepHP   = 0;
static uint8_t debounceChlP  = 0;
static uint8_t debounceWFM   = 0;
static uint8_t debounceHPM   = 0;
static uint8_t debounceWF    = 0;
static uint8_t debounceWiFi  = 0;
static uint8_t debounceMQL   = 0;
static uint8_t debounceSolV  = 0;

// variables for salt status cycling
static unsigned long lastSaltUpdate = 0;
static int saltDisplayState = 0;                        // 0: Polarity, 1: Salt Current, 2: Salt Status
static const unsigned long saltDisplayInterval = 2000;  // 2 seconds per state
// cycling between Uptime, ResetReason, Firmware, ResetTimestamp
static uint8_t displayState = 0;
static unsigned long lastDisplaySwitch = 0;
const unsigned long displayInterval = 2000;

// Structure holding the measurement values to display on the Nextion display
// Used to refresh only modified values
static struct TFTStruct
{
  float pH, pHRaw, Orp, OrpRaw, pHSP, OrpSP, WST, WIT, WBT, WWPT, WWTT, WTSP, AT, AH, AP, AIT, ST, SVLT, SRLT, PSI, flow, flow2, F1H, F1L, F2H, F2L, PsiH, PsiL, WTLow, pHPumpFR, ChlPumpFR, WaterFillFR, Ph_Kp, Ph_Ki, Ph_Kd, Orp_Kp, Orp_Ki, Orp_Kd, SaltCurrentValue, SaltNeeded, FiltPower, HeatPower, SaltCurrent_Raw, FilterCurrent_Raw, HeatCurrent_Raw;
  uint8_t FSta, FSto, FStaT0, FStoT1, SStaT0, SStoT1, pHTkFill, OrpTkFill, PIDpH, PIDChl, PubInt, DelayPID, pHPIDW, OrpPIDW, FLOW_Pulse, FLOW2_Pulse, SaltDiff, ResetReason;
  uint16_t PumpMaxUp, WFMaxUp, FillDur, WFMinLvl, WFMaxLvl;
  uint16_t MQTT_PORT;
  uint32_t Uptime;
  boolean WIFI_OnOff, MqttLogin, BUSA_B, Mode, SolarLoEx, SolarOnline, SolarMode, WaterFillMode, SaltMode, NetW, Filt, Robot, R0, R1, R2, pHUTErr, ChlUTErr, WFUTErr, WFErr, PSIErr, FLOWErr, FLOW2Err, pHTLErr, ChlTLErr, PhPump, ChlPump, Heat, HeatPump, SaltPump, SolarPump ,Salt_Chlor, SaltPolarity, ValveMode, CleanMode, ValveSwitch, WaterFill, HeatMode, SolarValve;
  unsigned long pHPpRT, OrpPpRT, SHRT, HPRT, SPUT, SPRT, SolPRT, FLRT, WFRT, WFAC;
  IPAddress MQTT_IP;
  DeviceAddress TW_Adr_1, TW_Adr_2, TW_Adr_3, TW_Adr_4, TW_Adr_5, TA_Adr_1, TA_Adr_2, TA_Adr_3, TA_Adr_4, TA_Adr_5;
  String FW, SSID, PASSW, MQTT_USER, MQTT_PASS, MQTT_NAME, SaltStatus, ResetTimestamp;
  std::string ELDTstate, ELDHstate, WPVstate, WPMstate, BOTTstate, SOLARstate;
} TFTStruc =
{ //default values to force update on next refresh
  -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., 0.0, -1., -1., -1., -1., -1.,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0,
  MQTT_SERVER_PORT,
  0U,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
  99,
  "",   "", "", "", "", "", "", "",
  "", "", "", "", "", "",
};

//Nextion TFT object. Choose which ever Serial port
//you wish to connect to (not "Serial" which is used for debug), here Serial2 UART
static EasyNex myNex(Serial1);

// When set, UpdateTFT() skips both NextionListen() AND the periodic widget
// writes so the Nextion OTA flash routine in Ota.cpp can have exclusive
// access to Serial1 (whmi-wri protocol).  See nextionPause()/nextionResume()
// below; called from updateNextion() before/after streaming the .tft file.
static volatile bool s_nextion_paused = false;

void nextionPause(void)  { s_nextion_paused = true;  }
void nextionResume(void) { s_nextion_paused = false; }

// Functions prototypes
void InitTFT(void);
void ResetTFT(void);
void UpdateTFT(void);
void UpdateWiFi(bool);
void getAddressString(DeviceAddress addr, char* temp, size_t tempSize);

// Format milliseconds as "HH : MM"
static void fmtUptime(char* buf, size_t bufSize, unsigned long ms) {
    int sec = ms / 1000;
    int min = sec / 60; sec %= 60;
    int hr  = min / 60; min %= 60;
    snprintf(buf, bufSize, "%02d : %02d", hr, min);
}

// Format integer with thousands separator (dot), e.g. 12345 → "12.345"
static void fmtThousands(char* buf, size_t bufSize, int value) {
    if (value >= 1000000)
        snprintf(buf, bufSize, "%d.%03d.%03d", value/1000000, (value/1000)%1000, value%1000);
    else if (value >= 1000)
        snprintf(buf, bufSize, "%d.%03d", value/1000, value%1000);
    else
        snprintf(buf, bufSize, "%d", value);
}

// ---------------------------------------------------------------------------
// Robust trigger helpers — generalise the FILT-button pattern (trigger6).
//
// Background: every "vabXxx.val" Nextion variable is just a *mirror* of a
// firmware-side state. If the mirror gets stale (e.g. the user navigated to
// another page that did not refresh it, or a previous read failed), reading
// the mirror would push a wrong "current" state and the toggle would no-op
// or flap.
//
// To stay correct we always derive the new state from the firmware truth
// source (pump.IsRunning(), storage.* flag, digitalRead(RELAY_*), …), build
// the matching JSON command, push it onto queueIn, and finally re-align the
// Nextion's vab variable so the touch UI reflects what the firmware will do
// — even before the next UpdateTFT pass.
// ---------------------------------------------------------------------------

// Push a JSON command into queueIn using a properly sized buffer
// (xQueueSendToBack copies QUEUE_ITEM_SIZE bytes; passing a smaller stack
// buffer to it would read past the end and is undefined behaviour).
static void enqueueJsonCmd(const char* json) {
    char qbuf[QUEUE_ITEM_SIZE] = {0};
    strncpy(qbuf, json, sizeof(qbuf) - 1);
    xQueueSendToBack(queueIn, &qbuf, 0);
}

// Push the current HH:MM:SS string to the active page's "p<N>Time" header
// element. The HMI uses a per-page text component named "p<page_no>Time"; if
// a particular page does not have one the Nextion silently rejects the write
// (no harm done). We always also keep the global "page0.vaTime.txt" mirror up
// to date so other pages can reference it via that variable if they wish.
//
// Called both from UpdateTFT() once per cycle AND directly from each page
// load trigger so the new page shows the time immediately instead of having
// to wait for the next UpdateTFT pass (which can be up to a second away).
static void pushTimeToCurrentPage(int page) {
    if (page < 0 || page > 30) return;
    char obj[16];
    snprintf(obj, sizeof(obj), "p%dTime.txt", page);
    myNex.writeStr(obj, HourBuffer);
}

// Common book-keeping for any "page N has finished loading" trigger.
// - remember the current page (used by UpdateTFT to know which page widgets to refresh)
// - push the header clock immediately so the new page does not show a stale
//   or empty time until the next UpdateTFT cycle
// - refresh LastAction so the auto-sleep timer restarts
static void onPageLoaded(int page) {
    CurrentPage = page;
    pushTimeToCurrentPage(page);
    LastAction = millis();
}

// Build a "{"<key>":<v>}" JSON command and enqueue it for ProcessCommand.
//
// Importantly we do NOT write the new state back to the HMI's vab*.val from
// inside the trigger handler. The single source of truth for what the
// display shows is UpdateTFT(): it polls the actual firmware state every
// cycle and writes the matching vab + picc values. That keeps the display
// guaranteed consistent with the real hardware/storage state — including the
// case where the firmware *rejects* the command (e.g. PhPump start denied
// because FiltrationPump is off): the vab/picc revert automatically once the
// per-button debounceX expires.
//
// Doing the writeNum here would also create a UART race with NextionListen()
// (the trigger handler runs while EasyNex is still parsing serial bytes) and
// can fight UpdateTFT() the next cycle when the firmware truth disagrees
// with the user's tap.
static void sendBoolCmd(const char* key, int target) {
    char json[64];
    snprintf(json, sizeof(json), "{\"%s\":%d}", key, target);
    enqueueJsonCmd(json);
    LastAction = millis();
}

// Determine the new target state for a toggle-style button. Strategy:
//   1. Read the HMI's vab*.val. After a touch press the HMI script may have
//      flipped it itself (this is how most of the Nextion buttons in this
//      project are configured — bMode, bRobot, …).
//   2. If vab differs from the firmware truth → user clearly wants the new
//      vab value → use it. This preserves the original UX for every button
//      that previously worked.
//   3. If vab equals firmware truth (HMI button does NOT toggle vab in its
//      press script — this is what bFilt does, hence the original "filter
//      pump cannot be switched from the display" bug) → fall back to
//      flipping the firmware state. This finally makes those buttons work.
//   4. If the Nextion read fails (sentinel 777777) → also fall back to
//      firmware-toggle so the user still gets a reaction.
static int computeToggleTarget(const char* nexVar, int firmwareState) {
    uint32_t v = myNex.readNumber(String(nexVar));
    if (v == 777777UL) {
        Debug.print(DBG_WARNING, "[Nextion] readNumber(%s) failed, using firmware-toggle fallback", nexVar);
        return firmwareState ? 0 : 1;
    }
    int iv = (int)v;
    if (iv != firmwareState) return iv;       // HMI toggled vab → user intent
    return firmwareState ? 0 : 1;             // HMI did not toggle vab → toggle ourselves
}

void getAddressString(DeviceAddress addr, char* temp, size_t tempSize) {
    size_t index = 0;
    for (int i = 0; i < 8; i++) {
        if (index >= tempSize - 1) break; // Sicherheitsprüfung, um Überläufe zu vermeiden
        if (addr[i] < 16) {
            temp[index++] = '0'; // Eine führende Null hinzufügen
        }
        if (index >= tempSize - 1) break;
        index += snprintf(&temp[index], tempSize - index, "%x", addr[i]); // Hexadezimale Zeichen hinzufügen
        if (i < 7 && index < tempSize - 1) {
            temp[index++] = ':'; // Doppelpunkte zwischen den Adressteilen
        }
    }
    temp[index] = '\0'; // Null-Terminierung
}

void InitTFT()
{
  myNex.begin(9600);
  //myNex.begin(9600, SERIAL_8N1, 44, 43);
}

//reset TFT at start of controller - Change transmission rate to 115200 bauds on both side (Nextion then ESP)
//could have been not in HMI file, but it is good to know that after reset the Nextion goes back to 9600 bauds
void ResetTFT()
{
  myNex.writeStr(F("rest"));
  delay(1000);
  myNex.writeStr("baud=115200");
  delay(100);
  myNex.begin(115200);
  LastAction = millis();
}

void UpdateWiFi(bool wifi){
  if(wifi){
    char wbuf[64];
#ifdef MATTER_ENABLED
    wifi_ap_record_t ap{};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
      snprintf(wbuf, sizeof(wbuf), "WiFi: %s", reinterpret_cast<const char*>(ap.ssid));
    } else {
      snprintf(wbuf, sizeof(wbuf), "WiFi: %s", storage.SSID.c_str());
    }
    myNex.writeStr("page0.vaSSID.txt", wbuf);
    char ipbuf[20];
    if (wifiStaGetIpv4String(ipbuf, sizeof(ipbuf)))
      snprintf(wbuf, sizeof(wbuf), "IP: %s", ipbuf);
    else
      snprintf(wbuf, sizeof(wbuf), "IP: --");
    myNex.writeStr("page0.vaIP.txt", wbuf);
#else
    snprintf(wbuf, sizeof(wbuf), "WiFi: %s", WiFi.SSID().c_str());
    myNex.writeStr("page0.vaSSID.txt", wbuf);
    IPAddress ip = WiFi.localIP();
    snprintf(wbuf, sizeof(wbuf), "IP: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    myNex.writeStr("page0.vaIP.txt", wbuf);
#endif
  } else {
    myNex.writeStr("page0.vaSSID.txt","not connected");
    myNex.writeStr("page0.vaIP.txt","");
  }
}



//function to update TFT display
//it updates the TFTStruct variables, the global variables of the TFT + the widgets of the active page
//call this function at least every second to ensure fluid display
void UpdateTFT()
{
  // Skip everything while a Nextion OTA flash is in progress — Ota.cpp owns
  // Serial1 exclusively for the duration (whmi-wri).  Concurrent listen/write
  // would corrupt the .tft stream.
  if (s_nextion_paused) return;

  myNex.NextionListen();

  UpdateWiFi(wifiStaConnected());

  poolEnsureEuropeBerlinTz();
  time_t nowWall = time(nullptr);
  struct tm lt = {};
  if (nowWall != (time_t)-1 && localtime_r(&nowWall, &lt) != nullptr) {
    snprintf(HourBuffer, sizeof(HourBuffer), "%02d:%02d:%02d",
             lt.tm_hour, lt.tm_min, lt.tm_sec);
  } else {
    snprintf(HourBuffer, sizeof(HourBuffer), "%02d:%02d:%02d",
             hour(), minute(), second());
  }
  myNex.writeStr("page0.vaTime.txt", HourBuffer);

  if (millis() - lastDisplaySwitch >= displayInterval || !refresh) {
      xSemaphoreTake(mutex, portMAX_DELAY);
      char temp[32]; // Buffer for formatted text
      switch (displayState) {
          case 0: // Uptime
              if (storage.Uptime != TFTStruc.Uptime || !refresh) {
                  TFTStruc.Uptime = storage.Uptime;
                  snprintf(temp, sizeof(temp), "Uptime: %lu h", TFTStruc.Uptime);
                  myNex.writeStr(F("page0.vaMCFW.txt"), temp);
                  if (CurrentPage == 11) {
                      myNex.writeStr(F("t1.txt"), temp);
                  }
                  Debug.print(DBG_VERBOSE, "[Nextion] Updated Uptime: %s", temp);
              }
              break;
          case 1: // ResetReason
              if (storage.ResetReason != TFTStruc.ResetReason || !refresh) {
                  TFTStruc.ResetReason = storage.ResetReason;
                  snprintf(temp, sizeof(temp), "Reset: %s", resetReasonToString(TFTStruc.ResetReason));
                  myNex.writeStr(F("page0.vaMCFW.txt"), temp);
                  if (CurrentPage == 11) {
                      myNex.writeStr(F("t1.txt"), temp);
                  }
                  Debug.print(DBG_VERBOSE, "[Nextion] Updated ResetReason: %s", temp);
              }
              break;
          case 2: // ResetTimestamp
              if (storage.ResetTimestamp != TFTStruc.ResetTimestamp || !refresh) {
                  TFTStruc.ResetTimestamp = storage.ResetTimestamp;
                  snprintf(temp, sizeof(temp), "Reset at: %s", TFTStruc.ResetTimestamp.c_str());
                  myNex.writeStr(F("page0.vaMCFW.txt"), temp);
                  if (CurrentPage == 11) {
                      myNex.writeStr(F("t1.txt"), temp);
                  }
                  Debug.print(DBG_VERBOSE, "[Nextion] Updated ResetTimestamp: %s", temp);
              }
              break;
          case 3: // Firmware
              if (Firmw != TFTStruc.FW || !refresh) {
                  TFTStruc.FW = Firmw;
                  snprintf(temp, sizeof(temp), "MC fw: v %s", TFTStruc.FW.c_str());
                  myNex.writeStr(F("page0.vaMCFW.txt"), temp);
                  if (CurrentPage == 11) {
                      myNex.writeStr(F("t1.txt"), temp);
                  }
                  Debug.print(DBG_VERBOSE, "[Nextion] Updated Firmware: %s", temp);
              }
              break;
      }
      xSemaphoreGive(mutex);
      displayState = (displayState + 1) % 4; // Cycle: 0 -> 1 -> 2 -> 3 -> 0
      lastDisplaySwitch = millis();
  }

  if (storage.WIFI_OnOff != TFTStruc.WIFI_OnOff || !refresh)
  {
    if ((debounceWiFi == 0) || (debounceWiFi > debounceCount))
    {
      debounceWiFi = 0;
      TFTStruc.WIFI_OnOff = storage.WIFI_OnOff;
      myNex.writeNum(F("page18.vabWiFi_OnOff.val"), TFTStruc.WIFI_OnOff);
      if (CurrentPage == 18)
      {
        if (TFTStruc.WIFI_OnOff == 1)
        myNex.writeStr(F("bWiFi_OnOff.picc=52"));
        else
        myNex.writeStr(F("bWiFi_OnOff.picc=51"));
      }
      else if (CurrentPage == 11)
      {
        if (TFTStruc.WIFI_OnOff == 1)
        myNex.writeStr(F("b5.picc=43"));
        else
        myNex.writeStr(F("b5.picc=42"));
      }
    }
    else
      debounceWiFi++;
  }

  if (storage.MQTTLOGIN_OnOff != TFTStruc.MqttLogin || !refresh)
  {
    if ((debounceMQL == 0) || (debounceMQL > debounceCount))
    {
      debounceMQL = 0;
      TFTStruc.MqttLogin = storage.MQTTLOGIN_OnOff;
      myNex.writeNum(F("page19.vabMqttLogin.val"), TFTStruc.MqttLogin);
      if (CurrentPage == 19)
      {
        if (TFTStruc.MqttLogin == 1)
        myNex.writeStr(F("bMqttLogin.picc=50"));
        else
        myNex.writeStr(F("bMqttLogin.picc=49"));
      }
    }
    else
      debounceMQL++;
  }
  
  if (storage.SSID != TFTStruc.SSID || !refresh)
  {
    TFTStruc.SSID = storage.SSID;
    myNex.writeStr(F("page0.vaWifiSSID.txt"), TFTStruc.SSID);
    Debug.print(DBG_DEBUG, "Updating TFT SSID: %s", TFTStruc.SSID.c_str());
    if (CurrentPage == 24)  myNex.writeStr(F("ssid.txt"), TFTStruc.SSID);
  }

  if (storage.WIFI_PASS != TFTStruc.PASSW || !refresh)
  {
    TFTStruc.PASSW = storage.WIFI_PASS;
    myNex.writeStr(F("page0.vaWifiPASS.txt"), TFTStruc.PASSW);
    Debug.print(DBG_DEBUG, "Updating TFT WiFi password: %s", TFTStruc.PASSW.c_str());
    if (CurrentPage == 24)  myNex.writeStr(F("pwd.txt"), TFTStruc.PASSW);
  }
  
  {
    const bool netOnline = nextionNetStatusOnline();
    if (netOnline != TFTStruc.NetW || !refresh) {
      TFTStruc.NetW = netOnline;
      myNex.writeNum(F("page1.vabNetW.val"), TFTStruc.NetW);
      const char* netwStr = TFTStruc.NetW ? "ONLINE" : "OFFLINE";
      myNex.writeStr(F("page0.vaMqttState.txt"), netwStr);
      myNex.writeStr(F("page19.MqttState.txt"), netwStr);

      // Generic indicator update: pXNetW.pic=5/6 (pages 0-19 except 11)
      if (CurrentPage >= 0 && CurrentPage <= 19) {
        char cmd[20];
        snprintf(cmd, sizeof(cmd), "p%dNetW.pic=%d", CurrentPage, TFTStruc.NetW ? 5 : 6);
        myNex.writeStr(cmd);
        if (CurrentPage == 11) {
          myNex.writeStr(TFTStruc.NetW ? F("b6.picc=43") : F("b6.picc=42"));
          myNex.writeStr(TFTStruc.NetW ? F("b6.picc2=42") : F("b6.picc2=43"));
        } else if (CurrentPage == 19) {
          myNex.writeStr(F("MqttState.txt"), netwStr);
        }
      }
    }
  }

  if (storage.MQTT_NAME != TFTStruc.MQTT_NAME || !refresh)
  {
    TFTStruc.MQTT_NAME = storage.MQTT_NAME;
    myNex.writeStr(F("page0.vaMqttName.txt"), TFTStruc.MQTT_NAME);
    if (CurrentPage == 19)  myNex.writeStr(F("MqttName.txt"), TFTStruc.MQTT_NAME);
  }

  if (storage.MQTT_USER != TFTStruc.MQTT_USER || !refresh)
  {
    TFTStruc.MQTT_USER = storage.MQTT_USER;
    myNex.writeStr(F("page0.vaMqttUser.txt"), TFTStruc.MQTT_USER);
    if (CurrentPage == 19)  myNex.writeStr(F("MqttUser.txt"), TFTStruc.MQTT_USER);
  }

  if (storage.MQTT_PASS != TFTStruc.MQTT_PASS || !refresh)
  {
    TFTStruc.MQTT_PASS = storage.MQTT_PASS;
    myNex.writeStr(F("page0.vaMqttPass.txt"), TFTStruc.MQTT_PASS);
    if (CurrentPage == 19)  myNex.writeStr(F("MqttPass.txt"), TFTStruc.MQTT_PASS);
  }

  if (storage.MQTT_PORT != TFTStruc.MQTT_PORT || !refresh)
  {
    TFTStruc.MQTT_PORT = storage.MQTT_PORT;
    snprintf(buf, sizeof(buf), "%u", TFTStruc.MQTT_PORT);
    myNex.writeStr(F("page0.vaMqttPort.txt"), buf);
    if (CurrentPage == 19)  myNex.writeStr(F("MqttPort.txt"), buf);
  }

  if (storage.MQTT_IP != TFTStruc.MQTT_IP || !refresh)
  {
    TFTStruc.MQTT_IP = storage.MQTT_IP;
    snprintf(buf, sizeof(buf), "%d.%d.%d.%d",
             TFTStruc.MQTT_IP[0], TFTStruc.MQTT_IP[1], TFTStruc.MQTT_IP[2], TFTStruc.MQTT_IP[3]);
    myNex.writeStr(F("page0.vaMqttIP.txt"), buf);
    Debug.print(DBG_DEBUG, "[MQTT / NEXTION] MQTT server IP address: %s", buf);
    if (CurrentPage == 19)  myNex.writeStr(F("MqttIP.txt"), buf);
  }

  if (storage.PhValue != TFTStruc.pH || !refresh)
  {
    TFTStruc.pH = storage.PhValue;
    int pHgauge = TFTStruc.pH * 10;
    int pHangle;
    snprintf(buf, sizeof(buf), "%.2f", TFTStruc.pH);
    myNex.writeStr(F("page0.vapH.txt"), buf);
      if(pHgauge >= 69 && pHgauge <= 72) {
        pHangle = map(pHgauge,69,72,1500,5600);
      } else if(pHgauge > 72 && pHgauge <= 80) {
        pHangle = map(pHgauge,72,80,5600,7200);
      } else if(pHgauge > 80) {
        pHangle = 7200;
      } else if(pHgauge < 69 && pHgauge > 60) {
        pHangle = map(pHgauge,60,69,1,1500);
      } else {
        pHangle = 0;
      }
    myNex.writeNum(F("page0.pHg.val"), pHangle);
    if (CurrentPage == 0) {
      myNex.writeStr(F("pH.txt"), buf);
      myNex.writeNum(F("pHg.val"), pHangle);
    }
  }

  // Übertragung der pH-Rohwerte
  if (storage.PhRawValue != TFTStruc.pHRaw || !refresh)
  {
      TFTStruc.pHRaw = storage.PhRawValue;
      snprintf(buf, sizeof(buf), "%.2f", TFTStruc.pHRaw);
      myNex.writeStr(F("page0.vapHrw.txt"), buf);
      if (CurrentPage == 0)  myNex.writeStr(F("vapHrw.txt"), buf);
  }

  if (storage.OrpValue != TFTStruc.Orp || !refresh)
  {
    TFTStruc.Orp = storage.OrpValue;
    int Orpgauge = TFTStruc.Orp;
    int Orpangle;
    snprintf(buf, sizeof(buf), "%.0f", TFTStruc.Orp);
    myNex.writeStr(F("page0.vaOrp.txt"), buf);
      if(Orpgauge >= 750 && Orpgauge <= 850) {
        Orpangle = map(Orpgauge,750,850,1500,5600);
      } else if(Orpgauge > 850 && Orpgauge <= 900) {
        Orpangle = map(Orpgauge,850,900,5600,7200);
      } else if(Orpgauge > 900) {
        Orpangle = 7200;
      } else if(Orpgauge < 750 && Orpgauge > 500) {
        Orpangle = map(Orpgauge,500,750,1,1500);
      } else {
        Orpangle = 0;
      }
      myNex.writeNum(F("page0.Orpg.val"), Orpangle);
    if (CurrentPage == 0) {
      myNex.writeStr(F("Orp.txt"), buf);
      myNex.writeNum(F("Orpg.val"), Orpangle);
    }
  }

  // Übertragung der ORP-Rohwerte
  if (storage.OrpRawValue != TFTStruc.OrpRaw || !refresh)
  {
    TFTStruc.OrpRaw = storage.OrpRawValue;
    snprintf(buf, sizeof(buf), "%.0f", TFTStruc.OrpRaw);
    myNex.writeStr(F("page0.vaOrprw.txt"), buf);
    if (CurrentPage == 0)  myNex.writeStr(F("vaOrprw.txt"), buf);
  }

  if (storage.Ph_SetPoint != TFTStruc.pHSP || !refresh)
  {
    TFTStruc.pHSP = storage.Ph_SetPoint;
    snprintf(buf, sizeof(buf), "(%.1f)", TFTStruc.pHSP);
    myNex.writeStr(F("page0.vapHSP.txt"), buf);
    if (CurrentPage == 0 || CurrentPage == 15)  myNex.writeStr(F("pHSP.txt"), buf);
  }

  if (storage.Orp_SetPoint != TFTStruc.OrpSP || !refresh)
  {
    TFTStruc.OrpSP = storage.Orp_SetPoint;
    snprintf(buf, sizeof(buf), "(%d)", (int)TFTStruc.OrpSP);
    myNex.writeStr(F("page0.vaOrpSP.txt"), buf);
    if (CurrentPage == 0 || CurrentPage == 13 || CurrentPage == 16)  myNex.writeStr(F("OrpSP.txt"), buf);
  }

  if (PhPID.GetMode() != TFTStruc.PIDpH || !refresh)
  {
    if ((debouncepHP == 0) || (debouncepHP > debounceCount))
    {
    debouncepHP = 0;
    TFTStruc.PIDpH = PhPID.GetMode();
    myNex.writeNum(F("page15.vapHMode.val"), TFTStruc.PIDpH);
    if (CurrentPage == 0)
      {
        if (TFTStruc.PIDpH == 1)
        {
          myNex.writeStr(F("bpHP.picc=29"));
          myNex.writeStr(F("bpHP.picc2=0"));
        }
        else
        {
          myNex.writeStr(F("bpHP.picc=0"));
          myNex.writeStr(F("bpHP.picc2=29"));
        }
      }else if (CurrentPage == 15)
      {
        if (TFTStruc.PIDpH == 1)
        {
          myNex.writeStr(F("bpHMode.picc=36"));
          myNex.writeStr(F("bpHMode.picc2=37"));
        }
        else
        {
          myNex.writeStr(F("bpHMode.picc=37"));
          myNex.writeStr(F("bpHMode.picc2=36"));
        }
      }
    }
    else
      debouncepHP++;
  }

    if (OrpPID.GetMode() != TFTStruc.PIDChl || !refresh)
  {
    if ((debounceChlP == 0) || (debounceChlP > debounceCount))
    {
    debouncepHP = 0;
    TFTStruc.PIDChl = OrpPID.GetMode();
    myNex.writeNum(F("page16.vaChlMode.val"), TFTStruc.PIDChl);
    if (CurrentPage == 0)
      {
        if (TFTStruc.PIDChl == 1)
        {
          myNex.writeStr(F("bChlP.picc=29"));
          myNex.writeStr(F("bChlP.picc2=0"));
        }
        else
        {
          myNex.writeStr(F("bChlP.picc=0"));
          myNex.writeStr(F("bChlP.picc2=29"));
        }
      }else if (CurrentPage == 16)
      {
        if (TFTStruc.PIDpH == 1)
        {
          myNex.writeStr(F("bChlMode.picc=32"));
          myNex.writeStr(F("bChlMode.picc2=33"));
        }
        else
        {
          myNex.writeStr(F("bChlMode.picc=33"));
          myNex.writeStr(F("bChlMode.picc2=32"));
        }
      }
    }
    else
      debounceChlP++;
  }

  if (storage.BUS_A_B != TFTStruc.BUSA_B || !refresh)
  {
    if ((debounceB == 0) || (debounceB > debounceCount))
    {
      debounceB = 0;
      TFTStruc.BUSA_B = storage.BUS_A_B;
      myNex.writeNum(F("pageTempArray.vabBUSA_B.val"), TFTStruc.BUSA_B);
      if (CurrentPage == 27)
      {
        if (TFTStruc.BUSA_B == 1)
        myNex.writeStr(F("b2.pic=61"));
        else
        myNex.writeStr(F("b2.pic=60"));
      }
    }
    else
      debounceB++;
  }

  /*
  if (storage.BUS_A_B != TFTStruc.BUSA_B || !refresh) {
  TFTStruc.BUSA_B = storage.BUS_A_B;
  if (CurrentPage == 27)
      {
        if (TFTStruc.BUSA_B == 1)
        // Display data for Sensor W
        for (int i = 0; i < MAX_ADDRESSES; i++)
        {
          DeviceAddress addr;
          memcpy(&addr, &DS18B20_W[i], sizeof(DeviceAddress));
          String name = NV_STORAGE_MAPPING_W[DS18B20_Mapping_W[i]];
          String num = String(i + 1);
          String pos = String(DS18B20_Mapping_W[i]);
          myNex.writeStr("pageTempArray.s" + num + ".txt", getAddressString(addr) + " | " + name);
          myNex.writeStr("pageTempArray.a" + num + ".txt", num);
          myNex.writeStr("pageTempArray.t" + pos + ".txt", pos);

          Debug.print(DBG_VERBOSE, "Address %d for Sensor W: %s, Name: %s, Number: %s, Position: %s", i+1, getAddressString(addr).c_str(), name.c_str(), num.c_str(), pos.c_str());
        }
        else
        // Display data for Sensor A
        for (int i = 0; i < MAX_ADDRESSES; i++)
        {
            DeviceAddress addr;
            memcpy(&addr, &DS18B20_A[i], sizeof(DeviceAddress));
            String name = NV_STORAGE_MAPPING_A[DS18B20_Mapping_A[i]];
            String num = String(i + 1);
            String pos = String(DS18B20_Mapping_A[i]);
            myNex.writeStr("pageTempArray.s" + num + ".txt", getAddressString(addr) + " | " + name);
            myNex.writeStr("pageTempArray.a" + num + ".txt", num);
            myNex.writeStr("pageTempArray.t" + pos + ".txt", pos);

            Debug.print(DBG_VERBOSE, "Address %d for Sensor A: %s, Name: %s, Number: %s, Position: %s", i+1, getAddressString(addr).c_str(), name.c_str(), num.c_str(), pos.c_str());
        }
    }
  } */

  if (storage.BUS_A_B != TFTStruc.BUSA_B || !refresh)
  {
    TFTStruc.BUSA_B = storage.BUS_A_B;
    if (CurrentPage == 27)
    {
      if (TFTStruc.BUSA_B == 1)
      {
        // Display data for Sensor W
        for (int i = 0; i < MAX_ADDRESSES; i++)
        {
        const char* name = NV_STORAGE_MAPPING_W[storage.Array_W[i]];
        char addrStr[18];
        byte storedAddr[8];
        memcpy(storedAddr, dsAddrSlotW(i), 8);
        snprintf(addrStr, sizeof(addrStr), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                storedAddr[0], storedAddr[1], storedAddr[2], storedAddr[3],
                storedAddr[4], storedAddr[5], storedAddr[6], storedAddr[7]);
        char sval[48]; snprintf(sval, sizeof(sval), "%s | %s", addrStr, name);
        char sobj[28]; snprintf(sobj, sizeof(sobj), "pageTempArray.s%d.txt", i + 1);
        char aobj[28]; snprintf(aobj, sizeof(aobj), "pageTempArray.a%d.txt", i + 1);
        char tobj[28]; snprintf(tobj, sizeof(tobj), "pageTempArray.t%d.txt", storage.Array_W[i]);
        char numStr[4]; snprintf(numStr, sizeof(numStr), "%d", i + 1);
        char posStr[4]; snprintf(posStr, sizeof(posStr), "%d", storage.Array_W[i]);
        myNex.writeStr(sobj, sval);
        myNex.writeStr(aobj, numStr);
        myNex.writeStr(tobj, posStr);
        Debug.print(DBG_VERBOSE, "Address %d for Sensor W: %s, Name: %s, Number: %s, Position: %s", i + 1, addrStr, name, numStr, posStr);
        }
      }
      else
      {
        // Display data for Sensor A
        for (int i = 0; i < MAX_ADDRESSES; i++)
        {
        const char* name = NV_STORAGE_MAPPING_A[storage.Array_A[i]];
        char addrStr[18];
        byte storedAddr[8];
        memcpy(storedAddr, dsAddrSlotA(i), 8);
        snprintf(addrStr, sizeof(addrStr), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                storedAddr[0], storedAddr[1], storedAddr[2], storedAddr[3],
                storedAddr[4], storedAddr[5], storedAddr[6], storedAddr[7]);
        char sval[48]; snprintf(sval, sizeof(sval), "%s | %s", addrStr, name);
        char sobj[28]; snprintf(sobj, sizeof(sobj), "pageTempArray.s%d.txt", i + 1);
        char aobj[28]; snprintf(aobj, sizeof(aobj), "pageTempArray.a%d.txt", i + 1);
        char tobj[28]; snprintf(tobj, sizeof(tobj), "pageTempArray.t%d.txt", storage.Array_A[i]);
        char numStr[4]; snprintf(numStr, sizeof(numStr), "%d", i + 1);
        char posStr[4]; snprintf(posStr, sizeof(posStr), "%d", storage.Array_A[i]);
        myNex.writeStr(sobj, sval);
        myNex.writeStr(aobj, numStr);
        myNex.writeStr(tobj, posStr);
        Debug.print(DBG_VERBOSE, "Address %d for Sensor A: %s, Name: %s, Number: %s, Position: %s", i + 1, addrStr, name, numStr, posStr);
        }
      }
    }
  }

  if (storage.WaterSTemp != TFTStruc.WST || !refresh)
  {
    TFTStruc.WST = storage.WaterSTemp;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.WST);
    myNex.writeStr(F("page0.vaWT.txt"), buf);
    if (CurrentPage == 0 || CurrentPage == 2 || CurrentPage == 3 || CurrentPage == 7)
      myNex.writeStr(F("W.txt"), buf);
  }

  if (storage.WaterITemp != TFTStruc.WIT || !refresh)
  {
    TFTStruc.WIT = storage.WaterITemp;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.WIT);
    myNex.writeStr(F("page0.vaWIT.txt"), buf);
    if (CurrentPage == 0 || CurrentPage == 7)  myNex.writeStr(F("WIT.txt"), buf);
  }

  if (storage.WaterBTemp != TFTStruc.WBT || !refresh)
  {
    TFTStruc.WBT = storage.WaterBTemp;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.WBT);
    myNex.writeStr(F("page0.vaWBT.txt"), buf);
    if (CurrentPage == 0 || CurrentPage == 7)  myNex.writeStr(F("WBT.txt"), buf);
  }

  if (storage.WaterWPTemp != TFTStruc.WWPT || !refresh)
  {
    TFTStruc.WWPT = storage.WaterWPTemp;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.WWPT);
    myNex.writeStr(F("page0.vaWWPT.txt"), buf);
    if (CurrentPage == 0 || CurrentPage == 7)  myNex.writeStr(F("WWPT.txt"), buf);
  }

  if (storage.WaterWTTemp != TFTStruc.WWTT || !refresh)
  {
    TFTStruc.WWTT = storage.WaterWTTemp;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.WWTT);
    myNex.writeStr(F("page0.vaWWTT.txt"), buf);
    if (CurrentPage == 0 || CurrentPage == 7)  myNex.writeStr(F("WWTT.txt"), buf);
  }

  if (storage.AirInTemp != TFTStruc.AIT || !refresh)
  {
    TFTStruc.AIT = storage.AirInTemp;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.AIT);
    myNex.writeStr(F("page0.vaAIT.txt"), buf);
    if (CurrentPage == 0 || CurrentPage == 7)  myNex.writeStr(F("AIT.txt"), buf);
  }

  if (storage.SolarVLTemp != TFTStruc.SVLT || !refresh)
  {
    TFTStruc.SVLT = storage.SolarVLTemp;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.SVLT);
    myNex.writeStr(F("page0.vaSVLT.txt"), buf);
    if (CurrentPage == 0 || CurrentPage == 7)  myNex.writeStr(F("SVLT.txt"), buf);
  }

  if (storage.SolarRLTemp != TFTStruc.SRLT || !refresh)
  {
    TFTStruc.SRLT = storage.SolarRLTemp;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.SRLT);
    myNex.writeStr(F("page0.vaSRLT.txt"), buf);
    if (CurrentPage == 0 || CurrentPage == 7)  myNex.writeStr(F("SRLT.txt"), buf);
  }

  if (storage.WaterTemp_SetPoint != TFTStruc.WTSP || !refresh)
  {
    TFTStruc.WTSP = storage.WaterTemp_SetPoint;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.WTSP);
    myNex.writeStr(F("page0.vaWSP.txt"), buf);
    if (CurrentPage == 2)  myNex.writeStr(F("WSP.txt"), buf);
  }

  if (storage.WaterTempLowThreshold != TFTStruc.WTLow || !refresh)
  {
    TFTStruc.WTLow = storage.WaterTempLowThreshold;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.WTLow);
    myNex.writeStr(F("page0.vaWTempLow.txt"), buf);
    if (CurrentPage == 14)  myNex.writeStr(F("WTempLow.txt"), buf);
  }

  if (storage.AirTemp != TFTStruc.AT || !refresh)
  {
    TFTStruc.AT = storage.AirTemp;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.AT);
    myNex.writeStr(F("page0.vaAT.txt"), buf);
    if (CurrentPage == 0 || CurrentPage == 3 || CurrentPage == 7)  myNex.writeStr(F("A.txt"), buf);
  }

  if (storage.AirHum != TFTStruc.AH || !refresh)
  {
    TFTStruc.AH = storage.AirHum;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.AH);
    myNex.writeStr(F("page0.vaAH.txt"), buf);
    if (CurrentPage == 0 || CurrentPage == 7)  myNex.writeStr(F("AH.txt"), buf);
  }

  if (storage.AirPress != TFTStruc.AP || !refresh)
  {
    TFTStruc.AP = storage.AirPress;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.AP);
    myNex.writeStr(F("page0.vaAP.txt"), buf);
    if (CurrentPage == 0 || CurrentPage == 7)  myNex.writeStr(F("AP.txt"), buf);
  }

  if (storage.SolarTemp != TFTStruc.ST || storage.SolarOnline != TFTStruc.SolarOnline || storage.SolarLocExt != TFTStruc.SolarLoEx || !refresh) {
    TFTStruc.ST = storage.SolarTemp;
    TFTStruc.SolarOnline = storage.SolarOnline;
    TFTStruc.SolarLoEx = storage.SolarLocExt;
    if (TFTStruc.SolarLoEx == 1 && !TFTStruc.SolarOnline)
      snprintf(buf, sizeof(buf), "Offline");
    else
      snprintf(buf, sizeof(buf), "%.1f", TFTStruc.ST);
    myNex.writeStr(F("page0.vaST.txt"), buf);
    if (CurrentPage == 0 || CurrentPage == 2 || CurrentPage == 7)  myNex.writeStr(F("S.txt"), buf);
  }

  if (storage.PSIValue != TFTStruc.PSI || !refresh)
  {
    TFTStruc.PSI = storage.PSIValue;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.PSI);
    int psiInt = TFTStruc.PSI * 100;
    int slider;
    myNex.writeStr(F("page0.vaPSI.txt"), buf);
    if      (psiInt >= 40 && psiInt <= 70)  slider = map(psiInt, 40, 70, 30, 70);
    else if (psiInt > 70  && psiInt <= 90)  slider = map(psiInt, 70, 90, 70, 90);
    else if (psiInt > 90)                   slider = 100;
    else if (psiInt > 10  && psiInt < 40)   slider = map(psiInt, 10, 40, 10, 30);
    else                                    slider = 0;
    myNex.writeNum(F("page0.PSL.val"), slider);
    if (CurrentPage == 0 || CurrentPage == 1 || CurrentPage == 8) {
      myNex.writeStr(F("P.txt"), buf);
      if (CurrentPage == 0) myNex.writeNum(F("PSL.val"), slider);
    }
  }

  if (storage.PSI_HighThreshold != TFTStruc.PsiH || !refresh)
  {
    TFTStruc.PsiH = storage.PSI_HighThreshold;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.PsiH);
    myNex.writeStr(F("page0.vaPsiH.txt"), buf);
    if (CurrentPage == 12)  myNex.writeStr(F("PsiH.txt"), buf);
  }

  if (storage.PSI_MedThreshold != TFTStruc.PsiL || !refresh)
  {
    TFTStruc.PsiL = storage.PSI_MedThreshold;
    snprintf(buf, sizeof(buf), "%.1f", TFTStruc.PsiL);
    myNex.writeStr(F("page0.vaPsiL.txt"), buf);
    if (CurrentPage == 12)  myNex.writeStr(F("PsiL.txt"), buf);
  }

  if (storage.SaltCurrentValue != TFTStruc.SaltCurrent_Raw || !refresh)
  {
    TFTStruc.SaltCurrent_Raw = storage.SaltCurrentValue;
    snprintf(buf, sizeof(buf), "%.2f", TFTStruc.SaltCurrent_Raw);
    myNex.writeStr(F("pageAmpCalib.vaSaltCur.txt"), buf);
    if (CurrentPage == 29)  myNex.writeStr(F("vaSaltCur.txt"), buf);
  }

  if (storage.FilterCurrentValue != TFTStruc.FilterCurrent_Raw || !refresh)
  {
    TFTStruc.FilterCurrent_Raw = storage.FilterCurrentValue;
    snprintf(buf, sizeof(buf), "%.2f", TFTStruc.FilterCurrent_Raw);
    myNex.writeStr(F("pageAmpCalib.vaFiltCur.txt"), buf);
    if (CurrentPage == 29)  myNex.writeStr(F("vaFiltCur.txt"), buf);
  }

  if (storage.HeatCurrentValue != TFTStruc.HeatCurrent_Raw || !refresh)
  {
    TFTStruc.HeatCurrent_Raw = storage.HeatCurrentValue;
    snprintf(buf, sizeof(buf), "%.2f", TFTStruc.HeatCurrent_Raw);
    myNex.writeStr(F("pageAmpCalib.vaHeatCur.txt"), buf);
    if (CurrentPage == 29)  myNex.writeStr(F("vaHeatCur.txt"), buf);
  }

  if (storage.FLOWValue != TFTStruc.flow || !refresh)
  {
    TFTStruc.flow = storage.FLOWValue;
    snprintf(buf, sizeof(buf), "%.0f", TFTStruc.flow);
    int flowInt = (int)TFTStruc.flow;
    int slider;
    myNex.writeStr(F("page0.vaF1.txt"), buf);
    if      (flowInt >= 50 && flowInt <= 90)  slider = map(flowInt, 50, 90, 30, 70);
    else if (flowInt > 90  && flowInt <= 100) slider = map(flowInt, 90, 100, 70, 90);
    else if (flowInt > 100)                   slider = 100;
    else if (flowInt > 30  && flowInt < 50)   slider = map(flowInt, 30, 50, 10, 30);
    else                                      slider = 0;
    myNex.writeNum(F("page0.F1SL.val"), slider);
    if (CurrentPage == 0 || CurrentPage == 1) {
      myNex.writeStr(F("F1.txt"), buf);
      if (CurrentPage == 0) myNex.writeNum(F("F1SL.val"), slider);
    }
  }

  if (storage.FLOW_Pulse != TFTStruc.FLOW_Pulse || !refresh)
  {
    TFTStruc.FLOW_Pulse = storage.FLOW_Pulse;
    snprintf(buf, sizeof(buf), "%u", TFTStruc.FLOW_Pulse);
    myNex.writeStr(F("page0.vaF1P.txt"), buf);
    if (CurrentPage == 12)  myNex.writeStr(F("F1P.txt"), buf);
  }

  if (storage.FLOW_HighThreshold != TFTStruc.F1H || !refresh)
  {
    TFTStruc.F1H = storage.FLOW_HighThreshold;
    snprintf(buf, sizeof(buf), "%.0f", TFTStruc.F1H);
    myNex.writeStr(F("page0.vaF1H.txt"), buf);
    if (CurrentPage == 12)  myNex.writeStr(F("F1H.txt"), buf);
  }

  if (storage.FLOW_MedThreshold != TFTStruc.F1L || !refresh)
  {
    TFTStruc.F1L = storage.FLOW_MedThreshold;
    snprintf(buf, sizeof(buf), "%.0f", TFTStruc.F1L);
    myNex.writeStr(F("page0.vaF1L.txt"), buf);
    if (CurrentPage == 12)  myNex.writeStr(F("F1L.txt"), buf);
  }

  if (storage.FLOW2Value != TFTStruc.flow2 || !refresh)
  {
    TFTStruc.flow2 = storage.FLOW2Value;
    snprintf(buf, sizeof(buf), "%.0f", TFTStruc.flow2);
    int flow2Int = (int)TFTStruc.flow2;
    int slider;
    myNex.writeStr(F("page0.vaF2.txt"), buf);
    if      (flow2Int >= 8  && flow2Int <= 20) slider = map(flow2Int, 8, 20, 30, 70);
    else if (flow2Int > 20  && flow2Int <= 30) slider = map(flow2Int, 20, 30, 70, 90);
    else if (flow2Int > 100)                   slider = 100;
    else if (flow2Int >= 5  && flow2Int < 8)   slider = map(flow2Int, 5, 8, 10, 30);
    else                                       slider = 0;
    myNex.writeNum(F("page0.F2SL.val"), slider);
    if (CurrentPage == 0) {
      myNex.writeStr(F("F2.txt"), buf);
      myNex.writeNum(F("F2SL.val"), slider);
    }
  }

  if (storage.FLOW2_Pulse != TFTStruc.FLOW2_Pulse || !refresh)
  {
    TFTStruc.FLOW2_Pulse = storage.FLOW2_Pulse;
    snprintf(buf, sizeof(buf), "%u", TFTStruc.FLOW2_Pulse);
    myNex.writeStr(F("page0.vaF2P.txt"), buf);
    if (CurrentPage == 12)  myNex.writeStr(F("F2P.txt"), buf);
  }

  if (storage.FLOW2_HighThreshold != TFTStruc.F2H || !refresh)
  {
    TFTStruc.F2H = storage.FLOW2_HighThreshold;
    snprintf(buf, sizeof(buf), "%.0f", TFTStruc.F2H);
    myNex.writeStr(F("page0.vaF2H.txt"), buf);
    if (CurrentPage == 12)  myNex.writeStr(F("F2H.txt"), buf);
  }

  if (storage.FLOW2_MedThreshold != TFTStruc.F2L || !refresh)
  {
    TFTStruc.F2L = storage.FLOW2_MedThreshold;
    snprintf(buf, sizeof(buf), "%.0f", TFTStruc.F2L);
    myNex.writeStr(F("page0.vaF2L.txt"), buf);
    if (CurrentPage == 12)  myNex.writeStr(F("F2L.txt"), buf);
  }

  if ((storage.FiltrationStop != TFTStruc.FSto) || (storage.FiltrationStart != TFTStruc.FSta) || !refresh)
  {
    TFTStruc.FSto = storage.FiltrationStop;
    TFTStruc.FSta = storage.FiltrationStart;
    char stasto[12];
    snprintf(stasto, sizeof(stasto), "%d/%dh", TFTStruc.FSta, TFTStruc.FSto);
    myNex.writeStr(F("page0.vaStaSto.txt"), stasto);
    if (CurrentPage >= 0 && CurrentPage <= 19) {
      char obj[18];
      snprintf(obj, sizeof(obj), "p%dStaSto.txt", CurrentPage);
      myNex.writeStr(obj, stasto);
    }
  }

  if (storage.FiltrationStartMin != TFTStruc.FStaT0 || !refresh)
  {
    TFTStruc.FStaT0 = storage.FiltrationStartMin;
    snprintf(buf, sizeof(buf), "%u", TFTStruc.FStaT0);
    myNex.writeStr(F("page0.vaFiltT0.txt"), buf);
    if (CurrentPage == 1)  myNex.writeStr(F("FiltT0.txt"), buf);
  }

  if (storage.FiltrationStopMax != TFTStruc.FStoT1 || !refresh)
  {
    TFTStruc.FStoT1 = storage.FiltrationStopMax;
    snprintf(buf, sizeof(buf), "%u", TFTStruc.FStoT1);
    myNex.writeStr(F("page0.vaFiltT1.txt"), buf);
    if (CurrentPage == 1)  myNex.writeStr(F("FiltT1.txt"), buf);
  }

  if (storage.SolarStartMin != TFTStruc.SStaT0 || !refresh)
  {
    TFTStruc.SStaT0 = storage.SolarStartMin;
    snprintf(buf, sizeof(buf), "%u", TFTStruc.SStaT0);
    myNex.writeStr(F("page0.vaSolT0.txt"), buf);
    if (CurrentPage == 2)  myNex.writeStr(F("SolT0.txt"), buf);
  }

  if (storage.SolarStopMax != TFTStruc.SStoT1 || !refresh)
  {
    TFTStruc.SStoT1 = storage.SolarStopMax;
    snprintf(buf, sizeof(buf), "%u", TFTStruc.SStoT1);
    myNex.writeStr(F("page0.vaSolT1.txt"), buf);
    if (CurrentPage == 2)  myNex.writeStr(F("SolT1.txt"), buf);
  }

  if ((ChlPump.UpTime != TFTStruc.OrpPpRT) || !refresh)
  {
    TFTStruc.OrpPpRT = ChlPump.UpTime;
    snprintf(buf, sizeof(buf), "%.1fmin", (float)TFTStruc.OrpPpRT / 60000.0f);
    myNex.writeStr(F("page0.vaOrpd.txt"), buf);
    if (CurrentPage == 0)  myNex.writeStr(F("Orpd.txt"), buf);
  }

  if (((int)ChlPump.GetTankFill() != TFTStruc.OrpTkFill) || !refresh)
  {
    TFTStruc.OrpTkFill = (int)round(ChlPump.GetTankFill());
    snprintf(buf, sizeof(buf), "%d", TFTStruc.OrpTkFill);
    myNex.writeStr(F("page0.vaOrpTk.txt"), buf);
    myNex.writeNum(F("page0.vaOrpPg.val"), TFTStruc.OrpTkFill);
    if (CurrentPage == 0) {
      myNex.writeStr(F("OrpTk.txt"), buf);
      myNex.writeNum(F("vaOrpPg.val"), TFTStruc.OrpTkFill);
    }
  }

  if ((PhPump.UpTime != TFTStruc.pHPpRT) || !refresh)
  {
    TFTStruc.pHPpRT = PhPump.UpTime;
    snprintf(buf, sizeof(buf), "%.1fmin", (float)TFTStruc.pHPpRT / 60000.0f);
    myNex.writeStr(F("page0.vapHd.txt"), buf);
    if (CurrentPage == 0)  myNex.writeStr(F("pHd.txt"), buf);
  }

  if (((int)PhPump.GetTankFill() != TFTStruc.pHTkFill) || !refresh)
  {
    TFTStruc.pHTkFill = (int)round(PhPump.GetTankFill());
    snprintf(buf, sizeof(buf), "%d", TFTStruc.pHTkFill);
    myNex.writeStr(F("page0.vapHTk.txt"), buf);
    myNex.writeNum(F("page0.vapHPg.val"), TFTStruc.pHTkFill);
    if (CurrentPage == 0) {
      myNex.writeStr(F("pHTk.txt"), buf);
      myNex.writeNum(F("vapHPg.val"), TFTStruc.pHTkFill);
    }
  }

  if (storage.AutoMode != TFTStruc.Mode || !refresh)
  {
    if ((debounceM == 0) || (debounceM > debounceCount))
    {
      debounceM = 0;
      TFTStruc.Mode = storage.AutoMode;
      const char* modeStr = TFTStruc.Mode ? "AUTO" : "MANU";
      myNex.writeNum(F("page1.vabMode.val"), storage.AutoMode);
      if (CurrentPage == 1) {
        myNex.writeStr(F("p1Mode.txt"), modeStr);
        myNex.writeStr(F("t1Mode.txt"), modeStr);
        myNex.writeStr(storage.AutoMode == 1 ? F("bMode.picc=9") : F("bMode.picc=8"));
      } else if (CurrentPage >= 0 && CurrentPage <= 19) {
        char obj[16];
        snprintf(obj, sizeof(obj), "p%dMode.txt", CurrentPage);
        myNex.writeStr(obj, modeStr);
      }
    }
    else
      debounceM++;
  }

  if (FiltrationPump.IsRunning() != TFTStruc.Filt || !refresh)
  {
    if ((debounceF == 0) || (debounceF > debounceCount))
    {
      debounceF = 0;
      TFTStruc.Filt = FiltrationPump.IsRunning();
      myNex.writeNum(F("page1.vabFilt.val"), TFTStruc.Filt);
      if (CurrentPage == 1)
      {
        if (TFTStruc.Filt == 1)
          myNex.writeStr(F("bFilt.picc=9"));
        else
          myNex.writeStr(F("bFilt.picc=8"));
      }
    }
    else
      debounceF++;
  }

  if (storage.SolarLocExt != TFTStruc.SolarLoEx || !refresh)
  {
    if ((debounceSolLE == 0) || (debounceSolLE > debounceCount))
    {
      debounceSolLE = 0;
      TFTStruc.SolarLoEx = storage.SolarLocExt;
      myNex.writeNum(F("page4.vabSolLoEx.val"), TFTStruc.SolarLoEx);
      if (CurrentPage == 18)
      {
        if (TFTStruc.SolarLoEx == 1)
        myNex.writeStr(F("bSolMode.picc=11"));
        else
        myNex.writeStr(F("bSolMode.picc=10"));
      }
    }
    else
      debounceSolLE++;
  }

  if (storage.HeatPumpMode != TFTStruc.HeatMode || !refresh)
  {
    if ((debounceHPM == 0) || (debounceHPM > debounceCount))
    {
      debounceHPM = 0;
      TFTStruc.HeatMode = storage.HeatPumpMode;
      myNex.writeNum(F("page3.vabHeatMode.val"), TFTStruc.HeatMode);
      if (CurrentPage == 3)
      {
        if (TFTStruc.HeatMode == 1)
          myNex.writeStr(F("page3.bHeatMode.picc=9"));
        else
          myNex.writeStr(F("page3.bHeatMode.picc=8"));
      }
    }
    else
      debounceHPM++;
  }
  
  if (storage.ValveMode != TFTStruc.ValveMode || !refresh)
  {
    if ((debounceVM == 0) || (debounceVM > debounceCount))
    {
      debounceVM = 0;
      TFTStruc.ValveMode = storage.ValveMode;
      myNex.writeNum(F("page4.vabValveMode.val"), TFTStruc.ValveMode);
      if (CurrentPage == 4)
      {
        if (TFTStruc.ValveMode == 1)
        myNex.writeStr(F("bValveMode.picc=15"));
        else
        myNex.writeStr(F("bValveMode.picc=14"));
      }
    }
    else
      debounceVM++;
  }

  if (storage.CleanMode != TFTStruc.CleanMode || !refresh)
  {
    if ((debounceCM == 0) || (debounceCM > debounceCount))
    {
      debounceCM = 0;
      TFTStruc.CleanMode = storage.CleanMode;
      myNex.writeNum(F("page4.vabCleanMode.val"), TFTStruc.CleanMode);
      if (CurrentPage == 4)
      {
        if (TFTStruc.CleanMode == 1)
        myNex.writeStr(F("bCleanMode.picc=15"));
        else
        myNex.writeStr(F("bCleanMode.picc=14"));
      }
    }
    else
      debounceCM++;
  }

  if (storage.ValveSwitch != TFTStruc.ValveSwitch || !refresh)
  {
    if ((debounceVS == 0) || (debounceVS > debounceCount))
    {
      debounceVS = 0;
      TFTStruc.ValveSwitch = storage.ValveSwitch;
      myNex.writeNum(F("page4.vabCleanDir.val"), TFTStruc.ValveSwitch);
      if (CurrentPage == 4)
      {
        if (TFTStruc.ValveSwitch == 1)
        myNex.writeStr(F("bCleanDir.picc=15"));
        else
        myNex.writeStr(F("bCleanDir.picc=14"));
      }
    }
    else
      debounceVS++;
  }

  if (storage.WaterFillMode != TFTStruc.WaterFillMode || !refresh)
  {
    if ((debounceWFM == 0) || (debounceWFM > debounceCount))
    {
      debounceWFM = 0;
      TFTStruc.WaterFillMode = storage.WaterFillMode;
      myNex.writeNum(F("page10.vabFillMode.val"), TFTStruc.WaterFillMode);
      if (CurrentPage == 10)
      {
        if (TFTStruc.WaterFillMode == 1)
        myNex.writeStr(F("bFillMode.picc=27"));
        else
        myNex.writeStr(F("bFillMode.picc=26"));
      }
    }
    else
      debounceWFM++;
  }

  if (ELD_Treppe.getStatus() != TFTStruc.ELDTstate || !refresh)
  {
    TFTStruc.ELDTstate = ELD_Treppe.getStatus();
    const char* s = TFTStruc.ELDTstate.c_str();
    myNex.writeStr(F("page4.vaELDTstate.txt"), s);
    if (CurrentPage == 4)
    {
      myNex.writeStr(F("ELDTstate.txt"), s);
      if      (TFTStruc.ELDTstate == "AUF")  { myNex.writeStr(F("bValNoT.picc=14")); myNex.writeStr(F("ELDTstate.picc=14")); myNex.writeStr(F("ELDTstate.pco=65535")); }
      else if (TFTStruc.ELDTstate == "HALB") { myNex.writeStr(F("bValNoT.picc=16")); myNex.writeStr(F("ELDTstate.picc=16")); myNex.writeStr(F("ELDTstate.pco=0")); }
      else if (TFTStruc.ELDTstate == "ZU")   { myNex.writeStr(F("bValNoT.picc=15")); myNex.writeStr(F("ELDTstate.picc=15")); myNex.writeStr(F("ELDTstate.pco=0")); }
      else if (TFTStruc.ELDTstate == "öffne" || TFTStruc.ELDTstate == "schließe") { myNex.writeStr(F("bValNoT.picc=14")); myNex.writeStr(F("ELDTstate.picc=14")); myNex.writeStr(F("ELDTstate.pco=2016")); }
      else if (TFTStruc.ELDTstate == "calibr...") { myNex.writeStr(F("bValNoT.picc=14")); myNex.writeStr(F("ELDTstate.picc=14")); myNex.writeStr(F("ELDTstate.pco=63488")); }
      else if (ELD_Treppe.CurrentAngle() <= (ELD_Treppe.StartAngle() + 5)) { myNex.writeStr(F("bValNoT.picc=15")); myNex.writeStr(F("ELDTstate.picc=15")); myNex.writeStr(F("ELDTstate.pco=0")); }
      else if (ELD_Treppe.CurrentAngle() <= (ELD_Treppe.HalfAngle() + (ELD_Treppe.MaxAngle() - ELD_Treppe.HalfAngle()) / 2)) { myNex.writeStr(F("bValNoT.picc=16")); myNex.writeStr(F("ELDTstate.picc=16")); myNex.writeStr(F("ELDTstate.pco=0")); }
      else { myNex.writeStr(F("bValNoT.picc=14")); myNex.writeStr(F("ELDTstate.picc=14")); myNex.writeStr(F("ELDTstate.pco=65535")); }
    }
  }

  if (ELD_Hinten.getStatus() != TFTStruc.ELDHstate || !refresh)
  {
    TFTStruc.ELDHstate = ELD_Hinten.getStatus();
    const char* s = TFTStruc.ELDHstate.c_str();
    myNex.writeStr(F("page4.vaELDHstate.txt"), s);
    if (CurrentPage == 4)
    {
      myNex.writeStr(F("ELDHstate.txt"), s);
      if      (TFTStruc.ELDHstate == "AUF")  { myNex.writeStr(F("bValNoH.picc=14")); myNex.writeStr(F("ELDHstate.picc=14")); myNex.writeStr(F("ELDHstate.pco=65535")); }
      else if (TFTStruc.ELDHstate == "HALB") { myNex.writeStr(F("bValNoH.picc=16")); myNex.writeStr(F("ELDHstate.picc=16")); myNex.writeStr(F("ELDHstate.pco=0")); }
      else if (TFTStruc.ELDHstate == "ZU")   { myNex.writeStr(F("bValNoH.picc=15")); myNex.writeStr(F("ELDHstate.picc=15")); myNex.writeStr(F("ELDHstate.pco=0")); }
      else if (TFTStruc.ELDHstate == "öffne" || TFTStruc.ELDHstate == "schließe") { myNex.writeStr(F("bValNoH.picc=14")); myNex.writeStr(F("ELDHstate.picc=14")); myNex.writeStr(F("ELDHstate.pco=2016")); }
      else if (TFTStruc.ELDHstate == "calibr...") { myNex.writeStr(F("bValNoH.picc=14")); myNex.writeStr(F("ELDHstate.picc=14")); myNex.writeStr(F("ELDHstate.pco=63488")); }
      else if (ELD_Hinten.CurrentAngle() <= (ELD_Hinten.StartAngle() + 5)) { myNex.writeStr(F("bValNoH.picc=15")); myNex.writeStr(F("ELDHstate.picc=15")); myNex.writeStr(F("ELDHstate.pco=0")); }
      else if (ELD_Hinten.CurrentAngle() <= (ELD_Hinten.HalfAngle() + (ELD_Hinten.MaxAngle() - ELD_Hinten.HalfAngle()) / 2)) { myNex.writeStr(F("bValNoH.picc=16")); myNex.writeStr(F("ELDHstate.picc=16")); myNex.writeStr(F("ELDHstate.pco=0")); }
      else { myNex.writeStr(F("bValNoH.picc=14")); myNex.writeStr(F("ELDHstate.picc=14")); myNex.writeStr(F("ELDHstate.pco=65535")); }
    }
  }

  if (WP_Vorlauf.getStatus() != TFTStruc.WPVstate || !refresh)
  {
    TFTStruc.WPVstate = WP_Vorlauf.getStatus();
    const char* s = TFTStruc.WPVstate.c_str();
    myNex.writeStr(F("page4.vaWPVstate.txt"), s);
    if (CurrentPage == 4)
    {
      myNex.writeStr(F("WPVstate.txt"), s);
      if      (TFTStruc.WPVstate == "AUF")  { myNex.writeStr(F("bValWPV.picc=14")); myNex.writeStr(F("WPVstate.picc=14")); myNex.writeStr(F("WPVstate.pco=65535")); }
      else if (TFTStruc.WPVstate == "HALB") { myNex.writeStr(F("bValWPV.picc=16")); myNex.writeStr(F("WPVstate.picc=16")); myNex.writeStr(F("WPVstate.pco=0")); }
      else if (TFTStruc.WPVstate == "ZU")   { myNex.writeStr(F("bValWPV.picc=15")); myNex.writeStr(F("WPVstate.picc=15")); myNex.writeStr(F("WPVstate.pco=0")); }
      else if (TFTStruc.WPVstate == "öffne" || TFTStruc.WPVstate == "schließe") { myNex.writeStr(F("bValWPV.picc=14")); myNex.writeStr(F("WPVstate.picc=14")); myNex.writeStr(F("WPVstate.pco=2016")); }
      else if (TFTStruc.WPVstate == "calibr...") { myNex.writeStr(F("bValWPV.picc=14")); myNex.writeStr(F("WPVstate.picc=14")); myNex.writeStr(F("WPVstate.pco=63488")); }
      else if (WP_Vorlauf.CurrentAngle() <= (WP_Vorlauf.StartAngle() + 5)) { myNex.writeStr(F("bValWPV.picc=15")); myNex.writeStr(F("WPVstate.picc=15")); myNex.writeStr(F("WPVstate.pco=0")); }
      else if (WP_Vorlauf.CurrentAngle() <= (WP_Vorlauf.HalfAngle() + (WP_Vorlauf.MaxAngle() - WP_Vorlauf.HalfAngle()) / 2)) { myNex.writeStr(F("bValWPV.picc=16")); myNex.writeStr(F("WPVstate.picc=16")); myNex.writeStr(F("WPVstate.pco=0")); }
      else { myNex.writeStr(F("bValWPV.picc=14")); myNex.writeStr(F("WPVstate.picc=14")); myNex.writeStr(F("WPVstate.pco=65535")); }
    }
  }

  if (WP_Mischer.getStatus() != TFTStruc.WPMstate || !refresh)
  {
    TFTStruc.WPMstate = WP_Mischer.getStatus();
    const char* s = TFTStruc.WPMstate.c_str();
    myNex.writeStr(F("page4.vaWPMstate.txt"), s);
    if (CurrentPage == 4)
    {
      myNex.writeStr(F("WPMstate.txt"), s);
      if      (TFTStruc.WPMstate == "AUF")  { myNex.writeStr(F("bValWPM.picc=14")); myNex.writeStr(F("WPMstate.picc=14")); myNex.writeStr(F("WPMstate.pco=65535")); }
      else if (TFTStruc.WPMstate == "HALB") { myNex.writeStr(F("bValWPM.picc=16")); myNex.writeStr(F("WPMstate.picc=16")); myNex.writeStr(F("WPMstate.pco=0")); }
      else if (TFTStruc.WPMstate == "ZU")   { myNex.writeStr(F("bValWPM.picc=15")); myNex.writeStr(F("WPMstate.picc=15")); myNex.writeStr(F("WPMstate.pco=0")); }
      else if (TFTStruc.WPMstate == "öffne" || TFTStruc.WPMstate == "schließe") { myNex.writeStr(F("bValWPM.picc=14")); myNex.writeStr(F("WPMstate.picc=14")); myNex.writeStr(F("WPMstate.pco=2016")); }
      else if (TFTStruc.WPMstate == "calibr...") { myNex.writeStr(F("bValWPM.picc=14")); myNex.writeStr(F("WPMstate.picc=14")); myNex.writeStr(F("WPMstate.pco=63488")); }
      else if (WP_Mischer.CurrentAngle() <= (WP_Mischer.StartAngle() + 5)) { myNex.writeStr(F("bValWPM.picc=15")); myNex.writeStr(F("WPMstate.picc=15")); myNex.writeStr(F("WPMstate.pco=0")); }
      else if (WP_Mischer.CurrentAngle() <= (WP_Mischer.HalfAngle() + (WP_Mischer.MaxAngle() - WP_Mischer.HalfAngle()) / 2)) { myNex.writeStr(F("bValWPM.picc=16")); myNex.writeStr(F("WPMstate.picc=16")); myNex.writeStr(F("WPMstate.pco=0")); }
      else { myNex.writeStr(F("bValWPM.picc=14")); myNex.writeStr(F("WPMstate.picc=14")); myNex.writeStr(F("WPMstate.pco=65535")); }
    }
  }

  if (Bodenablauf.getStatus() != TFTStruc.BOTTstate || !refresh)
  {
    TFTStruc.BOTTstate = Bodenablauf.getStatus();
    const char* s = TFTStruc.BOTTstate.c_str();
    myNex.writeStr(F("page4.vaBOTTstate.txt"), s);
    if (CurrentPage == 4)
    {
      myNex.writeStr(F("BOTTstate.txt"), s);
      if      (TFTStruc.BOTTstate == "AUF")  { myNex.writeStr(F("bValBott.picc=14")); myNex.writeStr(F("BOTTstate.picc=14")); myNex.writeStr(F("BOTTstate.pco=65535")); }
      else if (TFTStruc.BOTTstate == "HALB") { myNex.writeStr(F("bValBott.picc=16")); myNex.writeStr(F("BOTTstate.picc=16")); myNex.writeStr(F("BOTTstate.pco=0")); }
      else if (TFTStruc.BOTTstate == "ZU")   { myNex.writeStr(F("bValBott.picc=15")); myNex.writeStr(F("BOTTstate.picc=15")); myNex.writeStr(F("BOTTstate.pco=0")); }
      else if (TFTStruc.BOTTstate == "öffne" || TFTStruc.BOTTstate == "schließe") { myNex.writeStr(F("bValBott.picc=14")); myNex.writeStr(F("BOTTstate.picc=14")); myNex.writeStr(F("BOTTstate.pco=2016")); }
      else if (TFTStruc.BOTTstate == "calibr...") { myNex.writeStr(F("bValBott.picc=14")); myNex.writeStr(F("BOTTstate.picc=14")); myNex.writeStr(F("BOTTstate.pco=63488")); }
      else if (Bodenablauf.CurrentAngle() <= (Bodenablauf.StartAngle() + 5)) { myNex.writeStr(F("bValBott.picc=15")); myNex.writeStr(F("BOTTstate.picc=15")); myNex.writeStr(F("BOTTstate.pco=0")); }
      else if (Bodenablauf.CurrentAngle() <= (Bodenablauf.HalfAngle() + (Bodenablauf.MaxAngle() - Bodenablauf.HalfAngle()) / 2)) { myNex.writeStr(F("bValBott.picc=16")); myNex.writeStr(F("BOTTstate.picc=16")); myNex.writeStr(F("BOTTstate.pco=0")); }
      else { myNex.writeStr(F("bValBott.picc=14")); myNex.writeStr(F("BOTTstate.picc=14")); myNex.writeStr(F("BOTTstate.pco=65535")); }
    }
  }

  {
    // Solar valve: track full MotorValve status string for vaSOLVstate / SOLARstate display
    std::string currentSOLARstate = (storage.SolarLocExt == 0)
        ? std::string(Solarvalve.getStatus())
        : (storage.ValveStatus ? std::string("AUF") : std::string("ZU"));
    if (currentSOLARstate != TFTStruc.SOLARstate || !refresh)
    {
      if ((debounceSolV == 0) || (debounceSolV > debounceCount))
      {
        debounceSolV = 0;
        TFTStruc.SOLARstate = currentSOLARstate;
        bool currentValveState = (storage.SolarLocExt == 0) ? Solarvalve.isOpen() : storage.ValveStatus;
        TFTStruc.SolarValve = currentValveState;
        myNex.writeNum(F("page2.vabSolVal.val"), TFTStruc.SolarValve);
        const char* s = TFTStruc.SOLARstate.c_str();
        myNex.writeStr(F("page2.vaSOLVstate.txt"), s);
        if (CurrentPage == 2)
        {
          myNex.writeStr(F("SOLARstate.txt"), s);
          if (storage.SolarLocExt == 0)
          {
            if      (TFTStruc.SOLARstate == "AUF")  { myNex.writeStr(F("bSolVal.picc=47")); myNex.writeStr(F("SOLARstate.picc=47")); myNex.writeStr(F("SOLARstate.pco=65535")); }
            else if (TFTStruc.SOLARstate == "HALB") { myNex.writeStr(F("bSolVal.picc=48")); myNex.writeStr(F("SOLARstate.picc=48")); myNex.writeStr(F("SOLARstate.pco=0")); }
            else if (TFTStruc.SOLARstate == "ZU")   { myNex.writeStr(F("bSolVal.picc=48")); myNex.writeStr(F("SOLARstate.picc=48")); myNex.writeStr(F("SOLARstate.pco=0")); }
            else if (TFTStruc.SOLARstate == "öffne" || TFTStruc.SOLARstate == "schließe") { myNex.writeStr(F("bSolVal.picc=47")); myNex.writeStr(F("SOLARstate.picc=47")); myNex.writeStr(F("SOLARstate.pco=2016")); }
            else if (TFTStruc.SOLARstate == "calibr...") { myNex.writeStr(F("bSolVal.picc=47")); myNex.writeStr(F("SOLARstate.picc=47")); myNex.writeStr(F("SOLARstate.pco=63488")); }
          }
          else
          {
            myNex.writeStr(TFTStruc.SolarValve ? F("bSolVal.picc=11") : F("bSolVal.picc=10"));
          }
        }
      }
      else
        debounceSolV++;
    }
  }

  if (storage.Salt_Chlor != TFTStruc.Salt_Chlor || !refresh)
  {
    if ((debounceSC == 0) || (debounceSC > debounceCount))
    {
      debounceSC = 0;
      TFTStruc.Salt_Chlor = storage.Salt_Chlor;
      myNex.writeNum(F("page13.vabSaltMode.val"), TFTStruc.Salt_Chlor);
      if (CurrentPage == 0)
      {
        if (TFTStruc.Salt_Chlor == 1)
        myNex.writeStr(F("b14.picc=5"));
        else
        myNex.writeStr(F("b14.picc=6"));
      }
      if (CurrentPage == 13)
      {
        if (TFTStruc.Salt_Chlor == 1)
          myNex.writeStr(F("bSaltMode.picc=41"));
        else
          myNex.writeStr(F("bSaltMode.picc=40"));
      }
    }
    else
      debounceSC++;
  }

  if (storage.SaltMode != TFTStruc.SaltMode || !refresh)
  {
    if ((debounceSM == 0) || (debounceSM > debounceCount))
    {
      debounceSM = 0;
      TFTStruc.SaltMode = storage.SaltMode;
      myNex.writeNum(F("page13.vabSaltMode.val"), TFTStruc.SaltMode);
      if (CurrentPage == 0)
      {
        if (TFTStruc.SaltMode == 1)
        myNex.writeStr(F("b14.picc=5"));
        else
        myNex.writeStr(F("b14.picc=6"));
      }
      if (CurrentPage == 13)
      {
        if (TFTStruc.SaltMode == 1)
          myNex.writeStr(F("bSaltMode.picc=41"));
        else
          myNex.writeStr(F("bSaltMode.picc=40"));
      }
    }
    else
      debounceSM++;
  }

  if (SaltPump.IsRunning() != TFTStruc.SaltPump || !refresh)
  {
    if ((debounceSP == 0) || (debounceSP > debounceCount))
    {
      debounceSP = 0;
      TFTStruc.SaltPump = SaltPump.IsRunning();
      myNex.writeNum(F("page13.vabSaltPum.val"), TFTStruc.SaltPump);
      if (CurrentPage == 13)
      {
        if (TFTStruc.SaltPump == 1)
          myNex.writeStr(F("bSaltPum.picc=41"));
        else
          myNex.writeStr(F("bSaltPum.picc=40"));
      }
    }
    else
      debounceSP++;
  }

    {
      bool currentPumpState = (storage.SolarLocExt == 0) ? SolarPump.IsRunning() : storage.SolarPumpStatus;
      if (currentPumpState != TFTStruc.SolarPump || !refresh)
      {
          if ((debounceSolP == 0) || (debounceSolP > debounceCount))
          {
              debounceSolP = 0;
              TFTStruc.SolarPump = currentPumpState;
              myNex.writeNum(F("page2.vabSolPum.val"), TFTStruc.SolarPump);
              if (CurrentPage == 2)
              {
                  if (TFTStruc.SolarPump == 1)
                      myNex.writeStr(F("bSolPum.picc=11"));
                  else
                      myNex.writeStr(F("bSolPum.picc=10"));
              }
          }
          else
              debounceSolP++;
      }
  }

  // Salt status cyclic display
  if (millis() - lastSaltUpdate >= saltDisplayInterval) {
    char saltBuf[32];
    bool useRedColor = false;

    switch (saltDisplayState) {
      case 0: // Polarity
          snprintf(saltBuf, sizeof(saltBuf), "%s", storage.SaltPolarity == POLARITY_DIRECT ? "DIREKT" : "VERPOLT");
          break;
      case 1: // Salt Current
          snprintf(saltBuf, sizeof(saltBuf), " %.1fA", storage.SaltCurrentValue);
          break;
      case 2: // Leistung
      {
          float power = storage.SaltCurrentValue * ELECTROLYSIS_VOLTAGE;
          snprintf(saltBuf, sizeof(saltBuf), " %.0fW", power);
          break;
      }
      case 3: // Salt Status
          if (storage.SaltStatus == "LOW Salt") {
              snprintf(saltBuf, sizeof(saltBuf), "+ %.0f kg Salz", storage.SaltNeeded);
              useRedColor = true;
          } else if (storage.SaltStatus == "HIGH Salt") {
              snprintf(saltBuf, sizeof(saltBuf), "HIGH Salt");
              useRedColor = true;
          } else if (storage.SaltStatus == "OK") {
              snprintf(saltBuf, sizeof(saltBuf), "Salt: OK");
              saltDisplayState = -1; // Skip to polarity next
          } else {
              snprintf(saltBuf, sizeof(saltBuf), "%s", TFTStruc.SaltStatus.length() == 0 ? "...Wait" : TFTStruc.SaltStatus.c_str());
              useRedColor = (TFTStruc.SaltStatus == "LOW Salt" || TFTStruc.SaltStatus == "HIGH Salt");
              Debug.print(DBG_VERBOSE, "[Nextion] Skipping Unknown, using: %s", saltBuf);
              saltDisplayState = -1; // Skip to polarity
          }
          TFTStruc.SaltStatus = saltBuf;
          break;
      default:
          saltBuf[0] = '\0';
          break;
  }

    // Update display only if text or color has changed
    if (TFTStruc.SaltStatus != saltBuf || !refresh) {
      TFTStruc.SaltStatus = saltBuf;
      myNex.writeStr(F("page0.vaSaltDir.txt"), saltBuf);
      myNex.writeNum(F("page0.vaSaltDir.pco"), useRedColor ? 63488 : 65535);
      if (CurrentPage == 13) {
        myNex.writeStr(F("SaltDir.txt"), saltBuf);
        myNex.writeNum(F("SaltDir.pco"), useRedColor ? 63488 : 65535);
      }
      Debug.print(DBG_VERBOSE, "Updated salt display: %s, color: %d", saltBuf, useRedColor ? 63488 : 65535);
    }

    // Update salt current
    if (storage.SaltCurrentValue != TFTStruc.SaltCurrentValue || !refresh) {
      TFTStruc.SaltCurrentValue = storage.SaltCurrentValue;
      Debug.print(DBG_VERBOSE, "Updated salt current: %.2f A", storage.SaltCurrentValue);
    }

    // Update salt needed
    if (storage.SaltNeeded != TFTStruc.SaltNeeded || !refresh) {
      TFTStruc.SaltNeeded = storage.SaltNeeded;
      Debug.print(DBG_VERBOSE, "Updated salt needed: %.1f kg", storage.SaltNeeded);
    }

    // Cycle to next state
    saltDisplayState = (saltDisplayState + 1) % (storage.SaltStatus == "OK" || storage.SaltStatus == "Unknown" ? 3 : 4);
    lastSaltUpdate = millis();
    }

  if (storage.SaltDiff != TFTStruc.SaltDiff || !refresh)
  {
    TFTStruc.SaltDiff = storage.SaltDiff;
    snprintf(buf, sizeof(buf), "%d", TFTStruc.SaltDiff);
    myNex.writeStr(F("page0.vaSaltDiff.txt"), buf);
    if (CurrentPage == 13)  myNex.writeStr(F("SaltDiff.txt"), buf);
  }

  if ((FiltrationPump.UpTime != TFTStruc.FLRT) || !refresh)
  {
    TFTStruc.FLRT = FiltrationPump.UpTime;
    char tbuf[10];
    fmtUptime(tbuf, sizeof(tbuf), TFTStruc.FLRT);
    myNex.writeStr(F("page0.vaFiltDur.txt"), tbuf);
    if (CurrentPage == 1)  myNex.writeStr(F("FiltDur.txt"), tbuf);
  }

  float filterPower = storage.FilterCurrentValue * FILTER_VOLTAGE;
  if ((filterPower != TFTStruc.FiltPower) || !refresh) {
    TFTStruc.FiltPower = filterPower;
    char tbuf[12];
    fmtThousands(tbuf, sizeof(tbuf), (int)filterPower);
    myNex.writeStr(F("page0.vawFilt.txt"), tbuf);
    if (CurrentPage == 1) myNex.writeStr(F("page1.wFilt.txt"), tbuf);
  }

  if ((WaterFill.UpTime != TFTStruc.WFRT) || !refresh)
  {
    TFTStruc.WFRT = WaterFill.UpTime;
    char tbuf[10];
    fmtUptime(tbuf, sizeof(tbuf), TFTStruc.WFRT);
    myNex.writeStr(F("page0.vaWFDur.txt"), tbuf);
    if (CurrentPage == 10)  myNex.writeStr(F("WFDur.txt"), tbuf);
  }

  if ((storage.WaterFillAnCon != TFTStruc.WFAC) || !refresh)
  {
    TFTStruc.WFAC = storage.WaterFillAnCon;
    char tbuf[12];
    fmtThousands(tbuf, sizeof(tbuf), (int)TFTStruc.WFAC);
    myNex.writeStr(F("page0.vaWFAnCon.txt"), tbuf);
    if (CurrentPage == 10)  myNex.writeStr(F("WFAnCon.txt"), tbuf);
  }

  if ((SaltPump.UpTime != TFTStruc.SPUT) || !refresh)
  {
    TFTStruc.SPUT = SaltPump.UpTime;
    char tbuf[10];
    fmtUptime(tbuf, sizeof(tbuf), TFTStruc.SPUT);
    myNex.writeStr(F("page0.vaSaltDur.txt"), tbuf);
    if (CurrentPage == 13)  myNex.writeStr(F("SaltDur.txt"), tbuf);
  }

  if ((storage.SaltPumpRunTime != TFTStruc.SPRT) || !refresh)
  {
    TFTStruc.SPRT = storage.SaltPumpRunTime;
    char tbuf[10];
    fmtUptime(tbuf, sizeof(tbuf), TFTStruc.SPRT);
    myNex.writeStr(F("page0.vaSaltRT.txt"), tbuf);
    if (CurrentPage == 13)  myNex.writeStr(F("SaltRT.txt"), tbuf);
  }

  if (PhPump.IsRunning() != TFTStruc.PhPump || !refresh)
  {
    if ((debouncepH == 0) || (debouncepH > debounceCount))
    {
      debouncepH = 0;
      TFTStruc.PhPump = PhPump.IsRunning();
      myNex.writeNum(F("page15.vabpHPum.val"), TFTStruc.PhPump);
      if (CurrentPage == 15)
      {
        if (TFTStruc.PhPump == 1)
          myNex.writeStr(F("bpHPum.picc=37"));
        else
          myNex.writeStr(F("bpHPum.picc=36"));
      }
    }
    else
      debouncepH++;
  }

  if (ChlPump.IsRunning() != TFTStruc.ChlPump || !refresh)
  {
    if ((debounceChl == 0) || (debounceChl > debounceCount))
    {
      debounceChl = 0;
      TFTStruc.ChlPump = ChlPump.IsRunning();
      myNex.writeNum(F("vabChlPum.val"), TFTStruc.ChlPump);
      if (CurrentPage == 16)
      {
        if (TFTStruc.ChlPump == 1)
          myNex.writeStr(F("bChlPum.picc=33"));
        else
          myNex.writeStr(F("bChlPum.picc=32"));
      }
    }
    else
      debounceChl++;
  }

  if (RobotPump.IsRunning() != TFTStruc.Robot || !refresh)
  {
    if ((debounceH == 0) || (debounceH > debounceCount))
    {
      debounceH = 0;
      TFTStruc.Robot = RobotPump.IsRunning();
      myNex.writeNum(F("page5.vabRobot.val"), TFTStruc.Robot);
      if (CurrentPage == 5)
      {
        if (TFTStruc.Robot == 1)
          myNex.writeStr(F("bRobot.picc=18"));
        else
          myNex.writeStr(F("bRobot.picc=17"));
      }
    }
    else
      debounceH++;
  }

  if (HeatPump.IsRunning() != TFTStruc.HeatPump || !refresh)
  {
    if ((debounceHP == 0) || (debounceHP > debounceCount))
    {
      debounceHP = 0;
      TFTStruc.HeatPump = HeatPump.IsRunning();
      myNex.writeNum(F("page3.vabHeatPum.val"), TFTStruc.HeatPump);
      if (CurrentPage == 3)
      {
        if (TFTStruc.HeatPump == 1)
          myNex.writeStr(F("bHeatPum.picc=13"));
        else
          myNex.writeStr(F("bHeatPum.picc=12"));
      }
    }
    else
      debounceHP++;
  }

  if (WaterFill.IsRunning() != TFTStruc.WaterFill || !refresh)
  {
    if ((debounceWF == 0) || (debounceWF > debounceCount))
    {
      debounceWF = 0;
      TFTStruc.WaterFill = WaterFill.IsRunning();
      myNex.writeNum(F("page10.vabTap.val"), TFTStruc.WaterFill);
      if (CurrentPage == 10)
      {
        if (TFTStruc.WaterFill == 1)
          myNex.writeStr(F("bTap.picc=27"));
        else
          myNex.writeStr(F("bTap.picc=26"));
      }
    }
    else
      debounceWF++;
  }
  
  if ((HeatPump.UpTime != TFTStruc.HPRT) || !refresh)
  {
    TFTStruc.HPRT = HeatPump.UpTime;
    char tbuf[10];
    fmtUptime(tbuf, sizeof(tbuf), TFTStruc.HPRT);
    myNex.writeStr(F("page0.vaWPDur.txt"), tbuf);
    if (CurrentPage == 3)  myNex.writeStr(F("WPDur.txt"), tbuf);
  }

  float heatPower = storage.HeatCurrentValue * HEAT_VOLTAGE;
  if ((heatPower != TFTStruc.HeatPower) || !refresh) {
      TFTStruc.HeatPower = heatPower;
      char hbuf[12];
      fmtThousands(hbuf, sizeof(hbuf), (int)heatPower);
      myNex.writeStr(F("page0.vaWWP.txt"), hbuf);
      if (CurrentPage == 3) myNex.writeStr(F("page3.WWP.txt"), hbuf);
  }

  if ((SolarPump.UpTime != TFTStruc.SHRT) || !refresh)
  {
    TFTStruc.SHRT = SolarPump.UpTime;
    char tbuf[10];
    fmtUptime(tbuf, sizeof(tbuf), TFTStruc.SHRT);
    myNex.writeStr(F("page0.vaSolDur.txt"), tbuf);
    if (CurrentPage == 2)  myNex.writeStr(F("SolDur.txt"), tbuf);
  }

  if (digitalRead(RELAY_R0) != TFTStruc.R0 || !refresh)
  {
    if ((debounceR0 == 0) || (debounceR0 > debounceCount))
    {
      debounceR0 = 0;
      TFTStruc.R0 = digitalRead(RELAY_R0);
      myNex.writeNum(F("page1.vabR0.val"), !TFTStruc.R0);
      if (CurrentPage == 1)
      {
        if (TFTStruc.R0 == 0)
          myNex.writeStr(F("bR0.picc=8"));
        else
          myNex.writeStr(F("bR0.picc=7"));
      }
    }
    else
      debounceR0++;
  }

  if (digitalRead(RELAY_R1) != TFTStruc.R1 || !refresh)
  {
    if ((debounceR1 == 0) || (debounceR1 > debounceCount))
    {
      debounceR1 = 0;
      TFTStruc.R1 = digitalRead(RELAY_R1);
      myNex.writeNum(F("page1.vabR1.val"), !TFTStruc.R1);
      if (CurrentPage == 1)
      {
        if (TFTStruc.R1 == 0)
          myNex.writeStr(F("bR1.picc=8"));
        else
          myNex.writeStr(F("bR1.picc=7"));
      }
    }
    else
      debounceR1++;
  }

  if (storage.WinterMode != TFTStruc.R2 || !refresh)
  {
    if ((debounceR2 == 0) || (debounceR2 > debounceCount))
    {
      debounceR2 = 0;
      TFTStruc.R2 = storage.WinterMode;
      myNex.writeNum(F("page1.vabWinMode.val"), TFTStruc.R2);
      if (CurrentPage == 1)
      {
        if (TFTStruc.R2 == 1)
          myNex.writeStr(F("bWinMode.picc=9"));
        else
          myNex.writeStr(F("WinMode.picc=8"));
      }
    }
    else
      debounceR2++;
  }

  if (ChlPump.TankLevel() != TFTStruc.ChlTLErr || !refresh)
  {
    TFTStruc.ChlTLErr = ChlPump.TankLevel();
    if (!TFTStruc.ChlTLErr)
    {
      myNex.writeStr(F("page0.vaChlLevel.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaChlLevel.val=0"));
  }

  if (PhPump.TankLevel() != TFTStruc.pHTLErr || !refresh)
  {
    TFTStruc.pHTLErr = PhPump.TankLevel();
    if (!TFTStruc.pHTLErr)
    {
      myNex.writeStr(F("page0.vaAcidLevel.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaAcidLevel.val=0"));
  }

  if (digitalRead(WATER_MAX_LVL) != TFTStruc.WFMaxLvl || !refresh)
  {
    TFTStruc.WFMaxLvl = digitalRead(WATER_MAX_LVL);
    myNex.writeNum(F("page0.vabValLvMax.val"), !TFTStruc.WFMaxLvl);
  }

  if (digitalRead(WATER_MIN_LVL) != TFTStruc.WFMinLvl || !refresh)
  {
    TFTStruc.WFMinLvl = digitalRead(WATER_MIN_LVL);
    myNex.writeNum(F("page0.vabValLvMin.val"), !TFTStruc.WFMinLvl);
  }

  if (PSIError != TFTStruc.PSIErr || !refresh)
  {
    TFTStruc.PSIErr = PSIError;
    if (TFTStruc.PSIErr)
    {
      myNex.writeStr(F("page0.vaPSIErr.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaPSIErr.val=0"));
  }

  if (FLOWError != TFTStruc.FLOWErr || !refresh)
  {
    TFTStruc.FLOWErr = FLOWError;
    if (TFTStruc.FLOWErr)
    {
      myNex.writeStr(F("page0.vaFLOWErr.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaFLOWErr.val=0"));
  }

  if (FLOW2Error != TFTStruc.FLOW2Err || !refresh)
  {
    TFTStruc.FLOW2Err = FLOW2Error;
    if (TFTStruc.FLOW2Err)
    {
      myNex.writeStr(F("page0.vaFLOW2Err.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaFLOW2Err.val=0"));
  }

  if (ChlPump.UpTimeError != TFTStruc.ChlUTErr || !refresh)
  {
    TFTStruc.ChlUTErr = ChlPump.UpTimeError;
    if (TFTStruc.ChlUTErr)
    {
      myNex.writeStr(F("page0.vaChlUTErr.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaChlUTErr.val=0"));
  }

  if (PhPump.UpTimeError != TFTStruc.pHUTErr || !refresh)
  {
    TFTStruc.pHUTErr = PhPump.UpTimeError;
    if (TFTStruc.pHUTErr)
    {
      myNex.writeStr(F("page0.vapHUTErr.val=1"));
    }
    else
      myNex.writeStr(F("page0.vapHUTErr.val=0"));
  }

  if (WaterFillError != TFTStruc.WFErr || !refresh)
  {
    TFTStruc.WFErr = WaterFillError;
    if (TFTStruc.WFErr)
    {
      myNex.writeStr(F("page0.vaWFErr.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaWFErr.val=0"));
  }

  if (WaterFill.UpTimeError != TFTStruc.WFUTErr || !refresh)
  {
    TFTStruc.WFUTErr = WaterFill.UpTimeError;
    if (TFTStruc.WFUTErr)
    {
      myNex.writeStr(F("page0.vaWFUTErr.val=1"));
    }
    else
      myNex.writeStr(F("page0.vaWFUTErr.val=0"));
  }

  if (storage.PublishPeriod != TFTStruc.PubInt || !refresh)
  {
    TFTStruc.PubInt = storage.PublishPeriod;
    snprintf(buf, sizeof(buf), "%.0f", TFTStruc.PubInt / 1.6f);
    myNex.writeStr(F("page0.vaPubInt.txt"), buf);
    if (CurrentPage == 11)  myNex.writeStr(F("PubInt.txt"), buf);
  }

  if (storage.DelayPIDs != TFTStruc.DelayPID || !refresh)
  {
    TFTStruc.DelayPID = storage.DelayPIDs;
    snprintf(buf, sizeof(buf), "%d", TFTStruc.DelayPID);
    myNex.writeStr(F("page0.vaDelayPID.txt"), buf);
    if (CurrentPage == 14)  myNex.writeStr(F("DelayPID.txt"), buf);
  }

  if ((storage.Ph_Kp != TFTStruc.Ph_Kp) || (storage.Ph_Ki != TFTStruc.Ph_Ki) || (storage.Ph_Kd != TFTStruc.Ph_Kd) || !refresh)
  {
    TFTStruc.Ph_Kp = storage.Ph_Kp;
    TFTStruc.Ph_Ki = storage.Ph_Ki;
    TFTStruc.Ph_Kd = storage.Ph_Kd;
    snprintf(buf, sizeof(buf), "%.1f/%.0f/%.1f", TFTStruc.Ph_Kp / 10000.0f, (float)TFTStruc.Ph_Ki, (float)TFTStruc.Ph_Kd);
    myNex.writeStr(F("page0.vapHPIDD.txt"), buf);
    if (CurrentPage == 15)  myNex.writeStr(F("pHPIDD.txt"), buf);
  }

  if ((storage.Orp_Kp != TFTStruc.Orp_Kp) || (storage.Orp_Ki != TFTStruc.Orp_Ki) || (storage.Orp_Kd != TFTStruc.Orp_Kd) || !refresh)
  {
    TFTStruc.Orp_Kp = storage.Orp_Kp;
    TFTStruc.Orp_Ki = storage.Orp_Ki;
    TFTStruc.Orp_Kd = storage.Orp_Kd;
    snprintf(buf, sizeof(buf), "%.1f/%.0f/%.1f", TFTStruc.Orp_Kp / 10000.0f, (float)TFTStruc.Orp_Ki, (float)TFTStruc.Orp_Kd);
    myNex.writeStr(F("page0.vaOrpPIDD.txt"), buf);
    if (CurrentPage == 16)  myNex.writeStr(F("OrpPIDD.txt"), buf);
  } 

  if (storage.PhPIDWindowSize != TFTStruc.pHPIDW || !refresh)
  {
    TFTStruc.pHPIDW = storage.PhPIDWindowSize;
    snprintf(buf, sizeof(buf), "%.0f", TFTStruc.pHPIDW / 60000.0f);
    myNex.writeStr(F("page0.vapHPIDW.txt"), buf);
    if (CurrentPage == 15)  myNex.writeStr(F("pHPIDW.txt"), buf);
  }

  if (storage.OrpPIDWindowSize != TFTStruc.OrpPIDW || !refresh)
  {
    TFTStruc.OrpPIDW = storage.OrpPIDWindowSize;
    snprintf(buf, sizeof(buf), "%.0f", TFTStruc.OrpPIDW / 60000.0f);
    myNex.writeStr(F("page0.vaOrpPIDW.txt"), buf);
    if (CurrentPage == 16)  myNex.writeStr(F("OrpPIDW.txt"), buf);
  }

  if (storage.pHPumpFR != TFTStruc.pHPumpFR || !refresh)
  {
    TFTStruc.pHPumpFR = storage.pHPumpFR;
    snprintf(buf, sizeof(buf), "%.1f", (float)TFTStruc.pHPumpFR);
    myNex.writeStr(F("page0.vapHPumpFR.txt"), buf);
    if (CurrentPage == 15)  myNex.writeStr(F("pHPumpFR.txt"), buf);
  }

  if (storage.ChlPumpFR != TFTStruc.ChlPumpFR || !refresh)
  {
    TFTStruc.ChlPumpFR = storage.ChlPumpFR;
    snprintf(buf, sizeof(buf), "%.1f", (float)TFTStruc.ChlPumpFR);
    myNex.writeStr(F("page0.vaChlPumpFR.txt"), buf);
    if (CurrentPage == 16)  myNex.writeStr(F("ChlPumpFR.txt"), buf);
  }

  const unsigned long phLimMin = storage.PhPumpUpTimeLimit / 60000UL;
  if (phLimMin != (unsigned long)TFTStruc.PumpMaxUp || !refresh)
  {
    TFTStruc.PumpMaxUp = (uint16_t)(phLimMin > 65535UL ? 65535UL : phLimMin);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)phLimMin);
    myNex.writeStr(F("page0.vaPumpsMaxUp.txt"), buf);
    if (CurrentPage == 14)  myNex.writeStr(F("PumpsMaxUp.txt"), buf);
  }

  if (storage.WaterFillFR != TFTStruc.WaterFillFR || !refresh)
  {
    TFTStruc.WaterFillFR = storage.WaterFillFR;
    snprintf(buf, sizeof(buf), "%.1f", (float)TFTStruc.WaterFillFR);
    myNex.writeStr(F("page0.vaWFFR.txt"), buf);
    if (CurrentPage == 17)  myNex.writeStr(F("WFFR.txt"), buf);
  }

  const unsigned long wfLimMin = storage.WaterFillUpTimeLimit / 60000UL;
  if (wfLimMin != (unsigned long)TFTStruc.WFMaxUp || !refresh)
  {
    TFTStruc.WFMaxUp = (uint16_t)(wfLimMin > 65535UL ? 65535UL : wfLimMin);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)wfLimMin);
    myNex.writeStr(F("page0.vaWFMaxUp.txt"), buf);
    if (CurrentPage == 17)  myNex.writeStr(F("WFMaxUp.txt"), buf);
  }

  const unsigned long fillDurMin = storage.WaterFillDuration / 60000UL;
  if (fillDurMin != (unsigned long)TFTStruc.FillDur || !refresh)
  {
    TFTStruc.FillDur = (uint16_t)(fillDurMin > 65535UL ? 65535UL : fillDurMin);
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)fillDurMin);
    myNex.writeStr(F("page0.vaFillDur.txt"), buf);
    if (CurrentPage == 17)  myNex.writeStr(F("FillDur.txt"), buf);
  }

  if(CurrentPage == 0) {
    snprintf(buf, sizeof(buf), "%.2f", storage.PhValue);
    myNex.writeStr(F("page0.vapH.txt"), buf);
    snprintf(buf, sizeof(buf), "%.0f", storage.OrpValue);
    myNex.writeStr(F("page0.vaOrp.txt"), buf);
    snprintf(buf, sizeof(buf), "%.2f", storage.PhRawValue);
    myNex.writeStr(F("page0.vapHrw.txt"), buf);
    snprintf(buf, sizeof(buf), "%.2f", storage.OrpRawValue);
    myNex.writeStr(F("page0.vaOrprw.txt"), buf);
  }

  // Update time at top of displayed page. We deliberately cover the full
  // range of pages (0..30) instead of only 0..19 — the original limit
  // skipped the salt/solar/water-fill/system pages and was the reason the
  // header clock only updated reliably on page 0. Pages without a
  // "p<N>Time" element will simply ignore the write.
  pushTimeToCurrentPage(CurrentPage);
  //put TFT in sleep mode with wake up on touch and force page 0 load to trigger an event
  if((unsigned long)(millis() - LastAction) >= TFT_SLEEP && TFT_ON && CurrentPage != 22 && CurrentPage != 11 && CurrentPage != 21 && CurrentPage != 25 && CurrentPage != 28 && CurrentPage != 29 && CurrentPage != 30)
  {
    myNex.writeStr("thup=1");
    myNex.writeStr("wup=0");
    myNex.writeStr("sleep=1");
    TFT_ON = false;
  }
  refresh = true;
}

//Page 0 has finished loading
//printh 23 02 54 01
void trigger1()
{
  if(!TFT_ON)
  {
    UpdateWiFi(wifiStaConnected());
    TFT_ON = true;
    refresh = false;
  }
  onPageLoaded(0);
}

//Page 1 has finished loading
//printh 23 02 54 02
void trigger2()  { onPageLoaded(1); }

//Page 2 has finished loading
//printh 23 02 54 03
void trigger3()  { onPageLoaded(2); }

//Page 3 has finished loading
//printh 23 02 54 04
void trigger4()  { onPageLoaded(3); }

//MODE button was toggled
//printh 23 02 54 05
void trigger5()
{
  int target = computeToggleTarget("vabMode.val", storage.AutoMode ? 1 : 0);
  TFTStruc.Mode = (boolean)target;
  debounceM = 1;
  Debug.print(DBG_INFO, "[Nextion] MODE (storage=%d, target=%d)",
              (int)storage.AutoMode, target);
  sendBoolCmd("Mode", target);
}

//FILT button was toggled
//printh 23 02 54 06
void trigger6()
{
  int target = computeToggleTarget("vabFilt.val", FiltrationPump.IsRunning() ? 1 : 0);
  TFTStruc.Filt = (boolean)target;
  debounceF = 1;
  Debug.print(DBG_INFO, "[Nextion] FILT (pump=%d, target=%d)",
              (int)FiltrationPump.IsRunning(), target);
  sendBoolCmd("FiltPump", target);
}

//Robot button was toggled
//printh 23 02 54 07
void trigger7()
{
  int target = computeToggleTarget("vabRobot.val", RobotPump.IsRunning() ? 1 : 0);
  TFTStruc.Robot = (boolean)target;
  debounceH = 1;
  Debug.print(DBG_INFO, "[Nextion] Robot (pump=%d, target=%d)",
              (int)RobotPump.IsRunning(), target);
  sendBoolCmd("RobotPump", target);
}

//Relay 0 button was toggled
//printh 23 02 54 08
// Relay outputs are active-LOW: digitalRead==0 means relay is energised (ON).
void trigger8()
{
  int wasOn  = (digitalRead(RELAY_R0) == LOW) ? 1 : 0;
  int target = computeToggleTarget("vabR0.val", wasOn);
  TFTStruc.R0 = (boolean)target;
  debounceR0 = 1;
  Debug.print(DBG_INFO, "[Nextion] Relay 0 (was=%d, target=%d)", wasOn, target);
  char json[32];
  snprintf(json, sizeof(json), "{\"Relay\":[0,%d]}", target);
  enqueueJsonCmd(json);
  LastAction = millis();
}

//Relay 1 button was toggled
//printh 23 02 54 09
void trigger9()
{
  int wasOn  = (digitalRead(RELAY_R1) == LOW) ? 1 : 0;
  int target = computeToggleTarget("vabR1.val", wasOn);
  TFTStruc.R1 = (boolean)target;
  debounceR1 = 1;
  Debug.print(DBG_INFO, "[Nextion] Relay 1 (was=%d, target=%d)", wasOn, target);
  char json[32];
  snprintf(json, sizeof(json), "{\"Relay\":[1,%d]}", target);
  enqueueJsonCmd(json);
  LastAction = millis();
}

//Winter button was toggled
//printh 23 02 54 0A
void trigger10()
{
  int target = computeToggleTarget("vabWinMode.val", storage.WinterMode ? 1 : 0);
  TFTStruc.R2 = (boolean)target;
  debounceR2 = 1;
  Debug.print(DBG_INFO, "[Nextion] Winter (storage=%d, target=%d)",
              (int)storage.WinterMode, target);
  sendBoolCmd("Winter", target);
}

//Probe calibration completed or new pH, Orp or Water Temp setpoints or New tank
//printh 23 02 54 0B
void trigger11()
{
  Debug.print(DBG_VERBOSE,"Calibration complete or new pH, Orp, Water Temp, Backwash Trigger, Filterpressure max setpoints, MotoValve position or new tank event");
  String s = myNex.readStr(F("pageCalibs.vaCommand.txt"));
  enqueueJsonCmd(s.c_str());
  Debug.print(DBG_VERBOSE,"Nextion cal page command: %s",s.c_str());
  LastAction = millis();
}

//Clear Errors button pressed
//printh 23 02 54 0C
void trigger12()
{
  Debug.print(DBG_VERBOSE,"Clear errors event");
  enqueueJsonCmd("{\"Clear\":1}");
  LastAction = millis();
}

//pH PID button pressed
//printh 23 02 54 0D
void trigger13()
{
  int target = computeToggleTarget("page15.vabpHMode.val",
                                   storage.Ph_RegulationOnOff ? 1 : 0);
  TFTStruc.PIDpH = (boolean)target;
  debouncepHP = 1;
  Debug.print(DBG_INFO, "[Nextion] pH PID (storage=%d, target=%d)",
              (int)storage.Ph_RegulationOnOff, target);
  sendBoolCmd("PhPID", target);
}

//Orp PID button pressed
//printh 23 02 54 0E
void trigger14()
{
  int target = computeToggleTarget("page16.vabChlMode.val",
                                   storage.Orp_RegulationOnOff ? 1 : 0);
  TFTStruc.PIDChl = (boolean)target;
  debounceChlP = 1;
  Debug.print(DBG_INFO, "[Nextion] Orp PID (storage=%d, target=%d)",
              (int)storage.Orp_RegulationOnOff, target);
  sendBoolCmd("OrpPID", target);
}

//HEAT MODE button was toggled (Haus-Wasserheizung → MQTT "Heat"). Nutzt dasselbe
// vabHeatMode wie die WP-Automatik auf page 3; unqualifiziert = Variable der aktiven Seite.
//printh 23 02 54 0F
void trigger15()
{
  int target = computeToggleTarget("vabHeatMode.val", storage.WaterHeat ? 1 : 0);
  TFTStruc.Heat = (boolean)target;
  debounceH = 1;
  Debug.print(DBG_INFO, "[Nextion] HEAT MODE (storage=%d, target=%d)",
              (int)storage.WaterHeat, target);
  sendBoolCmd("Heat", target);
}

//Page 4 has finished loading
//printh 23 02 54 10
void trigger16() { onPageLoaded(4); }

//Page 5 has finished loading
//printh 23 02 54 11
void trigger17() { onPageLoaded(5); }

//Page 6 has finished loading
//printh 23 02 54 12
void trigger18() { onPageLoaded(6); }

//Page 7 has finished loading
//printh 23 02 54 13
void trigger19() { onPageLoaded(7); }

//Page 8 has finished loading
//printh 23 02 54 14
void trigger20() { onPageLoaded(8); }

//Page 9 has finished loading
//printh 23 02 54 15
void trigger21() { onPageLoaded(9); }

//Page 10 has finished loading
//printh 23 02 54 16
void trigger22() { onPageLoaded(10); }

//SALT_CHLOR button was toggled
//printh 23 02 54 17
void trigger23()
{
  int target = computeToggleTarget("vabSaltChl.val", storage.Salt_Chlor ? 1 : 0);
  TFTStruc.Salt_Chlor = (boolean)target;
  debounceSM = 1;
  Debug.print(DBG_INFO, "[Nextion] SALT_CHLOR (storage=%d, target=%d)",
              (int)storage.Salt_Chlor, target);
  sendBoolCmd("Salt_Chlor", target);
}

//Page 11 has finished loading
//printh 23 02 54 18
void trigger24() { onPageLoaded(11); }

//Page 12 has finished loading
//printh 23 02 54 19
void trigger25() { onPageLoaded(12); }

//Page 13 has finished loading
//printh 23 02 54 1A
void trigger26() { onPageLoaded(13); }

//Page 14 has finished loading
//printh 23 02 54 1B
void trigger27() { onPageLoaded(14); }

//Page 15 has finished loading
//printh 23 02 54 1C
void trigger28() { onPageLoaded(15); }

//pHPump button was toggled
//printh 23 02 54 1D
void trigger29()
{
  int target = computeToggleTarget("page15.vabpHPum.val", PhPump.IsRunning() ? 1 : 0);
  TFTStruc.PhPump = (boolean)target;
  debouncepH = 1;
  Debug.print(DBG_INFO, "[Nextion] PhPump (pump=%d, target=%d)",
              (int)PhPump.IsRunning(), target);
  sendBoolCmd("PhPump", target);
}

//Page 16 has finished loading
//printh 23 02 54 1E
void trigger30() { onPageLoaded(16); }

//ChlPump button was toggled
//printh 23 02 54 1F
void trigger31()
{
  int target = computeToggleTarget("page16.vabChlPum.val", ChlPump.IsRunning() ? 1 : 0);
  TFTStruc.ChlPump = (boolean)target;
  debounceChl = 1;
  Debug.print(DBG_INFO, "[Nextion] ChlPump (pump=%d, target=%d)",
              (int)ChlPump.IsRunning(), target);
  sendBoolCmd("ChlPump", target);
}

//Reset PSI Calib button pressed
//printh 23 02 54 20
void trigger32()
{
  Debug.print(DBG_VERBOSE,"Reset PSI Calib event");
  enqueueJsonCmd("{\"RstPSICal\":1}");
  LastAction = millis();
}

//Reboot button pressed
//printh 23 02 54 21
void trigger33()
{
  Debug.print(DBG_VERBOSE,"Reboot event");
  enqueueJsonCmd("{\"Reboot\":1}");
  LastAction = millis();
}

//Reset pH probe button pressed
//printh 23 02 54 22
void trigger34()
{
  Debug.print(DBG_VERBOSE,"Reset pH Probe event");
  enqueueJsonCmd("{\"RstpHCal\":1}");
  LastAction = millis();
}

//Reset Orp probe button pressed
//printh 23 02 54 23
void trigger35()
{
  Debug.print(DBG_VERBOSE,"Reset Orp Probe event");
  enqueueJsonCmd("{\"RstOrpCal\":1}");
  LastAction = millis();
}

//HeatPump button was toggled
//printh 23 02 54 24
void trigger36()
{
  int target = computeToggleTarget("page3.vabHeatPum.val", HeatPump.IsRunning() ? 1 : 0);
  TFTStruc.HeatPump = (boolean)target;
  debounceHP = 1;
  Debug.print(DBG_INFO, "[Nextion] HeatPump (pump=%d, target=%d)",
              (int)HeatPump.IsRunning(), target);
  sendBoolCmd("HeatPump", target);
}

//SALT MODE button was toggled
//printh 23 02 54 25
void trigger37()
{
  int target = computeToggleTarget("vabSaltMode.val", storage.SaltMode ? 1 : 0);
  TFTStruc.SaltMode = (boolean)target;
  debounceSM = 1;
  Debug.print(DBG_INFO, "[Nextion] SALT MODE (storage=%d, target=%d)",
              (int)storage.SaltMode, target);
  sendBoolCmd("SaltMode", target);
}

//SaltPump button was toggled
//printh 23 02 54 26
void trigger38()
{
  int target = computeToggleTarget("vabSaltPum.val", SaltPump.IsRunning() ? 1 : 0);
  TFTStruc.SaltPump = (boolean)target;
  debounceSP = 1;
  Debug.print(DBG_INFO, "[Nextion] SaltPump (pump=%d, target=%d)",
              (int)SaltPump.IsRunning(), target);
  sendBoolCmd("SaltPump", target);
}

//VALVE MODE button was toggled
//printh 23 02 54 27
void trigger39()
{
  int target = computeToggleTarget("vabValveMode.val", storage.ValveMode ? 1 : 0);
  TFTStruc.ValveMode = (boolean)target;
  debounceVM = 1;
  Debug.print(DBG_INFO, "[Nextion] VALVE MODE (storage=%d, target=%d)",
              (int)storage.ValveMode, target);
  sendBoolCmd("ValveMode", target);
}

//CLEAN MODE button was toggled
//printh 23 02 54 28
void trigger40()
{
  int target = computeToggleTarget("vabCleanMode.val", storage.CleanMode ? 1 : 0);
  TFTStruc.CleanMode = (boolean)target;
  debounceCM = 1;
  Debug.print(DBG_INFO, "[Nextion] CLEAN MODE (storage=%d, target=%d)",
              (int)storage.CleanMode, target);
  sendBoolCmd("CleanMode", target);
}

//VALVE SWITCH button was toggled
//printh 23 02 54 29
void trigger41()
{
  int target = computeToggleTarget("vabCleanDir.val", storage.ValveSwitch ? 1 : 0);
  TFTStruc.ValveSwitch = (boolean)target;
  debounceVS = 1;
  Debug.print(DBG_INFO, "[Nextion] VALVE SWITCH (storage=%d, target=%d)",
              (int)storage.ValveSwitch, target);
  sendBoolCmd("ValveSwitch", target);
}

//WATERFILL MODE button was toggled
//printh 23 02 54 2A
void trigger42()
{
  int target = computeToggleTarget("vabFillMode.val", storage.WaterFillMode ? 1 : 0);
  TFTStruc.WaterFillMode = (boolean)target;
  debounceWFM = 1;
  Debug.print(DBG_INFO, "[Nextion] WATERFILL MODE (storage=%d, target=%d)",
              (int)storage.WaterFillMode, target);
  sendBoolCmd("FillMode", target);
}

//Page 17 has finished loading
//printh 23 02 54 2B
void trigger43() { onPageLoaded(17); }

//Reset Anual WaterConsumtion button pressed
//printh 23 02 54 2C
void trigger44()
{
  Debug.print(DBG_VERBOSE,"Reset Anual WaterConsumtion event");
  enqueueJsonCmd("{\"RstWatCons\":1}");
  LastAction = millis();
}

//WaterFill tap button was toggled
//printh 23 02 54 2D
void trigger45()
{
  int target = computeToggleTarget("vabTap.val", WaterFill.IsRunning() ? 1 : 0);
  TFTStruc.WaterFill = (boolean)target;
  debounceWF = 1;
  Debug.print(DBG_INFO, "[Nextion] WaterFill (tap=%d, target=%d)",
              (int)WaterFill.IsRunning(), target);
  sendBoolCmd("WaterFill", target);
}

//Page 18 has finished loading
//printh 23 02 54 2E
void trigger46() { onPageLoaded(18); }

//WIFIOnOff button was toggled
//printh 23 02 54 2F
void trigger47()
{
  int target = computeToggleTarget("vabWiFi_OnOff.val", storage.WIFI_OnOff ? 1 : 0);
  TFTStruc.WIFI_OnOff = (boolean)target;
  debounceWiFi = 1;
  Debug.print(DBG_INFO, "[Nextion] WIFI_OnOff (storage=%d, target=%d)",
              (int)storage.WIFI_OnOff, target);
  sendBoolCmd("WIFI_OnOff", target);
}

//SOLAR MODE button was toggled
//printh 23 02 54 30
void trigger48()
{
  int target = computeToggleTarget("vabSolMode.val", storage.SolarMode ? 1 : 0);
  TFTStruc.SolarMode = (boolean)target;
  debounceSolM = 1;
  Debug.print(DBG_INFO, "[Nextion] SOLAR MODE (storage=%d, target=%d)",
              (int)storage.SolarMode, target);
  sendBoolCmd("SolarMode", target);
}

//SolarPump button was toggled
//printh 23 02 54 31
void trigger49()
{
  int target = computeToggleTarget("vabSolPum.val", SolarPump.IsRunning() ? 1 : 0);
  TFTStruc.SolarPump = (boolean)target;
  debounceSolP = 1;
  Debug.print(DBG_INFO, "[Nextion] SolarPump (pump=%d, target=%d)",
              (int)SolarPump.IsRunning(), target);
  sendBoolCmd("SolarPump", target);
}

//SOLAR LOKAL EXTERN button was toggled
//printh 23 02 54 32
void trigger50()
{
  int target = computeToggleTarget("vabSolLoEx.val", storage.SolarLocExt ? 1 : 0);
  TFTStruc.SolarLoEx = (boolean)target;
  debounceSolLE = 1;
  Debug.print(DBG_INFO, "[Nextion] SOLAR LOKAL EXTERN (storage=%d, target=%d)",
              (int)storage.SolarLocExt, target);
  sendBoolCmd("SolarLocExt", target);
}

//Page 19 has finished loading
//printh 23 02 54 33
void trigger51() { onPageLoaded(19); }

//MQTT Login button was toggled
//printh 23 02 54 34
void trigger52()
{
  int target = computeToggleTarget("vabMqttLogin.val", storage.MQTTLOGIN_OnOff ? 1 : 0);
  TFTStruc.MqttLogin = (boolean)target;
  debounceMQL = 1;
  Debug.print(DBG_INFO, "[Nextion] MQTT Login (storage=%d, target=%d)",
              (int)storage.MQTTLOGIN_OnOff, target);
  sendBoolCmd("MqttLogin", target);
}

//BUS_AB button was toggled
//printh 23 02 54 35
void trigger53()
{
  int target = computeToggleTarget("vabBUSA_B.val", storage.BUS_A_B ? 1 : 0);
  TFTStruc.BUSA_B = (boolean)target;
  debounceB = 1;
  Debug.print(DBG_INFO, "[Nextion] BUSA_B (storage=%d, target=%d)",
              (int)storage.BUS_A_B, target);
  sendBoolCmd("Bus_A_B", target);
}

//Page 27 has finished loading
//printh 23 02 54 36
void trigger54() { onPageLoaded(27); }

//Page KeyPad has finished loading
//printh 23 02 54 37
void trigger55() { onPageLoaded(22); }

// HEATPUMP MODE (HeatPumpMode / MQTT "HeatPumpMode") — page 3 Wärmepumpe.
// printh 23 02 54 38
//
// Mit trigger15 (WaterHeat / "Heat") wird dasselbe Variablennamen-Schema genutzt,
// aber auf einer anderen Seite. Hier page3.vabHeatMode.val qualifizieren, damit
// readNumber immer die WP-Seite trifft. UpdateTFT() schreibt page3.vabHeatMode + picc.
void trigger56()
{
  int target = computeToggleTarget("page3.vabHeatMode.val", storage.HeatPumpMode ? 1 : 0);
  TFTStruc.HeatMode = (boolean)target;
  debounceHPM = 1;
  Debug.print(DBG_INFO, "[Nextion] HEATPUMP MODE (storage=%d, target=%d)",
              (int)storage.HeatPumpMode, target);
  sendBoolCmd("HeatPumpMode", target);
}