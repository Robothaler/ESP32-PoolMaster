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

static volatile int CurrentPage = 0;
static volatile bool TFT_ON = true;           // display status
static volatile bool refresh = false;         // flag to force display refresh

static String temp;
static unsigned long LastAction = 0; // Last action time done on TFT. Go to sleep after TFT_SLEEP
static char HourBuffer[9];

static uint8_t debounceCount = 2;
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
static uint8_t debounceWF    = 0;
static uint8_t debounceWiFi  = 0;
static uint8_t debounceMQL   = 0;


// Structure holding the measurement values to display on the Nextion display
// Used to refresh only modified values
static struct TFTStruct
{
  float pH, Orp, pHSP, OrpSP, WST, WIT, WBT, WWPT, WWTT, WTSP, AT, AH, AP, AIT, ST, SVLT, SRLT, PSI, flow, flow2, F1H, F1L, F2H, F2L, PsiH, PsiL, WTLow, pHPumpFR, ChlPumpFR, WaterFillFR, Ph_Kp, Ph_Ki, Ph_Kd, Orp_Kp, Orp_Ki, Orp_Kd;
  uint8_t FSta, FSto, FStaT0, FStoT1, SStaT0, SStoT1, pHTkFill, OrpTkFill, PIDpH, PIDChl, PubInt, PumpMaxUp, WFMaxUp, FillDur, WFMinLvl, WFMaxLvl, DelayPID, pHPIDW, OrpPIDW, FLOW_Pulse, FLOW2_Pulse, SaltDiff;
  uint16_t MQTT_PORT;
  boolean WIFI_OnOff, MqttLogin, BUSA_B, Mode, SolarLoEx, SolarMode, WaterFillMode, SaltMode, NetW, Filt, Robot, R0, R1, R2, pHUTErr, ChlUTErr, WFUTErr, WFErr, PSIErr, FLOWErr, FLOW2Err, pHTLErr, ChlTLErr, PhPump, ChlPump, Heat, HeatPump, SaltPump, SolarPump ,Salt_Chlor, SaltPolarity, ValveMode, CleanMode, ValveSwitch, WaterFill;
  unsigned long pHPpRT, OrpPpRT, SHRT, HPRT, SPUT, SPRT, SolPRT, FLRT, WFRT, WFAC;
  IPAddress MQTT_IP;
  DeviceAddress TW_Adr_1, TW_Adr_2, TW_Adr_3, TW_Adr_4, TW_Adr_5, TA_Adr_1, TA_Adr_2, TA_Adr_3, TA_Adr_4, TA_Adr_5;
  String FW, SSID, PASSW, MQTT_USER, MQTT_PASS, MQTT_NAME;
  std::string ELDTstate, ELDHstate, WPVstate, WPMstate, BOTTstate, SOLARstate;
} TFTStruc =
{ //default values to force update on next refresh
  -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1.,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
  99,
  "", "", "", "", "", "",
  "", "", "", "", "", "",
};

extern Preferences nvs;

//Nextion TFT object. Choose which ever Serial port
//you wish to connect to (not "Serial" which is used for debug), here Serial2 UART
static EasyNex myNex(Serial1);

// Functions prototypes
void InitTFT(void);
void ResetTFT(void);
void UpdateTFT(void);
void UpdateWiFi(bool);
void getAddressString(DeviceAddress addr, char* temp, size_t tempSize);

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
    temp = "WiFi: " + WiFi.SSID();
    myNex.writeStr("page0.vaSSID.txt",temp);
    temp = "IP: " + WiFi.localIP().toString();
    myNex.writeStr("page0.vaIP.txt",temp);
  } else
  {
    myNex.writeStr("page0.vaSSID.txt","not connected");
    myNex.writeStr("page0.vaIP.txt","");
  } 
}



//function to update TFT display
//it updates the TFTStruct variables, the global variables of the TFT + the widgets of the active page
//call this function at least every second to ensure fluid display
void UpdateTFT()
{
  myNex.NextionListen();

  sprintf(HourBuffer, "%02d:%02d:%02d", hour(), minute(), second());
  myNex.writeStr("page0.vaTime.txt", HourBuffer);

  if (Firmw != TFTStruc.FW)
  {
    TFTStruc.FW = F("MC fw: v ");
    TFTStruc.FW += Firmw;
    myNex.writeStr(F("page0.vaMCFW.txt"), TFTStruc.FW);
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
        temp = String(TFTStruc.SSID);
        myNex.writeStr(F("page0.vaWifiSSID.txt"), temp);
        Debug.print(DBG_DEBUG, "Updating TFT SSID: %s", temp.c_str());
        if (CurrentPage == 24)  myNex.writeStr(F("ssid.txt"), temp);
      }


  if (storage.WIFI_PASS != TFTStruc.PASSW || !refresh)
    {
      TFTStruc.PASSW = storage.WIFI_PASS;
      temp = String(TFTStruc.PASSW);
      myNex.writeStr(F("page0.vaWifiPASS.txt"), temp);
      Debug.print(DBG_DEBUG, "Updating TFT WiFi password: %s", temp.c_str());
      if (CurrentPage == 24)  myNex.writeStr(F("pwd.txt"), temp);
    }
  
  if (MQTTConnection != TFTStruc.NetW || !refresh) {
  TFTStruc.NetW = MQTTConnection;
  myNex.writeNum(F("page1.vabNetW.val"), TFTStruc.NetW);
  temp = TFTStruc.NetW ? F("ONLINE") : F("OFFLINE");
  myNex.writeStr(F("page0.vaMqttState.txt"), temp);
  myNex.writeStr(F("page19.MqttState.txt"), temp);

  switch (CurrentPage) {
    case 0:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p0NetW.pic=5") : F("p0NetW.pic=6"));
      break;
    case 1:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p1NetW.pic=5") : F("p1NetW.pic=6"));
      break;
    case 2:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p2NetW.pic=5") : F("p2NetW.pic=6"));
      break;
    case 3:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p3NetW.pic=5") : F("p3NetW.pic=6"));
      break;
    case 4:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p4NetW.pic=5") : F("p4NetW.pic=6"));
      break;
    case 5:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p5NetW.pic=5") : F("p5NetW.pic=6"));
      break;
    case 6:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p6NetW.pic=5") : F("p6NetW.pic=6"));
      break;
    case 7:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p7NetW.pic=5") : F("p7NetW.pic=6"));
      break;
    case 8:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p8NetW.pic=5") : F("p8NetW.pic=6"));
      break;
    case 9:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p9NetW.pic=5") : F("p9NetW.pic=6"));
      break;
    case 10:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p10NetW.pic=5") : F("p10NetW.pic=6"));
      break;
    case 11:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p11NetW.pic=5") : F("p11NetW.pic=6"));
      myNex.writeStr(TFTStruc.NetW == 1 ? F("b6.picc=43") : F("b6.picc=42"));
      myNex.writeStr(TFTStruc.NetW == 1 ? F("b6.picc2=42") : F("b6.picc2=43"));
      break;
    case 12:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p12NetW.pic=5") : F("p12NetW.pic=6"));
      break;
    case 13:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p13NetW.pic=5") : F("p13NetW.pic=6"));
      break;
    case 14:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p14NetW.pic=5") : F("p14NetW.pic=6"));
      break;
    case 15:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p15NetW.pic=5") : F("p15NetW.pic=6"));
      break;
    case 16:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p16NetW.pic=5") : F("p16NetW.pic=6"));
      break;
    case 17:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p17NetW.pic=5") : F("p17NetW.pic=6"));
      break;
    case 18:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p18NetW.pic=5") : F("p18NetW.pic=6"));
      break;
    case 19:
      myNex.writeStr(TFTStruc.NetW == 1 ? F("p19NetW.pic=5") : F("p19NetW.pic=6"));
      myNex.writeStr(F("MqttState.txt"), temp);
      break;
    }
  }

  if (storage.MQTT_NAME != TFTStruc.MQTT_NAME || !refresh)
    {
      TFTStruc.MQTT_NAME = storage.MQTT_NAME;
      temp = String(TFTStruc.MQTT_NAME);
      myNex.writeStr(F("page0.vaMqttName.txt"), temp);
      if (CurrentPage == 19)  myNex.writeStr(F("MqttName.txt"), temp);
    }

  if (storage.MQTT_USER != TFTStruc.MQTT_USER || !refresh)
    {
      TFTStruc.MQTT_USER = storage.MQTT_USER;
      temp = String(TFTStruc.MQTT_USER);
      myNex.writeStr(F("page0.vaMqttUser.txt"), temp);
      if (CurrentPage == 19)  myNex.writeStr(F("MqttUser.txt"), temp);
    }

  if (storage.MQTT_PASS != TFTStruc.MQTT_PASS || !refresh)
    {
      TFTStruc.MQTT_PASS = storage.MQTT_PASS;
      temp = String(TFTStruc.MQTT_PASS);
      myNex.writeStr(F("page0.vaMqttPass.txt"), temp);
      if (CurrentPage == 19)  myNex.writeStr(F("MqttPass.txt"), temp);
    }

  if (storage.MQTT_PORT != TFTStruc.MQTT_PORT || !refresh)
    {
      TFTStruc.MQTT_PORT = storage.MQTT_PORT;
      temp = String(TFTStruc.MQTT_PORT);
      myNex.writeStr(F("page0.vaMqttPort.txt"), temp);
      if (CurrentPage == 19)  myNex.writeStr(F("MqttPort.txt"), temp);
    }
  
  if (storage.MQTT_IP != TFTStruc.MQTT_IP || !refresh)
    {
      TFTStruc.MQTT_IP = storage.MQTT_IP;
      temp = TFTStruc.MQTT_IP.toString();
      myNex.writeStr(F("page0.vaMqttIP.txt"), temp);
      Debug.print(DBG_DEBUG, "[MQTT / NEXTION] MQTT server IP address: %s", storage.MQTT_IP.toString().c_str());
      if (CurrentPage == 19)  myNex.writeStr(F("MqttIP.txt"), temp);
    }   

  if (storage.PhValue != TFTStruc.pH || !refresh)
  {
    TFTStruc.pH = storage.PhValue;
    int pHgauge = TFTStruc.pH * 10;
    int pHangle;
    myNex.writeStr(F("page0.vapH.txt"), String(TFTStruc.pH, 2));
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
    if (CurrentPage == 0)  myNex.writeStr(F("pH.txt"), String(TFTStruc.pH, 2));
      myNex.writeNum(F("pHg.val"), pHangle);
  }

  if (storage.OrpValue != TFTStruc.Orp || !refresh)
  {
    TFTStruc.Orp = storage.OrpValue;
    int Orpgauge = TFTStruc.Orp;
    int Orpangle;
    myNex.writeStr(F("page0.vaOrp.txt"), String(TFTStruc.Orp, 0));
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
    if (CurrentPage == 0)  myNex.writeStr(F("Orp.txt"), String(TFTStruc.Orp, 0));
      myNex.writeNum(F("Orpg.val"), Orpangle);
  }

  if (storage.Ph_SetPoint != TFTStruc.pHSP || !refresh)
  {
    TFTStruc.pHSP = storage.Ph_SetPoint;
    temp = "(" + String(TFTStruc.pHSP, 1) + ")";
    myNex.writeStr(F("page0.vapHSP.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("pHSP.txt"), temp);
    else if (CurrentPage == 15)  myNex.writeStr(F("pHSP.txt"), temp);
  }
  if (storage.Orp_SetPoint != TFTStruc.OrpSP || !refresh)
  {
    TFTStruc.OrpSP = storage.Orp_SetPoint;
    temp = "(" + String((int)TFTStruc.OrpSP) + ")";
    myNex.writeStr(F("page0.vaOrpSP.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("OrpSP.txt"), temp);
    else if (CurrentPage == 13)  myNex.writeStr(F("OrpSP.txt"), temp);
    else if (CurrentPage == 16)  myNex.writeStr(F("OrpSP.txt"), temp);
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
        myNex.writeStr(F("b2.pic=58"));
        else
        myNex.writeStr(F("b2.pic=59"));
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
        String name = NV_STORAGE_MAPPING_W[storage.Array_W[i]];
        char addressStr_W[18];                                                // Array to hold the formatted address string
        byte storedAddress_W[8];                                              // ByteArray to hold the stored address
        nvs.getBytes(("address_W_" + String(i)).c_str(), storedAddress_W, 8); // Read the stored address from NVS
        sprintf(addressStr_W, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                storedAddress_W[0], storedAddress_W[1], storedAddress_W[2], storedAddress_W[3],
                storedAddress_W[4], storedAddress_W[5], storedAddress_W[6], storedAddress_W[7]); // Format the address string
        String address = String(addressStr_W);
        String num = String(i + 1);
        String pos = String(storage.Array_W[i]);
        myNex.writeStr("pageTempArray.s" + num + ".txt", address + " | " + name);
        myNex.writeStr("pageTempArray.a" + num + ".txt", num);
        myNex.writeStr("pageTempArray.t" + pos + ".txt", pos);
        Debug.print(DBG_VERBOSE, "Address %d for Sensor W: %s, Name: %s, Number: %s, Position: %s", i + 1, address.c_str(), name.c_str(), num.c_str(), pos.c_str());
        }
      }
      else
      {
        // Display data for Sensor A
        for (int i = 0; i < MAX_ADDRESSES; i++)
        {
        String name = NV_STORAGE_MAPPING_A[storage.Array_A[i]];
        char addressStr_A[18];                                                // Array to hold the formatted address string
        byte storedAddress_A[8];                                              // ByteArray to hold the stored address
        nvs.getBytes(("address_A_" + String(i)).c_str(), storedAddress_A, 8); // Read the stored address from NVS
        sprintf(addressStr_A, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                storedAddress_A[0], storedAddress_A[1], storedAddress_A[2], storedAddress_A[3],
                storedAddress_A[4], storedAddress_A[5], storedAddress_A[6], storedAddress_A[7]); // Format the address string
        String address = String(addressStr_A);                                             // Convert the char array to a String
        String num = String(i + 1);
        String pos = String(storage.Array_A[i]);
        myNex.writeStr("pageTempArray.s" + num + ".txt", address + " | " + name);
        myNex.writeStr("pageTempArray.a" + num + ".txt", num);
        myNex.writeStr("pageTempArray.t" + pos + ".txt", pos);
        Debug.print(DBG_VERBOSE, "Address %d for Sensor A: %s, Name: %s, Number: %s, Position: %s", i + 1, address.c_str(), name.c_str(), num.c_str(), pos.c_str());
        }
      }
    }
  }

  if (storage.WaterSTemp != TFTStruc.WST || !refresh)
  {
    TFTStruc.WST = storage.WaterSTemp;
    temp = String(TFTStruc.WST, 1);
    myNex.writeStr(F("page0.vaWT.txt"), temp);
    if (CurrentPage == 0)
      myNex.writeStr(F("W.txt"), temp);
    else if (CurrentPage == 2)
      myNex.writeStr(F("W.txt"), temp);
    else if (CurrentPage == 3)
      myNex.writeStr(F("W.txt"), temp);
    else if (CurrentPage == 7)
      myNex.writeStr(F("W.txt"), temp);
  }

  if (storage.WaterITemp != TFTStruc.WIT || !refresh)
  {
    TFTStruc.WIT = storage.WaterITemp;
    temp = String(TFTStruc.WIT, 1);
    myNex.writeStr(F("page0.vaWIT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("WIT.txt"), temp);
    else if (CurrentPage == 7)  myNex.writeStr(F("WIT.txt"), temp);
  }

  if (storage.WaterBTemp != TFTStruc.WBT || !refresh)
  {
    TFTStruc.WBT = storage.WaterBTemp;
    temp = String(TFTStruc.WBT, 1);
    myNex.writeStr(F("page0.vaWBT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("WBT.txt"), temp);
    else if (CurrentPage == 7)  myNex.writeStr(F("WBT.txt"), temp);
  }

  if (storage.WaterWPTemp != TFTStruc.WWPT || !refresh)
  {
    TFTStruc.WWPT = storage.WaterWPTemp;
    temp = String(TFTStruc.WWPT, 1);
    myNex.writeStr(F("page0.vaWWPT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("WWPT.txt"), temp);
    else if (CurrentPage == 7)  myNex.writeStr(F("WWPT.txt"), temp);
  }

  if (storage.WaterWTTemp != TFTStruc.WWTT || !refresh)
  {
    TFTStruc.WWTT = storage.WaterWTTemp;
    temp = String(TFTStruc.WWTT, 1);
    myNex.writeStr(F("page0.vaWWTT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("WWTT.txt"), temp);
    else if (CurrentPage == 7)  myNex.writeStr(F("WWTT.txt"), temp);
  }

  if (storage.AirInTemp != TFTStruc.AIT || !refresh)
  {
    TFTStruc.AIT = storage.AirInTemp;
    temp = String(TFTStruc.AIT, 1);
    myNex.writeStr(F("page0.vaAIT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("AIT.txt"), temp);
    else if (CurrentPage == 7)  myNex.writeStr(F("AIT.txt"), temp);
  }

  if (storage.SolarVLTemp != TFTStruc.SVLT || !refresh)
  {
    TFTStruc.SVLT = storage.SolarVLTemp;
    temp = String(TFTStruc.SVLT, 1);
    myNex.writeStr(F("page0.vaSVLT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("SVLT.txt"), temp);
    else if (CurrentPage == 7)  myNex.writeStr(F("SVLT.txt"), temp);
  }

  if (storage.SolarRLTemp != TFTStruc.SRLT || !refresh)
  {
    TFTStruc.SRLT = storage.SolarRLTemp;
    temp = String(TFTStruc.SRLT, 1);
    myNex.writeStr(F("page0.vaSRLT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("SRLT.txt"), temp);
    else if (CurrentPage == 7)  myNex.writeStr(F("SRLT.txt"), temp);
  }

  if (storage.WaterTemp_SetPoint != TFTStruc.WTSP || !refresh)
  {
    TFTStruc.WTSP = storage.WaterTemp_SetPoint;
    temp = String(TFTStruc.WTSP, 1);
    myNex.writeStr(F("page0.vaWSP.txt"), temp);
    if (CurrentPage == 2)  myNex.writeStr(F("WSP.txt"), temp);
  }

  if (storage.WaterTempLowThreshold != TFTStruc.WTLow || !refresh)
  {
    TFTStruc.WTLow = storage.WaterTempLowThreshold;
    temp = String(TFTStruc.WTLow, 1);
    myNex.writeStr(F("page0.vaWTempLow.txt"), temp);
    if (CurrentPage == 14)  myNex.writeStr(F("WTempLow.txt"), temp);
  }

  if (storage.AirTemp != TFTStruc.AT || !refresh)
  {
    TFTStruc.AT = storage.AirTemp;
    temp = String(TFTStruc.AT, 1);
    myNex.writeStr(F("page0.vaAT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("A.txt"), temp);
    else if (CurrentPage == 3)  myNex.writeStr(F("A.txt"), temp);
    else if (CurrentPage == 7)  myNex.writeStr(F("A.txt"), temp);
  }

  if (storage.AirHum != TFTStruc.AH || !refresh)
  {
    TFTStruc.AH = storage.AirHum;
    temp = String(TFTStruc.AH, 1);
    myNex.writeStr(F("page0.vaAH.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("AH.txt"), temp);
    else if (CurrentPage == 7)  myNex.writeStr(F("AH.txt"), temp);
  }

  if (storage.AirPress != TFTStruc.AP || !refresh)
  {
    TFTStruc.AP = storage.AirPress;
    temp = String(TFTStruc.AP, 1);
    myNex.writeStr(F("page0.vaAP.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("AP.txt"), temp);
    else if (CurrentPage == 7)  myNex.writeStr(F("AP.txt"), temp);
  }

  if (storage.SolarTemp != TFTStruc.ST || !refresh)
  {
    TFTStruc.ST = storage.SolarTemp;
    temp = String(TFTStruc.ST, 1);
    myNex.writeStr(F("page0.vaST.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("S.txt"), temp);
    else if (CurrentPage == 2)  myNex.writeStr(F("S.txt"), temp);
    else if (CurrentPage == 7)  myNex.writeStr(F("S.txt"), temp);
  }

  if (storage.PSIValue != TFTStruc.PSI || !refresh)
  {
    TFTStruc.PSI = storage.PSIValue;
    temp = String(TFTStruc.PSI, 1);
    int tempInt = TFTStruc.PSI * 100;
    int slider;
    myNex.writeStr(F("page0.vaPSI.txt"), temp);
      if(tempInt >= 40 && tempInt <= 70) {
        slider = map(tempInt,40,70,30,70);
      } else if(tempInt > 70 && tempInt <= 90) {
        slider = map(tempInt,70,90,70,90);
      } else if(tempInt > 90) {
        slider = 100;
      } else if(tempInt < 40 && tempInt > 10) {
        slider = map(tempInt,10,40,10,30);
      } else {
        slider = 0;
      }
    myNex.writeNum(F("page0.PSL.val"), slider);
    if (CurrentPage == 0)  myNex.writeStr(F("P.txt"), temp);
    else if (CurrentPage == 1)  myNex.writeStr(F("P.txt"), temp);
    else if (CurrentPage == 8)  myNex.writeStr(F("P.txt"), temp);
    else if (CurrentPage == 0)  myNex.writeNum(F("PSL.val"), slider);
  }

  if (storage.PSI_HighThreshold != TFTStruc.PsiH || !refresh)
  {
    TFTStruc.PsiH = storage.PSI_HighThreshold;
    temp = String(TFTStruc.PsiH, 1);
    myNex.writeStr(F("page0.vaPsiH.txt"), temp);
    if (CurrentPage == 12)  myNex.writeStr(F("PsiH.txt"), temp);
  }

  if (storage.PSI_MedThreshold != TFTStruc.PsiL || !refresh)
  {
    TFTStruc.PsiL = storage.PSI_MedThreshold;
    temp = String(TFTStruc.PsiL, 1);
    myNex.writeStr(F("page0.vaPsiL.txt"), temp);
    if (CurrentPage == 12)  myNex.writeStr(F("PsiL.txt"), temp);
  }

  if (storage.FLOWValue != TFTStruc.flow || !refresh)
  {
    TFTStruc.flow = storage.FLOWValue;
    temp = String(TFTStruc.flow, 0);
    int tempInt = TFTStruc.flow;
    int slider;
    myNex.writeStr(F("page0.vaF1.txt"), temp);
    if(tempInt >= 50 && tempInt <= 90) {
        slider = map(tempInt,50,90,30,70);
      } else if(tempInt > 90 && tempInt <= 100) {
        slider = map(tempInt,90,100,70,90);
      } else if(tempInt > 100) {
        slider = 100;
      } else if(tempInt < 50 && tempInt > 30) {
        slider = map(tempInt,30,50,10,30);
      } else {
        slider = 0;
      }
    myNex.writeNum(F("page0.F1SL.val"), slider);
    if (CurrentPage == 0)  myNex.writeStr(F("F1.txt"), temp);
    else if (CurrentPage == 1)  myNex.writeStr(F("F1.txt"), temp);
    else if (CurrentPage == 0)  myNex.writeNum(F("F1SL.val"), slider);
  }

  if (storage.FLOW_Pulse != TFTStruc.FLOW_Pulse || !refresh)
  {
    TFTStruc.FLOW_Pulse = storage.FLOW_Pulse;
    temp = String(TFTStruc.FLOW_Pulse);
    myNex.writeStr(F("page0.vaF1P.txt"), temp);
    if (CurrentPage == 12)  myNex.writeStr(F("F1P.txt"), temp);
  }

  if (storage.FLOW_HighThreshold != TFTStruc.F1H || !refresh)
  {
    TFTStruc.F1H = storage.FLOW_HighThreshold;
    temp = String(TFTStruc.F1H, 0);
    myNex.writeStr(F("page0.vaF1H.txt"), temp);
    if (CurrentPage == 12)  myNex.writeStr(F("F1H.txt"), temp);
  }

  if (storage.FLOW_MedThreshold != TFTStruc.F1L || !refresh)
  {
    TFTStruc.F1L = storage.FLOW_MedThreshold;
    temp = String(TFTStruc.F1L, 0);
    myNex.writeStr(F("page0.vaF1L.txt"), temp);
    if (CurrentPage == 12)  myNex.writeStr(F("F1L.txt"), temp);
  }

  if (storage.FLOW2Value != TFTStruc.flow2 || !refresh)
  {
    TFTStruc.flow2 = storage.FLOW2Value;
    temp = String(TFTStruc.flow2, 0);
    int tempInt = TFTStruc.flow2;
    int slider;
    myNex.writeStr(F("page0.vaF2.txt"), temp);
    if(tempInt >= 8 && tempInt <= 20) {
        slider = map(tempInt,8,20,30,70);
      } else if(tempInt > 20 && tempInt <= 30) {
        slider = map(tempInt,20,30,70,90);
      } else if(tempInt > 100) {
        slider = 100;
      } else if(tempInt < 5 && tempInt > 8) {
        slider = map(tempInt,5,8,10,30);
      } else {
        slider = 0;
      }
    myNex.writeNum(F("page0.F2SL.val"), slider);
    if (CurrentPage == 0)  myNex.writeStr(F("F2.txt"), temp);
    else if (CurrentPage == 0)  myNex.writeNum(F("F2SL.val"), slider);
  }

if (storage.FLOW2_Pulse != TFTStruc.FLOW2_Pulse || !refresh)
  {
    TFTStruc.FLOW2_Pulse = storage.FLOW2_Pulse;
    temp = String(TFTStruc.FLOW2_Pulse);
    myNex.writeStr(F("page0.vaF2P.txt"), temp);
    if (CurrentPage == 12)  myNex.writeStr(F("F2P.txt"), temp);
  }

  if (storage.FLOW2_HighThreshold != TFTStruc.F2H || !refresh)
  {
    TFTStruc.F2H = storage.FLOW2_HighThreshold;
    temp = String(TFTStruc.F2H, 0);
    myNex.writeStr(F("page0.vaF2H.txt"), temp);
    if (CurrentPage == 12)  myNex.writeStr(F("F2H.txt"), temp);
  }

  if (storage.FLOW2_MedThreshold != TFTStruc.F2L || !refresh)
  {
    TFTStruc.F2L = storage.FLOW2_MedThreshold;
    temp = String(TFTStruc.F2L, 0);
    myNex.writeStr(F("page0.vaF2L.txt"), temp);
    if (CurrentPage == 12)  myNex.writeStr(F("F2L.txt"), temp);
  }

  if ((storage.FiltrationStop != TFTStruc.FSto) || (storage.FiltrationStart != TFTStruc.FSta) || !refresh)
  {
    TFTStruc.FSto = storage.FiltrationStop;
    TFTStruc.FSta = storage.FiltrationStart;
    temp = String(TFTStruc.FSta) + F("/") + String(TFTStruc.FSto) + F("h");
    myNex.writeStr(F("page0.vaStaSto.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("p0StaSto.txt"), temp);
    else if (CurrentPage == 1)  myNex.writeStr(F("p1StaSto.txt"), temp);
    else if (CurrentPage == 2)  myNex.writeStr(F("p2StaSto.txt"), temp);
    else if (CurrentPage == 3)  myNex.writeStr(F("p3StaSto.txt"), temp);
    else if (CurrentPage == 4)  myNex.writeStr(F("p4StaSto.txt"), temp);
    else if (CurrentPage == 5)  myNex.writeStr(F("p5StaSto.txt"), temp);
    else if (CurrentPage == 6)  myNex.writeStr(F("p6StaSto.txt"), temp);
    else if (CurrentPage == 7)  myNex.writeStr(F("p7StaSto.txt"), temp);
    else if (CurrentPage == 8)  myNex.writeStr(F("p8StaSto.txt"), temp);
    else if (CurrentPage == 9)  myNex.writeStr(F("p9StaSto.txt"), temp);
    else if (CurrentPage == 10)  myNex.writeStr(F("p10StaSto.txt"), temp);
    else if (CurrentPage == 11)  myNex.writeStr(F("p11StaSto.txt"), temp);
    else if (CurrentPage == 12)  myNex.writeStr(F("p12StaSto.txt"), temp);
    else if (CurrentPage == 13)  myNex.writeStr(F("p13StaSto.txt"), temp);
    else if (CurrentPage == 14)  myNex.writeStr(F("p14StaSto.txt"), temp);
    else if (CurrentPage == 15)  myNex.writeStr(F("p15StaSto.txt"), temp);
    else if (CurrentPage == 16)  myNex.writeStr(F("p16StaSto.txt"), temp);
    else if (CurrentPage == 17)  myNex.writeStr(F("p17StaSto.txt"), temp);
    else if (CurrentPage == 18)  myNex.writeStr(F("p18StaSto.txt"), temp);
    else if (CurrentPage == 19)  myNex.writeStr(F("p19StaSto.txt"), temp);
  }

  if (storage.FiltrationStartMin != TFTStruc.FStaT0 || !refresh)
  {
    TFTStruc.FStaT0 = storage.FiltrationStartMin;
    temp = String(TFTStruc.FStaT0);
    myNex.writeStr(F("page0.vaFiltT0.txt"), temp);
    if (CurrentPage == 1)  myNex.writeStr(F("FiltT0.txt"), temp);
  }

  if (storage.FiltrationStopMax != TFTStruc.FStoT1 || !refresh)
  {
    TFTStruc.FStoT1 = storage.FiltrationStopMax;
    temp = String(TFTStruc.FStoT1);
    myNex.writeStr(F("page0.vaFiltT1.txt"), temp);
    if (CurrentPage == 1)  myNex.writeStr(F("FiltT1.txt"), temp);
  }

  if (storage.SolarStartMin != TFTStruc.SStaT0 || !refresh)
  {
    TFTStruc.SStaT0 = storage.SolarStartMin;
    temp = String(TFTStruc.SStaT0);
    myNex.writeStr(F("page0.vaSolT0.txt"), temp);
    if (CurrentPage == 2)  myNex.writeStr(F("SolT0.txt"), temp);
  }

  if (storage.SolarStopMax != TFTStruc.SStoT1 || !refresh)
  {
    TFTStruc.SStoT1 = storage.SolarStopMax;
    temp = String(TFTStruc.SStoT1);
    myNex.writeStr(F("page0.vaSolT1.txt"), temp);
    if (CurrentPage == 2)  myNex.writeStr(F("SolT1.txt"), temp);
  }

  if ((ChlPump.UpTime != TFTStruc.OrpPpRT) || !refresh)
  {
    TFTStruc.OrpPpRT = ChlPump.UpTime;

    temp = String(float(TFTStruc.OrpPpRT)/1000./60., 1) + F("min");
    myNex.writeStr(F("page0.vaOrpd.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("Orpd.txt"), temp);
  }

  if (((int)ChlPump.GetTankFill() != TFTStruc.OrpTkFill) || !refresh)
  {
    TFTStruc.OrpTkFill = (int)round(ChlPump.GetTankFill());

    temp = String(TFTStruc.OrpTkFill);
    int tempInt = TFTStruc.OrpTkFill;
    myNex.writeStr(F("page0.vaOrpTk.txt"), temp);
    myNex.writeNum(F("page0.vaOrpPg.val"), tempInt);
    if (CurrentPage == 0)  myNex.writeStr(F("OrpTk.txt"), temp);
      myNex.writeNum(F("vaOrpPg.val"), tempInt);
  }

  if ((PhPump.UpTime != TFTStruc.pHPpRT) || !refresh)
  {
    TFTStruc.pHPpRT = PhPump.UpTime;

    temp = String(float(TFTStruc.pHPpRT)/1000./60., 1) + F("min");
    myNex.writeStr(F("page0.vapHd.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("pHd.txt"), temp);
  }

  if (((int)PhPump.GetTankFill() != TFTStruc.pHTkFill) || !refresh)
  {
    TFTStruc.pHTkFill = (int)round(PhPump.GetTankFill());

    temp = String(TFTStruc.pHTkFill);
    int tempInt = TFTStruc.pHTkFill;
    myNex.writeStr(F("page0.vapHTk.txt"), temp);
    myNex.writeNum(F("page0.vapHPg.val"), tempInt);
    if (CurrentPage == 0)  myNex.writeStr(F("pHTk.txt"), temp);
    myNex.writeNum(F("vapHPg.val"), tempInt);
  }

  if (storage.AutoMode != TFTStruc.Mode || !refresh)
  {
    if ((debounceM == 0) || (debounceM > debounceCount))
    {
      debounceM = 0;
      TFTStruc.Mode = storage.AutoMode;
      temp = TFTStruc.Mode ? F("AUTO") : F("MANU");
      myNex.writeNum(F("page1.vabMode.val"), storage.AutoMode);
      if (CurrentPage == 0) myNex.writeStr(F("p0Mode.txt"), temp);
      else if (CurrentPage == 1)
      {
        myNex.writeStr(F("p1Mode.txt"), temp);
        myNex.writeStr(F("t1Mode.txt"), temp);
        if (storage.AutoMode == 1) myNex.writeStr(F("bMode.picc=9"));
        else
          myNex.writeStr(F("bMode.picc=8"));
      }
      else if (CurrentPage == 2) myNex.writeStr(F("p2Mode.txt"), temp);
      else if (CurrentPage == 3) myNex.writeStr(F("p3Mode.txt"), temp);
      else if (CurrentPage == 4) myNex.writeStr(F("p4Mode.txt"), temp);
      else if (CurrentPage == 5) myNex.writeStr(F("p5Mode.txt"), temp);
      else if (CurrentPage == 6) myNex.writeStr(F("p6Mode.txt"), temp);
      else if (CurrentPage == 7) myNex.writeStr(F("p7Mode.txt"), temp);
      else if (CurrentPage == 8) myNex.writeStr(F("p8Mode.txt"), temp);
      else if (CurrentPage == 9) myNex.writeStr(F("p9Mode.txt"), temp);
      else if (CurrentPage == 10) myNex.writeStr(F("p10Mode.txt"), temp);
      else if (CurrentPage == 11) myNex.writeStr(F("p11Mode.txt"), temp);
      else if (CurrentPage == 12) myNex.writeStr(F("p12Mode.txt"), temp);
      else if (CurrentPage == 13) myNex.writeStr(F("p13Mode.txt"), temp);
      else if (CurrentPage == 14) myNex.writeStr(F("p14Mode.txt"), temp);
      else if (CurrentPage == 15) myNex.writeStr(F("p15Mode.txt"), temp);
      else if (CurrentPage == 16) myNex.writeStr(F("p16Mode.txt"), temp);
      else if (CurrentPage == 17) myNex.writeStr(F("p17Mode.txt"), temp);
      else if (CurrentPage == 18) myNex.writeStr(F("p18Mode.txt"), temp);
      else if (CurrentPage == 19) myNex.writeStr(F("p19Mode.txt"), temp);
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

  if (storage.SolarMode != TFTStruc.SolarMode || !refresh)
  {
    if ((debounceSolM == 0) || (debounceSolM > debounceCount))
    {
      debounceSolM = 0;
      TFTStruc.SolarMode = storage.SolarMode;
      myNex.writeNum(F("page4.vabSolMode.val"), TFTStruc.SolarMode);
      if (CurrentPage == 2)
      {
        if (TFTStruc.SolarMode == 1)
        myNex.writeStr(F("bSolMode.picc=11"));
        else
        myNex.writeStr(F("bSolMode.picc=10"));
      }
    }
    else
      debounceSolM++;
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
    temp = String(TFTStruc.ELDTstate.c_str());
    myNex.writeStr(F("page4.vaELDTstate.txt"), temp);
    if (CurrentPage == 4)
    {
      myNex.writeStr(F("ELDTstate.txt"), temp);
      if (temp == "AUF")
      {
        myNex.writeStr(F("bValNoT.picc=14"));
        myNex.writeStr(F("ELDTstate.picc=14"));
        myNex.writeStr(F("ELDTstate.pco=65535")); 
      }
      else if (temp == "HALB")
      {
        myNex.writeStr(F("bValNoT.picc=16"));
        myNex.writeStr(F("ELDTstate.picc=16"));
        myNex.writeStr(F("ELDTstate.pco=0"));
      }
      else if (temp == "ZU")
      {
        myNex.writeStr(F("bValNoT.picc=15"));
        myNex.writeStr(F("ELDTstate.picc=15"));
        myNex.writeStr(F("ELDTstate.pco=0")); 
      }
      else if (temp == "öffne" || "schließe")
      {
        myNex.writeStr(F("bValNoT.picc=14"));
        myNex.writeStr(F("ELDTstate.picc=14"));
        myNex.writeStr(F("ELDTstate.pco=2016"));  
      }
      else if (temp == "calibr...")
      {
        myNex.writeStr(F("bValNoT.picc=14"));
        myNex.writeStr(F("ELDTstate.picc=14"));
        myNex.writeStr(F("ELDTstate.pco=63488"));
      }
      else if (ELD_Treppe.CurrentAngle() <= (ELD_Treppe.StartAngle() + 5))
      {
        myNex.writeStr(F("bValNoT.picc=15"));
        myNex.writeStr(F("ELDTstate.picc=15"));
        myNex.writeStr(F("ELDTstate.pco=0"));
      }
      else if (ELD_Treppe.CurrentAngle() >= (ELD_Treppe.StartAngle() + 5) && ELD_Treppe.CurrentAngle() <= (ELD_Treppe.HalfAngle() + ((ELD_Treppe.MaxAngle() - ELD_Treppe.HalfAngle())) / 2))
      {
        myNex.writeStr(F("bValNoT.picc=16"));
        myNex.writeStr(F("ELDTstate.picc=16"));
        myNex.writeStr(F("ELDTstate.pco=0"));
      }
      else if (ELD_Treppe.CurrentAngle() >= (ELD_Treppe.HalfAngle() + ((ELD_Treppe.MaxAngle() - ELD_Treppe.HalfAngle())) / 2))
      {
        myNex.writeStr(F("bValNoT.picc=14"));
        myNex.writeStr(F("ELDTstate.picc=14"));
        myNex.writeStr(F("ELDTstate.pco=65535"));
      }
    }
  }

if (ELD_Hinten.getStatus() != TFTStruc.ELDHstate || !refresh)
  {
    TFTStruc.ELDHstate = ELD_Hinten.getStatus();
    temp = String(TFTStruc.ELDHstate.c_str());
    myNex.writeStr(F("page4.vaELDHstate.txt"), temp);
    if (CurrentPage == 4)
    {
      myNex.writeStr(F("ELDHstate.txt"), temp);
      if (temp == "AUF")
      {
        myNex.writeStr(F("bValNoH.picc=14"));
        myNex.writeStr(F("ELDHstate.picc=14"));
        myNex.writeStr(F("ELDHstate.pco=65535")); 
      }
      else if (temp == "HALB")
      {
        myNex.writeStr(F("bValNoH.picc=16"));
        myNex.writeStr(F("ELDHstate.picc=16"));
        myNex.writeStr(F("ELDHstate.pco=0"));
      }
      else if (temp == "ZU")
      {
        myNex.writeStr(F("bValNoH.picc=15"));
        myNex.writeStr(F("ELDHstate.picc=15"));
        myNex.writeStr(F("ELDHstate.pco=0")); 
      }
      else if (temp == "öffne" || "schließe")
      {
        myNex.writeStr(F("bValNoH.picc=14"));
        myNex.writeStr(F("ELDHstate.picc=14"));
        myNex.writeStr(F("ELDHstate.pco=2016"));  
      }
      else if (temp == "calibr...")
      {
        myNex.writeStr(F("bValNoH.picc=14"));
        myNex.writeStr(F("ELDHstate.picc=14"));
        myNex.writeStr(F("ELDHstate.pco=63488"));
      }
      else if (ELD_Hinten.CurrentAngle() <= (ELD_Hinten.StartAngle() + 5))
      {
        myNex.writeStr(F("bValNoH.picc=15"));
        myNex.writeStr(F("ELDHstate.picc=15"));
        myNex.writeStr(F("ELDHstate.pco=0"));
      }
      else if (ELD_Hinten.CurrentAngle() >= (ELD_Hinten.StartAngle() + 5) && ELD_Hinten.CurrentAngle() <= (ELD_Hinten.HalfAngle() + ((ELD_Hinten.MaxAngle() - ELD_Hinten.HalfAngle())) / 2))
      {
        myNex.writeStr(F("bValNoH.picc=16"));
        myNex.writeStr(F("ELDHstate.picc=16"));
        myNex.writeStr(F("ELDHstate.pco=0"));
      }
      else if (ELD_Hinten.CurrentAngle() >= (ELD_Hinten.HalfAngle() + ((ELD_Hinten.MaxAngle() - ELD_Hinten.HalfAngle())) / 2))
      {
        myNex.writeStr(F("bValNoH.picc=14"));
        myNex.writeStr(F("ELDHstate.picc=14"));
        myNex.writeStr(F("ELDHstate.pco=65535"));
      }
    }
  }

  if ((WP_Vorlauf.getStatus() != TFTStruc.WPVstate) || !refresh)
  {
    TFTStruc.WPVstate = WP_Vorlauf.getStatus();

    temp = String(TFTStruc.WPVstate.c_str());
    myNex.writeStr(F("page4.vaWPVstate.txt"), temp);
    if (CurrentPage == 4)
    {
      myNex.writeStr(F("WPVstate.txt"), temp);
      if (temp == "AUF")
      {
        myNex.writeStr(F("bValWPV.picc=14"));
        myNex.writeStr(F("WPVstate.picc=14"));
        myNex.writeStr(F("WPVstate.pco=65535")); 
      }
      else if (temp == "HALB")
      {
        myNex.writeStr(F("bValWPV.picc=16"));
        myNex.writeStr(F("WPVstate.picc=16"));
        myNex.writeStr(F("WPVstate.pco=0"));
      }
      else if (temp == "ZU")
      {
        myNex.writeStr(F("bValWPV.picc=15"));
        myNex.writeStr(F("WPVstate.picc=15"));
        myNex.writeStr(F("WPVstate.pco=0")); 
      }
      else if (temp == "öffne" || "schließe")
      {
        myNex.writeStr(F("bValWPV.picc=14"));
        myNex.writeStr(F("WPVstate.picc=14"));
        myNex.writeStr(F("WPVstate.pco=2016"));  
      }
      else if (temp == "calibr...")
      {
        myNex.writeStr(F("bValWPV.picc=14"));
        myNex.writeStr(F("WPVstate.picc=14"));
        myNex.writeStr(F("WPVstate.pco=63488"));
      }
      else if (WP_Vorlauf.CurrentAngle() <= (WP_Vorlauf.StartAngle() + 5))
      {
        myNex.writeStr(F("bValWPV.picc=15"));
        myNex.writeStr(F("WPVstate.picc=15"));
        myNex.writeStr(F("WPVstate.pco=0"));
      }
      else if (WP_Vorlauf.CurrentAngle() >= (WP_Vorlauf.StartAngle() + 5) && WP_Vorlauf.CurrentAngle() <= (WP_Vorlauf.HalfAngle() + ((WP_Vorlauf.MaxAngle() - WP_Vorlauf.HalfAngle())) / 2))
      {
        myNex.writeStr(F("bValWPV.picc=16"));
        myNex.writeStr(F("WPVstate.picc=16"));
        myNex.writeStr(F("WPVstate.pco=0"));
      }
      else if (WP_Vorlauf.CurrentAngle() >= (WP_Vorlauf.HalfAngle() + ((WP_Vorlauf.MaxAngle() - WP_Vorlauf.HalfAngle())) / 2))
      {
        myNex.writeStr(F("bValWPV.picc=14"));
        myNex.writeStr(F("WPVstate.picc=14"));
        myNex.writeStr(F("WPVstate.pco=65535"));
      }
    }
  }

  if ((WP_Mischer.getStatus() != TFTStruc.WPMstate) || !refresh)
  {
    TFTStruc.WPMstate = WP_Mischer.getStatus();

    temp = String(TFTStruc.WPMstate.c_str());
    myNex.writeStr(F("page4.vaWPMstate.txt"), temp);
    if (CurrentPage == 4)
    {
      myNex.writeStr(F("WPMstate.txt"), temp);
      if (temp == "AUF")
      {
        myNex.writeStr(F("bValWPM.picc=14"));
        myNex.writeStr(F("WPMstate.picc=14"));
        myNex.writeStr(F("WPMstate.pco=65535")); 
      }
      else if (temp == "HALB")
      {
        myNex.writeStr(F("bValWPM.picc=16"));
        myNex.writeStr(F("WPMstate.picc=16"));
        myNex.writeStr(F("WPMstate.pco=0"));
      }
      else if (temp == "ZU")
      {
        myNex.writeStr(F("bValWPM.picc=15"));
        myNex.writeStr(F("WPMstate.picc=15"));
        myNex.writeStr(F("WPMstate.pco=0")); 
      }
      else if (temp == "öffne" || "schließe")
      {
        myNex.writeStr(F("bValWPM.picc=14"));
        myNex.writeStr(F("WPMstate.picc=14"));
        myNex.writeStr(F("WPMstate.pco=2016"));  
      }
      else if (temp == "calibr...")
      {
        myNex.writeStr(F("bValWPM.picc=14"));
        myNex.writeStr(F("WPMstate.picc=14"));
        myNex.writeStr(F("WPMstate.pco=63488"));
      }
      else if (WP_Mischer.CurrentAngle() <= (WP_Mischer.StartAngle() + 5))
      {
        myNex.writeStr(F("bValWPM.picc=15"));
        myNex.writeStr(F("WPMstate.picc=15"));
        myNex.writeStr(F("WPMstate.pco=0"));
      }
      else if (WP_Mischer.CurrentAngle() >= (WP_Mischer.StartAngle() + 5) && WP_Mischer.CurrentAngle() <= (WP_Mischer.HalfAngle() + ((WP_Mischer.MaxAngle() - WP_Mischer.HalfAngle())) / 2))
      {
        myNex.writeStr(F("bValWPM.picc=16"));
        myNex.writeStr(F("WPMstate.picc=16"));
        myNex.writeStr(F("WPMstate.pco=0"));
      }
      else if (WP_Mischer.CurrentAngle() >= (WP_Mischer.HalfAngle() + ((WP_Mischer.MaxAngle() - WP_Mischer.HalfAngle())) / 2))
      {
        myNex.writeStr(F("bValWPM.picc=14"));
        myNex.writeStr(F("WPMstate.picc=14"));
        myNex.writeStr(F("WPMstate.pco=65535"));
      }
    }
  }

  if ((Bodenablauf.getStatus() != TFTStruc.BOTTstate) || !refresh)
  {
    TFTStruc.BOTTstate = Bodenablauf.getStatus();

    temp = String(TFTStruc.BOTTstate.c_str());
    myNex.writeStr(F("page4.vaBOTTstate.txt"), temp);
    if (CurrentPage == 4)  myNex.writeStr(F("BOTTstate.txt"), temp);
    {
      myNex.writeStr(F("BOTTstate.txt"), temp);
      if (temp == "AUF")
      {
        myNex.writeStr(F("bValBott.picc=14"));
        myNex.writeStr(F("BOTTstate.picc=14"));
        myNex.writeStr(F("BOTTstate.pco=65535")); 
      }
      else if (temp == "HALB")
      {
        myNex.writeStr(F("bValBott.picc=16"));
        myNex.writeStr(F("BOTTstate.picc=16"));
        myNex.writeStr(F("BOTTstate.pco=0"));
      }
      else if (temp == "ZU")
      {
        myNex.writeStr(F("bValBott.picc=15"));
        myNex.writeStr(F("BOTTstate.picc=15"));
        myNex.writeStr(F("BOTTstate.pco=0")); 
      }
      else if (temp == "öffne" || "schließe")
      {
        myNex.writeStr(F("bValBott.picc=14"));
        myNex.writeStr(F("BOTTstate.picc=14"));
        myNex.writeStr(F("BOTTstate.pco=2016"));  
      }
      else if (temp == "calibr...")
      {
        myNex.writeStr(F("bValBott.picc=14"));
        myNex.writeStr(F("BOTTstate.picc=14"));
        myNex.writeStr(F("BOTTstate.pco=63488"));
      }
      else if (Bodenablauf.CurrentAngle() <= (Bodenablauf.StartAngle() + 5))
      {
        myNex.writeStr(F("bValBott.picc=15"));
        myNex.writeStr(F("BOTTstate.picc=15"));
        myNex.writeStr(F("BOTTstate.pco=0"));
      }
      else if (Bodenablauf.CurrentAngle() >= (Bodenablauf.StartAngle() + 5) && Bodenablauf.CurrentAngle() <= (Bodenablauf.HalfAngle() + ((Bodenablauf.MaxAngle() - Bodenablauf.HalfAngle())) / 2))
      {
        myNex.writeStr(F("bValBott.picc=16"));
        myNex.writeStr(F("BOTTstate.picc=16"));
        myNex.writeStr(F("WPMstate.pco=0"));
      }
      else if (Bodenablauf.CurrentAngle() >= (Bodenablauf.HalfAngle() + ((Bodenablauf.MaxAngle() - Bodenablauf.HalfAngle())) / 2))
      {
        myNex.writeStr(F("bValBott.picc=14"));
        myNex.writeStr(F("BOTTstate.picc=14"));
        myNex.writeStr(F("BOTTstate.pco=65535"));
      }
    }
  }

  if ((Solarvalve.getStatus() != TFTStruc.SOLARstate) || !refresh)
  {
    TFTStruc.SOLARstate = Solarvalve.getStatus();

    temp = String(TFTStruc.SOLARstate.c_str());
    myNex.writeStr(F("page2.vaSOLVstate.txt"), temp);
    if (CurrentPage == 2)  myNex.writeStr(F("SOLARstate.txt"), temp);
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

  if (SolarPump.IsRunning() != TFTStruc.SolarPump || !refresh)
  {
    if ((debounceSolP == 0) || (debounceSolP > debounceCount))
    {
      debounceSolP = 0;
      TFTStruc.SolarPump = SolarPump.IsRunning();
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

  if (storage.SaltPolarity != TFTStruc.SaltPolarity || !refresh)
  {
      TFTStruc.SaltPolarity = storage.SaltPolarity;
      temp = TFTStruc.SaltPolarity ? F("DIREKT") : F("VERPOLT");
      myNex.writeStr(F("page0.vaSaltDir.txt"), temp);
    if (CurrentPage == 13)  myNex.writeStr(F("SaltDir.txt"), temp);
  }

  if (storage.SaltDiff != TFTStruc.SaltDiff || !refresh)
  {
    TFTStruc.SaltDiff = storage.SaltDiff;
    temp = String(TFTStruc.SaltDiff);
    myNex.writeStr(F("page0.vaSaltDiff.txt"), temp);
    if (CurrentPage == 13)  myNex.writeStr(F("SaltDiff.txt"), temp);
  }

  if ((FiltrationPump.UpTime != TFTStruc.FLRT) || !refresh)
  {
    TFTStruc.FLRT = FiltrationPump.UpTime;
    int Sec = TFTStruc.FLRT/1000;
    int Min = Sec/60; Sec%=60;
    int Std = Min/60; Min%=60;

    char temp[10];
    sprintf(temp, "%02d : %02d", Std, Min);

    myNex.writeStr(F("page0.vaFiltDur.txt"), temp);
    if (CurrentPage == 1)  myNex.writeStr(F("FiltDur.txt"), temp);
  }

  if ((WaterFill.UpTime != TFTStruc.WFRT) || !refresh)
  {
    TFTStruc.WFRT = WaterFill.UpTime;
    int Sec = TFTStruc.WFRT/1000;
    int Min = Sec/60; Sec%=60;
    int Std = Min/60; Min%=60;

    char temp[10];
    sprintf(temp, "%02d : %02d", Std, Min);

    myNex.writeStr(F("page0.vaWFDur.txt"), temp);
    if (CurrentPage == 10)  myNex.writeStr(F("WFDur.txt"), temp);
  }

  if ((storage.WaterFillAnCon != TFTStruc.WFAC) || !refresh)
{
    TFTStruc.WFAC = storage.WaterFillAnCon;
    temp = String(TFTStruc.WFAC);
    String formattedTemp = "";
    int counter = 0;
    for (int i = temp.length() - 1; i >= 0; i--) {
        formattedTemp = temp[i] + formattedTemp;
        counter++;
        if (counter == 3 && i != 0) {
            formattedTemp = "." + formattedTemp;
            counter = 0;
        }
    }
    myNex.writeStr(F("page0.vaWFAnCon.txt"), formattedTemp);
    if (CurrentPage == 10)  myNex.writeStr(F("WFAnCon.txt"), formattedTemp);
}

  if ((SaltPump.UpTime != TFTStruc.SPUT) || !refresh)
  {
    TFTStruc.SPUT = SaltPump.UpTime;
    int Sec = TFTStruc.SPUT/1000;
    int Min = Sec/60; Sec%=60;
    int Std = Min/60; Min%=60;
    
    char temp[10];
    sprintf(temp, "%02d : %02d", Std, Min);

    myNex.writeStr(F("page0.vaSaltDur.txt"), temp);
    if (CurrentPage == 13)  myNex.writeStr(F("SaltDur.txt"), temp);
  }

  if ((storage.SaltPumpRunTime != TFTStruc.SPRT) || !refresh)
  {
    TFTStruc.SPRT = storage.SaltPumpRunTime;
    int Sec = TFTStruc.SPRT/1000;
    int Min = Sec/60; Sec%=60;
    int Std = Min/60; Min%=60;
    
    char temp[10];
    sprintf(temp, "%02d : %02d", Std, Min);

    myNex.writeStr(F("page0.vaSaltRT.txt"), temp);
    if (CurrentPage == 13)  myNex.writeStr(F("SaltRT.txt"), temp);
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
        if (TFTStruc.ChlPump == 16)
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
    int Sec = TFTStruc.HPRT/1000;
    int Min = Sec/60; Sec%=60;
    int Std = Min/60; Min%=60;
    
    char temp[10];
    sprintf(temp, "%02d : %02d", Std, Min);

    myNex.writeStr(F("page0.vaWPDur.txt"), temp);
    if (CurrentPage == 3)  myNex.writeStr(F("WPDur.txt"), temp);
  }

  if ((SolarPump.UpTime != TFTStruc.SHRT) || !refresh)
  {
    TFTStruc.SHRT = SolarPump.UpTime;
    int Sec = TFTStruc.SHRT/1000;
    int Min = Sec/60; Sec%=60;
    int Std = Min/60; Min%=60;
    
    char temp[10];
    sprintf(temp, "%02d : %02d", Std, Min);

    myNex.writeStr(F("page0.vaSolDur.txt"), temp);
    if (CurrentPage == 2)  myNex.writeStr(F("SolDur.txt"), temp);
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
    temp = String(float(TFTStruc.PubInt)/1.6, 0);
    myNex.writeStr(F("page0.vaPubInt.txt"), temp);
    if (CurrentPage == 11)  myNex.writeStr(F("PubInt.txt"), temp);
  }

  if (storage.DelayPIDs != TFTStruc.DelayPID || !refresh)
  {
    TFTStruc.DelayPID = storage.DelayPIDs;
    temp = String(TFTStruc.DelayPID);
    myNex.writeStr(F("page0.vaDelayPID.txt"), temp);
    if (CurrentPage == 14)  myNex.writeStr(F("DelayPID.txt"), temp);
  }

  if ((storage.Ph_Kp != TFTStruc.Ph_Kp) || (storage.Ph_Ki != TFTStruc.Ph_Ki) || (storage.Ph_Kd != TFTStruc.Ph_Kd) || !refresh)
  {
    TFTStruc.Ph_Kp = storage.Ph_Kp;
    TFTStruc.Ph_Ki = storage.Ph_Ki;
    TFTStruc.Ph_Kd = storage.Ph_Kd;
    temp = String(float(TFTStruc.Ph_Kp)/10000, 1) +F("/") + String(TFTStruc.Ph_Ki, 0) +F("/") + String(TFTStruc.Ph_Kd, 1);
    myNex.writeStr(F("page0.vapHPIDD.txt"), temp);
    if (CurrentPage == 15)  myNex.writeStr(F("pHPIDD.txt"), temp);
  }

  if ((storage.Orp_Kp != TFTStruc.Orp_Kp) || (storage.Orp_Ki != TFTStruc.Orp_Ki) || (storage.Orp_Kd != TFTStruc.Orp_Kd) || !refresh)
  {
    TFTStruc.Orp_Kp = storage.Orp_Kp;
    TFTStruc.Orp_Ki = storage.Orp_Ki;
    TFTStruc.Orp_Kd = storage.Orp_Kd;
    temp = String(float(TFTStruc.Orp_Kp)/10000, 1) +F("/") + String(TFTStruc.Orp_Ki, 0) +F("/") + String(TFTStruc.Orp_Kd, 1);
    myNex.writeStr(F("page0.vaOrpPIDD.txt"), temp);
    if (CurrentPage == 16)  myNex.writeStr(F("OrpPIDD.txt"), temp);
  } 

  if (storage.PhPIDWindowSize != TFTStruc.pHPIDW || !refresh)
  {
    TFTStruc.pHPIDW = storage.PhPIDWindowSize;
    temp = String(float(TFTStruc.pHPIDW)/1000./60., 0);
    myNex.writeStr(F("page0.vapHPIDW.txt"), temp);
    if (CurrentPage == 15)  myNex.writeStr(F("pHPIDW.txt"), temp);
  }

  if (storage.OrpPIDWindowSize != TFTStruc.OrpPIDW || !refresh)
  {
    TFTStruc.OrpPIDW = storage.OrpPIDWindowSize;
    temp = String(float(TFTStruc.OrpPIDW)/1000./60., 0);
    myNex.writeStr(F("page0.vaOrpPIDW.txt"), temp);
    if (CurrentPage == 16)  myNex.writeStr(F("OrpPIDW.txt"), temp);
  }

  if (storage.pHPumpFR != TFTStruc.pHPumpFR || !refresh)
  {
    TFTStruc.pHPumpFR = storage.pHPumpFR;
    temp = String(TFTStruc.pHPumpFR, 1);
    myNex.writeStr(F("page0.vapHPumpFR.txt"), temp);
    if (CurrentPage == 15)  myNex.writeStr(F("pHPumpFR.txt"), temp);
  }

  if (storage.ChlPumpFR != TFTStruc.ChlPumpFR || !refresh)
  {
    TFTStruc.ChlPumpFR = storage.ChlPumpFR;
    temp = String(TFTStruc.ChlPumpFR, 1);
    myNex.writeStr(F("page0.vaChlPumpFR.txt"), temp);
    if (CurrentPage == 16)  myNex.writeStr(F("ChlPumpFR.txt"), temp);
  }

  if (storage.PhPumpUpTimeLimit != TFTStruc.PumpMaxUp || !refresh)
  {
    TFTStruc.PumpMaxUp = storage.PhPumpUpTimeLimit / 60000; // milliseconds in minutes
    temp = String(TFTStruc.PumpMaxUp);
    myNex.writeStr(F("page0.vaPumpsMaxUp.txt"), temp);
    if (CurrentPage == 14)  myNex.writeStr(F("PumpsMaxUp.txt"), temp);
  }

  if (storage.WaterFillFR != TFTStruc.WaterFillFR || !refresh)
  {
    TFTStruc.WaterFillFR = storage.WaterFillFR;
    temp = String(TFTStruc.WaterFillFR, 1);
    myNex.writeStr(F("page0.vaWFFR.txt"), temp);
    if (CurrentPage == 17)  myNex.writeStr(F("WFFR.txt"), temp);
  }

  if (storage.WaterFillUpTimeLimit != TFTStruc.WFMaxUp || !refresh)
  {
    TFTStruc.WFMaxUp = storage.WaterFillUpTimeLimit / 60000; // milliseconds in minutes
    temp = String(TFTStruc.WFMaxUp);
    myNex.writeStr(F("page0.vaWFMaxUp.txt"), temp);
    if (CurrentPage == 17)  myNex.writeStr(F("WFMaxUp.txt"), temp);
  }

  if (storage.WaterFillDuration != TFTStruc.FillDur || !refresh)
  {
    TFTStruc.FillDur = storage.WaterFillDuration / 60000; // milliseconds in minutes
    temp = String(TFTStruc.FillDur);
    myNex.writeStr(F("page0.vaFillDur.txt"), temp);
    if (CurrentPage == 17)  myNex.writeStr(F("FillDur.txt"), temp);
  }

  if(CurrentPage == 2) {
    myNex.writeStr(F("page0.vapH.txt"), String(storage.PhValue, 2));
    myNex.writeStr(F("page0.vaOrp.txt"), String(storage.OrpValue, 0));
  }

  //update time at top of displayed page
  switch (CurrentPage)
  {
    case 0: {
        myNex.writeStr(F("p0Time.txt"), HourBuffer);       
        break;
      }
    case 1: {
        myNex.writeStr(F("p1Time.txt"), HourBuffer);
        break;
      }
    case 2: {
        myNex.writeStr(F("p2Time.txt"), HourBuffer);      
        break;
      }
    case 3: {
        myNex.writeStr(F("p3Time.txt"), HourBuffer);      
        break;
      }
    case 4: {
        myNex.writeStr(F("p4Time.txt"), HourBuffer);      
        break;
      }
    case 5: {
        myNex.writeStr(F("p5Time.txt"), HourBuffer);      
        break;
      }
    case 6: {
        myNex.writeStr(F("p6Time.txt"), HourBuffer);      
        break;
      }
    case 7: {
        myNex.writeStr(F("p7Time.txt"), HourBuffer);      
        break;
      }
    case 8: {
        myNex.writeStr(F("p8Time.txt"), HourBuffer);      
        break;
      }
    case 9: {
        myNex.writeStr(F("p9Time.txt"), HourBuffer);      
        break;
      }
    case 10: {
        myNex.writeStr(F("p10Time.txt"), HourBuffer);      
        break;
      }
    case 11: {
        myNex.writeStr(F("p11Time.txt"), HourBuffer);      
        break;
      }
    case 12: {
        myNex.writeStr(F("p12Time.txt"), HourBuffer);      
        break;
      }
    case 13: {
        myNex.writeStr(F("p13Time.txt"), HourBuffer);      
        break;
      }
    case 14: {
        myNex.writeStr(F("p14Time.txt"), HourBuffer);      
        break;
      }
    case 15: {
        myNex.writeStr(F("p15Time.txt"), HourBuffer);      
        break;
      }
    case 16: {
        myNex.writeStr(F("p16Time.txt"), HourBuffer);      
        break;
      }
    case 17: {
        myNex.writeStr(F("p17Time.txt"), HourBuffer);      
        break;
      }
    case 18: {
        myNex.writeStr(F("p18Time.txt"), HourBuffer);      
        break;
      }
    case 19: {
        myNex.writeStr(F("p19Time.txt"), HourBuffer);      
        break;
      }
  }
  //put TFT in sleep mode with wake up on touch and force page 0 load to trigger an event
  if((unsigned long)(millis() - LastAction) >= TFT_SLEEP && TFT_ON && CurrentPage != 2)
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
  CurrentPage = 0;
  if(!TFT_ON)
  {
    UpdateWiFi(WiFi.status() == WL_CONNECTED);
    TFT_ON = true;  
    refresh = false;
  }
  LastAction = millis();
}

//Page 1 has finished loading
//printh 23 02 54 02
void trigger2()
{
  CurrentPage = 1;
  LastAction = millis();
}

//Page 2 has finished loading
//printh 23 02 54 03
void trigger3()
{
  CurrentPage = 2;
  LastAction = millis();  
}

//Page 3 has finished loading
//printh 23 02 54 04
void trigger4()
{
  CurrentPage = 3;
  LastAction = millis();
}

//MODE button was toggled
//printh 23 02 54 05
void trigger5()
{
  TFTStruc.Mode = (boolean)myNex.readNumber(F("vabMode.val"));
  debounceM = 1;
  Debug.print(DBG_VERBOSE,"MODE button");
  char Cmd[100] = "{\"Mode\":0}";
  if(TFTStruc.Mode) Cmd[8] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion Mode cmd: %s",Cmd);
  LastAction = millis();
}

//FILT button was toggled
//printh 23 02 54 06
void trigger6()
{
  TFTStruc.Filt = (boolean)myNex.readNumber(F("vabFilt.val"));
  debounceF = 1;
  Debug.print(DBG_VERBOSE,"FILT button");
  char Cmd[100] = "{\"FiltPump\":0}";
  if(TFTStruc.Filt) Cmd[12] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion Filt command: %s",Cmd);
  LastAction = millis();
}

//Robot button was toggled
//printh 23 02 54 07
void trigger7()
{
  TFTStruc.Robot = (boolean)myNex.readNumber(F("vabRobot.val"));
  debounceH = 1;
  Debug.print(DBG_VERBOSE,"Robot button");
  char Cmd[100] = "{\"RobotPump\":0}";
  if(TFTStruc.Robot) Cmd[13] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion Robot command: %s",Cmd);
  LastAction = millis();
}

//Relay 0 button was toggled
//printh 23 02 54 08
void trigger8()
{
  TFTStruc.R0 = (boolean)myNex.readNumber(F("vabR0.val"));
  debounceR0 = 1;
  Debug.print(DBG_VERBOSE,"Relay 0 button");
  char Cmd[100] = "{\"Relay\":[0,0]}";
  if(TFTStruc.R0) Cmd[12] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion R0 command: %s",Cmd);
  LastAction = millis();
}

//Relay 1 button was toggled
//printh 23 02 54 09
void trigger9()
{
  TFTStruc.R1 = (boolean)myNex.readNumber(F("vabR1.val"));
  debounceR1 = 1;
  Debug.print(DBG_VERBOSE,"Relay 1 button");
  char Cmd[100] = "{\"Relay\":[1,0]}";
  if(TFTStruc.R1) Cmd[12] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion R1 command: %s",Cmd);
  LastAction = millis();
}

//Winter button was toggled
//printh 23 02 54 0A
void trigger10()
{
  TFTStruc.R2 = (boolean)myNex.readNumber(F("vabWinMode.val"));
  debounceR2 = 1;
  Debug.print(DBG_VERBOSE,"Winter button");
  char Cmd[100] = "{\"Winter\":0}";
  if(TFTStruc.R2) Cmd[10] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion Winter command: %s",Cmd);
  LastAction = millis();
}

//Probe calibration completed or new pH, Orp or Water Temp setpoints or New tank
//printh 23 02 54 0B
void trigger11()
{
  Debug.print(DBG_VERBOSE,"Calibration complete or new pH, Orp, Water Temp, Backwash Trigger, Filterpressure max setpoints, MotoValve position or new tank event");
  char Cmd[100] = "";
  strcpy(Cmd,myNex.readStr(F("pageCalibs.vaCommand.txt")).c_str());
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion cal page command: %s",Cmd);
  LastAction = millis();
}

//Clear Errors button pressed
//printh 23 02 54 0C
void trigger12()
{
  Debug.print(DBG_VERBOSE,"Clear errors event");
  char Cmd[100] = "{\"Clear\":1}";
  xQueueSendToBack(queueIn,&Cmd,0);
  LastAction = millis();
}

//pH PID button pressed
//printh 23 02 54 0D
void trigger13()
{
  TFTStruc.PIDpH = (boolean)myNex.readNumber(F("page15.vabpHMode.val"));
  debouncepHP = 1;
  Debug.print(DBG_VERBOSE,"pH PID button");
  char Cmd[100] = "{\"PhPID\":0}";
  if(TFTStruc.PIDpH) Cmd[9] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion pH PID cmd: %s",Cmd);
  LastAction = millis();
}

//Orp PID button pressed
//printh 23 02 54 0E
void trigger14()
{
  TFTStruc.PIDChl = (boolean)myNex.readNumber(F("page16.vabChlMode.val"));
  debounceChlP = 1;
  Debug.print(DBG_VERBOSE,"Orp PID button");
  char Cmd[100] = "{\"OrpPID\":0}";
  if(TFTStruc.PIDChl) Cmd[10] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion Orp PID cmd: %s",Cmd);
  LastAction = millis();
}

//HEAT MODE button was toggled
//printh 23 02 54 0F
void trigger15()
{
  TFTStruc.Heat = (boolean)myNex.readNumber(F("vabHeatMode.val"));
  debounceH = 1;
  Debug.print(DBG_VERBOSE,"HEAT MODE button");
  char Cmd[100] = "{\"Heat\":0}";
  if(TFTStruc.Heat) Cmd[8] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion Heat Mode cmd: %s",Cmd);
  LastAction = millis();
}

//Page 4 has finished loading
//printh 23 02 54 10
void trigger16()
{
  CurrentPage = 4;
  LastAction = millis();
}

//Page 5 has finished loading
//printh 23 02 54 11
void trigger17()
{
  CurrentPage = 5;
  LastAction = millis();
}

//Page 6 has finished loading
//printh 23 02 54 12
void trigger18()
{
  CurrentPage = 6;
  LastAction = millis();
}

//Page 7 has finished loading
//printh 23 02 54 13
void trigger19()
{
  CurrentPage = 7;
  LastAction = millis();
}

//Page 8 has finished loading
//printh 23 02 54 14
void trigger20()
{
  CurrentPage = 8;
  LastAction = millis();
}

//Page 9 has finished loading
//printh 23 02 54 15
void trigger21()
{
  CurrentPage = 9;
  LastAction = millis();
}

//Page 10 has finished loading
//printh 23 02 54 16
void trigger22()
{
  CurrentPage = 10;
  LastAction = millis();
}

//SALT_CHLOR button was toggled
//printh 23 02 54 17
void trigger23()
{
  TFTStruc.Salt_Chlor = (boolean)myNex.readNumber(F("vabSaltChl.val"));
  debounceSM = 1;
  Debug.print(DBG_VERBOSE,"SALT_CHLOR button");
  char Cmd[100] = "{\"Salt_Chlor\":0}";
  if(TFTStruc.Salt_Chlor) Cmd[14] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion Salt_Chlor cmd: %s",Cmd);
  LastAction = millis();
}

//Page 11 has finished loading
//printh 23 02 54 18
void trigger24()
{
  CurrentPage = 11;
  LastAction = millis();
}

//Page 12 has finished loading
//printh 23 02 54 19
void trigger25()
{
  CurrentPage = 12;
  LastAction = millis();
}

//Page 13 has finished loading
//printh 23 02 54 1A
void trigger26()
{
  CurrentPage = 13;
  LastAction = millis();
}

//Page 14 has finished loading
//printh 23 02 54 1B
void trigger27()
{
  CurrentPage = 14;
  LastAction = millis();
}

//Page 15 has finished loading
//printh 23 02 54 1C
void trigger28()
{
  CurrentPage = 15;
  LastAction = millis();
}

//pHPump button was toggled
//printh 23 02 54 1D
void trigger29()
{
  TFTStruc.PhPump = (boolean)myNex.readNumber(F("page15.vabpHPum.val"));
  debouncepH = 1;
  Debug.print(DBG_VERBOSE,"PhPump button");
  char Cmd[100] = "{\"PhPump\":0}";
  if(TFTStruc.PhPump) Cmd[10] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion PhPump command: %s",Cmd);
  LastAction = millis();
}

//Page 16 has finished loading
//printh 23 02 54 1E
void trigger30()
{
  CurrentPage = 16;
  LastAction = millis();
}

//ChlPump button was toggled
//printh 23 02 54 1F
void trigger31()
{
  TFTStruc.ChlPump = (boolean)myNex.readNumber(F("page16.vabChlPum.val"));
  debounceChl = 1;
  Debug.print(DBG_VERBOSE,"ChlPump button");
  char Cmd[100] = "{\"ChlPump\":0}";
  if(TFTStruc.ChlPump) Cmd[11] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion ChlPump command: %s",Cmd);
  LastAction = millis();
}

//Reset PSI Calib button pressed
//printh 23 02 54 20
void trigger32()
{
  Debug.print(DBG_VERBOSE,"Reset PSI Calib event");
  char Cmd[100] = "{\"RstPSICal\":1}";
  xQueueSendToBack(queueIn,&Cmd,0);
  LastAction = millis();
}

//Reboot button pressed
//printh 23 02 54 21
void trigger33()
{
  Debug.print(DBG_VERBOSE,"Reboot event");
  char Cmd[100] = "{\"Reboot\":1}";
  xQueueSendToBack(queueIn,&Cmd,0);
  LastAction = millis();
}

//Reset pH probe button pressed
//printh 23 02 54 22
void trigger34()
{
  Debug.print(DBG_VERBOSE,"Reset pH Probe event");
  char Cmd[100] = "{\"RstpHCal\":1}";
  xQueueSendToBack(queueIn,&Cmd,0);
  LastAction = millis();
}

//Reset Orp probe button pressed
//printh 23 02 54 23
void trigger35()
{
  Debug.print(DBG_VERBOSE,"Reset Orp Probe event");
  char Cmd[100] = "{\"RstOrpCal\":1}";
  xQueueSendToBack(queueIn,&Cmd,0);
  LastAction = millis();
}

//HeatPump button was toggled
//printh 23 02 54 24
void trigger36()
{
  TFTStruc.HeatPump = (boolean)myNex.readNumber(F("vabHeatPum.val"));
  debounceHP = 1;
  Debug.print(DBG_VERBOSE,"HeatPump button");
  char Cmd[100] = "{\"HeatPump\":0}";
  if(TFTStruc.HeatPump) Cmd[12] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion HeatPump command: %s",Cmd);
  LastAction = millis();
}

//SALT MODE button was toggled
//printh 23 02 54 25
void trigger37()
{
  TFTStruc.SaltMode = (boolean)myNex.readNumber(F("vabSaltMode.val"));
  debounceSM = 1;
  Debug.print(DBG_VERBOSE,"SALT MODE button");
  char Cmd[100] = "{\"SaltMode\":0}";
  if(TFTStruc.SaltMode) Cmd[12] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion SaltMode cmd: %s",Cmd);
  LastAction = millis();
}

//SaltPump button was toggled
//printh 23 02 54 26
void trigger38()
{
  TFTStruc.SaltPump = (boolean)myNex.readNumber(F("vabSaltPum.val"));
  debounceSP = 1;
  Debug.print(DBG_VERBOSE,"SaltPump button");
  char Cmd[100] = "{\"SaltPump\":0}";
  if(TFTStruc.SaltPump) Cmd[12] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion SaltPump command: %s",Cmd);
  LastAction = millis();
}

//VALVE MODE button was toggled
//printh 23 02 54 27
void trigger39()
{
  TFTStruc.ValveMode = (boolean)myNex.readNumber(F("vabValveMode.val"));
  debounceVM = 1;
  Debug.print(DBG_VERBOSE,"VALVE MODE button");
  char Cmd[100] = "{\"ValveMode\":0}";
  if(TFTStruc.ValveMode) Cmd[13] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion ValveMode cmd: %s",Cmd);
  LastAction = millis();
}

//CLEAN MODE button was toggled
//printh 23 02 54 28
void trigger40()
{
  TFTStruc.CleanMode = (boolean)myNex.readNumber(F("vabCleanMode.val"));
  debounceCM = 1;
  Debug.print(DBG_VERBOSE,"CLEAN MODE button");
  char Cmd[100] = "{\"CleanMode\":0}";
  if(TFTStruc.CleanMode) Cmd[13] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion CleanMode cmd: %s",Cmd);
  LastAction = millis();
}

//VALVE SWITCH button was toggled
//printh 23 02 54 29
void trigger41()
{
  TFTStruc.ValveSwitch = (boolean)myNex.readNumber(F("vabCleanDir.val"));
  debounceVS = 1;
  Debug.print(DBG_VERBOSE,"VALVE SWITCH button");
  char Cmd[100] = "{\"ValveSwitch\":0}";
  if(TFTStruc.ValveSwitch) Cmd[15] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion ValveSwitch cmd: %s",Cmd);
  LastAction = millis();
}

//WATERFILL MODE button was toggled
//printh 23 02 54 2A
void trigger42()
{
  TFTStruc.WaterFillMode = (boolean)myNex.readNumber(F("vabFillMode.val"));
  debounceWFM = 1;
  Debug.print(DBG_VERBOSE,"WATERFILL MODE button");
  char Cmd[100] = "{\"FillMode\":0}";
  if(TFTStruc.WaterFillMode) Cmd[12] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion FillMode cmd: %s",Cmd);
  LastAction = millis();
}

//Page 17 has finished loading
//printh 23 02 54 2B
void trigger43()
{
  CurrentPage = 17;
  LastAction = millis();
}

//Reset Anual WaterConsumtion button pressed
//printh 23 02 54 2C
void trigger44()
{
  Debug.print(DBG_VERBOSE,"Reset Anual WaterConsumtion event");
  char Cmd[100] = "{\"RstWatCons\":1}";
  xQueueSendToBack(queueIn,&Cmd,0);
  LastAction = millis();
}

//WaterFill tap button was toggled
//printh 23 02 54 2D
void trigger45()
{
  TFTStruc.WaterFill = (boolean)myNex.readNumber(F("vabTap.val"));
  debounceWF = 1;
  Debug.print(DBG_VERBOSE,"WaterFill tap button");
  char Cmd[100] = "{\"WaterFill\":0}";
  if(TFTStruc.WaterFill) Cmd[13] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion WaterFill command: %s",Cmd);
  LastAction = millis();
}

//Page 18 has finished loading
//printh 23 02 54 2E
void trigger46()
{
  CurrentPage = 18;
  LastAction = millis();
}

//WIFIOnOff button was toggled
//printh 23 02 54 2F
void trigger47()
{
  TFTStruc.WIFI_OnOff = (boolean)myNex.readNumber(F("vabWiFi_OnOff.val"));
  debounceWiFi = 1;
  Debug.print(DBG_VERBOSE,"WIFI_OnOff button");
  char Cmd[100] = "{\"WIFI_OnOff\":0}";
  if(TFTStruc.WIFI_OnOff) Cmd[14] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion WIFI_OnOff command: %s",Cmd);
  LastAction = millis();
}

//SOLAR MODE button was toggled
//printh 23 02 54 30
void trigger48()
{
  TFTStruc.SolarMode = (boolean)myNex.readNumber(F("vabSolMode.val"));
  debounceSolM = 1;
  Debug.print(DBG_VERBOSE,"SOLAR MODE button");
  char Cmd[100] = "{\"SolarMode\":0}";
  if(TFTStruc.SolarMode) Cmd[13] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion SolarMode cmd: %s",Cmd);
  LastAction = millis();
}

//SolarPump button was toggled
//printh 23 02 54 31
void trigger49()
{
  TFTStruc.SolarPump = (boolean)myNex.readNumber(F("vabSolPum.val"));
  debounceSolP = 1;
  Debug.print(DBG_VERBOSE,"SolarPump button");
  char Cmd[100] = "{\"SolarPump\":0}";
  if(TFTStruc.SolarPump) Cmd[13] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion SolarPump command: %s",Cmd);
  LastAction = millis();
}

//SOLAR LOKAL EXTERN button was toggled
//printh 23 02 54 32
void trigger50()
{
  TFTStruc.SolarLoEx = (boolean)myNex.readNumber(F("vabSolLoEx.val"));
  debounceSolLE = 1;
  Debug.print(DBG_VERBOSE,"SOLAR LOKAL EXTERN button");
  char Cmd[100] = "{\"SolarLocExt\":0}";
  if(TFTStruc.SolarLoEx) Cmd[15] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion SolarLocExt cmd: %s",Cmd);
  LastAction = millis();
}

//Page 19 has finished loading
//printh 23 02 54 33
void trigger51()
{
  CurrentPage = 19;
  LastAction = millis();
}

//MQTT Login button was toggled
//printh 23 02 54 34
void trigger52()
{
  TFTStruc.MqttLogin = (boolean)myNex.readNumber(F("vabMqttLogin.val"));
  debounceMQL = 1;
  Debug.print(DBG_VERBOSE,"MQTT Login button");
  char Cmd[100] = "{\"MqttLogin\":0}";
  if(TFTStruc.MqttLogin) Cmd[13] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion MqttLogin cmd: %s",Cmd);
  LastAction = millis();
}

//BUS_AB button was toggled
//printh 23 02 54 35
void trigger53()
{
  TFTStruc.BUSA_B = (boolean)myNex.readNumber(F("vabBUSA_B.val"));
  debounceB = 1;
  Debug.print(DBG_VERBOSE,"BUSA_B button");
  char Cmd[100] = "{\"Bus_A_B\":0}";
  if(TFTStruc.BUSA_B) Cmd[11] = '1';
  xQueueSendToBack(queueIn,&Cmd,0);
  Debug.print(DBG_VERBOSE,"Nextion BUSA_B cmd: %s",Cmd);
  LastAction = millis();
}

//Page 27 has finished loading
//printh 23 02 54 36
void trigger54()
{
  CurrentPage = 27;
  LastAction = millis();
}