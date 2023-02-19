/*
  NEXTION TFT related code, based on EasyNextion library by Seithan / Athanasios Seitanis (https://github.com/Seithan/EasyNextionLibrary)
  The trigger(s) functions at the end are called by the Nextion library on event (buttons, page change).

  (c) Loic74 <loic74650@gmail.com> 2018-2020

  Modified to implement display sleep mode.
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
static uint8_t debounceSM    = 0;
static uint8_t debounceVM    = 0;
static uint8_t debounceCM    = 0;
static uint8_t debounceVS    = 0;
static uint8_t debounceSC    = 0;
static uint8_t debounceSP    = 0;
static uint8_t debounceSPl   = 0;
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


// Structure holding the measurement values to display on the Nextion display
// Used to refresh only modified values
static struct TFTStruct
{
  float pH, Orp, pHSP, OrpSP, WT, WTSP, AT, PSI, flow, flow2, F1H, F1L, F2H, F2L, PsiH, PsiL, WTLow, pHPumpFR, ChlPumpFR, Ph_Kp, Ph_Ki, Ph_Kd, Orp_Kp, Orp_Ki, Orp_Kd;
  uint8_t FSta, FSto, FStaT0, FStoT1, SStaT0, SStoT1, pHTkFill, OrpTkFill, PIDpH, PIDChl, PubInt, PumpMaxUp, DelayPID, pHPIDW, OrpPIDW, FLOW_Pulse, FLOW2_Pulse, SaltDiff;
  boolean Mode, SaltMode, NetW, Filt, Robot, R0, R1, R2, pHUTErr, ChlUTErr, PSIErr, FLOWErr, FLOW2Err, pHTLErr, ChlTLErr, PhPump, ChlPump, Heat, HeatPump, SaltPump, Salt_Chlor, SaltPolarity, ValveMode, CleanMode, ValveSwitch;
  unsigned long pHPpRT, OrpPpRT, SHRT, HPRT, SPRT;
  String FW;
  std::string ELDTstate, ELDHstate, WPVstate, WPMstate, BOTTstate;
} TFTStruc =
{ //default values to force update on next refresh
  -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1., -1.,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  99, 99, 99, 99, 99,
  "",
  "", "", "", "", "",
};



//Nextion TFT object. Choose which ever Serial port
//you wish to connect to (not "Serial" which is used for debug), here Serial2 UART
static EasyNex myNex(Serial2);

// Functions prototypes
void InitTFT(void);
void ResetTFT(void);
void UpdateTFT(void);
void UpdateWiFi(bool);


void InitTFT()
{
  myNex.begin(9600);
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

  if (MQTTConnection != TFTStruc.NetW || !refresh)
  {
    TFTStruc.NetW = MQTTConnection;
    myNex.writeNum(F("page1.vabNetW.val"), TFTStruc.NetW);

    if (CurrentPage == 0)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p0NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p0NetW.pic=6"));
      }
    }
    else if (CurrentPage == 1)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p1NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p1NetW.pic=6"));
      }
    }
    else if (CurrentPage == 2)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p2NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p2NetW.pic=6"));
      }
    }
    else if (CurrentPage == 3)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
    else if (CurrentPage == 4)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
    else if (CurrentPage == 5)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
    else if (CurrentPage == 6)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
    else if (CurrentPage == 7)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
    else if (CurrentPage == 7)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
    else if (CurrentPage == 8)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
    else if (CurrentPage == 9)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
    else if (CurrentPage == 10)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
    else if (CurrentPage == 11)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
    else if (CurrentPage == 12)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
    else if (CurrentPage == 13)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
    else if (CurrentPage == 14)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
    else if (CurrentPage == 15)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
    else if (CurrentPage == 16)
    {
      if (TFTStruc.NetW == 1)
      {
        myNex.writeStr(F("p3NetW.pic=5"));
      }
      else
      {
        myNex.writeStr(F("p3NetW.pic=6"));
      }
    }
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
        pHangle = map(pHgauge,60,69,0,1500);
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
      } else if(Orpgauge < 500 && Orpgauge > 750) {
        Orpangle = map(Orpgauge,500,750,0,1500);
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

  if (storage.TempValue != TFTStruc.WT || !refresh)
  {
    TFTStruc.WT = storage.TempValue;
    temp = String(TFTStruc.WT, 1);
    myNex.writeStr(F("page0.vaWT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("W.txt"), temp);
    else if (CurrentPage == 2)  myNex.writeStr(F("W.txt"), temp);
    else if (CurrentPage == 3)  myNex.writeStr(F("W.txt"), temp);
    else if (CurrentPage == 7)  myNex.writeStr(F("W.txt"), temp);
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

  if (storage.TempExternal != TFTStruc.AT || !refresh)
  {
    TFTStruc.AT = storage.TempExternal;
    temp = String(TFTStruc.AT, 1);
    myNex.writeStr(F("page0.vaAT.txt"), temp);
    if (CurrentPage == 0)  myNex.writeStr(F("A.txt"), temp);
    else if (CurrentPage == 3)  myNex.writeStr(F("A.txt"), temp);
    else if (CurrentPage == 7)  myNex.writeStr(F("A.txt"), temp);
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
      if (CurrentPage == 0)
      {
        myNex.writeStr(F("p0Mode.txt"), temp);
      }
      else if (CurrentPage == 1)
      {
        myNex.writeStr(F("p1Mode.txt"), temp);
        myNex.writeStr(F("t1Mode.txt"), temp);
        if (storage.AutoMode == 1)
          myNex.writeStr(F("bMode.picc=9"));
        else
          myNex.writeStr(F("bMode.picc=8"));
      }
      else if (CurrentPage == 2)
      {
        myNex.writeStr(F("p2Mode.txt"), temp);
      }
      else if (CurrentPage == 3)
      {
        myNex.writeStr(F("p3Mode.txt"), temp);
      }
      else if (CurrentPage == 4)
      {
        myNex.writeStr(F("p4Mode.txt"), temp);
      }
      else if (CurrentPage == 5)
      {
        myNex.writeStr(F("p5Mode.txt"), temp);
      }
      else if (CurrentPage == 6)
      {
        myNex.writeStr(F("p6Mode.txt"), temp);
      }
      else if (CurrentPage == 7)
      {
        myNex.writeStr(F("p7Mode.txt"), temp);
      }
      else if (CurrentPage == 8)
      {
        myNex.writeStr(F("p8Mode.txt"), temp);
      }
      else if (CurrentPage == 9)
      {
        myNex.writeStr(F("p9Mode.txt"), temp);
      }
      else if (CurrentPage == 10)
      {
        myNex.writeStr(F("p10Mode.txt"), temp);
      }
      else if (CurrentPage == 11)
      {
        myNex.writeStr(F("p11Mode.txt"), temp);
      }
      else if (CurrentPage == 12)
      {
        myNex.writeStr(F("p12Mode.txt"), temp);
      }
      else if (CurrentPage == 13)
      {
        myNex.writeStr(F("p13Mode.txt"), temp);
      }
      else if (CurrentPage == 14)
      {
        myNex.writeStr(F("p14Mode.txt"), temp);
      }
      else if (CurrentPage == 15)
      {
        myNex.writeStr(F("p15Mode.txt"), temp);
      }
      else if (CurrentPage == 16)
      {
        myNex.writeStr(F("p16Mode.txt"), temp);
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

  if (storage.ValveMode != TFTStruc.ValveMode || !refresh)
  {
    if ((debounceVM == 0) || (debounceVM > debounceCount))
    {
      debounceVM = 0;
      TFTStruc.ValveMode = storage.ValveMode;
      myNex.writeNum(F("page4.vabValveMode.val"), TFTStruc.ValveMode);
      if (CurrentPage == 4)
      {
        myNex.writeStr(F("bValveMode.picc=15"));
      } else {
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
        myNex.writeStr(F("bCleanMode.picc=15"));
      } else {
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
        myNex.writeStr(F("bCleanDir.picc=15"));
      } else {
        myNex.writeStr(F("bCleanDir.picc=14"));
      }
    }
    else
      debounceVS++;
  }

  if (ELD_Treppe.getStatus() != TFTStruc.ELDTstate || !refresh)
  {
    TFTStruc.ELDTstate = ELD_Treppe.getStatus();
    temp = String(TFTStruc.ELDTstate.c_str());
    myNex.writeStr(F("page4.vaELDTstate.txt"), temp);
    if (CurrentPage == 4)  myNex.writeStr(F("ELDTstate.txt"), temp);
  }

if (ELD_Hinten.getStatus() != TFTStruc.ELDHstate || !refresh)
  {
    TFTStruc.ELDHstate = ELD_Hinten.getStatus();
    temp = String(TFTStruc.ELDHstate.c_str());
    myNex.writeStr(F("page4.vaELDHstate.txt"), temp);
    if (CurrentPage == 4)  myNex.writeStr(F("ELDHstate.txt"), temp);
  }

  if ((WP_Vorlauf.getStatus() != TFTStruc.WPVstate) || !refresh)
  {
    TFTStruc.WPVstate = WP_Vorlauf.getStatus();

    temp = String(TFTStruc.WPVstate.c_str());
    myNex.writeStr(F("page4.vaWPVstate.txt"), temp);
    if (CurrentPage == 4)  myNex.writeStr(F("WPVstate.txt"), temp);
  }

  if ((WP_Mischer.getStatus() != TFTStruc.WPMstate) || !refresh)
  {
    TFTStruc.WPMstate = WP_Mischer.getStatus();

    temp = String(TFTStruc.WPMstate.c_str());
    myNex.writeStr(F("page4.vaWPMstate.txt"), temp);
    if (CurrentPage == 4)  myNex.writeStr(F("WPMstate.txt"), temp);
  }

  if ((Bodenablauf.getStatus() != TFTStruc.BOTTstate) || !refresh)
  {
    TFTStruc.BOTTstate = Bodenablauf.getStatus();

    temp = String(TFTStruc.BOTTstate.c_str());
    myNex.writeStr(F("page4.vaBOTTstate.txt"), temp);
    if (CurrentPage == 4)  myNex.writeStr(F("BOTTstate.txt"), temp);
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
        myNex.writeStr(F("b14.picc=5"));
      } else {
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
        myNex.writeStr(F("b14.picc=5"));
      } else {
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

  if (storage.SaltPolarity != TFTStruc.SaltPolarity || !refresh)
  {
    if ((debounceSPl == 0) || (debounceSPl > debounceCount))
    {
      debounceSPl = 0;
      TFTStruc.SaltPolarity = storage.SaltPolarity;
      if (storage.SaltPolarity == 0)
        temp = "DIREKT";
      else
        temp = "UMGEPOLT";
      myNex.writeStr(F("page0.vaSaltDir.txt"), temp);
    if (CurrentPage == 13)  myNex.writeStr(F("SaltDir.txt"), temp);
    }
    else
      debounceSPl++;
  }

  if (storage.SaltDiff != TFTStruc.SaltDiff || !refresh)
  {
    TFTStruc.SaltDiff = storage.SaltDiff;
    temp = String(TFTStruc.SaltDiff);
    myNex.writeStr(F("page0.vaSaltDiff.txt"), temp);
    if (CurrentPage == 13)  myNex.writeStr(F("SaltDiff.txt"), temp);
  }

  if ((SaltPump.UpTime != TFTStruc.SPRT) || !refresh)
  {
    TFTStruc.SPRT = SaltPump.UpTime;
    int Sec = TFTStruc.SPRT/100/60;
    int Min = Sec/60; Sec%=60;
    int Std = Min/60; Min%=60;
    
    if(Std < 10 && Min >= 10) {
        temp = "0" + String(Std) +F(" : ") + String(Min);
      } else if(Std >= 10 && Min < 10) {
        temp = String(Std) +F(" : 0") + String(Min);
      } else if(Std < 10 && Min < 10) {
        temp = "0" + String(Std) +F(" : 0") + String(Min);
      } else {
        temp = String(Std) +F(" : ") + String(Min);
      }
    myNex.writeStr(F("page0.vaSaltDur.txt"), temp);
    if (CurrentPage == 13)  myNex.writeStr(F("SaltDur.txt"), temp);
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
          myNex.writeStr(F("bHeatPum.pic=12"));
      }
    }
    else
      debounceHP++;
  }
  
  if ((HeatPump.UpTime != TFTStruc.HPRT) || !refresh)
  {
    TFTStruc.HPRT = HeatPump.UpTime;
    int Sec = TFTStruc.HPRT/100/60;
    int Min = Sec/60; Sec%=60;
    int Std = Min/60; Min%=60;
    
    if(Std < 10 && Min >= 10) {
        temp = "0" + String(Std) +F(" : ") + String(Min);
      } else if(Std >= 10 && Min < 10) {
        temp = String(Std) +F(" : 0") + String(Min);
      } else if(Std < 10 && Min < 10) {
        temp = "0" + String(Std) +F(" : 0") + String(Min);
      } else {
        temp = String(Std) +F(" : ") + String(Min);
      }
    myNex.writeStr(F("page0.vaWPDur.txt"), temp);
    if (CurrentPage == 3)  myNex.writeStr(F("WPDur.txt"), temp);
  }

  if ((SolarHeatPump.UpTime != TFTStruc.SHRT) || !refresh)
  {
    TFTStruc.SHRT = SolarHeatPump.UpTime;
    int Sec = TFTStruc.SHRT/100/60;
    int Min = Sec/60; Sec%=60;
    int Std = Min/60; Min%=60;
    
    if(Std < 10 && Min >= 10) {
        temp = "0" + String(Std) +F(" : ") + String(Min);
      } else if(Std >= 10 && Min < 10) {
        temp = String(Std) +F(" : 0") + String(Min);
      } else if(Std < 10 && Min < 10) {
        temp = "0" + String(Std) +F(" : 0") + String(Min);
      } else {
        temp = String(Std) +F(" : ") + String(Min);
      }
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
          myNex.writeStr(F("bR0.pic=8"));
        else
          myNex.writeStr(F("bR0.pic=7"));
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
          myNex.writeStr(F("bR1.pic=8"));
        else
          myNex.writeStr(F("bR1.pic=7"));
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
    temp = String(float(TFTStruc.pHPIDW), 0);
    myNex.writeStr(F("page0.vapHPIDW.txt"), temp);
    if (CurrentPage == 15)  myNex.writeStr(F("pHPIDW.txt"), temp);
  }

  if (storage.OrpPIDWindowSize != TFTStruc.OrpPIDW || !refresh)
  {
    TFTStruc.OrpPIDW = storage.OrpPIDWindowSize;
    temp = String(float(TFTStruc.OrpPIDW), 0);
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

  if (storage.ChlPumpFR != TFTStruc.pHPumpFR || !refresh)
  {
    TFTStruc.pHPumpFR = storage.ChlPumpFR;
    temp = String(TFTStruc.ChlPumpFR, 1);
    myNex.writeStr(F("page0.vaChlPumpFR.txt"), temp);
    if (CurrentPage == 16)  myNex.writeStr(F("ChlPumpFR.txt"), temp);
  }

  if (storage.PhPumpUpTimeLimit != TFTStruc.PumpMaxUp || !refresh)
  {
    TFTStruc.PumpMaxUp = storage.PhPumpUpTimeLimit;
    temp = String(TFTStruc.PumpMaxUp);
    myNex.writeStr(F("page0.vaPumpsMaxUp.txt"), temp);
    if (CurrentPage == 14)  myNex.writeStr(F("PumpsMaxUp.txt"), temp);
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
  }
  //put TFT in sleep mode with wake up on touch and force page 0 load to trigger an event
  if((unsigned long)(millis() - LastAction) >= TFT_SLEEP && TFT_ON)
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
  TFTStruc.Heat = (boolean)myNex.readNumber(F("vabSolMode.val"));
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