#include <Arduino.h>                // Arduino framework
#include "Config.h"
#include "PoolMaster.h"

extern Preferences nvs;

// Setup oneWire instances to communicate with temperature sensors (one bus per sensor)
static OneWire oneWire_W(ONE_WIRE_BUS_W);
static OneWire oneWire_A(ONE_WIRE_BUS_A);
// Pass our oneWire reference to Dallas Temperature library instance
static DallasTemperature sensors_W(&oneWire_W);
static DallasTemperature sensors_A(&oneWire_A);

// global variable for numbers of connected sensors
uint8_t numSensors_W;
uint8_t numSensors_A;

// DS18B20 SENSOR-Mapping to  map the sensoradress with the Tempname
const char* NV_STORAGE_MAPPING_A[] = {"SolarTemp", "SolarVLTemp", "SolarRLTemp", "AirInTemp", "AirTemp"}; // Mapping of A-BUS-Sensors to NVS
const char* NV_STORAGE_MAPPING_W[] = {"WaterSTemp", "WaterITemp", "WaterBTemp", "WaterWPTemp", "WaterWTTemp"}; // Mapping of W-BUS-Sensors to NVS

// Setup an ADS1115 instance for analog measurements
static ADS1115Scanner adc_int(INT_ADS1115_ADDR);
#ifdef EXT_ADS1115
static ADS1115Scanner adc_ph(PH_ADS1115_ADDR);
static ADS1115Scanner adc_orp(ORP_ADS1115_ADDR);
#endif

static float ph_sensor_value;     // pH sensor current value
static float orp_sensor_value;    // ORP sensor current value
static float psi_sensor_value;    // PSI sensor current value

// Setup instance for flow measurements
long flow_currentMillis = 0;
long flow_previousMillis = 0;
int flow_interval = 1000;
// The hall-effect flow sensor outputs approximately 4.5 pulses per second per litre/minute of flow.
// 13.51 is to make 100 for the full flow
float flow_calibrationFactor;
volatile byte flow_pulseCount;
byte flow_pulse1Sec = 0;

// Setup instance for flow2 measurements
long flow2_currentMillis = 0;
long flow2_previousMillis = 0;
int flow2_interval = 1000;
// The hall-effect flow sensor outputs approximately 4.5 pulses per second per litre/minute of flow.
// 13.51 is to make 100 for the full flow
float flow2_calibrationFactor;
volatile byte flow2_pulseCount;
byte flow2_pulse1Sec = 0;


// Signal filtering library sample buffers
static RunningMedian samples_A_Temp[5] = { RunningMedian(11), RunningMedian(11), RunningMedian(11), RunningMedian(11), RunningMedian(11) };
static RunningMedian samples_W_Temp[5] = { RunningMedian(11), RunningMedian(11), RunningMedian(11), RunningMedian(11), RunningMedian(11) };
static RunningMedian samples_Ph        = RunningMedian(11);
static RunningMedian samples_Orp       = RunningMedian(11);
static RunningMedian samples_PSI       = RunningMedian(11);
static RunningMedian samples_ATemp     = RunningMedian(11);
static RunningMedian samples_AHum      = RunningMedian(11);
static RunningMedian samples_AP        = RunningMedian(11);
static RunningMedian samples_Flow      = RunningMedian(11);
static RunningMedian samples_Flow2     = RunningMedian(11);

void stack_mon(UBaseType_t&);
void lockI2C();
void unlockI2C();

//Update loop for ADS1115 measurements
// Some explanations: The sampling rate is set to 16sps in order to be sure that 
// every 125ms (which is the period of the task) there is a sample available. As it takes 3ms to
// update and restart the ADC, the whole loop takes a minimum of 3 + 1/SPS ms. If SPS was set to 
// 8sps, that means 128ms instead of 125ms. With 16sps, we have 3 + 62.5 < 125ms which is OK.
// With those settings, we get a value for each channel roughly every second: 9 values asked 
// (3 per channel), with height values retrieved per second -> 0,89 value per second. The value
// returned for each channel is the median of the three samples. Then, among the last 
// 11 samples returned, we take the 5 median ones and compute the mean as consolidated value.

// With the "Loulou74" board, the sampling is different: we sample PSI at 8sps, and pH, Orp at 
// 4sps each. Filtering and average is then performed as usual to get a new value every second.
//We have here two sections of code here of which only one will be compiled depending on the
//configuration

#ifdef EXT_ADS1115
//----------------------------

// PulseCounter for FLOW
void IRAM_ATTR flow_pulseCounter()
{
  // Increment the pulse counter
  flow_pulseCount++;
}

// PulseCounter for FLOW2
void IRAM_ATTR flow2_pulseCounter()
{
  // Increment the pulse counter
  flow2_pulseCount++;
}

void AnalogInit()
{
  // Initialize the ADS1115 instances PSI measurements
  adc_int.setSpeed(ADS1115_SPEED_16SPS);
  adc_int.addChannel(ADS1115_CHANNEL2, ADS1115_RANGE_6144);
  adc_int.setSamples(8);
  // Initialize the ADS1115 instances pH measurements
  adc_ph.setSpeed(ADS1115_SPEED_16SPS);
  adc_ph.addChannel(ADS1115_CHANNEL01, ADS1115_RANGE_6144);
  adc_ph.setSamples(4);
  // Initialize the ADS1115 instances Orp measurements
  adc_orp.setSpeed(ADS1115_SPEED_16SPS);
  adc_orp.addChannel(ADS1115_CHANNEL01, ADS1115_RANGE_6144);
  adc_orp.setSamples(4);
}

void AnalogPoll(void *pvParameters)
{
  while (!startTasks) ;
  TickType_t period = PT1;  
  TickType_t ticktime = xTaskGetTickCount(); 
  static UBaseType_t hwm=0;
  lockI2C();
  adc_int.start();
  adc_ph.start();
  adc_orp.start();
  unlockI2C();
  vTaskDelayUntil(&ticktime,period);
  
  for(;;)
  {
    lockI2C();
    adc_ph.update();
    if(adc_ph.ready()){              // all conversions done ?
      // As an int is 32 bits long for ESP32 and as the ADS1115 is wired in differential, we have to manage
      // negative voltage as follow
        ph_sensor_value  = adc_ph.readFilter(0);
        if(ph_sensor_value >= 32768) ph_sensor_value= ph_sensor_value - 65536;      // pH sensor current value
        adc_ph.start();
        
        //Ph
        samples_Ph.add(ph_sensor_value);          // compute average of pH from center 5 measurements among 11
        storage.PhValue = (samples_Ph.getAverage(5)*0.1875/1000.)*storage.pHCalibCoeffs0 + storage.pHCalibCoeffs1;
    }

    adc_orp.update();
    if(adc_orp.ready()){              // all conversions done ?
      // As an int is 32 bits long for ESP32 and as the ADS1115 is wired in differential, we have to manage
      // negative voltage as follow
        orp_sensor_value = adc_orp.readFilter(0);
        if(orp_sensor_value >= 32768) orp_sensor_value = orp_sensor_value - 65536;  // ORP sensor current value
        adc_orp.start(); 
        
        //ORP
        samples_Orp.add(orp_sensor_value);        // compute average of ORP from last 5 measurements
        storage.OrpValue = (samples_Orp.getAverage(5)*0.1875/1000.)*storage.OrpCalibCoeffs0 + storage.OrpCalibCoeffs1;
    }
    
    adc_int.update();
    if(adc_int.ready()){
      psi_sensor_value = adc_int.readFilter(0) ;    // psi sensor current value
      adc_int.start();
      //PSI (water pressure)
      samples_PSI.add(psi_sensor_value);        // compute average of PSI from last 5 measurements
      storage.PSIValue = (samples_PSI.getAverage(5)*0.1875/1000.)*storage.PSICalibCoeffs0 + storage.PSICalibCoeffs1;
      Debug.print(DBG_DEBUG,"pH: %5.0f - %4.2f - ORP: %5.0f - %3.0fmV - PSI: %5.0f - %4.2fBar\r",
            ph_sensor_value,storage.PhValue,orp_sensor_value,storage.OrpValue,psi_sensor_value,storage.PSIValue);
    }
    unlockI2C();
    stack_mon(hwm);
    vTaskDelayUntil(&ticktime,period);
  }  
}
#else //EXT_ADS1115
//-----------------

// PulseCounter for FLOW
void IRAM_ATTR flow_pulseCounter()
{
  // Increment the pulse counter
  flow_pulseCount++;
}

// PulseCounter for FLOW2
void IRAM_ATTR flow2_pulseCounter()
{
  // Increment the pulse counter
  flow2_pulseCount++;
}

void AnalogInit()
{
  adc_int.setSpeed(ADS1115_SPEED_16SPS);
  adc_int.addChannel(ADS1115_CHANNEL0, ADS1115_RANGE_6144);
  adc_int.addChannel(ADS1115_CHANNEL1, ADS1115_RANGE_6144);
  adc_int.addChannel(ADS1115_CHANNEL2, ADS1115_RANGE_6144);
  adc_int.setSamples(3);
}

void AnalogPoll(void *pvParameters)
{
  while (!startTasks) ;

  TickType_t period = PT1;  
  TickType_t ticktime = xTaskGetTickCount(); 
  static UBaseType_t hwm=0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  lockI2C();
  adc_int.start();
  unlockI2C();
  vTaskDelayUntil(&ticktime,period);
  
  for(;;)
  {
    #ifdef CHRONO
    td = millis();
    #endif

    lockI2C();
    adc_int.update();

    if(adc_int.ready()){                              // all conversions done ?
        orp_sensor_value = adc_int.readFilter(0) ;    // ORP sensor current value
        ph_sensor_value  = adc_int.readFilter(1) ;    // pH sensor current value
        psi_sensor_value = adc_int.readFilter(2) ;    // psi sensor current value
        adc_int.start();  
        
        //Ph
        samples_Ph.add(ph_sensor_value);          // compute average of pH from center 5 measurements among 11
        storage.PhValue = (samples_Ph.getAverage(5)*0.1875/1000.)*storage.pHCalibCoeffs0 + storage.pHCalibCoeffs1;

#ifdef SIMU
        if(!init_simu){
            if(newpHOutput) {
                pHTab[iw] = storage.PhPIDOutput;
                pHCumul = pHTab[0]+pHTab[1]+pHTab[2];
                iw++;
                iw %= 3;
            }
            storage.PhValue = pHLastValue + pHCumul/4500000.*(double)((millis()-pHLastTime)/3600000.);
            pHLastValue = storage.PhValue;
            pHLastTime = millis();
        } else {
            init_simu = false;
            pHLastTime = millis();
            pHLastValue = 7.0;
            storage.PhValue = pHLastValue;
            storage.OrpValue = OrpLastValue;
            OrpLastTime = millis();
            OrpLastValue = 730.0;
            for(uint8_t i=0;i<3;i++) {
                pHTab[i] = 0.;
                ChlTab[i] = 0.;
            }  
        }  
#endif

        //ORP
        samples_Orp.add(orp_sensor_value);         // compute average of ORP from last 5 measurements
        storage.OrpValue = (samples_Orp.getAverage(5)*0.1875/1000.)*storage.OrpCalibCoeffs0 + storage.OrpCalibCoeffs1;

#ifdef SIMU
        if(!init_simu){
            if(newChlOutput) {
            ChlTab[jw] = storage.OrpPIDOutput;
            ChlCumul = ChlTab[0]+ChlTab[1]+ChlTab[2];
            jw++;
            jw %= 3;
            }    
            storage.OrpValue = OrpLastValue + ChlCumul/36000.*(double)((millis()-OrpLastTime)/3600000.);
            OrpLastValue = storage.OrpValue;
            OrpLastTime = millis();    
        } 
#endif

        //PSI (water pressure)
        samples_PSI.add(psi_sensor_value);        // compute average of PSI from last 5 measurements
        storage.PSIValue = (samples_PSI.getAverage(5)*0.1875/1000.)*storage.PSICalibCoeffs0 + storage.PSICalibCoeffs1;

        Debug.print(DBG_DEBUG,"pH: %5.0f - %4.2f - ORP: %5.0f - %3.0fmV - PSI: %5.0f - %4.2fBar\r",
            ph_sensor_value,storage.PhValue,orp_sensor_value,storage.OrpValue,psi_sensor_value,storage.PSIValue);
    }
    unlockI2C();

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[AnalogPoll] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif 

    stack_mon(hwm);
    vTaskDelayUntil(&ticktime,period);
  }  
}

#endif //EXT_ADS1115
//-----------------

void FlowInit()
{
  flow_pulseCount = 0;
  flow_previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(FLOW), flow_pulseCounter, RISING);
}

void Flow2Init()
{
  flow2_pulseCount = 0;
  flow2_previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(FLOW2), flow2_pulseCounter, RISING);
}


void StatusLights(void *pvParameters)
{
  static uint8_t line = 0;
  uint8_t status;

  while (!startTasks) ;
  vTaskDelay(DT10);                                // Scheduling offset 

  TickType_t period = PT10;  
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  for(;;)
  {
    #ifdef CHRONO
    td = millis();
    #endif 

    status = 0;
    status |= (line & 1) << 1;
    if(line == 0)
    {
        line = 1;
        status |= (storage.AutoMode & 1) << 2;
        status |= (AntiFreezeFiltering & 1) << 3;
        status |= (PSIError & 1) << 7;
        status |= (FLOWError & 1) << 7;
        status |= (FLOW2Error & 1) << 7;
    } else
    {
        line = 0;
        status |= (PhPID.GetMode() & 1) << 2;
        status |= (OrpPID.GetMode() & 1) << 3;
        status |= (!PhPump.TankLevel() & 1) << 4;
        status |= (!ChlPump.TankLevel() & 1) << 5;
        status |= (PhPump.UpTimeError & 1) << 6;
        status |= (ChlPump.UpTimeError & 1) << 7;  
    }
    (status & 0xF0) ? digitalWrite(BUZZER,HIGH) : digitalWrite(BUZZER,LOW) ;
    if(WiFi.status() == WL_CONNECTED) status |= 0x01;
        else status &= 0xFE;
    Debug.print(DBG_VERBOSE,"Status LED : 0x%02x",status);
    lockI2C();
    Wire.beginTransmission(PCF8574_ADR);
    Wire.write(~status);
    Wire.endTransmission();
    unlockI2C();

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[StatusLights] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif

    stack_mon(hwm);
    vTaskDelayUntil(&ticktime,period);
  }  
}

//Ph regulation loop
void pHRegulation(void *pvParameters)
{
  while (!startTasks) ;
  vTaskDelay(DT8);                                // Scheduling offset 

  TickType_t period = PT8;  
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  for(;;)
  {
    #ifdef CHRONO
    td = millis();
    #endif 

    //do not compute PID if filtration pump is not running
    // Set also a lower limit at 30s (a lower pump duration doesn't mean anything)

    if (FiltrationPump.IsRunning()  && (PhPID.GetMode() == AUTOMATIC))
    {  
      if(PhPID.Compute()){
        Debug.print(DBG_VERBOSE,"Ph  regulation: %10.2f, %13.9f, %13.9f, %17.9f",storage.PhPIDOutput,storage.PhValue,storage.Ph_SetPoint,storage.Ph_Kp);
        if(storage.PhPIDOutput < (double)30000.) storage.PhPIDOutput = 0.;
        Debug.print(DBG_INFO,"Ph  regulation: %10.2f",storage.PhPIDOutput);
    #ifdef SIMU
        newpHOutput = true;
    #endif            
      }
    #ifdef SIMU
      else newpHOutput = false;
    #endif    
      /************************************************
       turn the Acid pump on/off based on pid output
      ************************************************/
	  unsigned long now = millis();
      if (now - storage.PhPIDwindowStartTime > storage.PhPIDWindowSize)
      {
        //time to shift the Relay Window
        storage.PhPIDwindowStartTime += storage.PhPIDWindowSize;
      }
      if ((unsigned long)storage.PhPIDOutput <= now - storage.PhPIDwindowStartTime)
        PhPump.Stop();
      else
        PhPump.Start();   
    }

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[pHRegulation] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif

    stack_mon(hwm);
    vTaskDelayUntil(&ticktime,period);
  }  
}

//Orp regulation loop
void OrpRegulation(void *pvParameters)
{
  while (!startTasks) ;
  vTaskDelay(DT6);                                // Scheduling offset 

  TickType_t period = PT6;  
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  for(;;)
  { 
    #ifdef CHRONO
    td = millis();
    #endif 
     
        //do not compute PID if filtration pump is not running
        // Set also a lower limit at 30s (a lower pump duration does'nt mean anything)

        if (!storage.Salt_Chlor && FiltrationPump.IsRunning() && (OrpPID.GetMode() == AUTOMATIC))
        {
          if(OrpPID.Compute()){
            Debug.print(DBG_VERBOSE,"ORP regulation: %10.2f, %13.9f, %12.9f, %17.9f",storage.OrpPIDOutput,storage.OrpValue,storage.Orp_SetPoint,storage.Orp_Kp);
            if(storage.OrpPIDOutput < (double)30000.) storage.OrpPIDOutput = 0.;    
              Debug.print(DBG_INFO,"Orp regulation: %10.2f",storage.OrpPIDOutput);
        #ifdef SIMU
              newChlOutput = true;
        #endif      
            }
        #ifdef SIMU
            else newChlOutput = false;
        #endif    
          /************************************************
           turn the Chl pump on/off based on pid output
          ************************************************/
        unsigned long now = millis();
          if (now - storage.OrpPIDwindowStartTime > storage.OrpPIDWindowSize)
          {
            //time to shift the Relay Window
            storage.OrpPIDwindowStartTime += storage.OrpPIDWindowSize;
          }
          if ((unsigned long)storage.OrpPIDOutput <= now - storage.OrpPIDwindowStartTime)
            ChlPump.Stop();
          else
            ChlPump.Start();
        }

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[OrpRegulation] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif

    stack_mon(hwm);    
    vTaskDelayUntil(&ticktime,period);
  }
}

//Salt regulation loop
void SaltRegulation(void *pvParameters)
{
  while (!startTasks) ;
  vTaskDelay(DT7);                                // Scheduling offset 

  TickType_t period = PT7;  
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  for(;;)
  { 
    #ifdef CHRONO
    td = millis();
    #endif


    // The polarity of the Salt electrolysis should be changed every 4 hours to prevent calcification of the electrolysis plates.
    if (FiltrationPump.IsRunning())
    {
        static bool polarity_reversed = storage.SaltPolarity;
        static unsigned long last_switch_time = 0;
        static unsigned long lastUpTime = 0;
        static unsigned long last_runtime = 0;

        if (SaltPump.UpTime != lastUpTime) { // check if SaltPump.UpTime has changed
            last_runtime = SaltPump.UpTime - lastUpTime;
            storage.SaltPumpRunTime += last_runtime; // add to SaltPumpRunTime
            lastUpTime = SaltPump.UpTime;
        }

        static unsigned long switch_polarity_time = 240; // polarity switch time in minutes

        // Check if it's time to reverse polarity
        if (storage.SaltPumpRunTime >= switch_polarity_time * 60 * 1000) {
            storage.SaltPumpRunTime = 0;
            polarity_reversed = !polarity_reversed;

            // Stop SaltPump 200 milliseconds before switching polarity
                if (SaltPump.IsRunning()) {
                    SaltPump.Stop();
                    unsigned long stop_time = millis();
                    while (millis() - stop_time < 500) {
                        // wait for 500 milliseconds
                    }
                }

                // Switch polarity
                if (polarity_reversed) {
                    digitalWrite(SALT_POL, DIRECT);
                    storage.SaltPolarity = 0;
                } else {
                    digitalWrite(SALT_POL, REVERSE);
                    storage.SaltPolarity = 1;
                }

                // Start SaltPump 500 milliseconds after switching polarity
                unsigned long switch_time = millis();
                while (millis() - switch_time < 500) {
                    // wait for 500 milliseconds
                }
                SaltPump.Start();
            }
    }
    
    // start or stopp salt electrolys based on orp value
    if (FiltrationPump.IsRunning() && storage.SaltMode && storage.Salt_Chlor && !FLOW2Error && !storage.WinterMode && storage.WaterSTemp >= storage.WaterTempLowThreshold)
        {
          //Stop OrpPID
          OrpPID.SetMode(MANUAL);
          storage.Orp_RegulationOnOff = 0;
          storage.OrpPIDOutput = 0.0;
          ChlPump.Stop();

        if (storage.OrpValue < (storage.Orp_SetPoint - storage.SaltDiff)) {
          Debug.print(DBG_VERBOSE,"Salt regulation on: %13.9f, %12.9f, %17.9f",storage.OrpValue,storage.Orp_SetPoint,storage.SaltDiff);
          SaltPump.Start();
        } else if (storage.OrpValue > (storage.Orp_SetPoint + storage.SaltDiff)) {
          Debug.print(DBG_INFO,"Salt regulation off: %13.9f, %12.9f, %17.9f",storage.OrpValue,storage.Orp_SetPoint,storage.SaltDiff);
          SaltPump.Stop();
        }
      } else {
        SaltPump.Stop();
      }

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[SaltRegulation] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif

    stack_mon(hwm);    
    vTaskDelayUntil(&ticktime,period);
  }
}

//Flow measurements loop
void FlowMeasures(void *pvParameters)
{
  while (!startTasks) ;
  vTaskDelay(DT9);                                // Scheduling offset 

  TickType_t period = PT9;  
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif


  for(;;)
  {        
    #ifdef CHRONO
    td = millis();
    #endif

    flow_currentMillis = millis();
    if (flow_currentMillis - flow_previousMillis > flow_interval)
    {
        flow_pulse1Sec = flow_pulseCount;
        flow_pulseCount = 0;
        flow_calibrationFactor = storage.FLOW_Pulse;

        // Because this loop may not complete in exactly 1 second intervals we calculate
        // the number of milliseconds that have passed since the last execution and use
        // that to scale the output. We also apply the calibrationFactor to scale the output
        // based on the number of pulses per second per units of measure (litres/minute in
        // this case) coming from the sensor.
        storage.FLOWValue = ((1000.0 / (millis() - flow_previousMillis)) * flow_pulse1Sec) / flow_calibrationFactor; // FLOWValue = liter/min.

        samples_Flow.add(storage.FLOWValue);
        storage.FLOWValue = samples_Flow.getAverage(5);

        flow_previousMillis = millis();
    }

    flow2_currentMillis = millis();
    if (flow2_currentMillis - flow2_previousMillis > flow2_interval)
    {
        flow2_pulse1Sec = flow2_pulseCount;
        flow2_pulseCount = 0;
        flow2_calibrationFactor = storage.FLOW2_Pulse;

        // Because this loop may not complete in exactly 1 second intervals we calculate
        // the number of milliseconds that have passed since the last execution and use
        // that to scale the output. We also apply the calibrationFactor to scale the output
        // based on the number of pulses per second per units of measure (litres/minute in
        // this case) coming from the sensor.
        storage.FLOW2Value = ((1000.0 / (millis() - flow2_previousMillis)) * flow2_pulse1Sec) / flow2_calibrationFactor; // FLOW2Value = liter/min.

        samples_Flow2.add(storage.FLOW2Value);
        storage.FLOW2Value = samples_Flow2.getAverage(5);

        flow2_previousMillis = millis();
    }

    Debug.print(DBG_VERBOSE, "[FLOW] Flow: %4.1f l/min - Flow2: %4.1f l/min\r",
                storage.FLOWValue, storage.FLOW2Value);

#ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[FlowMeasures] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif

    stack_mon(hwm);
    vTaskDelayUntil(&ticktime,period);
  }

}


//Init DS18B20 one-wire library
void TempInit()
{
  bool error = false;

  DeviceAddress tempDeviceAddress_A;  // Adresse des gefundenen Sensors
  DeviceAddress tempDeviceAddress_W;  // Adresse des gefundenen Sensors
  uint8_t sensorCount_A = 0;          // Anzahl der gefundenen Sensoren
  uint8_t sensorCount_W = 0;          // Anzahl der gefundenen Sensoren

  // Start up the library
  sensors_W.begin();
  sensors_W.begin(); // two times to work-around of a OneWire library bug for enumeration
  sensors_A.begin();

  Debug.print(DBG_INFO, "[DS18B20 - INIT] 1wire W devices: %d device(s) found", sensors_W.getDeviceCount());
  Debug.print(DBG_INFO, "[DS18B20 - INIT] 1wire A devices: %d device(s) found", sensors_A.getDeviceCount());

  // Search for all sensors on bus A and save their addresses in tempDeviceAddress
  Debug.print(DBG_INFO, "[DS18B20 - INIT] Searching for sensors on bus A\n");
  String foundSensors_A = ""; // Variable to store list of found sensors
  while (sensors_A.getAddress(tempDeviceAddress_A, sensorCount_A) && sensorCount_A < MAX_ADDRESSES)
  {
    for (uint8_t i = 0; i < 8; i++)
    {
        foundSensors_A += String(tempDeviceAddress_A[i], HEX) + " ";
    }

    // Check if address is already stored in NVS
    bool addressFound = false;
    for (uint8_t i = 0; i < MAX_ADDRESSES; i++)
    {
        uint8_t storedAddress[8];
        nvs.getBytes(("address_A_" + String(i)).c_str(), storedAddress, 8);
        if (memcmp(storedAddress, tempDeviceAddress_A, 8) == 0)
        {
          addressFound = true;
          break;
        }
    }

    // Save address in NVS if not already stored
    if (!addressFound)
    {
        for (uint8_t i = 0; i < MAX_ADDRESSES; i++)
        {
          uint8_t storedAddress[8];
          nvs.getBytes(("address_A_" + String(i)).c_str(), storedAddress, 8);
          if (memcmp(storedAddress, tempDeviceAddress_A, 8) == 0)
          {
                    // Address already stored
                    break;
          }
          else if (memcmp(storedAddress, "\x00\x00\x00\x00\x00\x00\x00\x00", 8) == 0)
          {
                    // Unused storage space found
                    Debug.print(DBG_INFO, "[DS18B20 - INIT] Storing new address in NVS\n");
                    saveParam(("address_A_" + String(i)).c_str(), tempDeviceAddress_A, 8);
                    break;
          }
        }
    }

    // Debug output for each sensor found
    Debug.print(DBG_VERBOSE, "[DS18B20] Sensor A%d - Address: %02X%02X%02X%02X%02X%02X%02X%02X", sensorCount_A, tempDeviceAddress_A[0], tempDeviceAddress_A[1], tempDeviceAddress_A[2],
    tempDeviceAddress_A[3], tempDeviceAddress_A[4], tempDeviceAddress_A[5], tempDeviceAddress_A[6], tempDeviceAddress_A[7]);

    sensorCount_A++;
  }

  // Search for all sensors on bus W and save their addresses in tempDeviceAddress
  Debug.print(DBG_INFO, "[DS18B20 - INIT] Searching for sensors on bus W\n");
  String foundSensors_W = ""; // Variable to store list of found sensors
  while (sensors_W.getAddress(tempDeviceAddress_W, sensorCount_W) && sensorCount_W < MAX_ADDRESSES)
  {
    for (uint8_t i = 0; i < 8; i++)
    {
        foundSensors_W += String(tempDeviceAddress_W[i], HEX) + " ";
    }

    // Check if address is already stored in NVS
    bool addressFound = false;
    for (uint8_t i = 0; i < MAX_ADDRESSES; i++)
    {
        uint8_t storedAddress[8];
        nvs.getBytes(("address_W_" + String(i)).c_str(), storedAddress, 8);
        if (memcmp(storedAddress, tempDeviceAddress_W, 8) == 0)
        {
          addressFound = true;
          break;
        }
    }

    // Save address in NVS if not already stored
    if (!addressFound)
    {
        for (uint8_t i = 0; i < MAX_ADDRESSES; i++)
        {
          uint8_t storedAddress[8];
          nvs.getBytes(("address_W_" + String(i)).c_str(), storedAddress, 8);
          if (memcmp(storedAddress, tempDeviceAddress_W, 8) == 0)
          {
                    // Address already stored
                    break;
          }
          else if (memcmp(storedAddress, "\x00\x00\x00\x00\x00\x00\x00\x00", 8) == 0)
          {
                    // Unused storage space found
                    Debug.print(DBG_INFO, "[DS18B20 - INIT] Storing new address in NVS\n");
                    saveParam(("address_W_" + String(i)).c_str(), tempDeviceAddress_W, 8);
                    break;
          }
        }
    }

    // Debug output for each sensor found
    Debug.print(DBG_VERBOSE, "[DS18B20 - INIT]  Sensor W%d - Address: %02X%02X%02X%02X%02X%02X%02X%02X", sensorCount_W, tempDeviceAddress_W[0], tempDeviceAddress_W[1], tempDeviceAddress_W[2],
    tempDeviceAddress_W[3], tempDeviceAddress_W[4], tempDeviceAddress_W[5], tempDeviceAddress_W[6], tempDeviceAddress_W[7]);

    sensorCount_W++;
  }

    // set resolution for all sensors
    sensors_W.setResolution(TEMPERATURE_RESOLUTION);
    sensors_A.setResolution(TEMPERATURE_RESOLUTION);

    if (error)
    {
        Debug.print(DBG_ERROR, "[DS18B20 - INIT] Error initializing temperature sensors");
    }
    else
    {
        Debug.print(DBG_INFO, "[DS18B20 - INIT] Temperature sensors initialized\n");
    }
  }


//Request temperature asynchronously
//in case of reading error, the buffer is not updated and the last value is kept
void getTemp(void *pvParameters)
{
  while (!startTasks) ;
  vTaskDelay(DT4);                                // Scheduling offset 

  TickType_t period = PT4;  
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  sensors_W.requestTemperatures();
  sensors_A.requestTemperatures();
  vTaskDelayUntil(&ticktime,period);
  
  for(;;)
  {        
    #ifdef CHRONO
    td = millis();
    #endif 

  // Read temperatures from DS18B20_A (Air and Solar sensors)
  for (int i = 0; i < 5; i++) {
    byte currentAddress_A[8];
    uint8_t storedAddress_A[8];
    nvs.getBytes(("address_A_" + String(i)).c_str(), storedAddress_A, 8);
    memcpy(currentAddress_A, storedAddress_A, 8);

    sensors_A.requestTemperaturesByAddress(currentAddress_A);
    float temp = sensors_A.getTempC(currentAddress_A);

    if (temp == NAN || temp == -127) {
      Debug.print(DBG_WARNING, "[DS18B20] Error getting temperature for sensor A%d", i);
    } else {
      samples_A_Temp[i].add(temp);
      float averagedTemp = samples_A_Temp[i].getAverage(5);
      switch (storage.Array_A[i]) {
              case 0: storage.SolarTemp = averagedTemp; break;
              case 1: storage.SolarVLTemp = averagedTemp; break;
              case 2: storage.SolarRLTemp = averagedTemp; break;
              case 3: storage.AirInTemp = averagedTemp; break;
              case 4:  if (bme.begin(0x76)) {
                          storage.AirTemp = averagedTemp; break;
                          } else {
                            break;
                          }
        default: break;
      }
      Debug.print(DBG_DEBUG, "[DS18B20] Sensor A%d - Address: %02X%02X%02X%02X%02X%02X%02X%02X - Temperature: %6.2f°C", i, currentAddress_A[0], currentAddress_A[1], currentAddress_A[2],
      currentAddress_A[3], currentAddress_A[4], currentAddress_A[5], currentAddress_A[6], currentAddress_A[7], averagedTemp);
      }
    }

  Debug.print(DBG_DEBUG, "[DS18B20] SolarTemp: %6.2f°C", storage.SolarTemp);
  Debug.print(DBG_DEBUG, "[DS18B20] SolarVLTemp: %6.2f°C", storage.SolarVLTemp);
  Debug.print(DBG_DEBUG, "[DS18B20] SolarRLTemp: %6.2f°C", storage.SolarRLTemp);
  Debug.print(DBG_DEBUG, "[DS18B20] AirInTemp: %6.2f°C", storage.AirInTemp);
  Debug.print(DBG_DEBUG, "[DS18B20] AirTemp: %6.2f°C", storage.AirTemp);

  // Read temperatures from DS18B20_W (Water sensors)
  for (int i = 0; i < 5; i++) {
    byte currentAddress_W[8];
    uint8_t storedAddress_W[8];
    nvs.getBytes(("address_W_" + String(i)).c_str(), storedAddress_W, 8);
    memcpy(currentAddress_W, storedAddress_W, 8);

    sensors_W.requestTemperaturesByAddress(currentAddress_W);
    float temp = sensors_W.getTempC(currentAddress_W);

    if (temp == NAN || temp == -127) {
      Debug.print(DBG_WARNING, "[DS18B20] Error getting temperature for sensor W%d", i);
    } else {
      samples_W_Temp[i].add(temp);
      float averagedTemp = samples_W_Temp[i].getAverage(5);
      switch (storage.Array_W[i]) {
			        case 0: storage.WaterSTemp = temp; break;
              case 1: storage.WaterITemp = temp; break;
              case 2: storage.WaterBTemp = temp; break;
              case 3: storage.WaterWPTemp = temp; break;
              case 4: storage.WaterWTTemp = temp; break;
              default: break;
      }
      Debug.print(DBG_DEBUG, "[DS18B20] Sensor W%d - Address: %02X%02X%02X%02X%02X%02X%02X%02X - Temperature: %6.2f°C", i, currentAddress_W[0], currentAddress_W[1], currentAddress_W[2], currentAddress_W[3], currentAddress_W[4], currentAddress_W[5], currentAddress_W[6], currentAddress_W[7], averagedTemp);
    }
  }

    Debug.print(DBG_DEBUG,"[DS18B20] WaterSTemp: %6.2f°C",storage.WaterSTemp);
    Debug.print(DBG_DEBUG,"[DS18B20] WaterITemp: %6.2f°C",storage.WaterITemp);
    Debug.print(DBG_DEBUG,"[DS18B20] WaterBTemp: %6.2f°C",storage.WaterBTemp);
    Debug.print(DBG_DEBUG,"[DS18B20] WaterWPTemp: %6.2f°C",storage.WaterWPTemp);
    Debug.print(DBG_DEBUG,"[DS18B20] WaterWTTemp: %6.2f°C",storage.WaterWTTemp);
    
    sensors_W.requestTemperatures();
    sensors_A.requestTemperatures();

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO,"[getTemp] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f",td,t_act,t_min,t_max,t_mean);
    #endif

    stack_mon(hwm);
    vTaskDelayUntil(&ticktime,period);
  }
}

// Asynchronous reading of BME280 data
void readBME280(void *pvParameters)
{
  while (!startTasks);
  vTaskDelay(DT5); // Scheduling offset

  TickType_t period = PT5;
  TickType_t ticktime = xTaskGetTickCount();
  static UBaseType_t hwm = 0;

  #ifdef CHRONO
  unsigned long td;
  int t_act=0,t_min=999,t_max=0;
  float t_mean=0.;
  int n=1;
  #endif

  for(;;)
  {
    #ifdef CHRONO
    td = millis();
    #endif

    // Attempt to read BME280 data
    bool success = bme.begin(0x76);

    // If reading was successful, evaluate and store the data
    if (success) {
      lockI2C();
      storage.AirHum = bme.readHumidity();
      storage.AirPress = bme.readPressure() / 100.0F;
      storage.AirTemp = bme.readTemperature();
      samples_AHum.add(storage.AirHum); // Add the data to the RunningMedian objects
      samples_AP.add(storage.AirPress);
      samples_ATemp.add(storage.AirTemp);
      storage.AirHum = samples_AHum.getAverage(); // Calculate the median values and store them in the storage variables
      storage.AirPress = samples_AP.getAverage();
      storage.AirTemp = samples_ATemp.getAverage();
      Debug.print(DBG_DEBUG, "[BME280] BME280: T=%6.2f°C P=%7.2fhPa H=%6.2f%%",
                  storage.AirTemp, storage.AirPress, storage.AirHum);
      unlockI2C();
    } else {
      Debug.print(DBG_WARNING, "[BME280] Error getting BME280 data");
    }

    #ifdef CHRONO
    t_act = millis() - td;
    if(t_act > t_max) t_max = t_act;
    if(t_act < t_min) t_min = t_act;
    t_mean += (t_act - t_mean)/n;
    ++n;
    Debug.print(DBG_INFO, "[readBME280] td: %d t_act: %d t_min: %d t_max: %d t_mean: %4.1f", td, t_act, t_min, t_max, t_mean);
    #endif

    stack_mon(hwm);
    vTaskDelayUntil(&ticktime, period);
  }
}