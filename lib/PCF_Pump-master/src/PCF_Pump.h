/*
            PCF_Pump - a simple library to handle home-pool filtration and peristaltic pumps
                 (c) Loic74 <loic74650@gmail.com> 2017-2020
Features: 

- keeps track of running time
- keeps track of Tank Levels
- set max running time limit

NB: all timings are in milliseconds
*/

#ifndef PCF_PUMP_h
#define PCF_PUMP_h
#define PCF_PUMP_VERSION "1.0.2"

//Constants used in some of the functions below
#define PUMP_ON  0
#define PUMP_OFF 1
#define TANK_FULL  1
#define TANK_EMPTY 0
#define INTERLOCK_OK  0
#define INTERLOCK_NOK 1
#define NO_LEVEL 170           // Pump with tank but without level switch
#define NO_TANK 255            // Pump without tank
#define NO_INTERLOCK 255  

#define DefaultMaxUpTime 30*60*1000 //default value is 30mins

extern bool lockI2C();
extern void unlockI2C();
 
class PCF_Pump{
  public:

  PCF_Pump(uint8_t startPin, uint8_t statePin, uint8_t levelPin, uint8_t interlockPin,
    float flowRate, float tankVolume, float tankFill, uint8_t address,
    uint8_t interlockAddress = 0xFF, bool activeLow = true);
    void setExternalMutexControl(bool useExternal);    
    void loop();
    bool Start();
    bool Stop();
    bool IsRunning();
    bool TankLevel();
    double GetTankUsage();    
    void SetTankVolume(double Volume);
    void SetFlowRate(double FlowRate);
    bool Interlock();
    void SetMaxUpTime(unsigned long Max);
    void ResetUpTime();
    void SetTankFill(double);
    double GetTankFill();

    void ClearErrors();
    
    unsigned long UpTime;
    unsigned long MaxUpTime;
    unsigned long CurrMaxUpTime;
    bool UpTimeError;
    unsigned long StartTime;
    unsigned long LastStartTime;
    unsigned long StopTime; 
    double flowrate, tankvolume, tankfill;          
    
  private:
  uint8_t getPCFState(uint8_t address);
  bool writePCFState(uint8_t address, uint8_t state);
  uint8_t _startPin;
  uint8_t _statePin;
  uint8_t _levelPin;
  uint8_t _interlockPin;
  uint8_t _address;             // Adress for startPin/statePin
  uint8_t _interlockAddress;    // Adress for interlockPin
  bool _activeLow;
  unsigned long _lastStartTime;
  unsigned long _upTime;
  unsigned long _maxUpTime;
  float _flowRate;
  float _tankVolume;
  float _tankFill;

};
#endif
