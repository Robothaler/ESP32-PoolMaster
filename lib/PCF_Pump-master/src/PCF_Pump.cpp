#include "Arduino.h"
#include "PCF_Pump.h"
#include "PoolMaster.h"

//Constructor
//PumpPin is the Arduino relay output pin number to be switched to start/stop the pump
//TankLevelPin is the Arduino digital input pin number connected to the tank level switch
//Interlockpin is the Arduino digital input number connected to an "interlock". 
//If this input is LOW, pump is stopped and/or cannot start. This is used for instance to stop
//the Orp or pH pumps in case filtration pump is not running
//IsRunningSensorPin is the pin which is checked to know whether the pump is running or not. 
//It can be the same pin as "PumpPin" in case there is no sensor on the pump (pressure, current, etc) which is not as robust. 
//This option is especially useful in the case where the filtration pump is not managed by the Arduino. 
//FlowRate is the flow rate of the pump in Liters/Hour, typically 1.5 or 3.0 L/hour for peristaltic pumps for pools. This is used to compute how much of the tank we have emptied out
//TankVolume is used here to compute the percentage fill used
PCF_Pump::PCF_Pump(uint8_t PumpPin, uint8_t IsRunningSensorPin, uint8_t TankLevelPin, 
           uint8_t Interlockpin, double FlowRate, double TankVolume, double TankFill)
{
  pumppin = PumpPin;
  isrunningsensorpin = IsRunningSensorPin;
  tanklevelpin = TankLevelPin;
  interlockpin = Interlockpin;
  flowrate = FlowRate; //in Liters per hour
  tankvolume = TankVolume; //in Liters
  tankfill = TankFill; // in percent
  StartTime = 0;
  LastStartTime = 0;
  StopTime = 0;
  UpTime = 0;        
  UpTimeError = 0;
  MaxUpTime = DefaultMaxUpTime;
  CurrMaxUpTime = MaxUpTime;
}     

//Call this in the main loop, for every loop, as often as possible
void PCF_Pump::loop()
{
  if(pcf8574_I.digitalRead(isrunningsensorpin) == PUMP_ON)
  {
    UpTime += millis() - StartTime;
    StartTime = millis();
  }

  if((CurrMaxUpTime > 0) && (UpTime >= CurrMaxUpTime))
  {
    Stop();
    UpTimeError = true;
  }

  if(!this->PCF_Pump::TankLevel()) this->PCF_Pump::Stop();

  if(interlockpin != NO_INTERLOCK)
  {
    if(pcf8574_I.digitalRead(interlockpin) == INTERLOCK_NOK)
       Stop();
  }
}

//Switch pump ON if over time was not reached, tank is not empty and interlock is OK
bool PCF_Pump::Start()
{
  if((pcf8574_I.digitalRead(isrunningsensorpin) == PUMP_OFF) 
    && !UpTimeError
    && this->PCF_Pump::TankLevel()
    && ((interlockpin == NO_INTERLOCK) || (pcf8574_I.digitalRead(interlockpin) == INTERLOCK_OK)))    //if((pcf8574.digitalRead(pumppin) == false))
  {
    pcf8574_I.digitalWrite(pumppin, PUMP_ON);
    StartTime = LastStartTime = millis(); 
    return true; 
  }
  else return false;
}

//Switch pump OFF
bool PCF_Pump::Stop()
{
  if(pcf8574_I.digitalRead(isrunningsensorpin) == PUMP_ON)
  {
    pcf8574_I.digitalWrite(pumppin, PUMP_OFF);
    UpTime += millis() - StartTime; 
    return true;
  }
  else return false;
}

//Reset the tracking of running time
//This is typically called every day at midnight
void PCF_Pump::ResetUpTime()
{
  StartTime = 0;
  StopTime = 0;
  UpTime = 0;
  CurrMaxUpTime = MaxUpTime;
}

//Set a maximum running time (in millisecs) per day (in case ResetUpTime() is called once per day)
//Once reached, pump is stopped and "UpTimeError" error flag is raised
//Set "Max" to 0 to disable limit
void PCF_Pump::SetMaxUpTime(unsigned long Max)
{
  MaxUpTime = Max;
  CurrMaxUpTime = MaxUpTime;
}

//Clear "UpTimeError" error flag and allow the pump to run for an extra MaxUpTime
void PCF_Pump::ClearErrors()
{
  if(UpTimeError)
  {
    CurrMaxUpTime += MaxUpTime;
    UpTimeError = false;
  }
}

//tank level status (true = full, false = empty)
bool PCF_Pump::TankLevel()
{
  if(tanklevelpin == NO_TANK)
  {
    return true;
  }
  else if (tanklevelpin == NO_LEVEL)
  {
    return (this->PCF_Pump::GetTankFill() > 5.); //alert below 5% 
  }
  else
  {
    return (pcf8574_I.digitalRead(tanklevelpin) == TANK_FULL);
  } 
}

//Return the percentage used since last reset of UpTime
double PCF_Pump::GetTankUsage() 
{
  float PercentageUsed = -1.0;
  if((tankvolume != 0.0) && (flowrate !=0.0))
  {
    double MinutesOfUpTime = (double)UpTime/1000.0/60.0;
    double Consumption = flowrate/60.0*MinutesOfUpTime;
    PercentageUsed = Consumption/tankvolume*100.0;
  }
  return (PercentageUsed);  
}

//Return the remaining quantity in tank in %. When resetting UpTime, SetTankFill must be called accordingly
double PCF_Pump::GetTankFill()
{
  return (tankfill - this->PCF_Pump::GetTankUsage());
}

//Set Tank volume
//Typically call this function when changing tank and set it to the full volume
void PCF_Pump::SetTankVolume(double Volume)
{
  tankvolume = Volume;
}

//Set flow rate of the pump in Liters/hour
void PCF_Pump::SetFlowRate(double FlowRate)
{
  flowrate = FlowRate;
}

//Set tank fill (percentage of tank volume)
void PCF_Pump::SetTankFill(double TankFill)
{
  tankfill = TankFill;
}

//interlock status
bool PCF_Pump::Interlock()
{
  return (pcf8574_I.digitalRead(interlockpin) == INTERLOCK_OK);
}

//pump status
bool PCF_Pump::IsRunning()
{
  return (pcf8574_I.digitalRead(isrunningsensorpin) == PUMP_ON);
}
