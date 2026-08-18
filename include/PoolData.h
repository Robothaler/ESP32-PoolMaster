#pragma once
// =============================================================================
//  PoolData.h — domain split of the former monolithic StoreStruct
// =============================================================================
//  One global `storage` object remains (see PoolMaster.h) so existing
//  `storage.PhValue` / `storage.AutoMode` call sites stay valid.
//  Layout is four consecutive bases for cache locality:
//
//    PoolConfig       — NVS-persisted settings (cold, written on change)
//    PoolMeasures     — live sensors (hot, 125 ms polling, NEVER written to NVS)
//    PoolRuntime      — PID windows, inventory, uptime (some keys via saveParam)
//    PoolSolarRemote  — SolarControl snapshot (Matter preferred, HTTP fallback)
//
//  NVS key names are unchanged so a firmware upgrade does not factory-reset
//  pool settings.  Live measurements that older firmware stored (WaterSTemp,
//  PhValue, …) are ignored on load.
// =============================================================================

#include <Arduino.h>
#include <IPAddress.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// Persistent configuration (NVS namespace "PoolMaster")
// ---------------------------------------------------------------------------
struct PoolConfig
{
    uint8_t ConfigVersion;
    uint8_t MatterVersion;
    String SSID, WIFI_PASS, MQTT_USER, MQTT_PASS, MQTT_NAME;
    IPAddress MQTT_IP;
    uint32_t MQTT_PORT;
    bool WIFI_OnOff, MQTTLOGIN_OnOff, BUS_A_B;
    bool Ph_RegulationOnOff, Orp_RegulationOnOff, AutoMode;
    bool SolarLocExt, SolarMode;
    bool Salt_Chlor, SaltMode, WinterMode, WaterHeat;
    bool ValveMode, CleanMode, ValveSwitch, WaterFillMode, HeatPumpMode;
    uint8_t FiltrationDuration, FiltrationStart, FiltrationStop;
    uint8_t FiltrationStartMin, FiltrationStopMax, DelayPIDs;
    uint8_t SolarStartMin, SolarStopMax;
    uint8_t address_A_0[8], address_A_1[8], address_A_2[8], address_A_3[8], address_A_4[8], Array_A[5];
    uint8_t address_W_0[8], address_W_1[8], address_W_2[8], address_W_3[8], address_W_4[8], Array_W[5];
    unsigned long PhPumpUpTimeLimit, ChlPumpUpTimeLimit, WaterFillUpTimeLimit;
    unsigned long WaterFillDuration, PublishPeriod;
    unsigned long PhPIDWindowSize, OrpPIDWindowSize;
    double Ph_SetPoint, Orp_SetPoint;
    double PSI_HighThreshold, PSI_MedThreshold;
    double FLOW_Pulse, FLOW_HighThreshold, FLOW_MedThreshold;
    double FLOW2_Pulse, FLOW2_HighThreshold, FLOW2_MedThreshold;
    double WaterTempLowThreshold, WaterTemp_SetPoint;
    double pHCalibCoeffs0, pHCalibCoeffs1, OrpCalibCoeffs0, OrpCalibCoeffs1;
    double PSICalibCoeffs0, PSICalibCoeffs1, SaltDiff;
    double Ph_Kp, Ph_Ki, Ph_Kd, Orp_Kp, Orp_Ki, Orp_Kd;
    double pHTankVol, ChlTankVol, pHPumpFR, ChlPumpFR, WaterFillFR;
    double SaltCurrentCalibCoeffs0, SaltCurrentCalibCoeffs1;
    double FilterCurrentCalibCoeffs0, FilterCurrentCalibCoeffs1;
    double HeatCurrentCalibCoeffs0, HeatCurrentCalibCoeffs1;
    float CellConstant, PoolVolume;
};

// ---------------------------------------------------------------------------
// Live measurements — written by CombinedPolling / TempTask / BME / Flow.
// Not persisted.  Consecutive doubles keep the 125 ms hot path on few cache lines.
// ---------------------------------------------------------------------------
struct PoolMeasures
{
    double PhValue, PhRawValue, OrpValue, OrpRawValue;
    double PSIValue, FLOWValue, FLOW2Value;
    double WaterSTemp, WaterITemp, WaterBTemp, WaterWPTemp, WaterWTTemp;
    double AirInTemp, AirTemp, AirHum, AirPress;
    double SolarTemp, SolarVLTemp, SolarRLTemp;
    double SaltCurrentValue, FilterCurrentValue, HeatCurrentValue;
};

// ---------------------------------------------------------------------------
// Runtime / inventory — millis-based PID windows, tank fill, uptime.
// A subset is still saved via saveParam() (AcidFill, Uptime, SaltStatus, …).
// ---------------------------------------------------------------------------
struct PoolRuntime
{
    String SaltStatus, ResetTimestamp;
    uint32_t Uptime, LastUptimeUpdate;
    bool SolarOnline, SaltPolarity;
    uint8_t ResetReason, SolarPumpStatus, ValveStatus;
    unsigned long SaltPumpRunTime, PhPIDwindowStartTime, OrpPIDwindowStartTime, WaterFillAnCon;
    double PhPIDOutput, OrpPIDOutput;
    double AcidFill, ChlFill;
    float SaltConcentration, SaltNeeded;
};

// ---------------------------------------------------------------------------
// SolarControl remote snapshot (Matter subscription, else HTTP poll fallback)
// ---------------------------------------------------------------------------
struct PoolSolarRemote
{
    float solarRoofTemp;
    float solarBoilerTemp;
    float solarStorageTemp;
    float solarBackflowTemp;
    bool  solarPumpRunning;
    bool  solarValvePool;
    bool  solarValveOK;
};

// Compatible drop-in for the former flat StoreStruct (storage.Field still works).
struct StoreStruct : PoolConfig, PoolMeasures, PoolRuntime, PoolSolarRemote
{
};
