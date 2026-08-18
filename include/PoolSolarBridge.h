#pragma once
// HTTP pool ↔ Solar bridge (LAN): SolarControl polls GET /api/pool-solar/v1/read
// Optional fallback: Pool polls Solar GET {base}/api/pool-solar/v1/solar
//   (skipped automatically while Matter SolarControl reports are fresh).
//   Incoming JSON may include: roofTemp_C, boilerTemp_C, storageTemp_C, backflowTemp_C or returnTemp_C,
//   pumpOn, valvePool, valveFeedbackEp3, mode, modeEp1, circulationOn, illuminationOn, poolModeRequestHw.
//
// NVS namespace "PoolMaster" (Preferences), keys max 15 chars:
//   psolBrTok   — Bearer token for /read (empty = no auth). User doc: pool_solar_bridge_token
//   psolBrUrl   — Solar base URL without trailing slash (default http://192.168.178.176 if unset in NVS)
//   psolPollS   — Poll interval seconds (5–300), default 10 when URL set
//
// JSON commands (MQTT/PoolServer queue, same as other keys):
//   PoolSolBrTok, PoolSolBrUrl, PoolSolPollS

#include <Arduino.h>
#include <Preferences.h>

class AsyncWebServerRequest;

void poolSolarBridgeLoadFromNvs(Preferences& p);

/** If a token is configured, require Authorization: Bearer <exact token>. */
bool poolSolarBridgeAuthorizeRead(AsyncWebServerRequest* req);

String poolSolarBridgeBuildReadJson();

/**
 * Solar pool heat request — single source of truth for HTTP GET /read (solarModeRequest),
 * Matter Solar-Mode-Request OnOff, and regulation intent.
 * True only in external solar mode when filtration/time window is active and collector is hot enough vs pool.
 */
bool poolSolarBridgeSolarModeRequest(void);

/**
 * MQTT publishSolarMode / SolarControl mode hint: 1 = pool, 2 = puffer, 3 = off,
 * -1 = hold previous command (active window but hysteresis middle zone).
 */
int poolSolarBridgeExternalSolarPublishEvent(void);

/** Called from otaTask loop; polls Solar when URL + WiFi are configured. */
void poolSolarBridgePollTick();

int poolSolarBridgeHttpPollLastCode();
/** Milliseconds since last successful solar JSON parse, or -1 if never. */
int32_t poolSolarBridgeHttpPollAgeMs();
const char* poolSolarBridgeHttpPollMode();
uint8_t poolSolarBridgeHttpPollModeEp1();
bool poolSolarBridgeHttpPollCirculationOn();
bool poolSolarBridgeHttpPollIlluminationOn();
bool poolSolarBridgeHttpPollPoolModeRequestHw();

bool poolSolarBridgeSetToken(const String& tok);
bool poolSolarBridgeSetSolarBaseUrl(const String& url);
bool poolSolarBridgeSetPollIntervalSec(uint32_t sec);

String poolSolarBridgeTokenMaskedForSettings();
const String& poolSolarBridgeSolarBaseUrlRef();
uint32_t poolSolarBridgePollIntervalSec();
