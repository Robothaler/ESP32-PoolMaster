#pragma once
// WebUI — AsyncWebServer + WebSocket backend for ESP32-PoolMaster
// Routes are added to the shared AsyncWebServer instance (defined in Ota.cpp).
// Call initWebUI() before server.begin().

void initWebUI();          // register all HTTP + WebSocket routes
void webUIBroadcast();     // push current status JSON to all WS clients (call periodically)
