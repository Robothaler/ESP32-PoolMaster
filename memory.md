# PoolMaster + Matter — Erkenntnisse für die Fehlersuche

Diese Datei fasst **projektspezifische** Erkenntnisse zusammen, damit spätere Debugging-Runden nicht bei Null anfangen. Sie ersetzt keine offiziellen CHIP/esp_matter-Dokumente.

---

## Wie nutzen

- Bei neuen Symptomen zuerst **Abschnitt „Symptom → typische Ursache“** und die verlinkten Stellen in `include/Config.h`, `sdkconfig.defaults`, `platformio.ini`, `src/Tasks.cpp`, `src/MatterBridge.cpp` prüfen.
- **Log-Dateien** unter `logs/` benennen oft das Datum und das Szenario (z. B. `serial-20260503_162037.log`).

---

## Symptom → typische Ursache (Kurzübersicht)

| Symptom | Wo nachsehen |
|--------|----------------|
| Apple Home Timeout / BLE **0x213**, „schon in einem Zuhause“, kein Fortschritt nach GATT | Interner Heap knapp; zu viel Serial/printf in NimBLE-Pfaden; MQTT/WebSocket während Pairing; Wi‑Fi vs BLE Konkurrenz |
| **CASE** hängt / Sigma3 / „Long CASE“ / Zertifikatszeit | System-Wanduhr vs. CHIP; `settimeofday` / SNTP; unrealistische Epoch (nicht vor 2020) |
| **Fail-Safe 32** / Commissioning bricht ab | Event-Loop überlastet; zu wenig Yield für CHIPoBLE; ggf. zu kleine Platform-Queue |
| **TWDT** auf `PoolMaster` während PASE | Task suspendiert aber noch im WDT; siehe `Tasks.cpp` (WDT abmelden / nach PASE wieder anmelden) |
| **0x01000000** / „Failed to schedule work“ / Platform-Queue | Last + Queue-Größe; **GAP-Suspend** von App-Tasks war hier korreliert (Default jetzt **aus** — lieber Throttle) |
| Relais/PCF „spinnt“ bei minimalem Task-Set | **T15** muss laufen, sonst fehlt Worker + Serial-Spam → CHIP verhungert (`Config.h` erzwingt T14+T15 bei `MATTER_ENABLED` + minimalem Set) |

---

## Zeit / Wall Clock und CASE

- Matter **CASE** nutzt die **reale Uhr** für Zertifikatsgültigkeit. Wenn nur eine Arduino-Zeitbibliothek läuft, kann **CHIP** eine falsche oder fehlende Realzeit sehen → CASE-Probleme.
- Im Projekt: nach gültiger lokaler Zeit **`settimeofday`** (ESP-Systemzeit), Abgleich mit CHIP (`MatterBridge.cpp`: Sync der Chip-Wall-Clock; **Mindest-Epoch ≥ 2020**, kein „Y2K“-Poisoning).
- **`matterResyncChipWallClockAfterNtp()`** nach NTP (u. a. `Setup.cpp`) — SNTP ist in `sdkconfig.defaults` mit `CONFIG_ENABLE_SNTP_TIME_SYNC=y` unterstützt.

---

## Heap, PSRAM, NimBLE

- **`CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=512`** ist für PoolMaster **wichtig**: viele kleine Allocs bleiben intern, große gehen nach PSRAM. Ohne kann der **interne** Heap schnell unter ~8–10 KB fallen → BTP/PASE-Allocs scheitern, **BLE 0x213**.
- **`CONFIG_BT_NIMBLE_MEM_ALLOC_MODE_DEFAULT=y`**: NimBLE-mbufs nicht zusätzlich den internen Heap drücken (siehe Kommentare in `sdkconfig.defaults`).
- **`CONFIG_BT_CTRL_BLE_MAX_ACT=2`**: weniger BT-Controller-RAM, vermeidet Boot-OOM bei Matter.

---

## CHIP Platform Event Queue

- **`CONFIG_MAX_EVENT_QUEUE_SIZE=40`** in `sdkconfig.defaults` (Angleichung an übliche esp_matter/connectedhomeip-Empfehlungen bei Last).
- Zu kleine Queue → Fehler beim Posten von Arbeit auf die Platform-/CHIP-Events unter Last.

---

## Wi‑Fi, BLE, MQTT, Web-UI

- **2,4 GHz geteilt**: Während Commissioning **`MATTER_NO_MQTT_UNTIL_COMMISSIONED`** (Default 1) — MQTT bricht Pairing zuverlässig.
- **`MATTER_DEFER_HTTP_SERVER_UNTIL_COMMISSIONED`**: WebUI/AsyncTCP erst nach Fabric (oder Fallback-Timer) — weniger Konkurrenz zu CHIPoBLE.
- **`MATTER_WIFI_PS_MAX_WHILE_UNCOMMISSIONED`**: In `platformio.ini` für Matter-Envs oft **0**, damit nach AddNOC/CASE kein unnötiges Delay durch aggressives Wi‑Fi-Power-Save (Fail-Safe ~60 s).
- Wi‑Fi-STA während offener BLE-Session abschalten: experimentell, in Logs teils **0x213** — Defaults in `Config.h` vorsichtig (z. B. `MATTER_WIFI_STA_OFF_DURING_BLE_GAP=0`).

---

## App-Tasks: Suspend vs. Throttle, MatterSync, TWDT

- **`MATTER_SUSPEND_APP_TASKS_DURING_GAP_PASE`** — Default **0**. Suspend von PoolMaster o. Ä. während GAP korrelierte mit Platform-Queue-Fehlern und **0x213** nach erstem GATT-Write (`logs/serial-20260503_162037.log`).
- **`MATTER_THROTTLE_APP_TASKS_DURING_CHIPOBLE`** — Default **1**: kurzes `vTaskDelay` in hohen Pool-Tasks statt hartem Suspend → weniger Deadlock-Risiko, mehr CPU für NimBLE/CHIP.
- **MatterSyncTask (T14)** soll beim Suspend/Throttle-Konzept **nicht** mit abgeschaltet werden, wenn er für den CHIP-Pfad gebraucht wird (Projektregel aus Debugging).
- **TWDT**: Vor GAP-Suspend ggf. **`esp_task_wdt_delete`** für betroffene Tasks; nach PASE wieder anmelden (`matterPoolMasterWdtRearmIfNeededAfterPase()` o. ä.) — Details `Tasks.cpp` / `PoolMaster.cpp`.

---

## Minimale Builds vs. volle Steuerung (`platformio.ini`)

- **`POOLMASTER_ENABLE_ALL_POOL_TASKS=0`**: nur minimales Task-Set (siehe `Config.h`); für Commissioning-Bisektierung. **Normbetrieb / volle Pool-Steuerung:** Flag weglassen oder **=1** (Default in `Config.h` ist 1).
- **`MATTER_MINIMAL_DEVICE=1`**: ein einziges On/Off-Gerät ohne Bridge — weniger RAM, Pairing-Debug. **Wechsel minimal ↔ voll** → fast immer **Matter Factory Reset** (anderes Endpoint-Layout).
- Kommentare in `[env:matter_serial]` / `[env:matter_ota]` beschreiben die Umstellung für Bisect vs. Produktion.

---

## Matter Controller (SolarControl)

- In `sdkconfig.defaults`: **`CONFIG_ESP_MATTER_CONTROLLER_ENABLE=n`** (Stand Debugging 2026-04): Controller teilt sich Last mit BLE/CHIPoBLE; bei aktivem Controller waren **15 s Apple-Timeouts** problematisch. Wieder aktivieren nur wenn Commissioning stabil und Stack ggf. angepasst.

---

## Logging während Pairing

- **`CORE_DEBUG_LEVEL`** / **`MATTER_LOG_EXTRA_CHIP_TAGS`** / **`MATTER_BLE_GAP_DIAG_LISTENER`**: weniger Chat und weniger `Serial` **im NimBLE-GAP-Pfad** reduziert Verhungern des BLE-Stacks (siehe Kommentare in `platformio.ini` und `Config.h`).
- **`MATTER_AGENT_DEBUG_NDJSON`**: bewusst oft aus — `printf` kann NimBLE blockieren.

---

## Relevante Dateien (Checkliste)

| Datei | Inhalt |
|--------|--------|
| `include/Config.h` | Matter-/WiFi-/Task-Schalter, Task-Matrix T1–T15 |
| `sdkconfig.defaults` | PSRAM, NimBLE, `CONFIG_MAX_EVENT_QUEUE_SIZE`, SNTP, Controller |
| `platformio.ini` | `MATTER_ENABLED`, Task-/Minimal-Flags, Stack, Log-Level |
| `src/Tasks.cpp` | Task-Erzeugung, Skip-Logs, WDT/Suspend/Throttle-Hooks |
| `src/MatterBridge.cpp` | Bridge vs. Minimal-Gerät, Clock-Sync |
| `src/Setup.cpp` | Zeit/NTP, `settimeofday` |

---

## Changelog dieser Notiz

- **2026-05-03**: Erste Version aus Matter-Commissioning-Debug (CASE, Queue, TWDT, Minimal-Tasks, Clock, PSRAM/NimBLE).
