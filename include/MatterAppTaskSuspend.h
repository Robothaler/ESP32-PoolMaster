#pragma once

/**
 * Suspend / resume pool application tasks during Matter BLE commissioning (PASE).
 * Implemented in Tasks.cpp when MATTER_ENABLED && MATTER_SUSPEND_APP_TASKS_DURING_GAP_PASE.
 *
 * Suspend: GAP CONNECT success while no fabric yet.
 * Resume: GAP DISCONNECT, kCommissioningComplete, or CHIPoBLE connection closed/error
 *          (always on BLE teardown — commissioning may continue on Wi‑Fi while FabricCount is still 0;
 *          idempotent — safe to call multiple times).
 *
 * Uses vTaskSuspend — tasks must not hold locks across long sections; pool loops
 * should mostly be between iterations when NimBLE runs this from its host task.
 *
 * MatterSyncTask is NOT suspended: it may hold esp_matter::chip_stack_lock; suspend
 * there deadlocks CHIP/BLE (Platform event queue post failures, 0x01000000).
 *
 * Task watchdog: PoolMaster registers with TWDT — it must be removed before vTaskSuspend,
 * then re-registered in its loop (matterPoolMasterWdtRearmIfNeededAfterPase) or TWDT resets.
 */
#ifdef MATTER_ENABLED
void matterSuspendAppTasksForGapPase(void);
void matterResumeAppTasksAfterGapPase(void);
void matterPoolMasterWdtRearmIfNeededAfterPase(void);
#endif
