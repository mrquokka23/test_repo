#include "sleep_manager.h"
#include "app_conf.h"
#include "stm32_seq.h"
#include "app_ble.h"
#include "hw_conf.h"              // <-- Timer Server

// One-shot Timer Server timer id
static uint8_t s_sleepTimerId;

// Forward decl
static void StartAdvertisingTask(void);
static void Sleep_WakeCb(void);

void Sleep_RegisterTasks(void)
{
  // Post-wake task (sequencer)
  UTIL_SEQ_RegTask(1 << CFG_TASK_ADV_ON_WAKE_ID, UTIL_SEQ_RFU, StartAdvertisingTask);

  // Create a one-shot Timer Server timer that will wake us from Stop2
  HW_TS_Create(0, &s_sleepTimerId, hw_ts_SingleShot, Sleep_WakeCb);
}

void Sleep_ArmWakeupAndIdle(void)
{
  // Convert seconds to Timer Server ticks (tick period is CFG_TS_TICK_VAL microseconds)
  const uint32_t us = WAKE_PERIOD_SEC * 1000000UL;
  const uint32_t ticks = (us + (CFG_TS_TICK_VAL - 1)) / CFG_TS_TICK_VAL; // ceil div, avoid 0

  HW_TS_Stop(s_sleepTimerId);
  HW_TS_Start(s_sleepTimerId, ticks);

  // return to main; UTIL_SEQ_Idle() will enter Stop2
}

void Sleep_CancelWakeup(void)
{
  HW_TS_Stop(s_sleepTimerId);
}

static void Sleep_WakeCb(void)
{
  // We’re in Timer Server context; just post a task
  UTIL_SEQ_SetTask(1 << CFG_TASK_ADV_ON_WAKE_ID, CFG_SCH_PRIO_0);
}

static void StartAdvertisingTask(void)
{
  // Start advertising after wake
  APP_BLE_StartAdvertising();
}
