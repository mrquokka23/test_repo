#include "sleep_manager.h"
#include "stm32wbxx_hal_rtc.h"
#include "app_ble.h"
#include "stm32_seq.h"

static void StartAdvertisingTask(void);

extern RTC_HandleTypeDef hrtc;

void Sleep_RegisterTasks(void)
{
  // Register the task that runs after wake to start advertising
  UTIL_SEQ_RegTask(1 << TASK_ADV_ON_WAKE, UTIL_SEQ_RFU, StartAdvertisingTask);
}

static void StartAdvertisingTask(void)
{
  // Stop the wakeup timer (optional if you set one-shot)
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  // Your project should already expose this; otherwise call aci_gap_set_discoverable
  Adv_Request(APP_BLE_FAST_ADV);
}

void Sleep_ArmWakeupAndIdle(void)
{
  // ~10 s at 2048 Hz (RTC/16)
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, WAKE_PERIOD_SEC * 2048U, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

  // Nothing else scheduled? Let sequencer idle enter Stop2 for you.
  // Do NOT block here; just return to main loop (UTIL_SEQ_Run).
}

void Sleep_CancelWakeup(void)
{
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
}
