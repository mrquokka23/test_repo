#pragma once
#include "main.h"

#define WAKE_PERIOD_SEC   10U       // change to taste

void Sleep_ArmWakeupAndIdle(void);
void Sleep_CancelWakeup(void);

// Sequencer task ID you’ll register in app_entry / main
#define TASK_ADV_ON_WAKE  (CFG_SCH_PRIO_0) // pick a free bit
void Sleep_RegisterTasks(void); // call once at init
