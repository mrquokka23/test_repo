#pragma once
#include "main.h"

#define WAKE_PERIOD_SEC   10U

void Sleep_RegisterTasks(void);
void Sleep_ArmWakeupAndIdle(void);
void Sleep_CancelWakeup(void);
