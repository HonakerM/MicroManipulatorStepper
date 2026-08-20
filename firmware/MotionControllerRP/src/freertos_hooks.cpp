// --------------------------------------------------------------------------------------
// Hook functions required by the FreeRTOSConfig.h settings in this project
// (configCHECK_FOR_STACK_OVERFLOW=2, configUSE_MALLOC_FAILED_HOOK=1).
// --------------------------------------------------------------------------------------
#include "FreeRTOS.h"
#include "task.h"

#include "utilities/logging.h"

extern "C" void vApplicationStackOverflowHook(TaskHandle_t /*task*/, char* task_name) {
  error_trap(task_name ? task_name : "stack overflow");
}

extern "C" void vApplicationMallocFailedHook(void) {
  error_trap("FreeRTOS heap allocation failed");
}
