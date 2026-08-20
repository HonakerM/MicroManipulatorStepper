// --------------------------------------------------------------------------------------
// FreeRTOSConfig.h for MotionControllerRP (RP2350, dual-core SMP)
//
// Based on the config used by pico-examples/freertos (Raspberry Pi's own reference
// config for the RP2040/RP2350 SMP port), trimmed to what this project needs.
// See: https://github.com/raspberrypi/pico-examples/blob/master/freertos/FreeRTOSConfig_examples_common.h
// --------------------------------------------------------------------------------------
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Scheduler related */
#define configUSE_PREEMPTION                    1
#define configUSE_TICKLESS_IDLE                 0
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    32
#define configMINIMAL_STACK_SIZE                ((configSTACK_DEPTH_TYPE)1024)
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1

/* Synchronization related */
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_QUEUE_SETS                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 5

/* System */
#define configSTACK_DEPTH_TYPE                  uint32_t
#define configMESSAGE_BUFFER_LENGTH_TYPE        size_t

/* Memory allocation */
#ifndef configSUPPORT_STATIC_ALLOCATION
#define configSUPPORT_STATIC_ALLOCATION         0
#endif
#ifndef configSUPPORT_DYNAMIC_ALLOCATION
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#endif
// Total FreeRTOS heap. Increase this if you add more tasks or the servo/path-planner
// tasks need bigger stacks than configMINIMAL_STACK_SIZE.
#define configTOTAL_HEAP_SIZE                   (128 * 1024)
#define configAPPLICATION_ALLOCATED_HEAP        0

/* Hooks */
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            1
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/* Stats */
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* Co-routines: unused by this project */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         1

/* Software timers */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            1024

/* SMP (set by the RP2xxx port) --------------------------------------------------------
 * This project intentionally runs across both cores: the command-parser/path-planner
 * task is pinned to core 0 and the servo control task to core 1, replacing the original
 * bare multicore_launch_core1() busy loops. */
#if FREE_RTOS_KERNEL_SMP
#ifndef configNUMBER_OF_CORES
#define configNUMBER_OF_CORES                   2
#endif
#define configNUM_CORES                         configNUMBER_OF_CORES
#define configTICK_CORE                         0
#define configRUN_MULTIPLE_PRIORITIES           1
#if configNUMBER_OF_CORES > 1
#define configUSE_CORE_AFFINITY                 1
#endif
#define configUSE_PASSIVE_IDLE_HOOK             0
#endif

/* RP2040/RP2350 specific: lets FreeRTOS tasks interoperate with pico_sync primitives
 * (mutex/semaphore/queue) and the pico_time API. */
#define configSUPPORT_PICO_SYNC_INTEROP         1
#define configSUPPORT_PICO_TIME_INTEROP         1

#include <assert.h>
#define configASSERT(x) assert(x)

/* API inclusion */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xTaskResumeFromISR              1
#define INCLUDE_xQueueGetMutexHolder            1

#if PICO_RP2350
#define configENABLE_MPU                        0
#define configENABLE_TRUSTZONE                  0
#define configRUN_FREERTOS_SECURE_ONLY          1
#define configENABLE_FPU                        1
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    16
#endif

#endif /* FREERTOS_CONFIG_H */
