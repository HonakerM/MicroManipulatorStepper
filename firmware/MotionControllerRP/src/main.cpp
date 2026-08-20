// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------
//
// This is the FreeRTOS entry point. Previously the command-parser/path-planner loop and
// the servo-control loop ran as two bare, un-yielding while(true) loops on core 0 and
// core 1 respectively (via Arduino's setup()/loop() plus multicore_launch_core1()).
// They are now two FreeRTOS tasks, each pinned to a core via vTaskCoreAffinitySet(), so
// the scheduler has visibility into both loops (stack usage, priorities, the ability to
// add further tasks alongside them, etc.) instead of the cores being permanently owned
// by a single unmanaged loop.
//
// Both tasks still run essentially flat-out, matching the original behaviour: each
// iteration ends with taskYIELD() rather than a real vTaskDelay(), so no artificial rate
// limit is imposed on either the command-parsing/path-planning loop or the servo control
// loop. taskYIELD() gives the scheduler on that core a chance to run the idle task /
// timer service / any other same-or-lower priority task without slowing this one down.
// If you later want the servo loop to run at a fixed, deterministic rate instead (e.g.
// to guarantee CPU headroom on core 1 for other tasks), swap its taskYIELD() for
// vTaskDelayUntil() against a fixed period -- see the comment near SERVO_TASK below.
// --------------------------------------------------------------------------------------

#include "main.h"

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/vreg.h"

#include <algorithm>

#include "FreeRTOS.h"
#include "task.h"

#include "compat/Serial_compat.h"
#include "compat/Wire_compat.h"
#include "compat/LittleFS_compat.h"
#include "compat/Arduino_compat.h"

#include "robot.h"
#include "utilities/logging.h"
#include "utilities/frequency_counter.h"
#include "kinematic_models/kinematic_model_delta3d.h"
#include "version.h"
#include "hw_config.h"

#include "demo_gcode_generator.h"

//*** CONFIG ****************************************************************************

// Priorities: both application tasks sit above the idle task; the command/path-planner
// task on core 0 is given slightly higher priority than the servo loop on core 1 since
// they run on different cores anyway and this keeps their relative importance explicit
// if that ever changes.
#define COMMAND_TASK_PRIORITY (tskIDLE_PRIORITY + 2UL)
#define SERVO_TASK_PRIORITY   (tskIDLE_PRIORITY + 2UL)

#define COMMAND_TASK_STACK_SIZE configMINIMAL_STACK_SIZE
#define SERVO_TASK_STACK_SIZE   configMINIMAL_STACK_SIZE

//*** GLOBALS ***************************************************************************

Robot robot(0.01f);

//*** FUNCTIONS *************************************************************************

// Run before setup()
void overclock() {
  vreg_set_voltage(VREG_VOLTAGE_1_20);         // For >133 MHz
  busy_wait_us(10 * 1000);  // 10 ms delay
  set_sys_clock_khz(250000, true);             // Set to 250 MHz
}

void set_led_color(uint8_t r, uint8_t g, uint8_t b) {
  // NeoPixel status LED support was removed as part of the arduino-pico -> pico-sdk
  // port (the NeoPixelConnect library has no pico-sdk equivalent wired up here). If you
  // want it back, drive the WS2812 directly with the PIO program from
  // pico-examples/pio/ws2812 and re-implement this function on top of it.
  (void)r; (void)g; (void)b;
}

void led_blink(uint8_t r, uint8_t g, uint8_t b, int count, int period_time_ms) {
  for (int i = 0; i < count; i++) {
    gpio_put(PIN_BUILTIN_LED, 1);
    // set_led_color(r, g, b);
    sleep_ms(period_time_ms / 2);
    // set_led_color(0, 0, 0);
    gpio_put(PIN_BUILTIN_LED, 0);
    sleep_ms(period_time_ms / 2);
  }
}

//--- FreeRTOS tasks ----------------------------------------------------------------

// Runs on core 0: services serial gcode input and drives the path planner.
static void command_task(void* /*params*/) {
  #ifdef DEMO_MODE
    auto* demo_gcode_generator = new DemoGcodeGenerator(&robot);
    demo_gcode_generator->run();  // blocks forever
  #endif

  while (true) {
    robot.update_command_parser();
    robot.update_path_planner();
    taskYIELD();
  }
}

// Runs on core 1: the real-time servo control loop.
//
// This computes its own dt from a free-running hardware timer (time_us_64), exactly as
// the original bare-metal loop did, so its update rate is not tied to the FreeRTOS tick
// rate. It is throttled only by however fast update_servo_controllers() itself runs.
//
// To instead run this at a fixed, known frequency (trading a bit of throughput for
// predictable timing / headroom for other core-1 tasks), replace the body with a
// vTaskDelayUntil() against a target period, e.g.:
//   const TickType_t period = pdMS_TO_TICKS(1); // 1kHz
//   TickType_t last_wake = xTaskGetTickCount();
//   while (true) {
//     ...
//     vTaskDelayUntil(&last_wake, period);
//   }
static void servo_task(void* /*params*/) {
  LOG_INFO("Starting servo control loop on core %d...", portGET_CORE_ID());

  uint64_t last_time = time_us_64();
  while (true) {
    uint64_t time_us = time_us_64();
    float dt = float(time_us - last_time) * 1e-6f;
    last_time = time_us;

    // limit time delta
    dt = std::min(dt, 0.0001f);

    robot.update_servo_controllers(dt);
    taskYIELD();
  }
}

static void hardware_init() {
  gpio_init(PIN_BUILTIN_LED);
  gpio_set_dir(PIN_BUILTIN_LED, GPIO_OUT);

  led_blink(0, 0, 30, 3, 100);
  overclock();

  Logger::instance().begin(921600, false);
  #ifndef DEMO_MODE
  while (!Serial);
  #endif

  set_led_color(50, 10, 0);

  delay(100);  // Allow time for serial monitor to connect

  LOG_INFO("Open Micro Stage Firmware: %s", FIRMWARE_VERSION);
  LOG_INFO("System clock: %i Mhz", int32_t(clock_get_hz(clk_sys)) / 1000 / 1000);

  // LittleFS.format();
  if (!LittleFS.begin()) {
    LOG_ERROR("Mounting filesystem failed");
  } else {
    FSInfo fs_info;
    LittleFS.info(fs_info);
    LOG_INFO("Mounting filesystem successfully [%i/%i bytes used]",
             (int)fs_info.usedBytes, (int)fs_info.totalBytes);
  }

  LOG_INFO("Initializing device...");
  robot.init();

  set_led_color(0, 20, 0);
  LOG_INFO("Initialization finished");
  LOG_INFO(" ");

  gpio_put(PIN_BUILTIN_LED, 1);
}

static void launch_tasks() {
  TaskHandle_t command_task_handle = nullptr;
  TaskHandle_t servo_task_handle = nullptr;

  xTaskCreate(command_task, "CommandTask", COMMAND_TASK_STACK_SIZE, nullptr,
              COMMAND_TASK_PRIORITY, &command_task_handle);
  xTaskCreate(servo_task, "ServoTask", SERVO_TASK_STACK_SIZE, nullptr,
              SERVO_TASK_PRIORITY, &servo_task_handle);

  #if (configNUMBER_OF_CORES > 1) && configUSE_CORE_AFFINITY
  vTaskCoreAffinitySet(command_task_handle, (UBaseType_t)(1u << 0));  // core 0
  vTaskCoreAffinitySet(servo_task_handle, (UBaseType_t)(1u << 1));    // core 1
  #endif

  vTaskStartScheduler();
}

int main() {
  // stdio (USB CDC) is brought up once, here, for the whole application.
  stdio_init_all();

  hardware_init();
  launch_tasks();

  // vTaskStartScheduler() only returns on failure (e.g. out of heap for the idle/timer
  // tasks).
  while (true) { tight_loop_contents(); }
  return 0;
}
