add_executable(MotionControllerRP
    main.cpp
    freertos_hooks.cpp
    robot.cpp

    compat/Serial_compat.cpp
    compat/Wire_compat.cpp
    compat/LittleFS_compat.cpp

    command_parser/command_parser.cpp

    demo_gcode_generator.cpp

    hardware/MT6701_encoder.cpp
    hardware/MT6835_encoder.cpp
    hardware/TB6612_motor_driver.cpp
    hardware/spi_interface.cpp

    kinematic_models/kinematic_model_base.cpp
    kinematic_models/kinematic_model_delta3d.cpp

    motion_control/motion_controller.cpp
    motion_control/path_planner.cpp
    motion_control/path_segment.cpp

    peripheral_control/peripheral.cpp

    robot_joint/robot_joint.cpp

    robot_tool/pwm_tool.cpp

    servo_control/actuator_calibration.cpp
    servo_control/homing_controller.cpp
    servo_control/pid.cpp
    servo_control/servo_controller.cpp

    utilities/logging.cpp
    utilities/lookup_table.cpp
    utilities/utilities.cpp
    utilities/waveforms.cpp
)

target_include_directories(MotionControllerRP PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/.. # so "FreeRTOSConfig.h" at the project root is found
)

target_link_libraries(MotionControllerRP PRIVATE
    pico_stdlib
    pico_multicore

    hardware_clocks
    hardware_pll
    hardware_vreg
    hardware_i2c
    hardware_spi
    hardware_pwm
    hardware_pio
    hardware_dma
    hardware_exception

    FreeRTOS-Kernel-Heap4

    filesystem_default   # from pico-vfs: mounts littlefs on onboard flash at "/"
)

# USB CDC for stdio (Serial_compat is built on top of this); no UART stdio.
pico_enable_stdio_usb(MotionControllerRP 1)
pico_enable_stdio_uart(MotionControllerRP 0)

# Registers pico-vfs's default fs_init() and gives it a size budget on flash.
pico_enable_filesystem(MotionControllerRP
    SIZE 524288  # 512KB reserved for calibration lookup tables etc. Tune to taste.
)

# Set the firmware version string burned into the binary, mirroring the previous
# PlatformIO build (adjust as you see fit / wire up to git describe, etc).
target_compile_definitions(MotionControllerRP PRIVATE
    # DEMO_MODE   # uncomment to build the demo gcode generator instead of the real IO
)

pico_add_extra_outputs(MotionControllerRP)
