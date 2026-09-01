from open_micro_stage_api import OpenMicroStageInterface
from open_micro_stage_api.api import SerialInterface
import time
import csv



def select_serial_device():
    """List available serial devices and prompt the user to select one."""

    devices = SerialInterface.list_serial_devices()
    devices = sorted(devices, key=lambda d: d.port)  # Sort devices by port name

    if not devices:
        print("No serial devices found.")
        return None

    print("\n=== Available Serial Devices ===")
    print(
        f"{'#':<4}"
        f"{'PORT':<12}"
        f"{'DESCRIPTION':<35}"
        f"{'MANUFACTURER':<25}"
        f"{'VID:PID':<12}"
        f"{'SERIAL NUMBER':<25}"
    )
    print("-" * 113)

    for i, device in enumerate(devices, start=1):
        vid_pid = (
            f"{device.vid:04X}:{device.pid:04X}"
            if device.vid is not None and device.pid is not None
            else "N/A"
        )

        print(
            f"{i:<4}"
            f"{device.port:<12}"
            f"{(device.description or 'N/A')[:34]:<35}"
            f"{(device.manufacturer or 'N/A')[:24]:<25}"
            f"{vid_pid:<12}"
            f"{(device.serial_number or 'N/A')[:24]:<25}"
        )

    print()

    while True:
        choice = input(
            f"Select device (1-{len(devices)}) or 'q' to quit: "
        ).strip()

        if choice.lower() in {"q", "quit", "exit"}:
            return None

        try:
            index = int(choice) - 1
        except ValueError:
            print("Invalid selection. Please enter a number.")
            continue

        if 0 <= index < len(devices):
            selected = devices[index]
            print(f"Selected {selected.port}: {selected.description}")
            return selected

        print(f"Invalid selection. Please select 1-{len(devices)}.")



def get_user_mode():
    """Ask user which mode to run: calibration, set move, or free move"""
    while True:
        print("\n=== Mode Selection ===")
        print("1. Calibration")
        print("2. Set Move")
        print("3. Free Move")
        print("4. Home Single Axis")
        print("5. Send Raw Command")
        print("6. Exit")
        
        choice = input("Select mode (1-6): ").strip()
        
        if choice == '1':
            return 'calibration'
        elif choice == '2':
            return 'set_move'
        elif choice == '3':
            return 'free_move'
        elif choice == '4':
            return 'home_axis'
        elif choice == '5':
            return 'raw_command'
        elif choice == '6':
            return 'exit'
        else:
            print("Invalid choice. Please select 1-6.")


def run_calibration(oms):
    """Run calibration mode"""
    try:
        axis = int(input("Enter axis to calibrate (0-2): ").strip())
        if axis not in [0, 1, 2]:
            print("Invalid axis. Must be 0, 1, or 2.")
            return
        
        _, data = oms.calibrate_joint(axis, save_result=True)


        transposed_data = [list(row) for row in zip(*data)]
        with open(f'output_{axis}.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile, delimiter=',')
            writer.writerows(transposed_data)
        
        print(f"Calibration complete. Data saved to output_{axis}.csv")
    except ValueError:
        print("Invalid input. Please enter a number.")


def run_set_move(oms):
    """Run set move mode"""
    user_input = input("Press Enter to home or 's' to skip: ")
    if user_input.lower() != 's':
        oms.home()

    # input("Press Enter to move 90 degrees")
    # oms.set_rotation(90.0)

    # time.sleep(1)
    # input("Press Enter to move 180 degrees")
    # oms.set_rotation(180.0)
    
    # time.sleep(1)
    # input("Press Enter to move 270 degrees")
    # oms.set_rotation(270.0)
    
    # time.sleep(1)
    # input("Press Enter to move back to 0 degrees")
    # oms.set_rotation(0.0)

    input("Press Enter to move a little bit")
    x, y, z = oms.read_current_position()
    oms.set_pose(x+0.01, y+0.01, z)
    print("Moved via software")

    input("Press Enter to move to 3,4,z")
    oms.set_pose(3.0, 4.0, z)
    oms.wait_for_stop()

    input("Press Enter to move to 0,0,0")
    oms.set_pose(0.0, 0.0, z)
    oms.wait_for_stop()

    oms.read_device_state_info()

def run_rotation(oms):
    """Run rotation mode"""
    user_input = input("Press Enter to home or 's' to skip: ")
    if user_input.lower() != 's':
        oms.home()

    oms.wait_for_stop()

    while True:
        try:
            x, y, z = oms.read_current_position()
            print(f"\nCurrent position -> X:{x:.4f}, Y:{y:.4f}, Z:{z:.4f}")

            user_input = input("Enter target X,Y,Z (or 'q' to quit, 'i' to info): ")

            if user_input.lower() == 'q':
                break
            if user_input.lower() == 'i':
                oms.read_device_state_info()
                continue

            try:
                if "+" in user_input:
                    user_input = user_input.replace("+", ",")
                x_str, y_str, z_str = user_input.split(',')
                x_target = float(x_str.strip())
                y_target = float(y_str.strip())
                z_target = float(z_str.strip())

                print(f"Moving to X:{x_target}, Y:{y_target}, Z:{z_target}")
                resp = oms.set_pose(x_target, y_target, z_target)
                print("test and resp:", resp)
                if not resp:
                    print(f"Failed to move to target position. The pose may be unreachable.")
                oms.wait_for_stop()

            except ValueError:
                print("Invalid input. Use format: X,Y,Z or X+Y+Z")
        except ValueError:
            print("Invalid data from device. Please check encoder positioning")
            break
        except KeyboardInterrupt:
            print("\nExiting free move mode.")
            break

    oms.read_device_state_info()


def run_free_move(oms):
    """Run free move mode"""
    user_input = input("Press Enter to home or 's' to skip: ")
    if user_input.lower() != 's':
        oms.home()

    oms.wait_for_stop()

    while True:
        try:
            x, y, z = oms.read_current_position()
            print(f"\nCurrent position -> X:{x:.4f}, Y:{y:.4f}, Z:{z:.4f}")

            user_input = input("Enter target X,Y,Z (or 'q' to quit, 'd' to diag, 's' to send command,'i' to info): ")

            if user_input.lower() == 'q':
                break
            if user_input.lower() == 'i':
                oms.read_device_state_info()
                continue
            if user_input.lower() == 'd':
                oms.send_command("M60")
                oms.send_command("M61")
                continue
            if user_input.lower() == 's':
                command_input = input("Enter command to send: ").strip()
                oms.send_command(command_input)
                continue
            
            if "+" in user_input:
                user_input = user_input.replace("+", ",")
            x_str, y_str, z_str = user_input.split(',')
            x_target = float(x_str.strip())
            y_target = float(y_str.strip())
            z_target = float(z_str.strip())

            print(f"Moving to X:{x_target}, Y:{y_target}, Z:{z_target}")
            oms.move_to(x_target, y_target, z_target, 5, blocking=True, timeout=5.0)
            oms.wait_for_stop()

        except ValueError:
            print("Invalid input. Use format: X,Y,Z or X+Y+Z")
        except KeyboardInterrupt:
            print("\nExiting free move mode.")
            break

    oms.read_device_state_info()


def run_home_axis(oms):
    """Repeatedly home a single axis chosen by the user."""
    while True:
        user_input = input("Enter axis to home (0-2, or 'q' to quit): ").strip().lower()

        if user_input in {'q', 'quit', 'exit'}:
            break

        try:
            axis = int(user_input)
        except ValueError:
            print("Invalid axis. Please enter 0, 1, or 2.")
            continue

        if axis not in [0, 1, 2]:
            print("Invalid axis. Please enter 0, 1, or 2.")
            continue

        oms.home(axis_list=[axis])
        oms.wait_for_stop()
        print(f"Axis {axis} homed.")


def run_raw_command(oms):
    """Send arbitrary serial commands to the device."""
    while True:
        user_input = input("Enter command to send (or 'q' to quit): ").strip()

        if user_input.lower() in {'q', 'quit', 'exit'}:
            break

        if not user_input:
            print("Please enter a command string.")
            continue

        result, response = oms.send_custom_command(user_input)
        print(f"Status: {result.name}")
        if response:
            print(response)




# Select serial device
device = select_serial_device()

if device is None:
    print("No serial device selected. Exiting.")
    exit(0)

# Create interface and connect
oms = OpenMicroStageInterface(
    show_communication=True,
    show_log_messages=True,
)

oms.connect(device.port)
oms.read_device_state_info()

# Main - run once and exit
mode = get_user_mode()

if mode == 'calibration':
    run_calibration(oms)
elif mode == 'set_move':
    run_set_move(oms)
elif mode == 'free_move':
    run_free_move(oms)
elif mode == 'home_axis':
    run_home_axis(oms)
elif mode == 'raw_command':
    run_raw_command(oms)
