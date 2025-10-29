import socket
import time
import math
import random

# --- Configuration ---
SIMULATOR_IP = "127.0.0.1"
PLOT_LISTENER_PORT = 7778
SEND_INTERVAL_SECONDS = 0.05

# --- Socket Setup ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# --- Helper Functions ---
def get_current_plot_time_ms():
    """
    Gets the current absolute system time in milliseconds.
    This is the CORRECT way to generate timestamps for the plotter protocol.
    """
    return int(time.time() * 1000)

def get_float_input(prompt, default_value=None):
    """Safely gets a float from user input."""
    while True:
        try:
            value_str = input(prompt)
            if not value_str and default_value is not None:
                return default_value
            return float(value_str)
        except ValueError:
            print("Invalid input. Please enter a valid number.")

def get_int_input(prompt, default_value=None):
    """Safely gets an integer from user input."""
    while True:
        try:
            value_str = input(prompt)
            if not value_str and default_value is not None:
                return default_value
            return int(value_str)
        except ValueError:
            print("Invalid input. Please enter a valid integer.")

def get_string_input(prompt, default_value=None):
    """Safely gets a string from user input."""
    value_str = input(prompt)
    if not value_str and default_value is not None:
        return default_value
    return value_str

# --- Direct Message Sending Function ---
def send_message_direct(full_message_string):
    """Sends a pre-formatted string directly."""
    sock.sendto(full_message_string.encode('utf-8'), (SIMULATOR_IP, PLOT_LISTENER_PORT))

# --- Individual Message Senders for Menu (UPDATED to Space-Delimited Format) ---
def send_test_point_y(axis=1):
    axis_str, cmd = ("Y-Axis 1", "POINT") if axis == 1 else ("Y-Axis 2", "POINT2")
    print(f"\n--- Send Test Point to {axis_str} ---")
    y = get_float_input("Enter Y value (e.g., 50.0): ", 50.0)
    style = get_int_input("Enter style (1-10): ", 1)
    full_msg = f"{cmd} {get_current_plot_time_ms()} {style} {y:.6f}"
    send_message_direct(full_msg)

def send_test_line_y(axis=1):
    axis_str, cmd = ("Y-Axis 1", "LINE") if axis == 1 else ("Y-Axis 2", "LINE2")
    print(f"\n--- Send Test Line to {axis_str} ---")
    y = get_float_input("Enter Y value (e.g., 60.0): ", 60.0)
    style = get_int_input("Enter style (1-10): ", 1)
    full_msg = f"{cmd} {get_current_plot_time_ms()} {style} {y:.6f}"
    send_message_direct(full_msg)

def send_test_text_marker():
    print("\n--- Send Test Text Marker ---")
    text = get_string_input("Enter marker text: ", "Default Marker")
    pos = get_string_input("Enter position ('top', 'mid', 'bot'): ", "mid").lower().strip()
    full_msg = f"MARKER {get_current_plot_time_ms()} {pos} \"{text}\""
    send_message_direct(full_msg)

def send_test_key_value():
    print("\n--- Send Test Key-Value ---")
    key = get_string_input("Enter Key: ", "Status")
    value = get_string_input("Enter Value: ", "Idle")
    full_msg = f"KV {get_current_plot_time_ms()} \"{key}\" \"{value}\""
    send_message_direct(full_msg)

def send_test_set_y_limits(axis=1):
    axis_str, cmd = ("Y-Axis 1", "YLIMITS") if axis == 1 else ("Y-Axis 2", "YLIMITS2")
    print(f"\n--- Send Set {axis_str} Limits ---")
    min_y = get_float_input("Enter Min Y: ", 0.0)
    max_y = get_float_input(f"Enter Max Y (> {min_y:.2f}): ", 100.0)
    if max_y <= min_y:
        print("Max Y must be > Min Y. Aborting.")
        return
    full_msg = f"{cmd} {get_current_plot_time_ms()} {min_y:.6f} {max_y:.6f}"
    send_message_direct(full_msg)

def send_test_set_y_units(axis=1):
    axis_str, cmd = ("Y-Axis 1", "YUNITS") if axis == 1 else ("Y-Axis 2", "YUNITS2")
    print(f"\n--- Send Set {axis_str} Units ---")
    units = get_string_input("Enter Y units string: ", "Units")
    full_msg = f"{cmd} {get_current_plot_time_ms()} \"{units}\""
    send_message_direct(full_msg)

# --- Continuous Demo Function (UPDATED to Space-Delimited Format) ---
def run_continuous_demo(duration_seconds):
    print(f"\nRunning dual-axis demo for {duration_seconds} seconds...")

    # --- Setup both axes ---
    setup_time = get_current_plot_time_ms()
    send_message_direct(f"YLIMITS {setup_time} 0 6000")
    time.sleep(0.01)
    send_message_direct(f"YUNITS {setup_time + 1} \"RPM\"")
    time.sleep(0.01)
    send_message_direct(f"YLIMITS2 {setup_time + 2} -1.0 1.0")
    time.sleep(0.01)
    send_message_direct(f"YUNITS2 {setup_time + 3} \"Power\"")
    time.sleep(0.01)
    send_message_direct(f"MARKER {setup_time + 10} top \"Dual-Axis Demo Started\"")

    count = 0
    script_start_time_sec = time.time()
    demo_start_time_ms = get_current_plot_time_ms()

    while (time.time() - script_start_time_sec) < duration_seconds:
        current_time = get_current_plot_time_ms()
        elapsed_seconds = (current_time - demo_start_time_ms) / 1000.0

        target_rpm = 3000 + 2000 * math.sin(2 * math.pi * 0.1 * elapsed_seconds)
        actual_rpm = target_rpm + random.uniform(-150, 150)
        send_message_direct(f"LINE {current_time} 1 {target_rpm:.3f}")
        send_message_direct(f"LINE {current_time} 2 {actual_rpm:.3f}")

        motor_power = 0.8 * math.cos(2 * math.pi * 0.1 * elapsed_seconds)
        send_message_direct(f"LINE2 {current_time} 3 {motor_power:.3f}")

        if count % 20 == 0:
            send_message_direct(f"KV {current_time} \"Target RPM\" \"{target_rpm:.0f}\"")
            send_message_direct(f"KV {current_time} \"Power\" \"{motor_power:.2f}\"")

        time.sleep(SEND_INTERVAL_SECONDS)
        count += 1
    print("Continuous demo finished.")

# --- Main Menu Logic ---
def main_menu():
    print("\n--- Plotter Test Menu (Space-Delimited Format) ---")
    while True:
        print("\nChoose an action:")
        print(" 1. Send Line (Y1)")
        print(" 2. Send Point (Y1)")
        print(" 3. Set Y1 Limits")
        print(" 4. Set Y1 Units")
        print("-------------------------")
        print(" 5. Send Line (Y2)")
        print(" 6. Send Point (Y2)")
        print(" 7. Set Y2 Limits")
        print(" 8. Set Y2 Units")
        print("-------------------------")
        print(" 9. Send Text Marker")
        print("10. Send Key-Value")
        print("-------------------------")
        print("11. Run Dual-Axis Demo (20 seconds)")
        print(" 0. Exit")

        choice = input("Enter choice: ")

        if choice == '1': send_test_line_y(axis=1)
        elif choice == '2': send_test_point_y(axis=1)
        elif choice == '3': send_test_set_y_limits(axis=1)
        elif choice == '4': send_test_set_y_units(axis=1)
        elif choice == '5': send_test_line_y(axis=2)
        elif choice == '6': send_test_point_y(axis=2)
        elif choice == '7': send_test_set_y_limits(axis=2)
        elif choice == '8': send_test_set_y_units(axis=2)
        elif choice == '9': send_test_text_marker()
        elif choice == '10': send_test_key_value()
        elif choice == '11': run_continuous_demo(20)
        elif choice == '0': print("Exiting."); break
        else: print("Invalid choice.")
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        main_menu()
    finally:
        print("Closing UDP socket.")
        sock.close()
