import socket
import time
import random

# --- Configuration ---
UDP_IP = "127.0.0.1"  # localhost
UDP_PORT = 7777
SOCKET_TIMEOUT = 2.0 # Seconds to wait for a response (if any expected, not in this sender)

# --- Create UDP Socket ---
try:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    print(f"UDP socket created. Target: {UDP_IP}:{UDP_PORT}")
except socket.error as e:
    print(f"Error creating socket: {e}")
    exit()

def send_message(message):
    """Sends a message string to the configured UDP target."""
    try:
        sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))
        print(f"Sent: '{message}'")
    except socket.error as e:
        print(f"Error sending message '{message}': {e}")

def test_position():
    """Sends a sample position message."""
    x = round(random.uniform(-70, 70), 2)
    y = round(random.uniform(-70, 70), 2)
    heading = round(random.uniform(0, 359.9), 1)
    message = f"pos:{x},{y},{heading}"
    send_message(message)

def test_circle():
    """Sends a sample circle message with radius and heading."""
    radius = round(random.uniform(5, 30), 1)
    heading = round(random.uniform(0, 359.9), 1)
    message = f"cir:{radius},{heading}"
    send_message(message)

_line_names_counter = {}
def test_line():
    """Sends a sample named, styled line message."""
    base_name = "random_line"
    _line_names_counter[base_name] = _line_names_counter.get(base_name, 0) + 1
    name = "dynamic_test_line"

    x0 = round(random.uniform(-65, 65), 2)
    y0 = round(random.uniform(-65, 65), 2)
    x1 = round(random.uniform(-65, 65), 2)
    y1 = round(random.uniform(-65, 65), 2)
    style = random.randint(1, 3)
    message = f"line:{name},{x0},{y0},{x1},{y1},{style}"
    send_message(message)

def test_text():
    """Sends a sample text message."""
    message = f"txt:Hello from Robot! Val={random.randint(0,100)}"
    send_message(message)

def test_kv():
    """Sends a sample key-value message."""
    keys = ["Robot Status", "Target Lock", "Alliance", "Battery Voltage"]
    values = {
        "Robot Status": ["Idle", "Scanning", "Moving", "Executing"],
        "Target Lock": ["True", "False"],
        "Alliance": ["BLUE", "RED"],
        "Battery Voltage": [f"{round(random.uniform(11.8, 13.5), 2)}V"]
    }
    random_key = random.choice(keys)
    random_value = random.choice(values[random_key])
    message = f"kv:{random_key},{random_value}"
    send_message(message)

def run_sequence():
    """Runs a predefined sequence of commands with named, styled lines."""
    print("\n--- Running Predefined Sequence ---")
    send_message("kv:Sequence,Starting")
    send_message("kv:Alliance,BLUE")
    send_message("pos:0,0,0")
    time.sleep(0.5)
    send_message("line:target_vec_1,-30,-30,30,30,3")
    time.sleep(1)
    send_message("cir:20.0,0")
    time.sleep(0.5)
    send_message("line:target_vec_1,30,-30,-30,30,2")
    time.sleep(1)
    send_message("txt:Sequence Started")
    send_message("kv:Sub-task,Moving to Waypoint 1")
    time.sleep(1)
    send_message("pos:10,15,45")
    send_message("line:robot_to_poi,10,15,50,50,3")
    time.sleep(1)
    send_message("cir:10.5,90")
    time.sleep(1)
    send_message("line:perm_line_A,-50,-50,50,50,1")
    time.sleep(0.5)
    send_message("line:perm_line_B,50,-50,-50,50,1")
    send_message("kv:Sub-task,Moving to Waypoint 2")
    time.sleep(1)
    send_message("pos:-20,30,180")
    send_message("line:robot_to_poi,-20,30,-60,0,3")
    time.sleep(0.5)
    send_message("cir:15.0,270")
    time.sleep(0.5)
    send_message("txt:Sequence Complete!")
    send_message("kv:Sequence,Finished")
    send_message("kv:Sub-task,Idle")
    print("--- Sequence Complete ---")

if __name__ == "__main__":
    print("UDP Test Sender for FtcFieldSimulator")
    print("Commands:")
    print("  p                      - Send a random 'pos' message")
    print("  c                      - Send a random 'cir' message")
    print("  l                      - Send a random named, styled 'line' message")
    print("  t                      - Send a random 'txt' message")
    print("  k                      - Send a random 'kv' (key-value) message")
    print("  s                      - Run a predefined sequence of commands")
    print("  pos:x,y,h              - Send specific position (e.g., pos:10.5,20.0,45)")
    print("  cir:rad,head           - Send specific circle (e.g., cir:15.5,45.0)")
    print("  line:name,x0,y0,x1,y1,style - Send specific named line (e.g., line:myline,0,0,30,30,1)")
    print("                           (style: 1=SolidThick, 2=SolidThin, 3=Dotted)")
    print("  txt:your message       - Send specific text (e.g., txt:Target Acquired)")
    print("  kv:key,value           - Send specific key-value (e.g., kv:Mode,AUTO)")
    print("  q or quit              - Exit")
    print("-" * 30)

    try:
        while True:
            command_input = input("Enter command: ").strip()
            command_lower = command_input.lower()

            if command_lower == 'q' or command_lower == 'quit':
                break
            elif command_lower == 'p':
                test_position()
            elif command_lower == 'c':
                test_circle()
            elif command_lower == 'l':
                test_line()
            elif command_lower == 't':
                test_text()
            elif command_lower == 'k':
                test_kv()
            elif command_lower == 's':
                run_sequence()
            elif command_lower.startswith(("pos:", "cir:", "line:", "txt:", "kv:")): # Simplified check

                valid_format = True
                # Safely split command_input to get the prefix and content
                try:
                    prefix_part, content_part = command_input.split(':', 1)
                    prefix = prefix_part.lower() + ":" # Ensure prefix is lowercase and ends with ':'
                except ValueError: # Happens if there's no ':' in command_input
                    print(f"Invalid command format: '{command_input}'. Missing ':' delimiter.")
                    valid_format = False
                    prefix = "" # Initialize prefix to avoid further errors
                    content_part = ""


                if valid_format: # Proceed only if prefix and content_part could be reasonably extracted
                    if prefix == "cir:":
                        try:
                            params = content_part.split(',')
                            if len(params) != 2:
                                print("Invalid 'cir' format. Expected: cir:radius,heading")
                                valid_format = False
                            else:
                                float(params[0].strip())
                                float(params[1].strip())
                        except (ValueError, IndexError):
                            print("Invalid numeric value or format in 'cir' command.")
                            valid_format = False
                    elif prefix == "line:":
                        try:
                            params = content_part.split(',', 5)
                            if len(params) != 6:
                                print("Invalid 'line' format. Expected: line:name,x0,y0,x1,y1,styleCode")
                                valid_format = False
                            else:
                                name_part = params[0].strip()
                                if not name_part:
                                    print("Invalid 'line' format: Name cannot be empty.")
                                    valid_format = False
                                if ',' in name_part:
                                    print("Warning: Line name contains a comma. This might break parsing on the receiver.")
                                    # Consider auto-replacing or rejecting
                                float(params[1].strip())
                                float(params[2].strip())
                                float(params[3].strip())
                                float(params[4].strip())
                                style_code = int(params[5].strip())
                                if not (1 <= style_code <= 3):
                                    print(f"Warning: styleCode {style_code} might be out of expected range (1-3).")
                        except (ValueError, IndexError):
                            print("Invalid value or format in 'line' command (name, 4 floats, 1 int expected).")
                            valid_format = False
                    elif prefix == "pos:":
                        try:
                            params = content_part.split(',')
                            if len(params) != 3:
                                print("Invalid 'pos' format. Expected: pos:x,y,heading")
                                valid_format = False
                            else:
                                for p_val in params: float(p_val.strip())
                        except (ValueError, IndexError):
                            print("Invalid numeric value or format in 'pos' command.")
                            valid_format = False
                    elif prefix == "txt:":
                        if not content_part.strip(): # Check if content after "txt:" is empty or just whitespace
                            print("Invalid 'txt' format: Message content is missing.")
                            valid_format = False
                    elif prefix == "kv:":
                        try:
                            params = content_part.split(',', 1)
                            if len(params) != 2:
                                print("Invalid 'kv' format. Expected: kv:key,value")
                                valid_format = False
                            elif not params[0].strip():
                                print("Invalid 'kv' format: Key cannot be empty.")
                                valid_format = False
                        except IndexError:
                             print("Invalid 'kv' format. Expected: kv:key,value")
                             valid_format = False

                if valid_format:
                    send_message(command_input)
                else:
                    # The error message would have already been printed by the specific validation block
                    continue

            elif not command_input:
                continue
            else:
                print(f"Unknown command: '{command_input}'. Please try again.")

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        print("Closing socket.")
        sock.close()
