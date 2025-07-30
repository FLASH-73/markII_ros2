# diagnose_servos.py
import sts_driver
import time

# --- USER CONFIGURATION ---
# Ensure this matches the 'serial_port' parameter in your launch file
SERIAL_PORT = '/dev/ttyUSB0' 
# Ensure this matches the 'baudrate' parameter in your launch file
BAUDRATE = 1000000

#==============================================================================
#   DIAGNOSTIC FUNCTIONS
#==============================================================================

def check_connectivity():
    """
    Pings specific servo IDs to see if they are responsive.
    This uses the ping() method from sts_driver.py.
    """
    try:
        driver = sts_driver.ServoController(SERIAL_PORT, BAUDRATE)
        print(f"Driver initialized successfully on {SERIAL_PORT}.")
    except Exception as e:
        print(f"Error: Failed to initialize driver: {e}")
        print("Please check that the SERIAL_PORT is correct and no other program (like ROS) is using it.")
        return

    # IDs for the problematic joint
    servo_ids_to_check = [4, 5] 
    
    print("\n--- Pinging Servos [4, 5] ---")
    all_responsive = True
    for servo_id in servo_ids_to_check:
        print(f"Pinging servo ID {servo_id}...")
        if driver.ping(servo_id):
            print(f"  SUCCESS: Servo {servo_id} is responding.")
        else:
            print(f"  FAILURE: Servo {servo_id} did NOT respond.")
            all_responsive = False
    
    if all_responsive:
        print("\nResult: Both servos are connected and responding.")
    else:
        print("\nResult: One or more servos failed to respond. Check wiring and power.")
            
    driver.close()
    print("---------------------------------")


def test_individual_movement():
    """
    Enables torque, reads the initial position, moves the servo, and
    safely returns it to the starting position.
    """
    driver = None # Initialize driver to None
    try:
        driver = sts_driver.ServoController(SERIAL_PORT, BAUDRATE)
        print(f"Driver initialized successfully on {SERIAL_PORT}.")
    except Exception as e:
        print(f"Error: Failed to initialize driver: {e}")
        print("Please check that the SERIAL_PORT is correct and no other program (like ROS) is using it.")
        return

    servo_ids_to_test = [4, 5]
    
    print("\n--- Testing Individual Movement (with return to origin) ---")
    for servo_id in servo_ids_to_test:
        initial_position = None
        
        try:
            print(f"\n>>> Testing servo ID {servo_id}:")
            
            # 1. READ INITIAL STATE
            # The get_position() function is defined in sts_driver.py
            initial_position = driver.get_position(servo_id)
            if initial_position is None:
                print(f"  Error: Could not read initial position for servo {servo_id}. Skipping.")
                continue
            print(f"  Initial position recorded: {initial_position}")

            # 2. RUN TEST MOVEMENTS
            print("  Enabling torque...")
            driver.set_torque_enable(servo_id, True)
            time.sleep(0.1)
            
            print("  Moving to position 2048 (center)...")
            driver.set_position(servo_id, 2048)
            time.sleep(2)

            print("  Moving to position 1024...")
            driver.set_position(servo_id, 1024)
            time.sleep(2)

        finally:
            # 3. RETURN TO ORIGIN AND CLEAN UP
            # This block runs even if the script is interrupted
            if initial_position is not None:
                print(f"  Returning servo {servo_id} to initial position {initial_position}...")
                driver.set_position(servo_id, initial_position)
                time.sleep(2) # Allow time for the return movement

            print(f"  Disabling torque for servo {servo_id}.")
            driver.set_torque_enable(servo_id, False)

    if driver:
        driver.close()
    print("\nMovement test complete.")
    print("---------------------------------")


def find_all_servos():
    """
    Scans the bus to find all connected servo IDs.
    This uses the fast_scan_servos() method from sts_driver.py.
    """
    try:
        driver = sts_driver.ServoController(SERIAL_PORT, BAUDRATE)
        print(f"Driver initialized successfully on {SERIAL_PORT}.")
    except Exception as e:
        print(f"Error: Failed to initialize driver: {e}")
        print("Please check that the SERIAL_PORT is correct and no other program (like ROS) is using it.")
        return

    print("\n--- Scanning for all servos on the bus (IDs 1-20) ---")
    found_servos = driver.fast_scan_servos(start_id=1, end_id=20)
    
    if found_servos:
        print(f"\nResult: Found servos with IDs: {found_servos}")
    else:
        print("\nResult: No servos found on the bus.")
        
    driver.close()
    print("---------------------------------")


#==============================================================================
#   MAIN EXECUTION
#==============================================================================

if __name__ == '__main__':
    while True:
        print("\n===== Servo Diagnostics Menu =====")
        print("1: Check Connectivity (Ping Servos 4 & 5)")
        print("2: Test Individual Movement (Servos 4 & 5)")
        print("3: Find All Servos (Scan Bus)")
        print("q: Quit")
        
        choice = input("Enter your choice (1, 2, 3, or q): ")
        
        if choice == '1':
            check_connectivity()
        elif choice == '2':
            test_individual_movement()
        elif choice == '3':
            find_all_servos()
        elif choice.lower() == 'q':
            print("Exiting.")
            break
        else:
            print("Invalid choice, please try again.")