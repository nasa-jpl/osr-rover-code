import sys
from os import path
import json

# Add the roboclaw.py file to the path
sys.path.append(path.join(path.expanduser('~'), 'osr_ws/src/osr-rover-code/ROS/osr_control/osr_control'))
from roboclaw import Roboclaw

class RoverDebug:
    def __init__(self):
        self.roboclaw = Roboclaw("/dev/serial1", 115200)
        if not self.roboclaw.Open():
            raise Exception("Could not connect to RoboClaw on /dev/serial1")
        print("Connected to RoboClaw on /dev/serial1")

        self.motors = { 
            128: { "M1": "left_front_drive", "M2": "left_middle_drive" },
            129: { "M1": "left_rear_drive", "M2": "right_rear_drive" },
            130: { "M1": "right_middle_drive", "M2": "right_front_drive" },
            131: { "M1": "left_front_rotate", "M2": "left_rear_rotate" },
            132: { "M1": "right_rear_rotate", "M2": "right_front_rotate" }
        }

        self.motor_functions = [
            "left_front_rotate", "left_front_drive",
            "right_front_rotate", "right_front_drive",
            "left_rear_rotate", "left_rear_drive",
            "right_rear_rotate", "right_rear_drive",
            "left_middle_drive", "right_middle_drive"
        ]

        self.load_motor_map()

    def set_motor_speed(self, address, motor, speed):
        if speed >= 0:
            if motor == 'M1':
                self.roboclaw.ForwardM1(address, speed)
            else:
                self.roboclaw.ForwardM2(address, speed)
        else:
            if motor == 'M1':
                self.roboclaw.BackwardM1(address, abs(speed))
            else:
                self.roboclaw.BackwardM2(address, abs(speed))

    def stop_all_motors(self):
        self.roboclaw.ForwardM1(128, 0)
        self.roboclaw.ForwardM2(128, 0)
        self.roboclaw.ForwardM1(129, 0)
        self.roboclaw.ForwardM2(129, 0)
        self.roboclaw.ForwardM1(130, 0)
        self.roboclaw.ForwardM2(130, 0)

    def debug_motors(self):
        print("Starting motor debug sequence.")
        print("This will activate each motor individually.")
        print("Observe which part of the rover moves for each motor.")

        for address in self.motors:
            for motor in ['M1', 'M2']:
                while True:
                    print(f"\nTesting motor: Address {address}, Motor {motor}")
                    if self.motors[address][motor]:
                        print(f"Currently mapped to: {self.motors[address][motor]}")
                    print("1: Run Forward")
                    print("2: Run Backward")
                    print("3: Map this motor")
                    print("4: Skip to next motor")
                    print("q: Quit")
                    choice = input("Enter your choice: ").lower()

                    if choice == 'q':
                        return
                    elif choice in ['1', '2']:
                        speed = 64 if choice == '1' else -64
                        print(f"Running motor for 2 seconds...")
                        self.set_motor_speed(address, motor, speed)
                        import time
                        time.sleep(2)
                        self.set_motor_speed(address, motor, 0)
                    elif choice == '3':
                        self.map_motor(address, motor)
                    elif choice == '4':
                        break
                    else:
                        print("Invalid choice. Please try again.")

        print("Motor debug sequence completed.")
        self.save_motor_map()

    def map_motor(self, address, motor):
        print("\nAvailable motor functions:")
        for i, function in enumerate(self.motor_functions, 1):
            print(f"{i}: {function}")
        
        while True:
            choice = input("Enter the number of the motor function: ")
            try:
                index = int(choice) - 1
                if 0 <= index < len(self.motor_functions):
                    function = self.motor_functions[index]
                    self.motors[address][motor] = function
                    print(f"Mapped Address {address}, Motor {motor} to {function}")
                    break
                else:
                    print("Invalid choice. Please try again.")
            except ValueError:
                print("Please enter a number.")

    def load_motor_map(self):
        try:
            with open('motor_map.json', 'r') as f:
                self.motors = json.load(f)
            print("Loaded existing motor map.")
        except FileNotFoundError:
            print("No existing motor map found. Starting fresh.")

    def save_motor_map(self):
        with open('motor_map.json', 'w') as f:
            json.dump(self.motors, f, indent=2)
        print("Motor map saved.")

    def display_motor_map(self):
        print("\nCurrent Motor Mapping:")
        for address, motors in self.motors.items():
            for motor, function in motors.items():
                if function:
                    print(f"Address {address}, Motor {motor}: {function}")
                else:
                    print(f"Address {address}, Motor {motor}: Not mapped")

    def read_encoder(self, address, motor):
        if motor == 'M1':
            return self.roboclaw.ReadEncM1(address)
        else:
            return self.roboclaw.ReadEncM2(address)

    def debug_encoders(self):
        print("Starting encoder debug sequence.")
        print("This will display encoder values for each motor.")
        print("Move the rover's wheels manually to see changes.")
        print("Press Enter to update values, or type 'q' to quit.")

        while True:
            for address in self.motors:
                for motor in ['M1', 'M2']:
                    enc_value = self.read_encoder(address, motor)
                    function = self.motors[address][motor] or "Not mapped"
                    print(f"Address {address}, Motor {motor} ({function}): {enc_value[1]}")
            
            choice = input("\nPress Enter to update, or 'q' to quit: ").lower()
            if choice == 'q':
                break

        print("Encoder debug sequence completed.")

if __name__ == "__main__":
    try:
        rover_debug = RoverDebug()
        
        while True:
            print("\nRover Run Menu:")
            print("1: All Motors Forward")
            print("s: Stop All Motors")
            print("q: Quit")
            
            choice = input("Enter your choice: ").lower()
            
            if choice == '1':
                rover_debug.roboclaw.ForwardM1(128, 30)
                rover_debug.roboclaw.BackwardM2(128, 30)
                rover_debug.roboclaw.ForwardM1(129, 30)
                rover_debug.roboclaw.ForwardM2(129, 30)
                rover_debug.roboclaw.BackwardM1(130, 30)
                rover_debug.roboclaw.ForwardM2(130, 30)
            if choice == 's':
                rover_debug.stop_all_motors()
            elif choice == 'q':
                break
            else:
                print("Invalid choice. Please try again.")

        print("Exiting Rover Debug.")

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if 'rover_debug' in locals():
            rover_debug.stop_all_motors()