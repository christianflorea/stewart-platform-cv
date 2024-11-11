from smbus2 import SMBus

# Set bus address to match the Arduino's I2C address
addr = 0x8  # Arduino I2C address
bus = SMBus(1)  # Use /dev/i2c-1

print("Control Servos:")
print("Type 'servo_number direction' (e.g., '1 up' or '2 down').")
print("To stop the program, type 'exit'.")

while True:
    user_input = input("Enter command: ").strip().lower()
    if user_input == "exit":
        break
    try:
        servo_str, direction = user_input.split()
        servo_number = int(servo_str)
        if servo_number not in [1, 2, 3]:
            print("Invalid servo number. Please enter 1, 2, or 3.")
            continue
        if direction not in ["up", "down"]:
            print("Invalid direction. Please enter 'up' or 'down'.")
            continue
        # Prepare data to send
        # Send two bytes: servo number and direction
        servo_byte = servo_number  # 1, 2, or 3
        direction_byte = 0x1 if direction == "up" else 0x0  # 1 for up, 0 for down
        # Send data to Arduino
        bus.write_i2c_block_data(addr, servo_byte, [direction_byte])
        print(f"Command sent: Move Servo {servo_number} {direction}.")
    except ValueError:
        print("Invalid input format. Please enter 'servo_number direction'.")
    except Exception as e:
        print(f"An error occurred: {e}")
