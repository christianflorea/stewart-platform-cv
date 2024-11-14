# FOR I2C TESTING

from smbus2 import SMBus

addr = 0x8  
bus = SMBus(1)  

print("Control Servos:")
print("Type 'servo_number direction' (e.g., '1 up' or '2 down').")
print("To stop the program, type 'exit'.")

while True:
    user_input = input("Enter command: ").strip().lower()
    if user_input == "exit":
        break
    servo_str, direction = user_input.split()
    servo_number = int(servo_str)
    if servo_number not in [1, 2, 3]:
        print("Invalid servo number. Please enter 1, 2, or 3.")
        continue
    if direction not in ["up", "down"]:
        print("Invalid direction. Please enter 'up' or 'down'.")
        continue
    
    servo_byte = servo_number  
    direction_byte = 0x1 if direction == "up" else 0x0  
    
    bus.write_i2c_block_data(addr, servo_byte, [direction_byte])
    print(f"Command sent: Move Servo {servo_number} {direction}.")
