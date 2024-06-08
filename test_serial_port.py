import serial

# Serial port configuration
serial_port = '/dev/ttyS0'  # Adjust as needed for your system
baud_rate = 9600            # Adjust the baud rate as needed
timeout = 1                 # Timeout in seconds for read operations

try:
    # Open the serial port
    with serial.Serial(serial_port, baud_rate, timeout=timeout) as ser:
        print(f"Listening on {serial_port} at {baud_rate} baud rate.")

        while True:
            if ser.in_waiting > 0:  # Check if there is data waiting to be read
                data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                if data:
                    print(f"Received: {data.strip()}")
            # Optionally, you could add a small sleep here to reduce CPU usage
except serial.SerialException as e:
    print(f"Serial error: {e}")
except Exception as e:
    print(f"An error occurred: {e}")
