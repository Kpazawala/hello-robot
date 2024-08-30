import serial
import time

BAUDRATE = 1000000
ser = None
 
ser_port = None
def open_serial_port():
    ports = ['/dev/ttyUSB0', '/dev/ttyUSB1']
    ser_port = []
    for port in ports:
        try:
            ser_port.append(serial.Serial(port, BAUDRATE, timeout=1))
            print(f"Successfully opened port: {port}")
            
        except serial.SerialException as e:
            print(f"Failed to open port {port}: {e}")
    if ser_port:
        return ser_port
    raise RuntimeError("Failed to open any serial port.")
    
try:
    ser1, ser2 = open_serial_port()[:2]
    time.sleep(2)
except RuntimeError as e:
    print(f"Error: {e}")

try:
    while True:
        ser1.write("Message From Serial 1".encode('utf-8'))
        ser2.write("Message From Serial 2".encode('utf-8'))        
        response1 = ser1.readline().decode('utf-8', errors='replace').strip()        
        response2 = ser2.readline().decode('utf-8', errors='replace').strip()
        
        print("Serial 1 Message Recieved:", response1)
        print("Serial 2 Message Recieved:", response2)
        
        # Wait for a while before sending the next command.

except KeyboardInterrupt:
    print("Exiting program.")

# Close the serial connection when done.
ser1.close()
ser2.close()