import serial
import serial.tools.list_ports
import time

def list_serial_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[ERROR] No serial ports found.")
        return []
    print("[INFO] Available serial ports:")
    for i, port in enumerate(ports):
        print(f"  [{i}] {port.device} - {port.description}")
    return ports

def choose_serial_port():
    ports = list_serial_ports()
    if not ports:
        return None, None, None

    while True:
        try:
            index = int(input("Enter port number: "))
            if 0 <= index < len(ports):
                port = ports[index].device
                break
            else:
                print("[WARN] Invalid index. Try again.")
        except ValueError:
            print("[WARN] Please enter a number.")
    
    while True:
        try:
            baudrate = int(input("Enter baud rate (e.g., 9600): "))
            break
        except ValueError:
            print("[WARN] Invalid baud rate. Try again.")
    
    while True:
        try:
            bytes_per_line = int(input("Enter number of bytes per line (e.g., 8): "))
            if bytes_per_line > 0:
                break
            else:
                print("[WARN] Must be greater than 0.")
        except ValueError:
            print("[WARN] Invalid number. Try again.")

    return port, baudrate, bytes_per_line

def open_serial(port, baudrate, timeout=1):
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"[INFO] Connected to {port} at {baudrate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"[ERROR] Could not open serial port {port}: {e}")
        return None

def read_serial_hex(ser, bytes_per_line):
    buffer = bytearray()
    try:
        while True:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)
                while len(buffer) >= bytes_per_line:
                    chunk = buffer[:bytes_per_line]
                    buffer = buffer[bytes_per_line:]
                    hex_output = ' '.join(f'{b:02X}' for b in chunk)
                    print(hex_output)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        ser.close()
        print("[INFO] Serial port closed.")

if __name__ == "__main__":
    port, baudrate, bytes_per_line = choose_serial_port()
    if port:
        ser = open_serial(port, baudrate)
        if ser:
            read_serial_hex(ser, bytes_per_line)
