import serial
import serial.tools.list_ports
import threading
import time

# Flag to control read thread
keep_reading = True

def list_com_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def read_from_serial(ser):
    global keep_reading
    while keep_reading:
        try:
            if ser.in_waiting:
                data = ser.readline().decode(errors='replace').strip()
                if data:
                    print(f"\r[ESP] {data}\n> ", end="", flush=True)
            else:
                time.sleep(0.05)
        except (serial.SerialException, OSError):
            print("\n[!] Serial disconnected.")
            keep_reading = False
            break

def main():
    global keep_reading
    ports = list_com_ports()
    if not ports:
        print("No COM ports found.")
        return

    print("Available COM ports:")
    for i, port in enumerate(ports):
        print(f"{i + 1}. {port}")

    # Choose port
    while True:
        try:
            choice = int(input("Select COM port (number): ")) - 1
            if 0 <= choice < len(ports):
                port_name = ports[choice]
                break
            else:
                print("Invalid selection.")
        except ValueError:
            print("Enter a valid number.")

    # Choose baud rate
    baud = input("Enter baud rate (default 9600): ").strip()
    baud = int(baud) if baud else 9600

    print(f"\nConnecting to {port_name} at {baud} baud...")

    try:
        ser = serial.Serial(
            port=port_name,
            baudrate=baud,
            timeout=1,
            write_timeout=1
        )

        # Clear any garbage left in buffer
        ser.reset_input_buffer()
        ser.reset_output_buffer()

    except serial.SerialException as e:
        print(f"[ERROR] Cannot open serial port: {e}")
        return

    print("\nConnected! Type your commands below.")
    print("Example: AT+CIFSR\nPress Ctrl+C to exit.\n")

    # Start reading thread
    keep_reading = True
    reader_thread = threading.Thread(target=read_from_serial, args=(ser,), daemon=True)
    reader_thread.start()

    try:
        while keep_reading:
            user_input = input("> ").strip()
            if user_input:
                # Send command safely
                try:
                    ser.write((user_input + '\r\n').encode())
                    time.sleep(0.1)  # Give ESP time to process before reading more
                except serial.SerialTimeoutException:
                    print("[!] Write timeout. ESP may be unresponsive.")
    except KeyboardInterrupt:
        print("\n[INFO] Exiting...")
    finally:
        keep_reading = False
        ser.close()

if __name__ == "__main__":
    main()
