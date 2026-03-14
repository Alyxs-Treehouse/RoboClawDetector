import serial
import time

def probe_firmware():
    port = '/dev/tty.usbserial-210'
    commands = [
        '#000PRAD!\n',
        '#000P?\n',
        '#000P\n',
        '?000\n',
        '#000P0000T0000!\n',
        '0\n',
        '?\n',
        'help\n'
    ]
    
    print("Connecting to ESP32-S3 Controller...")
    try:
        s = serial.Serial(port, 115200, timeout=0.5)
        time.sleep(1) # wait for ESP32 reboot if DTR is triggered
        s.reset_input_buffer()
        print("Connected. Probing for read commands...")
        
        for cmd in commands:
            print(f"Sending: {cmd.strip()}")
            s.write(cmd.encode())
            time.sleep(0.1)
            response = s.read(100)
            if response:
                print(f"  Got Response: {response.decode(errors='ignore')}")
            else:
                print("  No Response")
                
        s.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    probe_firmware()
