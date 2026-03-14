import serial
import time

port = '/dev/tty.usbserial-210'
baudrate = 115200

try:
    s = serial.Serial(port, baudrate, timeout=0.5)
    
    # Try different query commands
    queries = [
        b'QP0\r\n',
        b'QP 0\r\n',
        b'PR0\r\n',
        b'PR 0\r\n',
        b'#000 PR\r\n',
        b'#0PR\r\n',
        b'P0\r\n',
        b'#000 P\r\n',
        b'#0 P\r\n'
    ]
    
    for q in queries:
        s.reset_input_buffer()
        print(f"Sending: {repr(q)}")
        s.write(q)
        time.sleep(0.3)
        resp = s.read_all()
        if resp:
            print(f"Response: {repr(resp)}")
        else:
            print("No response")
            
    s.close()
except Exception as e:
    print(f"Error: {e}")
