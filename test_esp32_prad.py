import serial
import time
import re

def test_prad_polling():
    port = '/dev/tty.usbserial-210'
    
    print("Connecting to ESP32-S3 Controller...")
    try:
        s = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(1) # wait for ESP32 reboot
        s.reset_input_buffer()
        print("Connected.")
        
        print("\n--- 请随意用手转动 0 号舵机（底座），观察读数是否变化 ---")
        print("注意：如果当前舵机有很大阻力转不动，请先关闭电源，手动转到一个角度，再开电源运行脚本查看读数跳变。\n")
        
        pattern = re.compile(r'#000P(\d+)!')
        
        try:
            for i in range(50):
                cmd = '#000PRAD!\n'
                s.write(cmd.encode())
                s.flush() # Ensure the command goes out
                time.sleep(0.15)
                
                resp = s.read(100).decode(errors='ignore')
                
                # The response will look like "#000PRAD!#000P1500!"
                # Use regex to extract the P value
                match = pattern.search(resp)
                if match:
                    pos = int(match.group(1))
                    print(f"[{i+1}/50] 成功读取角度数值: {pos}")
                else:
                    print(f"[{i+1}/50] 读取失败，原始返回: {repr(resp)}")
                    
                time.sleep(0.4)
        except KeyboardInterrupt:
            print("\n测试提前终止")
            
        s.close()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    test_prad_polling()
