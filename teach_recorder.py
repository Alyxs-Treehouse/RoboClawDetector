"""
Teach-by-demonstration recorder.

Uses "follow mode": polls each servo's current position and immediately
re-commands it there, so the servo offers minimal resistance and can be
pushed through the motion by hand.

Usage:
    python teach_recorder.py --serial /dev/ttyUSB0 --action shoot

Steps:
  1. Script enters follow mode — gently push the arm into position
  2. Press Enter to start recording
  3. Move the arm through the desired motion
  4. Press Enter to stop
  5. The recorded trajectory is saved into arm_actions.py as the named action
"""

import serial, time, re, sys, argparse, threading

SERVO_IDS  = [0, 1, 2, 3, 4]
RECORD_HZ  = 10   # samples per second during recording
FOLLOW_HZ  = 20   # follow-mode update rate
MIN_FRAMES = 3


def open_port(port):
    s = serial.Serial()
    s.port = port; s.baudrate = 115200
    s.timeout = 0.15; s.dtr = False; s.rts = False
    s.open()
    print("Waiting for ESP32 boot..."); time.sleep(5)
    s.reset_input_buffer()
    return s


def read_pos(ser, sid):
    ser.reset_input_buffer()
    ser.write(f"#{sid:03d}PRAD!\n".encode()); ser.flush()
    time.sleep(0.05)
    resp = ser.read(50).decode(errors='ignore')
    m = re.search(rf'#{sid:03d}P(\d+)!', resp)
    return int(m.group(1)) if m else None


def read_all(ser):
    return {sid: read_pos(ser, sid) for sid in SERVO_IDS}


def send_raw(ser, sid, pos, dur_ms=30):
    pos = max(500, min(2500, pos))
    ser.write(f"#{sid:03d}P{pos:04d}T{dur_ms:04d}!\n".encode())
    ser.flush()
    time.sleep(0.002)


def follow_mode(ser, stop_event):
    """Continuously read each servo's position and re-send it as target.
    This zeroes out the holding torque so the arm can be moved by hand."""
    interval = 1.0 / FOLLOW_HZ
    while not stop_event.is_set():
        t0 = time.time()
        for sid in SERVO_IDS:
            pos = read_pos(ser, sid)
            if pos is not None:
                send_raw(ser, sid, pos, dur_ms=50)
        elapsed = time.time() - t0
        time.sleep(max(0, interval - elapsed))


def record(ser):
    frames = []
    interval = 1.0 / RECORD_HZ
    print(f"\nRecording at {RECORD_HZ} Hz — move the arm now. Press Enter to stop.")

    stop_flag = [False]
    def wait_enter():
        input()
        stop_flag[0] = True
    threading.Thread(target=wait_enter, daemon=True).start()

    t0 = time.time()
    while not stop_flag[0]:
        t_start = time.time()
        pos = read_all(ser)
        if all(v is not None for v in pos.values()):
            frames.append((round(time.time() - t0, 3), dict(pos)))
            print(f"  t={frames[-1][0]:.1f}s  " +
                  "  ".join(f"s{sid}:{p}" for sid, p in pos.items()), end='\r')
        elapsed = time.time() - t_start
        time.sleep(max(0, interval - elapsed))

    print(f"\nRecorded {len(frames)} frames over {frames[-1][0]:.1f}s")
    return frames


def save_action(action_name, frames):
    path = 'arm_actions.py'
    with open(path, 'r') as f:
        src = f.read()

    keyframes_lines = ['        keyframes = [']
    for i, (t, pos) in enumerate(frames):
        dur_ms = 800 if i == 0 else max(50, int((frames[i][0] - frames[i-1][0]) * 1000))
        wait_sec = round(dur_ms / 1000.0 + 0.02, 3)
        pos_str = ', '.join(f'{sid}: {p}' for sid, p in pos.items())
        keyframes_lines.append(f'            ({{{pos_str}}}, {dur_ms}, {wait_sec}),')
    keyframes_lines.append('        ]')
    keyframes_body = '\n'.join(keyframes_lines)

    method = f'''
    def {action_name}(self):
        """Recorded motion: {action_name}"""
        print("🤖 动作: {action_name}")
{keyframes_body}
        for positions, duration_ms, wait_sec in keyframes:
            for sid, pos in positions.items():
                self._send_raw(sid, pos, duration_ms)
            time.sleep(wait_sec)
        self._run_sequence([(self.ALL_CENTER, 800, 1.0)])
        print("✓ {action_name} 完成")
'''

    existing = re.search(rf'\n    def {action_name}\(self\):', src)
    if existing:
        end = re.search(rf'\n    def (?!{action_name})', src[existing.start() + 1:])
        end_pos = existing.start() + 1 + (end.start() if end else len(src))
        src = src[:existing.start()] + method + src[end_pos:]
    else:
        marker = '\n# ─── CLI 入口'
        src = src.replace(marker, method + marker)

    if f'"{action_name}": arm.{action_name},' not in src:
        src = src.replace(
            '"shoot": arm.shoot,',
            f'"shoot": arm.shoot,\n        "{action_name}": arm.{action_name},'
        )

    with open(path, 'w') as f:
        f.write(src)
    print(f"✓ Saved '{action_name}' to arm_actions.py")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--serial', default='/dev/ttyUSB0')
    parser.add_argument('--action', default='recorded_shoot')
    args = parser.parse_args()

    ser = open_port(args.serial)

    # Enter follow mode — arm can be pushed by hand
    print("\nEntering follow mode (arm will be soft — push it gently)...")
    stop_follow = threading.Event()
    follow_thread = threading.Thread(target=follow_mode, args=(ser, stop_follow), daemon=True)
    follow_thread.start()

    input("Press Enter when ready to start recording...")

    # Stop follow mode, switch to record mode
    stop_follow.set()
    follow_thread.join(timeout=1)
    time.sleep(0.1)
    ser.reset_input_buffer()

    frames = record(ser)

    # Return to center
    print("\nReturning to center...")
    for sid in SERVO_IDS:
        send_raw(ser, sid, 1500, dur_ms=1000)
    time.sleep(1.5)

    ser.close()

    if len(frames) < MIN_FRAMES:
        print("Too few frames recorded, aborting.")
        sys.exit(1)

    save_action(args.action, frames)
    print(f"\nRun it with:  python arm_actions.py --serial {args.serial} {args.action}")


if __name__ == '__main__':
    main()
