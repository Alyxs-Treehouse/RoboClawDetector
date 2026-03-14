import time
from robot_ik import RobotIK
from servo_controller import ServoController

def main():
    print("Initializing IK and Servo Controller...")
    ik = RobotIK("robot_template.urdf")
    servo = ServoController('/dev/tty.usbserial-210', 115200)

    if not servo.connect():
        print("Failed to access servo control board.")
        return

    # A known reachable coordinate sequence
    targets = [
        [0.15, 0.0, 0.25],
        [0.15, -0.1, 0.20],
        [0.15, 0.1, 0.20],
        [0.10, 0.0, 0.35]
    ]

    for target in targets:
        print(f"\\nCalculating IK for {target}")
        solution = ik.solve_ik_multiple_attempts(target, num_attempts=5)
        if solution:
            print("IK Solution found. Moving...")
            servo.move_all_joints(solution, duration_ms=1500)
            time.sleep(2)
        else:
            print("IK failed for this target.")

    print("Demo complete.")
    servo.disconnect()

if __name__ == "__main__":
    main()
