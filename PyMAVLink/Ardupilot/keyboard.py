#!/usr/bin/env python3
import time
from pymavlink import mavutil
from pynput import keyboard
from argparse import ArgumentParser

def parse_args():
    parser = ArgumentParser(description="鍵盤控制無人機（WASD 前後左右，QE 偏航，UJ 高度，A 解鎖，T 起飛，L 降落，M 退出，含電池和 GPS 安全檢查）")
    parser.add_argument("--device", default="tcp:127.0.0.1:5760", help="MAVLink 連線地址（預設：SITL TCP）")
    parser.add_argument("--baudrate", type=int, default=115200, help="連線速率（預設：115200）")
    parser.add_argument("--source-system", type=int, default=255, help="MAVLink 來源系統 ID（預設：255）")
    return parser.parse_args()

def wait_heartbeat(m):
    print("等待心跳訊息...")
    msg = m.recv_match(type='HEARTBEAT', blocking=True)
    print(f"收到心跳訊息：系統 {m.target_system}，組件 {m.target_component}")
    return True

def set_mode(m, mode):
    mode_mapping = m.mode_mapping()
    if mode not in mode_mapping:
        print(f"未知模式：{mode}")
        return False
    mode_id = mode_mapping[mode]
    m.mav.set_mode_send(
        m.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    start_time = time.time()
    while time.time() - start_time < 5:
        msg = m.recv_match(type='HEARTBEAT', blocking=True)
        if msg.custom_mode == mode_id:
            print(f"模式已切換至 {mode}")
            return True
        time.sleep(0.1)
    print(f"模式切換至 {mode} 失敗")
    return False

def arm_vehicle(m):
    print("正在解鎖...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    start_time = time.time()
    while time.time() - start_time < 5:
        msg = m.recv_match(type='HEARTBEAT', blocking=True)
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("無人機已解鎖！")
            return True
        time.sleep(0.1)
    print("解鎖失敗")
    return False

def check_safety(m):
    print("正在檢查電池和 GPS 狀態...")
    start_time = time.time()
    battery_ok = False
    gps_ok = False

    while time.time() - start_time < 10:
        msg = m.recv_match(type=['BATTERY_STATUS', 'GPS_RAW_INT'], blocking=True)
        if msg.get_type() == 'BATTERY_STATUS':
            if msg.battery_remaining >= 20:
                print(f"電池電量：{msg.battery_remaining}%，符合要求")
                battery_ok = True
            else:
                print(f"電池電量：{msg.battery_remaining}%，低於 20%，無法起飛")
        elif msg.get_type() == 'GPS_RAW_INT':
            if msg.fix_type >= 3 and msg.satellites_visible >= 6:
                print(f"GPS 固定模式：{msg.fix_type}，衛星數：{msg.satellites_visible}，符合要求")
                gps_ok = True
            else:
                print(f"GPS 固定模式：{msg.fix_type}，衛星數：{msg.satellites_visible}，不符合要求")
        
        if battery_ok and gps_ok:
            print("安全檢查通過！")
            return True
        time.sleep(0.1)
    
    print("安全檢查未通過，無法起飛")
    return False

def takeoff(m, altitude):
    if not check_safety(m):
        return False
    print(f"正在起飛到 {altitude} 公尺...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude)
    start_time = time.time()
    while time.time() - start_time < 10:
        msg = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_alt = msg.relative_alt / 1000.0
        if current_alt >= altitude * 0.95:
            print("起飛完成！")
            return True
        time.sleep(0.1)
    printOJ("起飛失敗")
    return False

def land(m):
    print("正在降落...")
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)
    start_time = time.time()
    while time.time() - start_time < 30:
        msg = m.recv_match(type='HEARTBEAT', blocking=True)
        if msg.system_status == mavutil.mavlink.MAV_STATE_STANDBY:
            print("降落完成！")
            return True
        time.sleep(0.1)
    print("降落失敗")
    return False

def send_velocity(m, vx, vy, vz, yaw_rate):
    m.mav.set_position_target_local_ned_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111001000,  # 僅控制速度和偏航角速度
        0, 0, 0,  # 位置無效
        vx, vy, vz,  # 速度（機體坐標系）
        0, 0, 0,  # 加速度無效
        0, yaw_rate)  # 偏航角速度

def main():
    args = parse_args()
    print(f"連線到 {args.device}，速率 {args.baudrate}")
    master = mavutil.mavlink_connection(args.device, baud=args.baudrate, source_system=args.SOURCE_SYSTEM)
    wait_heartbeat(master)

    print("鍵盤控制無人機：")
    print("  WASD - 前進/後退/右移/左移")
    print("  QE - 偏航左/右")
    print("  UJ - 上升/下降")
    print("  A - 解鎖 (Arm)")
    print("  T - 起飛到 1.5 米 (Takeoff)")
    print("  L - 降落 (Land)")
    print("  M - 退出程式")

    # 鍵盤狀態
    keys = {'w': False, 'a': False, 's': False, 'd': False,
            'q': False, 'e': False, 'u': False, 'j': False}
    arm_pressed = False
    takeoff_pressed = False
    land_pressed = False

    def on_press(key):
        nonlocal arm_pressed, takeoff_pressed, land_pressed
        try:
            k = key.char.lower()
            if k in keys:
                keys[k] = True
            elif k == 'a' and not arm_pressed:
                arm_pressed = True
                arm_vehicle(master)
            elif k == 't' and not takeoff_pressed:
                takeoff_pressed = True
                if set_mode(master, 'GUIDED'):
                    takeoff(master, 1.5)
            elif k == 'l' and not land_pressed:
                land_pressed = True
                if set_mode(master, 'LAND'):
                    land(master)
            elif k == 'm':
                print("退出程式...")
                master.close()
                return False
        except AttributeError:
            pass

    def on_release(key):
        nonlocal arm_pressed, takeoff_pressed, land_pressed
        try:
            k = key.char.lower()
            if k in keys:
                keys[k] = False
            elif k == 'a':
                arm_pressed = False
            elif k == 't':
                takeoff_pressed = False
            elif k == 'l':
                land_pressed = False
        except AttributeError:
            pass

    # 啟動鍵盤監聽
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # 主循環，20Hz 更新
    while listener.running:
        if set_mode(master, 'GUIDED'):
            vx = vy = vz = yaw_rate = 0
            if keys['w']:
                vx = 1.0  # 前進
            if keys['s']:
                vx = -1.0  # 後退
            if keys['d']:
                vy = 1.0  # 右移
            if keys['a']:
                vy = -1.0  # 左移
            if keys['u']:
                vz = -0.5  # 上升
            if keys['j']:
                vz = 0.5  # 下降
            if keys['q']:
                yaw_rate = -0.5  # 左轉
            if keys['e']:
                yaw_rate = 0.5  # 右轉
            send_velocity(master, vx, vy, vz, yaw_rate)
        time.sleep(0.05)  # 20Hz

if __name__ == '__main__':
    main()