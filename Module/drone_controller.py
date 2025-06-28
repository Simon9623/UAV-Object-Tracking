# ------------------
#  處理所有無人機通信和控制邏輯
# ------------------

import time
import math
import logging
import threading
from pymavlink import mavutil

logger = logging.getLogger(__name__)

class DroneController:
    """處理所有與無人機的 MAVLink 通訊與控制。"""
    def __init__(self, config):
        self.config = config
        self.master = None
        self.is_connected = False
        self.has_taken_off = False
        
        # 緩存的狀態資訊
        self.cached_altitude = 0.0
        self.cached_attitude = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.last_update_time = 0
        
        self._update_thread = None
        self._stop_thread = threading.Event()

    def connect(self):
        """連接到無人機 (智慧判斷連線類型)。"""
        address = self.config.get('drone.address')
        baud = self.config.get('drone.baud')
        logger.info(f"正在嘗試連接到無人機: {address}")
        try:
            # 判斷連線位址是否為網路類型 (UDP 或 TCP)
            if address.startswith('udp:') or address.startswith('tcp:'):
                # 網路連線，不傳遞 baud 參數
                self.master = mavutil.mavlink_connection(address)
            else:
                # 序列埠連線，傳遞 baud 參數
                self.master = mavutil.mavlink_connection(address, baud=baud)
            # --- 修改區塊結束 ---

            if self._wait_for_heartbeat():
                self.is_connected = True
                self._start_update_thread()
                logger.info("無人機連接成功，並已啟動狀態更新線程。")
                return True
            else:
                logger.error("連接無人機失敗：未收到心跳包。")
                return False
        except Exception as e:
            logger.error(f"連接過程中發生錯誤: {e}")
            return False

    def disconnect(self):
        """斷開與無人機的連接。"""
        if self.is_connected:
            self._stop_thread.set()
            if self._update_thread:
                self._update_thread.join()
            self.master.close()
            self.is_connected = False
            logger.info("已與無人機斷開連接。")

    def _wait_for_heartbeat(self, timeout=10):
        logger.info("等待心跳訊息...")
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
        if msg:
            logger.info(f"收到心跳: 系統ID={self.master.target_system}, 組件ID={self.master.target_component}")
            return True
        return False

    def _start_update_thread(self):
        """啟動一個背景線程來持續更新無人機狀態。"""
        self._stop_thread.clear()
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()

    def _update_loop(self):
        """背景線程的迴圈，持續請求並接收無人機數據。"""
        # 請求數據流
        self.master.mav.request_data_stream_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1 # 10Hz
        )
        while not self._stop_thread.is_set():
            try:
                msg = self.master.recv_match(type=['GLOBAL_POSITION_INT', 'ATTITUDE'], blocking=True, timeout=1)
                if msg:
                    self.last_update_time = time.time()
                    if msg.get_type() == 'GLOBAL_POSITION_INT':
                        self.cached_altitude = msg.relative_alt / 1000.0
                    elif msg.get_type() == 'ATTITUDE':
                        self.cached_attitude = {
                            'roll': math.degrees(msg.roll),
                            'pitch': math.degrees(msg.pitch),
                            'yaw': math.degrees(msg.yaw)
                        }
            except Exception as e:
                logger.warning(f"更新無人機狀態時出錯: {e}")
                time.sleep(1) # 避免在錯誤時瘋狂重試

    def set_mode(self, mode_name):
        """設定無人機飛行模式。"""
        if not self.is_connected: return False
        mode_id = self.master.mode_mapping().get(mode_name)
        if mode_id is None:
            logger.error(f"未知的模式: {mode_name}")
            return False
        
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        
        logger.info(f"已發送切換至 {mode_name} 模式的指令。")
        # 實際應用中應加入確認模式是否切換成功的回饋檢查
        return True

    def arm(self):
        """解鎖無人機。"""
        if not self.is_connected: return False
        logger.info("正在解鎖...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        # 應加入確認是否解鎖成功的回饋檢查
        return True

    def takeoff(self):
        """起飛到設定高度。"""
        if not self.is_connected: return False
        altitude = self.config.get('drone.takeoff_altitude')
        logger.info(f"正在起飛至 {altitude} 公尺...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
        self.has_taken_off = True
        return True

    def land(self):
        """降落。"""
        return self.set_mode('LAND')

    def send_velocity_command(self, vx, vy, vz, yaw_rate=0):
        """發送基於機身座標系的速度指令。"""
        if not self.is_connected: return False
        
        type_mask = 0b010111000111 # 只使用 vx, vy, vz 和 yaw_rate
        
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            type_mask, 0, 0, 0, vx, vy, vz, 0, 0, 0, 0, yaw_rate)
        
        # logger.debug(f"Sent velocity: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, yaw_rate={yaw_rate:.2f}")
        return True
    
    def hover(self):
        """發送懸停指令。"""
        self.send_velocity_command(0, 0, 0, 0)