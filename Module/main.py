# ------------------
# 主應用程式
# ------------------

import cv2
import time
import logging
import os 
import csv
import numpy as np
from pynput import keyboard

# 導入我們自己的模組
from utils import ConfigManager, calculate_distance, setup_logger
from app_state import AppState
from drone_controller import DroneController
from object_tracker import ObjectTracker
from control_system import AutonomousController
from video_handler import VideoHandler
from roi_handler import ROIHandler

# 設置日誌
logger = setup_logger()

class AutoTrackApp:
    """主應用程式類別。"""
    def __init__(self):
        self.state = AppState.STARTING
        logger.info("應用程式啟動中...")

        # 載入設定
        self.config = ConfigManager('config.yaml')

        # 初始化各個模組
        self.drone = DroneController(self.config)
        self.tracker = ObjectTracker(self.config)
        self.controller = AutonomousController(self.config)
        self.video = VideoHandler(self.config)
        self.roi_handler = ROIHandler()

        # UI & 控制
        self.window_name = "Drone AutoTrack"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.roi_handler.mouse_callback)
        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_release)
        
        self.frame = None
        self.last_frame_time = time.time()
        self.fps = 0

        self.log_file = None
        self.log_writer = None
        self._setup_log_file()

    def _setup_log_file(self):
        """創建一個帶有時間戳的唯一日誌檔案並寫入標頭。"""
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        timestamp_str = time.strftime("%Y%m%d-%H%M%S")
        filename = os.path.join(log_dir, f"track_log_{timestamp_str}.csv")
        
        try:
            self.log_file = open(filename, 'w', newline='')
            self.log_writer = csv.writer(self.log_file)
            
            # 寫入CSV標頭
            header = [
                'timestamp',
                'setpoint_yaw_error', 'actual_yaw_error',
                'setpoint_height_error', 'actual_height_error',
                'setpoint_distance', 'actual_distance',
                'pid_output_vx', 'pid_output_vz', 'pid_output_yaw_rate',
                'drone_altitude',
                # 以下為路徑圖所需，如果沒有外部定位系統(如GPS/SLAM)，可以先留空
                'target_world_x', 'target_world_y', 
                'drone_world_x', 'drone_world_y'
            ]
            self.log_writer.writerow(header)
            logger.info(f"日誌檔案已創建: {filename}")
        except Exception as e:
            logger.error(f"創建日誌檔案失敗: {e}")

    def on_key_press(self, key):
        """處理鍵盤輸入，主要用於狀態切換和手動控制。"""
        try:
            k = key.char
        except AttributeError:
            return # 非字元按鍵，忽略

        # --- 狀態切換指令 ---
        if k == 'c': # 連接無人機
            if self.state == AppState.DISCONNECTED:
                if self.drone.connect():
                    self.state = AppState.WAITING_FOR_VIDEO
        elif k == 'v': # 切換自主模式
            if self.state == AppState.TRACKING:
                self.state = AppState.AUTONOMOUS_FLIGHT
                self.controller.reset() # 進入自主模式時重置PID
                logger.info("切換至 [自主飛行] 模式。")
            elif self.state == AppState.AUTONOMOUS_FLIGHT:
                self.state = AppState.TRACKING
                self.drone.hover() # 退出自主模式時懸停
                logger.info("切換至 [手動追蹤] 模式。")
        # on_key_press 方法中
        elif k == 'r': # 重置追蹤與狀態
            self.tracker.reset()
            # 擴充條件，允許在 LANDING 狀態下重置
            if self.state in [AppState.TRACKING, AppState.AUTONOMOUS_FLIGHT, AppState.LANDING]:
                logger.info(f"從 {self.state.name} 狀態重置系統至 IDLE 狀態。")
                self.state = AppState.IDLE
                if self.drone.is_connected:
                    self.drone.hover() # 發送懸停指令作為安全措施
        elif k == 'x': # 退出
            self.state = AppState.EXITING
            return False # 停止監聽

        # --- 飛行控制指令 ---
        if self.state in [AppState.IDLE, AppState.TRACKING]:
            if k == 't': self.drone.takeoff()
            elif k == 'l': self.state = AppState.LANDING
            elif k == 'g': self.drone.set_mode("GUIDED")
            elif k == 'z': self.drone.arm()
            
            # 手動飛行
            vel = self.config.get('control.max_velocity_ms') / 2
            yaw_rate = self.config.get('control.max_yaw_rate_rads') / 2
            if   k == 'w': self.drone.send_velocity_command(vel, 0, 0)
            elif k == 's': self.drone.send_velocity_command(-vel, 0, 0)
            elif k == 'a': self.drone.send_velocity_command(0, -vel, 0)
            elif k == 'd': self.drone.send_velocity_command(0, vel, 0)
            elif k == 'u': self.drone.send_velocity_command(0, 0, -vel)
            elif k == 'j': self.drone.send_velocity_command(0, 0, vel)
            elif k == 'q': self.drone.send_velocity_command(0, 0, 0, -yaw_rate)
            elif k == 'e': self.drone.send_velocity_command(0, 0, 0, yaw_rate)
            elif k == 'h': self.drone.hover() # 手動懸停
    
    def on_release(self, key):
        """處理鍵盤按鍵放開事件，用於停止手動移動。"""
        # 如果處於自主模式，則忽略按鍵放開事件
        if self.state == AppState.AUTONOMOUS_FLIGHT:
            return

        try:
            k = key.char
            # 如果放開的是移動、升降、偏航的按鍵，就發送懸停指令
            if k in ['w', 's', 'a', 'd', 'u', 'j', 'q', 'e']:
                if self.drone.is_connected:
                    self.drone.hover() # 發送 vx=0, vy=0, vz=0, yaw_rate=0 的指令
                    logger.debug(f"Key '{k}' released, stopping movement.")
        except AttributeError:
            pass # 忽略非字元按鍵

    def _draw_overlay(self, display_frame):
        """在影像上繪製所有狀態資訊和追蹤結果。"""
        height, width, _ = display_frame.shape
        
        # 繪製 ROI 選擇框
        self.roi_handler.draw_selection(display_frame)

        # 繪製追蹤結果
        # 繪製追蹤結果
        if self.tracker.is_tracking():
            state = self.tracker.state
            
            # 繪製多邊形邊框
            if 'polygon' in state and len(state['polygon']) > 0:
                location = np.array(state['polygon']).flatten()
                cv2.polylines(display_frame, [np.int64(location).reshape((-1, 1, 2))], True, (0, 255, 0), 2)
            
            # --- 新增：繪製物件分割遮罩 ---
            if 'mask' in state and state['mask'].size > 0:
                # 獲取遮罩並進行二值化
                mask = state['mask'] > state['p'].seg_thr
                # 將遮罩顏色設為紅色 (B, G, R)
                color_mask = np.zeros_like(display_frame, dtype=np.uint8)
                color_mask[mask] = [0, 0, 255] # 將遮罩區域設為紅色
                
                # 使用加權混合，讓遮罩有半透明效果
                display_frame = cv2.addWeighted(display_frame, 1.0, color_mask, 0.5, 0)
        
        # 顯示狀態文字
        texts = [
            f"FPS: {self.fps:.1f}",
            f"State: {self.state.name}",
            f"Drone Connected: {'Yes' if self.drone.is_connected else 'No'}",
            f"Altitude: {self.drone.cached_altitude:.2f} m"
        ]
        for i, text in enumerate(texts):
            cv2.putText(display_frame, text, (10, 30 + i * 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        return display_frame

    def run(self):
        """應用程式主迴圈。"""
        self.keyboard_listener.start()
        self.state = AppState.DISCONNECTED
        
        while self.state != AppState.EXITING:
            # --- 狀態機邏輯 ---
            if self.state == AppState.DISCONNECTED:
                # 等待 'c' 鍵連接
                pass 
            elif self.state == AppState.WAITING_FOR_VIDEO:
                if self.video.start():
                    self.state = AppState.IDLE
                else: # 啟動失敗
                    self.state = AppState.DISCONNECTED
            elif self.state == AppState.LANDING:
                self.drone.land() # 持續呼叫降落指令
                time.sleep(0.2)   # 避免以過高頻率發送指令

            # --- 影像處理與控制 ---
            elif self.state.value >= AppState.IDLE.value:
                self.frame = self.video.get_frame()
                if self.frame is None:
                    time.sleep(0.1)
                    continue

                # ROI 處理
                if self.roi_handler.roi_done:
                    self.tracker.initialize(self.frame, self.roi_handler.target_pos, self.roi_handler.target_sz)
                    # self.roi_handler.roi_done = False # 這行可以被 reset() 取代
                    if self.tracker.is_tracking():
                        self.state = AppState.TRACKING
                        self.roi_handler.reset() # <-- 新增：重置ROI Handler，清除選擇框
                    else:
                        # 如果初始化失敗，也重置
                        self.roi_handler.reset()
                
                # 追蹤
                if self.tracker.is_tracking():
                    track_state = self.tracker.track(self.frame)
                    if track_state is None: # 追蹤失敗
                        self.state = AppState.IDLE
                        self.drone.hover()
                
                # 自主飛行
                if self.state == AppState.AUTONOMOUS_FLIGHT and self.tracker.is_tracking():
                    location = np.array(self.tracker.state['polygon']).flatten().reshape(-1, 2)
                    target_center = np.mean(location, axis=0)
                    bottom_center_y = np.max(location[:, 1])
                    bottom_center_x = location[np.argmax(location[:, 1]), 0]
                    
                    distance = calculate_distance(
                        (bottom_center_x, bottom_center_y),
                        self.config.get('camera'),
                        self.drone.cached_altitude
                    )
                    
                    if distance is not None:
                        # 計算誤差
                        frame_width, frame_height = self.frame.shape[1], self.frame.shape[0]
                        error_x = (frame_width / 2) - target_center[0]
                        error_y = (frame_height / 2) - target_center[1]

                        # 獲取指令
                        commands = self.controller.calculate_commands(
                            frame_width, frame_height,
                            target_center, distance
                        )
                        self.drone.send_velocity_command(**commands)

                        # --- 新增：寫入日誌數據 ---
                        log_row = [
                            time.time(),
                            0, error_x,                                    # 偏航誤差 (設定點為0)
                            0, error_y,                                    # 高度誤差 (設定點為0)
                            self.controller.pid_distance.setpoint, distance, # 距離
                            commands['vx'], commands['vz'], commands['yaw_rate'], # PID輸出
                            self.drone.cached_altitude,                    # 無人機高度
                            # 以下為世界座標，需要從您的定位系統獲取
                            # 暫時用None或0填充
                            None, None,                                    # 目標世界座標
                            None, None                                     # 無人機世界座標
                        ]
                        self.log_writer.writerow(log_row)
                        # --- 新增結束 ---

                    else:
                        self.drone.hover()

            # --- 顯示與更新 ---
            display_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            if self.frame is not None:
                display_frame = self.frame.copy()
            
            display_frame = self._draw_overlay(display_frame)
            cv2.imshow(self.window_name, display_frame)

            # 更新 FPS
            now = time.time()
            self.fps = 1.0 / (now - self.last_frame_time)
            self.last_frame_time = now
            
            if cv2.waitKey(1) & 0xFF == 27: # 按 ESC 退出
                self.state = AppState.EXITING

        self.cleanup()

    def cleanup(self):
        """清理所有資源。"""
        logger.info("正在關閉應用程式...")
        if self.keyboard_listener.is_alive():
            self.keyboard_listener.stop()
        if self.log_file:
            self.log_file.close()
            logger.info("日誌檔案已儲存。")
        if self.drone.is_connected:
            self.drone.land()
            time.sleep(5) # 等待降落
            self.drone.disconnect()
        self.video.stop()
        cv2.destroyAllWindows()
        logger.info("清理完畢，程式已終止。")

if __name__ == "__main__":
    app = AutoTrackApp()
    app.run()