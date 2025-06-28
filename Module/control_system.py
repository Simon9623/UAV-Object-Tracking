# ------------------
# control_system.py
# ------------------

import time
import logging
import numpy as np

logger = logging.getLogger(__name__)

class PID:
    """一個簡單的 PID 控制器實現。"""
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self._last_error = 0
        self._integral = 0
        self._last_time = time.time()
        
        # 積分飽和限制
        self.integral_min = -1.0
        self.integral_max = 1.0

    def calculate(self, measurement):
        """計算 PID 控制量。"""
        current_time = time.time()
        dt = current_time - self._last_time
        if dt == 0:
            return 0 # 避免除以零

        error = self.setpoint - measurement
        
        # P (比例)
        p_term = self.Kp * error
        
        # I (積分)
        self._integral += error * dt
        self._integral = max(min(self._integral, self.integral_max), self.integral_min) # 抗積分飽和
        i_term = self.Ki * self._integral
        
        # D (微分)
        derivative = (error - self._last_error) / dt
        d_term = self.Kd * derivative
        
        # 更新狀態
        self._last_error = error
        self._last_time = current_time
        
        return p_term + i_term + d_term

    def reset(self):
        """重置 PID 控制器狀態。"""
        self._last_error = 0
        self._integral = 0
        self._last_time = time.time()


class AutonomousController:
    """管理自主飛行的控制系統。"""
    def __init__(self, config):
        control_cfg = config.get('control')
        
        self.pid_yaw = PID(**control_cfg['pid_yaw'])
        self.pid_height = PID(**control_cfg['pid_height'])
        self.pid_distance = PID(**control_cfg['pid_distance'], setpoint=control_cfg['desired_distance_m'])

        self.max_vx = control_cfg['max_velocity_ms']
        self.max_vz = control_cfg['max_vertical_velocity_ms']
        self.max_yaw_rate = control_cfg['max_yaw_rate_rads']

    def calculate_commands(self, frame_width, frame_height, target_center, distance_to_target):
        """
        根據目標狀態計算無人機的速度指令。

        Args:
            frame_width (int): 影像寬度。
            frame_height (int): 影像高度。
            target_center (tuple): 目標中心像素座標 (x, y)。
            distance_to_target (float): 與目標的估算距離。

        Returns:
            dict: 包含 {'vx', 'vy', 'vz', 'yaw_rate'} 的指令。
        """
        if target_center is None or distance_to_target is None:
            return {'vx': 0, 'vy': 0, 'vz': 0, 'yaw_rate': 0}

        target_x, target_y = target_center
        
        # 計算誤差
        error_x = (frame_width / 2) - target_x # 水平誤差，用於偏航
        error_y = (frame_height / 2) - target_y # 垂直誤差，用於升降

        # 計算控制量
        # 注意: vz 和 yaw_rate 的符號可能需要根據無人機的反應來調整
        yaw_rate_cmd = self.pid_yaw.calculate(error_x) # 讓 error_x 變小
        vz_cmd = self.pid_height.calculate(error_y)     # 讓 error_y 變小
        vx_cmd = self.pid_distance.calculate(distance_to_target)

        # 限制輸出範圍
        vx_cmd = np.clip(vx_cmd, -self.max_vx, self.max_vx)
        vz_cmd = np.clip(vz_cmd, -self.max_vz, self.max_vz)
        yaw_rate_cmd = np.clip(yaw_rate_cmd, -self.max_yaw_rate, self.max_yaw_rate)

        logger.debug(f"Control Errors: x={error_x:.1f}, y={error_y:.1f}, dist_err={(self.pid_distance.setpoint - distance_to_target):.2f}")
        logger.debug(f"Control Outputs: vx={vx_cmd:.2f}, vz={vz_cmd:.2f}, yaw={yaw_rate_cmd:.2f}")

        return {'vx': -vx_cmd, 'vy': 0, 'vz': 0, 'yaw_rate': yaw_rate_cmd}
    
    def reset(self):
        """重置所有 PID 控制器。"""
        self.pid_yaw.reset()
        self.pid_height.reset()
        self.pid_distance.reset()
        logger.info("控制系統已重置。")