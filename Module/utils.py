# ------------------
# 存放共用的輔助函式 (例如距離計算)
# ------------------

import yaml
import math
import cv2
import numpy as np
import logging

class ConfigManager:
    """從 YAML 檔案載入並管理設定。"""
    def __init__(self, config_path='config.yaml'):
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # 將相機矩陣和畸變係數轉換為 numpy array
        self.config['camera']['matrix'] = np.array(self.config['camera']['matrix'], dtype=np.float32)
        self.config['camera']['dist_coeffs'] = np.array(self.config['camera']['dist_coeffs'], dtype=np.float32)

    def get(self, key_path):
        """使用點記法獲取設定值，例如 'drone.address'。"""
        keys = key_path.split('.')
        value = self.config
        for key in keys:
            value = value[key]
        return value

def calculate_distance(target_bottom_center, camera_config, drone_altitude):
    """
    根據目標在影像中的底部中心點，計算與目標的水平距離。

    Args:
        target_bottom_center (tuple): 目標底部中心點的 (u, v) 像素座標。
        camera_config (dict): 包含相機矩陣、畸變係數和傾斜角度的設定。
        drone_altitude (float): 無人機目前的相對高度(公尺)。

    Returns:
        float or None: 計算出的水平距離(公尺)，如果無效則回傳 None。
    """
    u, v = target_bottom_center
    cam_matrix = camera_config['matrix']
    dist_coeffs = camera_config['dist_coeffs']
    fx, fy = cam_matrix[0, 0], cam_matrix[1, 1]
    cx, cy = cam_matrix[0, 2], cam_matrix[1, 2]
    
    # 畸變校正
    points = np.array([[[u, v]]], dtype=np.float32)
    undistorted_points = cv2.undistortPoints(points, cam_matrix, dist_coeffs, P=cam_matrix)
    u_corr, v_corr = undistorted_points[0, 0]

    # 計算俯角
    alpha = math.atan((v_corr - cy) / fy)
    theta = math.radians(camera_config['tilt_angle_deg'])

    # 計算距離
    denominator = math.tan(theta + alpha)
    if abs(denominator) < 1e-6 or drone_altitude <= 0:
        return None

    distance = drone_altitude / denominator
    return distance if distance > 0 else None

def setup_logger():
    """設定全域日誌記錄器。"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    # 關閉 pynput 的冗長 debug log
    logging.getLogger("pynput").setLevel(logging.WARNING)
    return logging.getLogger("AutoTrackApp")