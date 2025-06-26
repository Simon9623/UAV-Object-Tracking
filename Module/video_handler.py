# ------------------
# 負責接收和解碼 UDP 影像串流
# ------------------

# video_handler.py
import socket
import logging
import cv2
import numpy as np

logger = logging.getLogger(__name__)

class VideoHandler:
    """處理 UDP 影像串流的接收與解碼。"""
    def __init__(self, config):
        self.listen_ip = config.get('network.video_listen_ip')
        self.listen_port = config.get('network.video_listen_port')
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(2.0) # 2秒超時
        self.is_running = False

    def start(self):
        """綁定 Socket 並開始監聽。"""
        try:
            self.sock.bind((self.listen_ip, self.listen_port))
            logger.info(f"影像串流監聽已啟動於 udp://{self.listen_ip}:{self.listen_port}")
            self.is_running = True
            return True
        except Exception as e:
            logger.error(f"綁定影像串流 Socket 失敗: {e}")
            self.is_running = False
            return False

    def get_frame(self):
        """
        接收一幀影像。

        Returns:
            np.array or None: 解碼後的影像幀，或在超時/錯誤時回傳 None。
        """
        if not self.is_running:
            return None
        
        try:
            packet, _ = self.sock.recvfrom(65535)
            frame = cv2.imdecode(np.frombuffer(packet, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                logger.warning("收到一個無法解碼的影像封包。")
            return frame
        except socket.timeout:
            logger.warning("等待影像串流超時...")
            return None
        except Exception as e:
            logger.error(f"接收影像時發生錯誤: {e}")
            return None

    def stop(self):
        """關閉 Socket。"""
        self.is_running = False
        self.sock.close()
        logger.info("影像串流監聽已停止。")