# ------------------
# 處理滑鼠 ROI 框選邏輯
# ------------------


import cv2
import numpy as np

class ROIHandler:
    """
    透過一個類別來處理滑鼠 ROI 選擇，避免使用全域變數。
    """
    def __init__(self):
        self._start_point = None
        self._end_point = None
        self.is_selecting = False
        
        self.roi_done = False
        self.target_pos = None
        self.target_sz = None
        self.selection_box = None # 用於繪製的矩形框

    def mouse_callback(self, event, x, y, flags, param):
        """註冊到 cv2.setMouseCallback 的回呼函式。"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.reset() # 每次點擊都開始一次新的選擇
            self._start_point = (x, y)
            self.is_selecting = True
            self.roi_done = False

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.is_selecting:
                self._end_point = (x, y)
                self.selection_box = (self._start_point, self._end_point)

        elif event == cv2.EVENT_LBUTTONUP:
            if self.is_selecting:
                self._end_point = (x, y)
                self.is_selecting = False
                
                # 確保 start 在左上, end 在右下
                x1, y1 = self._start_point
                x2, y2 = self._end_point
                start_x, end_x = min(x1, x2), max(x1, x2)
                start_y, end_y = min(y1, y2), max(y1, y2)

                # 忽略太小的框
                if (end_x - start_x) > 10 and (end_y - start_y) > 10:
                    self.target_pos = np.array([(start_x + end_x) / 2, (start_y + end_y) / 2])
                    self.target_sz = np.array([end_x - start_x, end_y - start_y])
                    self.roi_done = True
                else:
                    self.reset() # 選擇無效，重置

    def draw_selection(self, frame):
        """在影像上繪製正在選擇的框。"""
        if self.selection_box:
            start, end = self.selection_box
            cv2.rectangle(frame, start, end, (0, 255, 0), 2)
        return frame

    def reset(self):
        """重置所有狀態。"""
        self._start_point = None
        self._end_point = None
        self.is_selecting = False
        self.roi_done = False
        self.target_pos = None
        self.target_sz = None
        self.selection_box = None