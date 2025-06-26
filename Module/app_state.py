# ------------------
# 定義應用程式的各種狀態
# ------------------

from enum import Enum, auto

class AppState(Enum):
    """定義應用程式的所有可能狀態。"""
    STARTING = auto()
    DISCONNECTED = auto()
    WAITING_FOR_VIDEO = auto()
    IDLE = auto()  # 已連線，待命中
    SELECTING_TARGET = auto()
    TRACKING = auto() # 正在追蹤，但由手動控制
    AUTONOMOUS_FLIGHT = auto()
    LANDING = auto()
    EXITING = auto()