# ------------------
# SiamMask 追蹤演算法
# ------------------

import torch
import logging
from os.path import isfile
from argparse import Namespace
import sys

# 根據你的SiamMask路徑進行修改
# 建議將SiamMask專案做成可安裝的Python套件
try:
    sys.path.append('/home/simmon/Desktop/SiamMask/experiments/siammask_sharp')
    from custom import Custom
    from test import siamese_init, siamese_track, load_pretrain, load_config
except ImportError as e:
    print(f"無法導入SiamMask模組，請檢查路徑設定: {e}")
    sys.exit(1)

logger = logging.getLogger(__name__)

class ObjectTracker:
    """封裝 SiamMask 物件追蹤器。"""
    def __init__(self, config):
        self.config = config
        self.device = self._get_device()
        # 創建一個假的 args 物件，來模擬 argparse 的輸出
        # 這是為了兼容 SiamMask 原始的 load_config 函式
        pseudo_args = Namespace()
        
        # 從我們的設定檔中讀取 json 路徑，並將其賦值給假 args 物件的 .config 屬性
        pseudo_args.config = self.config.get('tracker.config_path')
        
        # 現在用這個兼容的假 args 物件去呼叫 load_config
        self.model_config = load_config(pseudo_args)
        # self.model_config = load_config(self.config)
        self.model = self._load_model()
        self.state = None # 追蹤器的內部狀態

    def _get_device(self):
        """獲取運算設備 (GPU or CPU)。"""
        use_cpu = self.config.get('tracker.use_cpu')
        if torch.cuda.is_available() and not use_cpu:
            logger.info("檢測到 CUDA，將使用 GPU 進行運算。")
            return torch.device('cuda')
        else:
            logger.info("將使用 CPU 進行運算。")
            return torch.device('cpu')

    def _load_model(self):
        """載入 SiamMask 預訓練模型。"""
        resume_path = self.config.get('tracker.resume_path')
        logger.info(f"正在從 {resume_path} 載入模型...")
        
        siammask = Custom(anchors=self.model_config['anchors'])
        if not isfile(resume_path):
            logger.error(f"模型檔案不存在: {resume_path}")
            raise FileNotFoundError(f"Checkpoint not found at {resume_path}")
        
        model = load_pretrain(siammask, resume_path)
        model.eval().to(self.device)
        logger.info("SiamMask 模型載入成功。")
        return model

    def initialize(self, frame, target_pos, target_sz):
        """
        使用第一幀和目標 ROI 初始化追蹤器。
        
        Args:
            frame: 影像幀 (np.array)
            target_pos: 目標中心 (x, y)
            target_sz: 目標大小 (w, h)
        """
        logger.info(f"正在初始化追蹤器，目標位置: {target_pos}, 大小: {target_sz}")
        try:
            self.state = siamese_init(frame, target_pos, target_sz, self.model, self.model_config['hp'], self.device)
            logger.info("追蹤器初始化成功。")
        except Exception as e:
            logger.error(f"追蹤器初始化失敗: {e}")
            self.state = None

    def track(self, frame):
        """
        在新的影像幀中追蹤目標。

        Returns:
            dict or None: 包含追蹤結果的 state 物件，如果失敗則為 None。
        """
        if self.state is None:
            return None
        
        try:
            self.state = siamese_track(self.state, frame, mask_enable=True, refine_enable=True, device=self.device)
            return self.state
        except Exception as e:
            logger.error(f"追蹤過程中發生錯誤: {e}")
            self.reset() # 追蹤失敗時重置
            return None

    def is_tracking(self):
        """回傳目前是否正在追蹤。"""
        return self.state is not None

    def reset(self):
        """重置追蹤器狀態。"""
        logger.info("追蹤器已重置。")
        self.state = None