# -*- coding: utf-8 -*-
# ffmpeg + MQTT/UDP通信 + 控制变焦 + 模型开启选择 + 识别结果上传（增加坐标与类别）
# 移植到Windows环境，使用RTSP拉流
import subprocess
import threading
import time
import cv2
import numpy as np
from collections import deque
from ultralytics import YOLO
import warnings
import socket
import signal
import sys
import os
import gc
import re
import math
import traceback
import paho.mqtt.client as mqtt
import json
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
import logging
import requests
import torch
from minio import Minio
from minio.error import S3Error
import io
from scipy.optimize import linear_sum_assignment  # 用于匈牙利算法
from email.utils import parsedate_to_datetime

# ================== 日志配置 ==================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)
# ===========================================
warnings.filterwarnings("ignore")

# ================== 配置区 ==================
# ================== 视频流功能开关 ==================
ENABLE_VIS_STREAM = True   # 可见光视频流开关（拉流+推流+AI+定位+吊舱控制）。关闭时可见光流不运行
ENABLE_VIS_AI = True       # 可见光AI推理开关（关闭时仅推流不推理，定位依赖推理不可用）
ENABLE_INF_STREAM = True   # 红外视频流开关（拉流+推流+定位）。关闭时红外流不运行
ENABLE_INF_AI = True       # 红外AI推理开关（依赖红外流开启才有效；红外定位需要推理开启）
# ==================================================

# RTSP配置
RTSP_OUT = "rtsp://localhost:8554/live/car_test"  # RTSP推流地址
RTSP_HW_OUT = "rtsp://localhost:8554/live/car_test_hw"  # 红外RTSP推流地址

# 模型路径
ENGINE_PATH = "/home/amov/AI_model/panel/best.engine"  # TensorRT engine模型
#ENGINE_PATH = "/home/amov/AI_model/panel960/yolov8_imgsize960.engine"  # TensorRT engine模型
ENGINE_PATH_INF = "/home/amov/AI_model/panel_inf/best.engine"

TARGET_W = 1920  # 默认流分辨率宽（可见光用相机配置 image_width，红外用 image_width_inf，此处仅为回退值）
TARGET_H = 1080  # 默认流分辨率高（回退值，同上）
TARGET_FPS = 15  #输出帧率

# 推流看门狗：超过该秒数无成功推帧（正常约67ms一帧）且队列有积压时，判定推流卡死并强制重启 FFmpeg
# 4s ≈ 60帧无输出；配合"非重启中"和"队列有积压"两个条件，误杀风险很低
STALL_TIMEOUT = 4

# H.264 RTP 时钟频率（用于 PTS 到秒的转换）
RTP_CLOCK_RATE = 90000

# 北京时间 (UTC+8)
TZ_CST = timezone(timedelta(hours=8))

CONF_THRES = 0.45  # 检测置信度阈值（配合 SORT_MIN_HITS 确认机制，单帧误检不会产生ID，可适当放宽减少漏检）
YOLO_IMGSZ = 640        # 可见光推理尺寸
YOLO_IMGSZ_INF = 640    # 红外推理尺寸

# ================== 相机配置表 ==================
# 相机参数配置字典，支持多相机切换
CAMERA_CONFIGS = {
    0: {
        "name": "千决吊舱",
        # 吊舱相对RTK天线的安装偏移（机体坐标系，单位：米）
        # forward_m: 机头方向为正；right_m: 右翼方向为正；down_m: 垂直向下为正
        "lever_arm": {"forward_m": 0.0, "right_m": 0.0, "down_m": 0.0},
        # 红外镜头相对RTK天线的安装偏移（红外与可见光镜头安装位置不同，需单独实测）
        "lever_arm_inf": {"forward_m": 0.0, "right_m": 0.0, "down_m": 0.0},
        "focal_length": 4.25,      # 焦距（毫米）
        "pixel_size": 0.0029,      # 单个像素物理尺寸（毫米/像素）
        "image_width": 1920,       # 图像宽度（像素）
        "image_height": 1080,      # 图像高度（像素）
        "rtsp_in": "rtsp://192.168.1.100:554/stream0" , # RTSP拉流地址
        "rtsp_hw_in": "rtsp://192.168.1.100:554/stream2"  # 红外RTSP拉流地址
    },
    1: {
        "name": "先飞吊舱",
        # 吊舱相对RTK天线的安装偏移（机体坐标系，单位：米）
        # forward_m: 机头方向为正；right_m: 右翼方向为正；down_m: 垂直向下为正
        # 实测（机头朝北）：吊舱在RTK北562.439mm、西290.337mm、下方265.245mm
        # 北→前向(+)，西→右翼负向(-)，下→正向(+)，mm转m
        "lever_arm": {"forward_m": 0.562439, "right_m": -0.290337, "down_m": 0.265245},
        # 红外镜头相对RTK天线的安装偏移（红外与可见光镜头安装位置不同，需单独实测）
        "lever_arm_inf": {"forward_m": 0.562439, "right_m": -0.504344, "down_m": 0.250245},
        "focal_length": 4.8,       # 焦距（毫米）
        # 实测：1080p输出下像素为4K原生(1.45um)经2x2 binning后的等效尺寸，约为2.9um
        # 若用0.00145会导致GSD偏小约一半，定位偏移只有实际的一半
        "pixel_size": 0.0029,      # 单个像素物理尺寸（毫米/像素）
        "image_width": 1920,       # 图像宽度（像素）
        "image_height": 1080,      # 图像高度（像素）
        # 红外实测参数：焦距24mm，等效焦距52mm（等效值仅作视场参考，定位计算用实际焦距）
        "focal_length_inf": 24.0,  # 红外焦距（毫米）
        "pixel_size_inf": 0.012,   # 红外像元间距（毫米/像素，12um）
        "image_width_inf": 1280,   # 红外原生分辨率宽（像素）
        "image_height_inf": 1024,  # 红外原生分辨率高（像素）
        # 红外源黑边裁剪（ffmpeg crop语法 w:h:x:y）：源以1440×1080(4:3)容器输出，
        # 实际内容1352×1080(5:4按高度放大)，左右各44px纯黑边（2026-08-26实测）。
        # 裁掉黑边后由拉流scale还原为原生1280×1024，保证定位像素坐标与12um像元严格对应
        "inf_crop": "1352:1080:44:0",
        # 红外时间系："local"=妙算3时钟系（红外源为其MediaMTX，时钟无法NTP同步，遥测匹配用local_time逐条跟踪漂移）
        #             "ntp"=NTP同步时钟系（红外相机自带HTTP服务且已NTP对时，同可见光，遥测匹配用ntp_time）
        "inf_time_system": "local",
        "rtsp_in": "rtsp://user:0000@192.168.144.108:554/cam/realmonitor?channel=1&subtype=0",  # RTSP拉流地址
        "rtsp_hw_in": "rtsp://192.168.144.45:8554/h30t"  # RTSP拉流地址
    },
    2: {
        "name": "固定机芯",
        # 吊舱相对RTK天线的安装偏移（机体坐标系，单位：米）
        # forward_m: 机头方向为正；right_m: 右翼方向为正；down_m: 垂直向下为正
        "lever_arm": {"forward_m": 0.0, "right_m": 0.0, "down_m": 0.0},
        # 红外镜头相对RTK天线的安装偏移（红外与可见光镜头安装位置不同，需单独实测）
        "lever_arm_inf": {"forward_m": 0.0, "right_m": 0.0, "down_m": 0.0},
        "focal_length": 5.5,       # 焦距（毫米）
        "pixel_size": 0.00219,     # 单个像素物理尺寸（毫米/像素）
        "image_width": 1920,       # 图像宽度（像素）
        "image_height": 1080,       # 图像高度（像素）
        "rtsp_in": "rtsp://admin:qly123456@192.168.144.69:554/live",  # RTSP拉流地址
        "rtsp_hw_in": "rtsp://admin:qly123456@192.168.144.69:554/live"  # 红外RTSP拉流地址
    },
    3: {
        "name": "本地测试视频",
        # 吊舱相对RTK天线的安装偏移（机体坐标系，单位：米）
        # forward_m: 机头方向为正；right_m: 右翼方向为正；down_m: 垂直向下为正
        "lever_arm": {"forward_m": 0.0, "right_m": 0.0, "down_m": 0.0},
        # 红外镜头相对RTK天线的安装偏移（红外与可见光镜头安装位置不同，需单独实测）
        "lever_arm_inf": {"forward_m": 0.0, "right_m": 0.0, "down_m": 0.0},
        "focal_length": 5.5,       # 焦距（毫米）
        "pixel_size": 0.00219,     # 单个像素物理尺寸（毫米/像素）
        "image_width": 1920,       # 图像宽度（像素）
        "image_height": 1080,       # 图像高度（像素）
        "rtsp_in": "rtsp://localhost:8554/live",  # RTSP拉流地址
        "rtsp_hw_in": "rtsp://localhost:8554/live"  # RTSP拉流地址
    },
    4: {
        "name": "H30T",
        # 吊舱相对RTK天线的安装偏移（机体坐标系，单位：米）
        # forward_m: 机头方向为正；right_m: 右翼方向为正；down_m: 垂直向下为正
        "lever_arm": {"forward_m": 0.0, "right_m": 0.0, "down_m": 0.0},
        # 红外镜头相对RTK天线的安装偏移（红外与可见光镜头安装位置不同，需单独实测）
        "lever_arm_inf": {"forward_m": 0.562439, "right_m": -0.504344, "down_m": 0.250245},
        "focal_length": 7.1,       # 焦距（毫米）
        "pixel_size": 0.00097,     # 单个像素物理尺寸（毫米/像素）
        "image_width": 1440,       # 图像宽度（像素）
        "image_height": 1080,       # 图像高度（像素）
        # H30T 红外参数：
        # 源以1440×1080(4:3)容器输出5:4内容（左右各44px黑边，2026-08-26实测）
        "focal_length_inf": 24.0,  # 红外焦距（毫米）
        "pixel_size_inf": 0.012,   # 红外像元间距（毫米/像素，12um）
        "image_width_inf": 1280,   # 红外原生分辨率宽（像素）
        "image_height_inf": 1024,  # 红外原生分辨率高（像素）
        "inf_crop": "1352:1080:44:0",  # 红外源黑边裁剪（实测值）
        "inf_time_system": "local",    # 红外源为妙算3的MediaMTX，时间系同先飞
        "rtsp_in": "rtsp://192.168.144.45:8554/h30t/zoom",  # RTSP拉流地址
        "rtsp_hw_in": "rtsp://192.168.144.45:8554/h30t"  # RTSP拉流地址
    },

}

# 当前使用的相机索引（修改此值切换相机）
CURRENT_CAMERA_INDEX = 1

# 从相机配置表中加载参数
_current_camera = CAMERA_CONFIGS[CURRENT_CAMERA_INDEX]
FOCAL_LENGTH = _current_camera["focal_length"]   #相机焦距（毫米）
PIXEL_SIZE = _current_camera["pixel_size"]       #单个像素物理尺寸（毫米/像素）
IMAGE_WIDTH = _current_camera["image_width"]     #图像宽度（像素）
IMAGE_HEIGHT = _current_camera["image_height"]   #图像高度（像素）
RTSP_IN = _current_camera["rtsp_in"]              # RTSP拉流地址
RTSP_HW_IN = _current_camera["rtsp_hw_in"]        # 红外RTSP拉流地址

# 红外相机定位参数（未在相机配置表中配置 focal_length_inf/pixel_size_inf 时沿用可见光参数）
# 红外拉流/推流/推理/定位统一使用红外原生分辨率（如1280×1024），未配置原生分辨率时回退 TARGET_W×TARGET_H
FOCAL_LENGTH_INF = _current_camera.get("focal_length_inf", FOCAL_LENGTH)  # 红外焦距（毫米，实际焦距）
PIXEL_SIZE_INF = _current_camera.get("pixel_size_inf", PIXEL_SIZE)        # 红外像元间距（毫米/像素）
IMAGE_WIDTH_INF = _current_camera.get("image_width_inf", TARGET_W)        # 红外流分辨率宽（像素，原生）
IMAGE_HEIGHT_INF = _current_camera.get("image_height_inf", TARGET_H)      # 红外流分辨率高（像素，原生）
INF_CROP = _current_camera.get("inf_crop", "")                            # 红外源黑边裁剪（w:h:x:y，空=不裁剪）
# 红外时间系："local"=妙算3系（MediaMTX时间源，遥测匹配用local_time）/ "ntp"=NTP系（相机自带HTTP已对时，匹配用ntp_time）
INF_TIME_SYSTEM = _current_camera.get("inf_time_system", "ntp")

# 吊舱相对RTK天线的安装偏移（机体坐标系：前/右/下，单位米），用于修正吊舱定位
# 默认全0表示吊舱与RTK天线重合（即原逻辑），配置后按无人机姿态旋转补偿
LEVER_ARM = _current_camera.get("lever_arm", {"forward_m": 0.0, "right_m": 0.0, "down_m": 0.0})
# 红外镜头相对RTK天线的安装偏移（红外与可见光镜头安装位置不同，单独标定）
# 未配置 lever_arm_inf 的相机沿用可见光 LEVER_ARM
LEVER_ARM_INF = _current_camera.get("lever_arm_inf", LEVER_ARM)

print(f"[INFO] 当前使用相机: {_current_camera['name']} (索引: {CURRENT_CAMERA_INDEX})")
print(f"[INFO] 吊舱RTK安装偏移(前/右/下): {LEVER_ARM['forward_m']}/{LEVER_ARM['right_m']}/{LEVER_ARM['down_m']} 米")
print(f"[INFO] 红外镜头RTK安装偏移(前/右/下): {LEVER_ARM_INF['forward_m']}/{LEVER_ARM_INF['right_m']}/{LEVER_ARM_INF['down_m']} 米")
print(f"[INFO] 可见光流分辨率: {IMAGE_WIDTH}x{IMAGE_HEIGHT}")
print(f"[INFO] 红外定位参数: 焦距{FOCAL_LENGTH_INF}mm, 流分辨率{IMAGE_WIDTH_INF}x{IMAGE_HEIGHT_INF}, 像元{PIXEL_SIZE_INF}mm")
print(f"[INFO] 红外时间系: {INF_TIME_SYSTEM}" + ("（妙算3时钟，遥测匹配用local_time）" if INF_TIME_SYSTEM == 'local' else "（NTP对时，遥测匹配用ntp_time）"))
# ==============================================


heading = 0.0
yaw_now = 0.0
pitch_now = 0.0   # 无人机俯仰角
roll_now = 0.0    # 无人机横滚角
fused_lat = 0.0
fused_lon = 0.0
height_fusion = 0.0
timestamp = 0
ntp_time = 0

# ================== 无人机遥测数据缓冲（用于时间匹配） ==================
drone_data_buffer = deque(maxlen=500)
drone_buffer_lock = threading.Lock()


# ================== 缺陷目标追踪器配置 ==================
TARGET_CACHE_SIZE = 5        # 每个目标收集多少帧坐标用于平均
TARGET_TIMEOUT = 5.0         # 目标超时时间（秒），超过此时间未更新则视为消失
UPLOAD_QUEUE_MAX_SIZE = 100  # 上传队列最大长度

# SORT 算法参数
SORT_MAX_AGE = 8             # 目标最大存活帧数（连续未匹配帧数，加大以增强遮挡/漏检后的恢复能力）
SORT_MIN_HITS = 3            # 最小确认帧数：连续匹配 N 帧后才分配正式ID并参与上传收集，抑制误检ID抖动
SORT_IOU_THRESHOLD = 0.2     # IoU匹配阈值：>此值直接判定匹配；快速移动目标可再调低
SORT_CENTER_DIST_FACTOR = 1.0  # 中心距离辅助匹配系数：IoU不达标但中心距离<=factor*检测框最大边时仍匹配（兜底快速目标）

# 目标状态枚举
class TargetStatus:
    COLLECTING = "collecting"    # 正在收集坐标（时间窗口内）
    EXPIRED = "expired"          # 时间窗口结束，等待上传
    UPLOADED = "uploaded"        # 已上传，后续忽略

# 上传队列（线程安全）
upload_queue = deque()
upload_queue_lock = threading.Lock()

# 上传线程控制
upload_thread_running = True

# 目标追踪器（存储所有检测到的目标信息）
# 结构: {target_id: {'coords': [(lat, lon), ...], 'first_frame': frame, 'status': status, 'last_seen': timestamp, 'class_name': name}}
target_tracker = {}
target_tracker_lock = threading.Lock()

# 目标ID计数器
target_id_counter = 0
# ===========================================

# ================== MQTT & UDP配置 ==================
# MQTT配置
MQTT_BROKER = "192.168.144.125"
MQTT_PORT = 1883
MQTT_USER = "mqttuser"
MQTT_PASSWORD = "mqtt"
MQTT_CLIENT_ID = "XF-D80Pro"
SUBSCRIBE_TOPIC = "drone/1581FBDBW256500A2NXJ/payload/ptz/ctl"  # 指令订阅主题
FEEDBACK_TOPIC = "drone/1581FBDBW256500A2NXJ/payload/ptz/status"  # 反馈主题


# ================== 吊舱控制传输配置 ==================
# TCP连接参数（先飞吊舱使用，IP/端口在下方 GIMBAL_CONFIGS 中按相机配置）
TCP_CONNECT_TIMEOUT = 5  # TCP连接超时（秒）
TCP_RETRY_DELAY = 2      # TCP重连延迟（秒）
# ===========================================


# ================== MinIO配置 ==================
# MinIO配置
MINIO_CFG = {
    "addr": "192.168.144.125:9000",
    "user": "minioadmin",
    "password": "minioadmin123",
    "bucket": "inspection",
    "folder": "DroneFiles",      # 可见光图片存储目录
    "folder_inf": "INF_images"   # 红外图片存储目录（可见光与红外分开存储）
}

# 初始化MinIO客户端
minio_client = None
try:
    minio_client = Minio(
        MINIO_CFG["addr"],
        access_key=MINIO_CFG["user"],
        secret_key=MINIO_CFG["password"],
        secure=False  # 使用HTTP (设为True使用HTTPS)
    )
    # 确保桶存在
    if not minio_client.bucket_exists(MINIO_CFG["bucket"]):
        minio_client.make_bucket(MINIO_CFG["bucket"])
        logger.info(f"[MinIO] 创建桶: {MINIO_CFG['bucket']}")
    logger.info(f"[MinIO] 连接到 {MINIO_CFG['addr']}，桶 '{MINIO_CFG['bucket']}' 已准备")
except Exception as e:
    logger.error(f"[MinIO] 初始化失败: {str(e)}", exc_info=True)
    minio_client = None
# ==================================================
# ================== 吊舱控制配置表 ==================
# 每个相机对应一套吊舱控制协议与指令（切换 CURRENT_CAMERA_INDEX 时自动切换）
GIMBAL_CONFIGS = {
    # ---------------- 千决吊舱：UDP 控制（55 AA 4A ... 协议） ----------------
    0: {
        "protocol": "udp",
        "local_ip": "192.168.1.200",   # 本地绑定地址
        "local_port": 4000,
        "remote_ip": "192.168.1.100",  # 吊舱地址
        "remote_port": 1030,
        # SerialControl指令映射（1-4上下右左，0/5单次）5：回中，6：垂直向下 8:全消隐
        "serial_cmd_map": {
            1: "55 AA 4A 00 01 04 00 E1 FF A6 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 1A 42 E4 A3 21 02 EF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 CB",
            2: "55 AA 4A 00 01 04 00 1F 00 B6 FD 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 1A 42 77 7B 1F 02 EF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 7E",
            3: "55 AA 4A 00 01 04 00 7F 04 EE FF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 1A 42 C3 C6 E8 01 EF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 7B",
            4: "55 AA 4A 00 01 04 00 88 FA 12 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 1A 42 4B 30 FD 01 EF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 A6",
            5: "55 AA 4A 00 01 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 1A 42 DC 79 23 02 EF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 12",
            0: "55 AA 4A 00 01 04 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 1A 42 04 E3 E8 01 EF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 69",
            6: "55 AA 4A 00 01 0C 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 1A 42 32 24 24 02 EF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 1D",
            8: "55 AA 4A 00 00 00 00 00 00 00 00 00 00 00 00 00 00 51 02 00 01 00 00 00 00 00 00 00 00 00 1A E2 CF 03 C6 03 EF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 23"
        },
        "single_serial_datas": {1, 2, 3, 4},   # 需要"按下-保持-停止"的指令
        "serial_stop_delay": 1.5,              # 按下保持时长（秒）
        "serial_stop_cmd": 0,                  # 保持结束后发送的停止指令
        "down": {6},                           # 垂直向下指令
        "down_stop_delay": 0,                  # 下视完成后的延时（秒）
        "serial_down_autostop": False,         # SerialControl=下视时是否自动补发停止指令
        # ZoomControl指令映射（均为单次）
        "zoom_cmd_map": {
            1: "55 AA 4A 00 00 00 00 00 00 00 00 00 00 00 00 00 00 11 01 00 03 00 00 00 00 00 00 00 00 00 1A 42 65 6E 2A 02 EF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 A8",
            2: "55 AA 4A 00 00 00 00 00 00 00 00 00 00 00 00 00 00 11 02 00 03 00 00 00 00 00 00 00 00 00 1A 42 84 50 2C 02 EF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 AC",
            0: "55 AA 4A 00 00 00 00 00 00 00 00 00 00 00 00 00 00 11 03 00 00 00 00 00 00 00 00 00 00 00 1A 42 DA 22 35 02 EF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 DB"
        },
        "zoom_mode": "fixed",                  # 固定档位变焦
        "zoom_stop_delay": 1.5,                # 变焦按键保持时长（秒）
        "zoom_stop_cmd": 0,                    # 变焦结束发送的停止指令
        "empty_command": None,                 # 动态变倍包裹空指令（仅先飞协议使用）
        "cancel_command": None,                # 取消跟踪指令（仅先飞协议使用）
        "zoom_1_command": None,
    },
    # ---------------- 先飞吊舱：TCP 控制（A8 E5 48 ... 协议） ----------------
    1: {
        "protocol": "tcp",
        "server_ip": "192.168.144.121",  # 跟踪/吊舱服务器地址
        "server_port": 2332,
        # SerialControl指令映射（1-4循环按下，0/5单次）5：回中，6：垂直向下，7：停止 ，8：跟随
        "serial_cmd_map": {
            1: "A8E548000100009B00000005000000000000000000000000000000000000010000000000000124F2DF6516EEAA16A3A000000FB00C0615E608000000000000000000000000005539",
            2: "A8E5480001000061FF000005000000000000000000000000000000000000010000000000000124F2DF6516EEAA16A3A000000FB00C0615E608000000000000000000000000009A50",
            3: "A8E548000100000000E60005000000000000000000000000000000000000010000000000000124F2DF6516EEAA16A3A000000FB00C0615E60800000000000000000000000000DFE6",
            4: "A8E54800010000000023FF05000000000000000000000000000000000000010000000000000124F2DF6516EEAA16A3A000000FB00C0615E608000000000000000000000000007DE4",
            5: "A8E54800020000000000000000000000000000000000000000000000000001000000000000000000000000000000000000000000000000000000000000000000000000000003CD70",
            0: "A8E548000100000000000001000000000000000000000000000000000000010000000000000124F2DF6516EEAA16A3A000000FB00C0615E608000000000000000000000000000B2F",
            6: "A8E54800020000000000000000000000000000000000000000000000000001000000000000010000000000000000000000000000000000000000000000000000000000000013529D",
            7: "A8E548000100000000000001000000000000000000000000000000000000010000000000000124F2DF6516EEAA16A3A000000FB00C0615E60800000000000000000000000011093F",
            8: "A8E5480002000000000000000000000000000000000000000000000000000100000000000001000000000000000000000000000000000000000000000000000000000000001242BC"
        },
        "single_serial_datas": {1, 2, 3, 4},   # 需要连续按下的指令
        "serial_repeat": 4,                    # 连续发送次数（按下方式）
        "serial_repeat_interval": 0.1,         # 连续发送间隔（秒）
        "down": {6},                           # 垂直向下指令
        "down_stop_delay": 1,                  # 下视完成后的延时（秒）
        "serial_down_autostop": True,          # SerialControl=下视时是否自动补发停止指令
        # ZoomControl指令映射（动态变倍时的档位命令，实际走动态生成）
        "zoom_cmd_map": {
            1: "A8E549000100000000000001000000000000000000000000000000000000010000000000000124F2DF6516EEAA16A3A000000FB00C0615E60800000000000000000000000022018ED7",
            2: "A8E549000100000000000001000000000000000000000000000000000000010000000000000124F2DF6516EEAA16A3A000000FB00C0615E6080000000000000000000000002301BDE6",
            0: "A8E548000100000000000001000000000000000000000000000000000000010000000000000124F2DF6516EEAA16A3A000000FB00C0615E608000000000000000000000000000B2F"
        },
        "zoom_mode": "dynamic",                # 动态变倍（按 ZoomRatio 生成指令）
        "zoom_stop_delay": 0,
        "zoom_stop_cmd": 0,
        "empty_command": bytes.fromhex(
            "A8E5480001000000000000000000000000000000000000000000000000000100000000000000000000000000000000000000000000000000000000000000000000000000000028B2"),
        "cancel_command": bytes.fromhex(
            "A8E552000100000000000001000000000000000000000000000000000000010000000000000124F2DF6516EEAA16A3A000000FB00C0615E60800000000000000000000000017010000000000000000000215"),
        "zoom_1_command": bytes.fromhex(
            "A8E54B000100000000000001000000000000000000000000000000000000010000000000000124F2DF6516EEAA16A3A000000FB00C0615E6080000000000000000000000002501FA002E6F"),
    },
}
# 其他相机（固定机芯/本地测试/H30T 等）默认沿用千决吊舱（UDP）配置
for _cam_idx in CAMERA_CONFIGS:
    GIMBAL_CONFIGS.setdefault(_cam_idx, GIMBAL_CONFIGS[0])

# 从吊舱配置表加载当前吊舱参数（与 CURRENT_CAMERA_INDEX 联动）
_current_gimbal = GIMBAL_CONFIGS.get(CURRENT_CAMERA_INDEX, GIMBAL_CONFIGS[0])
PROTOCOL = _current_gimbal["protocol"]
SERIAL_CMD_MAP = _current_gimbal["serial_cmd_map"]
ZOOM_CMD_MAP = _current_gimbal["zoom_cmd_map"]
SINGLE_SERIAL_DATAS = _current_gimbal["single_serial_datas"]
Down = _current_gimbal["down"]
print(f"[INFO] 当前吊舱控制协议: {PROTOCOL.upper()}")

# 类别名称，若修改需同步修改current_enabled_classes处理
COCO_NAMES = ['panel', 'dirty', 'damaged', 'snow', 'cover', 'shadow', 'sun']
COCO_NAMES_HW =['panel','shadow','shortcircuit','lowtemp'] #hotspot
# value: 要启用的类别ID列表（list of int）
AI_MODE_CONFIG = {
    1: [0],  # 板子
    2: [1, 2],  # 脏污、损伤
    3: [3, 4, 5],  # 雪、遮盖、阴影
    4: [6],  # 太阳
    5: [7],  # 板子-红外
    6: [8],  # 热斑-红外
    7: [9],  # 短路-红外
    8: [10]  # 异常低温-红外
}
# ================== 全局AI状态（线程安全） ==================
ai_lock = threading.Lock()
work_mode = 0  # 0: 关闭, 1: 开启（带过滤）
ai_enabled_classes = set()  # 当前要绘制的目标类别ID（如 0=person）
ai_set_enabled = False  # 是否启用AI框绘制
ID_Value = ""  # 航线ID
AUTOSAVE = False  # 是否启用自动保存
ZOOMAUTO = False
ZoomRatio = 1 #默认变倍倍率
tracking_mode = 0  # 0: 未跟踪, 2: 点跟踪, 3: 框选跟踪
# ==========================================================
# ================== 初始化 YOLO ==================
model = None
if ENABLE_VIS_STREAM and ENABLE_VIS_AI:
    print("[INFO] 加载 YOLO 模型 ...")
    model = YOLO(ENGINE_PATH)
    print("[INFO] YOLO 加载完成")
else:
    print("[INFO] 可见光 AI 已禁用，跳过 YOLO 模型加载（ENABLE_VIS_AI=False）")

model_inf = None
if ENABLE_INF_STREAM and ENABLE_INF_AI:
    print("[INFO] 加载 YOLO 模型 (红外) ...")
    model_inf = YOLO(ENGINE_PATH_INF)
    print("[INFO] YOLO (红外) 加载完成")
else:
    print("[INFO] 红外 AI 已禁用，跳过红外 YOLO 模型加载（ENABLE_INF_AI=False）")
# ===========================================

# ================== 启动预热推理 ==================
def warmup_yolo(yolo_model, tag="可见光", imgsz=YOLO_IMGSZ):
    """
    启动时预热推理：.engine 文件是懒加载的，首次 predict() 才会反序列化引擎、
    创建执行上下文并分配显存。提前空跑一次推理，让显存在启动阶段就分配到位，
    避免开启绘制框时才出现显存跳变。

    注意：Jetson 8GB 统一内存较紧张，预热可能因显存不足偶尔失败（error 12 /
    CUDNN_STATUS_ALLOC_FAILED）。失败时重置 predictor，下次真实推理会重新加载引擎。
    """
    if yolo_model is None:
        return
    print(f"[INFO] 预热推理 ({tag}, imgsz={imgsz}) ...")
    for attempt in range(2):  # 失败重试一次
        try:
            warmup_img = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)
            yolo_model.predict(
                source=warmup_img,
                imgsz=imgsz,
                device=0,
                half=True,
                conf=CONF_THRES,
                max_det=10,
                verbose=False
            )
            torch.cuda.empty_cache()  # 归还缓存分配器中暂未使用的显存
            print(f"[INFO] 预热完成 ({tag})")
            return
        except Exception as e:
            yolo_model.predictor = None  # 清除损坏的predictor，避免后续推理复用
            print(f"[WARN] 预热失败 ({tag}) 第{attempt+1}次: {str(e)}")
            gc.collect()
            torch.cuda.empty_cache()

# 降低cuDNN算法探测带来的工作区占用（benchmark模式会分配较大workspace）
#torch.backends.cudnn.benchmark = False
# 预热前先回收Python缓存，尽量腾出可用内存
gc.collect()
torch.cuda.empty_cache()

if model is not None:
    warmup_yolo(model, "可见光", imgsz=YOLO_IMGSZ)
if model_inf is not None:
    warmup_yolo(model_inf, "红外", imgsz=YOLO_IMGSZ_INF)
# ===========================================
# ================== 上传配置（新增） ==================
# 1. 线程池（固定2个线程，避免过多线程占用网络/CPU资源）
UPLOAD_EXECUTOR = ThreadPoolExecutor(
    max_workers=2,
    thread_name_prefix="AsyncUpload-"
)
# 2. 上传频率控制（2秒内仅上传1次，防止高频次上传导致网络过载）
LAST_UPLOAD_TIMESTAMP = 0
UPLOAD_INTERVAL = 0.5  # 单位：秒
# 3. 定位排查日志（CSV）：每次定位记录 无人机位置/吊舱位置/姿态/定位结果/目标ID，便于飞行后排查偏差
LOCATION_LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "location_debug.csv")
LOCATION_LOG_LOCK = threading.Lock()
# ==========================================================
# ================== MQTT & UDP 工具函数 ==================
# MQTT客户端
mqtt_client = None
is_mqtt_connected = False
mqtt_lock = threading.Lock()


def init_mqtt():
    """初始化MQTT客户端"""
    global mqtt_client, is_mqtt_connected
    mqtt_client = None

    try:
        mqtt_client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        mqtt_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
        mqtt_client.on_connect = on_mqtt_connect
        mqtt_client.on_message = on_mqtt_message
        mqtt_client.reconnect_delay_set(min_delay=1, max_delay=5)

        mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
        mqtt_client.loop_start()
        is_mqtt_connected = True
        print("[MQTT] 客户端初始化完成，后台线程启动")
        return True
    except Exception as e:
        print(f"[MQTT] 初始化失败: {str(e)}")
        is_mqtt_connected = False
        return False


def on_mqtt_connect(client, userdata, flags, rc, properties=None):
    """MQTT连接回调"""
    global is_mqtt_connected
    if rc == 0:
        is_mqtt_connected = True
        client.subscribe(SUBSCRIBE_TOPIC, qos=1)
        print(f"[MQTT] 成功连接服务器，订阅主题: {SUBSCRIBE_TOPIC}")
    else:
        is_mqtt_connected = False
        print(f"[MQTT] 连接失败（返回码: {rc}）")


def on_mqtt_message(client, userdata, msg):
    """MQTT消息处理回调"""
    print(f"\n[MQTT] 收到消息（主题: {msg.topic}）")
    try:
        payload_str = msg.payload.decode("utf-8")
        payload = json.loads(payload_str)
        if "ControlType" not in payload:
            print("[MQTT] 消息缺少ControlType字段")
            return

        control_type = payload["ControlType"]
        if control_type == "SerialControl":
            handle_serial_control(payload)
        elif control_type == "ZoomControl":
            handle_zoom_control(payload)
        elif control_type == "WorkModel":
            handle_work_model(payload)
        elif control_type == "AiModel":
            handle_ai_model(payload)
        elif control_type == "AutoSave":
            handle_auto_save(payload)

        else:
            print(f"[MQTT] 未知指令类型: {control_type}")
    except json.JSONDecodeError:
        print("[MQTT] 消息无法解析为JSON")
    except Exception as e:
        print(f"[MQTT] 处理消息异常: {str(e)}")


# ================== UDP 工具函数 ==================
# UDP套接字
udp_socket = None
udp_lock = threading.Lock()

def init_udp():
    """初始化UDP套接字"""
    global udp_socket
    local_ip = _current_gimbal.get("local_ip")
    local_port = _current_gimbal.get("local_port")
    try:
        # 创建UDP套接字
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 绑定本地地址 (可选，如果不绑定，系统会自动分配)
        udp_socket.bind((local_ip, local_port))
        # 设置超时时间 (秒)
        udp_socket.settimeout(2)
        print(f"[UDP] 套接字初始化完成，本地绑定: {local_ip}:{local_port}")
        return True
    except Exception as e:
        print(f"[UDP] 初始化失败: {str(e)}")
        udp_socket = None
        return False

def close_udp():
    """关闭UDP连接"""
    global udp_socket
    try:
        if udp_socket:
            udp_socket.close()
            udp_socket = None
            print("[UDP] UDP 连接已关闭")
    except Exception as e:
        print(f"[UDP] 关闭连接失败: {str(e)}")

def send_udp_command(command_bytes, desc="未知指令"):
    """
    发送UDP指令
    注意：UDP是无连接的，不需要维护长连接状态，直接发送即可
    """
    global udp_socket
    remote_ip = _current_gimbal.get("remote_ip")
    remote_port = _current_gimbal.get("remote_port")
    try:
        # 确保套接字已初始化
        if udp_socket is None:
            if not init_udp():
                print(f"[UDP] {desc}发送失败：套接字未初始化")
                return False

        # 发送数据到指定的远程地址
        target_addr = (remote_ip, remote_port)
        udp_socket.sendto(command_bytes, target_addr)
        print(f"[UDP] {desc}发送成功 | 目标: {remote_ip}:{remote_port} | 数据: {command_bytes.hex().upper()}")
        return True

    except socket.timeout:
        print(f"[UDP] {desc}发送超时")
    except Exception as e:
        print(f"[UDP] {desc}发送异常: {str(e)}")
        # UDP出错通常不需要像TCP那样断开重连，下次直接发即可
        # 这里可以尝试重建套接字
        close_udp()
    return False


# ================== TCP 工具函数（先飞吊舱） ==================
# TCP套接字
tracking_socket = None
tcp_lock = threading.Lock()

def init_tcp():
    """初始化TCP连接（先飞吊舱协议）"""
    global tracking_socket
    try:
        tracking_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tracking_socket.settimeout(TCP_CONNECT_TIMEOUT)
        tracking_socket.connect((_current_gimbal["server_ip"], _current_gimbal["server_port"]))
        tracking_socket.settimeout(None)
        print(f"[TCP] 成功连接吊舱服务器: {_current_gimbal['server_ip']}:{_current_gimbal['server_port']}")
        return True
    except Exception as e:
        print(f"[TCP] 连接失败: {str(e)}")
        tracking_socket = None
        return False

def send_tcp_command(command_bytes, desc="未知指令", max_retries=2):
    """发送TCP指令（带重试机制）"""
    global tracking_socket
    retry_count = 0
    while retry_count <= max_retries:
        try:
            # 检查连接，断开则重连
            if not tracking_socket:
                print(f"[TCP] 连接断开，尝试重连以发送{desc}...")
                if not init_tcp():
                    retry_count += 1
                    time.sleep(TCP_RETRY_DELAY)
                    continue

            tracking_socket.sendall(command_bytes)
            print(f"[TCP] {desc}发送成功: {command_bytes.hex().upper()}")
            return True
        except Exception as e:
            print(f"[TCP] {desc}发送失败(第{retry_count + 1}次尝试): {str(e)}")
            tracking_socket = None
            retry_count += 1
            time.sleep(TCP_RETRY_DELAY)

    print(f"[TCP] 超过最大重试次数，{desc}发送取消")
    return False

def close_tcp():
    """关闭TCP连接"""
    global tracking_socket
    try:
        if tracking_socket:
            tracking_socket.close()
            tracking_socket = None
            print("[TCP] 吊舱服务器连接已关闭")
    except Exception as e:
        print(f"[TCP] 关闭连接失败: {str(e)}")

# ================== 统一吊舱指令发送（按当前相机协议自动选择 TCP/UDP） ==================
def send_gimbal_command(command_bytes, desc="未知指令"):
    """按当前吊舱协议发送指令"""
    if PROTOCOL == "tcp":
        return send_tcp_command(command_bytes, desc)
    return send_udp_command(command_bytes, desc)

def init_gimbal():
    """按当前吊舱协议初始化传输连接"""
    if PROTOCOL == "tcp":
        return init_tcp()
    return init_udp()

def close_gimbal():
    """按当前吊舱协议关闭传输连接"""
    if PROTOCOL == "tcp":
        close_tcp()
    else:
        close_udp()


# ========================= MQTT指令发送工具（适配统一吊舱协议） =========================
def send_single_hex_mqtt(cmd_map, data_key, data_value):
    """MQTT指令对应的吊舱发送（适配统一send_gimbal_command）"""
    if data_value not in cmd_map:
        logger.error(f"[{data_key[:-4]}Control] 无对应{data_key}={data_value}的指令")
        return False

    # 生成指令描述
    if data_key == "SerialData":
        desc = f"SerialControl指令（SerialData={data_value}）"
    elif data_key == "ZoomData":
        desc = f"ZoomControl指令（ZoomData={data_value}）"
    else:
        desc = f"{data_key}指令（值={data_value}）"

    # 发送指令
    hex_cmd = cmd_map[data_value]
    cmd_bytes = bytes.fromhex(hex_cmd)
    return send_gimbal_command(cmd_bytes, desc)


# 指令处理函数
def handle_serial_control(data):
    """处理SerialControl指令（按当前吊舱协议执行）"""
    try:
        serial_data = int(data.get("SerialData", -1))
        send_single_hex_mqtt(SERIAL_CMD_MAP, "SerialData", serial_data)

        # 先飞吊舱：1-4 需要连续按多次才转到位
        if serial_data in SINGLE_SERIAL_DATAS:
            repeat = _current_gimbal.get("serial_repeat")
            if repeat:
                for _ in range(repeat):
                    time.sleep(_current_gimbal.get("serial_repeat_interval", 0.1))
                    send_single_hex_mqtt(SERIAL_CMD_MAP, "SerialData", serial_data)
            # 千决吊舱：按下保持一段时间后发送停止指令
            else:
                time.sleep(_current_gimbal.get("serial_stop_delay", 1.5))
                send_single_hex_mqtt(SERIAL_CMD_MAP, "SerialData", _current_gimbal.get("serial_stop_cmd", 0))

        # 下视指令完成后补发收尾指令（先飞吊舱协议需要，统一发8）
        if serial_data in Down and _current_gimbal.get("serial_down_autostop"):
            time.sleep(_current_gimbal.get("down_stop_delay", 0))
            send_single_hex_mqtt(SERIAL_CMD_MAP, "SerialData", 8)

    except (ValueError, TypeError):
        print("[SerialControl] SerialData格式错误（需为整数）")

def calculate_crc16(data_bytes):
    """CRC16校验"""
    crc_ta = [
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
        0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
    ]
    crc = 0
    for byte in data_bytes:
        da = (crc >> 12) & 0x0F
        crc <<= 4
        crc ^= crc_ta[da ^ ((byte >> 4) & 0x0F)]
        da = (crc >> 12) & 0x0F
        crc <<= 4
        crc ^= crc_ta[da ^ (byte & 0x0F)]
        crc &= 0xFFFF
    return crc

def generate_zoom_command(delt_xy):
    """生成动态变焦指令"""

    def to_hex_le(val):
        return val.to_bytes(2, byteorder='little', signed=False).hex().upper()

    delt_xy_hex = to_hex_le(delt_xy)
    base_template = "A8E54B000100000000000001000000000000000000000000000000000000010000000000000124F2DF6516EEAA16A3A000000FB00C0615E6080000000000000000000000002501{}"
    base_str = base_template.format(delt_xy_hex)

    data_bytes = bytes.fromhex(base_str)
    crc = calculate_crc16(data_bytes)
    crc_hex = crc.to_bytes(2, byteorder='big', signed=False).hex().upper()
    full_str = base_str + crc_hex
    return bytes.fromhex(full_str)

def handle_zoom_control(data):
    """处理ZoomControl指令（fixed: 固定档位变焦 / dynamic: 动态变倍，按当前吊舱协议）"""
    global ZoomRatio, ZOOMAUTO
    try:
        zoom_data = int(data.get("ZoomData", -1))

        if _current_gimbal.get("zoom_mode") == "dynamic":
            # 先飞吊舱：按倍率生成动态变倍指令
            if zoom_data == 1:
                ZoomRatio += 1
                if ZoomRatio > 10:
                    ZoomRatio = 10
            elif zoom_data == 2:
                ZoomRatio -= 1
                if ZoomRatio < 1:
                    ZoomRatio = 1

            empty_cmd = _current_gimbal.get("empty_command")
            if empty_cmd is not None:
                send_gimbal_command(empty_cmd, "空指令（变倍前置）")
            zoom_cmd = generate_zoom_command(ZoomRatio * 250)
            send_gimbal_command(zoom_cmd, f"动态变倍指令（倍率{ZoomRatio}）")
            if empty_cmd is not None:
                send_gimbal_command(empty_cmd, "空指令（变倍后置）")
        else:
            # 千决吊舱：固定档位按键变焦，按下保持后发送停止指令
            send_single_hex_mqtt(ZOOM_CMD_MAP, "ZoomData", zoom_data)
            time.sleep(_current_gimbal.get("zoom_stop_delay", 1.5))
            send_single_hex_mqtt(ZOOM_CMD_MAP, "ZoomData", _current_gimbal.get("zoom_stop_cmd", 0))

        # ZoomAuto 自动变焦开关
        zoom_auto = int(data.get("ZoomAuto", -1))
        if zoom_auto == 1:
            ZOOMAUTO = True
        elif zoom_auto == 0:
            ZOOMAUTO = False

    except (ValueError, TypeError):
        print("[ZoomControl] ZoomData格式错误（需为整数）")

def handle_auto_save(data):
    """处理MQTT AutoSave指令"""
    global AUTOSAVE
    logger.info("=== 处理AutoSave指令 ===")
    try:
        auto_save = int(data.get("AutoSave", -1))
        if auto_save == 1:
            AUTOSAVE = True
        elif auto_save == 0:
            AUTOSAVE = False
        else:
            logger.error(f"[AutoSave] 格式错误")
    except (ValueError, TypeError):
        logger.error(f"[AutoSave] 格式错误")

def handle_work_model(data):
    """处理WorkModel指令"""
    global work_mode, ai_set_enabled, ID_Value, AUTOSAVE, ai_enabled_classes
    line_mission = False
    try:
        work_mode_val = int(data.get("WorkMode", -1))
        # **************新加判断flight************#
        if "flightId" in data:
            line_mission = True

        ID_Value = data.get("flightId", "")
        # **************新加判断flight************#
        if work_mode_val == 0:
            print("[WorkModel] 关闭AI模型（不绘制框）")
            AUTOSAVE = False
            with ai_lock:
                work_mode = 0
                ai_set_enabled = False
                ai_enabled_classes.clear()


        elif work_mode_val == 1:
            print("[WorkModel] 开启AI模型（可能需要额外的AiModel指令来设置具体行为）")
            if ID_Value and line_mission:  # 开启wokemode=1，有任务ID时且ID非空，自动下视并保存结果
                AUTOSAVE = True
                send_single_hex_mqtt(SERIAL_CMD_MAP, "SerialData", 5)
                time.sleep(_current_gimbal.get("down_stop_delay", 0))
                send_single_hex_mqtt(SERIAL_CMD_MAP, "SerialData", 6)
                # 下视完成后延时补发收尾指令（千决/先飞统一发8）
                time.sleep(_current_gimbal.get("down_stop_delay", 0))
                send_single_hex_mqtt(SERIAL_CMD_MAP, "SerialData", 8)
                global heading
                heading = yaw_now
            else:
                AUTOSAVE = False
            with ai_lock:
                work_mode = 1
                # AiSet 和 AiModel 的值需要通过 handle_ai_model 来设置
        elif work_mode_val == 2:
            print("[WorkModel] 开启点跟踪模式")
            # 实际应用中这里需要添加跟踪逻辑
        elif work_mode_val == 3:
            print("[WorkModel] 开启框选跟踪模式")
            # 实际应用中这里需要添加跟踪逻辑
        elif work_mode_val == 4:
            print("[WorkModel] 取消跟踪模式")
            # 发送取消跟踪指令（按当前吊舱协议，先飞吊舱需前置/后置空指令）
            cancel_cmd = _current_gimbal.get("cancel_command")
            if cancel_cmd is not None:
                empty_cmd = _current_gimbal.get("empty_command")
                if empty_cmd is not None:
                    send_gimbal_command(empty_cmd, "空指令（取消跟踪前置）")
                send_gimbal_command(cancel_cmd, "取消跟踪指令（WorkMode=4）")
                if empty_cmd is not None:
                    send_gimbal_command(empty_cmd, "空指令（取消跟踪后置）")
        else:
            print(f"[WorkModel] 不支持的WorkMode值: {work_mode_val}（仅支持2-4）")
    except (ValueError, TypeError):
        print("[WorkModel] WorkMode格式错误（需为整数）")


def handle_ai_model(data):
    """处理AiModel MQTT指令，控制目标识别类别"""
    global ai_enabled_classes, ai_set_enabled
    try:
        ai_model = int(data.get("AiModel", -1))
        ai_set = int(data.get("AiSet", -1))

        with ai_lock:
            if ai_model == 0:
                # 关闭所有目标框选
                ai_enabled_classes.clear()
                ai_set_enabled = False
                print("[AiModel] 关闭所有目标框选")

            else:
                if ai_model in AI_MODE_CONFIG:
                    target_classes = set(AI_MODE_CONFIG[ai_model])
                    if ai_set == 1:
                        # 开启：合并到当前集合
                        ai_enabled_classes.update(target_classes)
                        ai_set_enabled = True
                        print(f"[AiModel] 开启模式 {ai_model}: {sorted(target_classes)}")
                    elif ai_set == 0:
                        # 关闭：从当前集合中移除
                        ai_enabled_classes.difference_update(target_classes)
                        ai_set_enabled = (len(ai_enabled_classes) > 0)
                        print(f"[AiModel] 关闭模式 {ai_model}: {sorted(target_classes)}")
                    else:
                        print(f"[AiModel] AiSet 必须为 0 或 1，收到: {ai_set}")
                else:
                    print(f"[AiModel] 未知模式 AiModel={ai_model}（不在配置中）")


    except Exception as e:
        print(f"[AiModel] 处理异常：{str(e)}")


# ================== 核心新增：异步上传配置 ==================
# 1. 上传失败反馈函数
def send_upload_failure_mqtt(error_msg):
    """图片上传失败时发送MQTT反馈"""
    # 检查MQTT变量是否就绪
    if not (mqtt_client and FEEDBACK_TOPIC):
        logger.warning("[MQTT反馈] 变量未初始化，跳过失败反馈")
        return

    # 检查MQTT连接状态
    if not is_mqtt_connected:
        logger.warning("[MQTT反馈] 未连接，跳过失败反馈")
        return

    # 构造失败反馈消息
    failure_msg = {
        "flightId": ID_Value,
        "code": -1,
        "Msg": f"图片上传失败：{error_msg}",
        "fileUrl": "",
    }

    # 发布失败反馈
    failure_json = json.dumps(failure_msg, ensure_ascii=False)
    result = mqtt_client.publish(FEEDBACK_TOPIC, failure_json, qos=1)
    result.wait_for_publish(timeout=3)

    if result.rc == 0:
        logger.info(f"[MQTT反馈] 失败结果已发送 | 主题：{FEEDBACK_TOPIC} | 内容：{failure_json}")
    else:
        logger.error(f"[MQTT反馈] 失败反馈发布失败 | 错误码：{result.rc}")


# 2. 异步上传函数（无本地保存）
# ================== 新增：MinIO上传函数 ==================
def upload_image_minio(image_stream, image_name, detected_structs, timestamp_str, is_inf=False):
    global minio_client

    if minio_client is None:
        logger.error("[MinIO] MinIO客户端未初始化，无法上传")
        send_upload_failure_mqtt("MinIO客户端未初始化")
        return

    try:
        # 按来源选择存储目录（红外与可见光分开存储）
        folder = MINIO_CFG["folder_inf"] if is_inf else MINIO_CFG["folder"]
        # 构建完整的对象路径 (bucket/folder/filename)
        object_name = f"{folder}/{image_name}"

        # 使用 io.BytesIO 包装字节流
        stream = io.BytesIO(image_stream)

        # 上传文件
        minio_client.put_object(
            bucket_name=MINIO_CFG["bucket"],
            object_name=object_name,
            data=stream,  # 现在是文件对象，有 read() 方法
            length=len(image_stream),
            content_type='image/jpeg'
        )
        logger.info(f"[MinIO] 图片上传成功 | 对象: {object_name}")

        # 构造MQTT反馈消息
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        feedback_msg = {
            "flightId": ID_Value,
            "code": 200,
            "Msg": "Success,图片已成功上传至MinIO",
            "fileUrl": f"http://{MINIO_CFG['addr']}/{MINIO_CFG['bucket']}/{object_name}",
            "detectedTypes": detected_structs,
            "timestamp": timestamp_str
        }

        # 发布MQTT反馈
        feedback_json = json.dumps(feedback_msg, ensure_ascii=False)
        result = mqtt_client.publish(
            topic=FEEDBACK_TOPIC,
            payload=feedback_json,
            qos=1
        )
        result.wait_for_publish(timeout=3)

        if result.rc == 0:
            logger.info(f"[MQTT反馈] 成功发送 | 对象: {object_name}")
        else:
            logger.error(f"[MQTT反馈] 发布失败 | 错误码: {result.rc}")

    except S3Error as e:
        error_msg = f"MinIO S3错误: {str(e)}"
        logger.error(f"[MinIO] 上传失败: {error_msg}")
        send_upload_failure_mqtt(error_msg)
    except Exception as e:
        error_msg = f"上传异常: {str(e)}"
        logger.error(f"[MinIO] 上传异常: {error_msg}")
        send_upload_failure_mqtt(error_msg)


# ==================================================


# 3. 触发上传函数（无本地保存）
# ================== 修改：触发上传函数 ==================
def trigger_image_upload(detected_structs, img_save, capture_ts, target_id=None, is_inf=False):
    global LAST_UPLOAD_TIMESTAMP
    current_time = time.time()

    # 上传频率控制
    if current_time - LAST_UPLOAD_TIMESTAMP < UPLOAD_INTERVAL:
        logger.debug(f"[上传控制] 未超过{UPLOAD_INTERVAL}秒间隔，跳过上传")
        return
    LAST_UPLOAD_TIMESTAMP = current_time

    # 从 detected_structs 中提取类型名
    detected_types = [d["type"] for d in detected_structs]

    # 图像编码
    _, buf = cv2.imencode('.jpg', img_save)
    image_stream = buf.tobytes()

    # 文件名 (包含时间戳和检测类型)
    timestamp_name = datetime.fromtimestamp(capture_ts).strftime("%Y%m%d_%H%M%S_%f")[:-3]
    type_str = "_".join(detected_types) if detected_types else "unknown"
    # 追加目标ID数字（如 target_0012 -> 0012），区分同一帧的多个目标，避免MinIO同名覆盖
    id_suffix = f"_{target_id.split('_')[-1]}" if target_id else ""
    image_name = f"{timestamp_name}_{type_str}{id_suffix}.jpg"

    # 毫秒级时间戳字符串
    dt = datetime.fromtimestamp(capture_ts)
    timestamp_str = dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    # ===== 使用MinIO上传 =====
    UPLOAD_EXECUTOR.submit(
        upload_image_minio,
        image_stream=image_stream,
        image_name=image_name,
        detected_structs=detected_structs,
        timestamp_str=timestamp_str,
        is_inf=is_inf
    )

    # 按来源选择存储目录（红外与可见光分开存储）
    folder = MINIO_CFG["folder_inf"] if is_inf else MINIO_CFG["folder"]
    logger.info(f"[上传任务] 已提交 | MinIO对象: {folder}/{image_name} | targets={len(detected_structs)}")
# ==================================================


# ================== 位置计算相关函数（从location.py合并） ==================

def on_connect_location(client, userdata, flags, rc, properties):
    """MQTT连接成功回调函数 - 修正为5个参数以兼容Callback API版本2"""
    print(f"✅ 连接成功，返回码: {rc}")
    # 订阅无人机数据主题 - 修正主题名称，将QoS作为参数
    client.subscribe("drone/1581FBDBW256500A2NXJ/psdk/telemetry/pos", qos=0)


def on_message_location(client, userdata, msg):
    """MQTT消息接收回调函数 - 增强版"""
    try:
        # 1. 获取原始 payload 并解码
        payload_str = msg.payload.decode("utf-8")
        
        # 2. 解析 JSON
        data = json.loads(payload_str)
        global fused_lat, fused_lon, height_fusion, ntp_time
        # 3. 提取关键字段
        fused_lat = data.get('fused_position', {}).get('fused_lat')
        fused_lon = data.get('fused_position', {}).get('fused_lon')
        ntp_time = data.get('ntp_time')
        height_fusion = data.get('fused_position', {}).get('fused_height') 
        yaw = data.get('attitude', {}).get('yaw')
        pitch = data.get('attitude', {}).get('pitch')
        roll = data.get('attitude', {}).get('roll')

        global yaw_now, pitch_now, roll_now
        yaw_now = yaw
        pitch_now = pitch
        roll_now = roll

        if height_fusion is not None:
            # ===== 推入遥测数据缓冲用于时间匹配 =====
            if ntp_time is not None:
                # local_time: 妙算3本地时钟的 epoch 读数（同一时刻与 ntp_time 并存），
                # 供红外帧时间（妙算3系，来自其MediaMTX的Date）直接匹配，免疫两机时钟漂移
                local_time_raw = data.get('local_time')
                local_time_ms = None
                if isinstance(local_time_raw, (int, float)) and local_time_raw > 1e9:
                    local_time_ms = int(local_time_raw * 1000) if local_time_raw < 1e12 else int(local_time_raw)
                with drone_buffer_lock:
                    drone_data_buffer.append({
                        "ntp_time": ntp_time,
                        "local_time_ms": local_time_ms,
                        "lat": fused_lat,
                        "lon": fused_lon,
                        "height": height_fusion,
                        "yaw": yaw_now,
                        "pitch": pitch_now,
                        "roll": roll_now
                    })
                # loc_local_now = datetime.now(timezone.utc).astimezone(TZ_CST).strftime("%H:%M:%S.%f")[:-3]
                # ntp_dt = datetime.fromtimestamp(ntp_time / 1000, tz=timezone.utc).astimezone(TZ_CST)
                # ntp_local = ntp_dt.strftime("%H:%M:%S.%f")[:-3]
                # print(f"[定位时间] 定位ntp={ntp_local}  本地到达={loc_local_now}")
            #print("\n📊 无人机状态数据:")
            #print(f"  高度: **{height_fusion}米**")
            #print(f"  航向角: **{yaw}°**")
            #print(f"  纬度: **{fused_lat}°**")
            #print(f"  经度: **{fused_lon}°**")
        else:
            print("⚠️ 收到的消息中缺少位置信息")

    except json.JSONDecodeError:
        print(f"❌ JSON 解析错误: {payload_str}")
    except Exception as e:
        print(f"❌ 处理消息时出错: {e}")


def compute_gimbal_position(rtk_lat_deg, rtk_lon_deg, rtk_height_agl,
                            yaw_deg, pitch_deg, roll_deg, lever_arm=None):
    """
    计算吊舱（相机）的实际地理位置。

    RTK天线与吊舱的安装位置不在一起，两者之间存在固定的机体坐标系安装偏移（lever_arm）。
    该偏移会随无人机姿态变化，需先用机体系->地理系（NED）旋转矩阵把偏移旋转到地理坐标，
    再叠加到RTK天线位置，得到吊舱的真实经纬度和离地高度，供后续像素坐标->GPS换算使用。

    机体坐标系约定（与 pixel_to_gps_with_yaw 保持一致）：
        X轴：机头方向（前）   Y轴：右翼方向（右）   Z轴：垂直机身向下
    姿态角约定（与无人机遥测一致）：
        yaw   : 航向角，0°=正北，顺时针（向右转）为正
        pitch : 俯仰角，机头向上为正，向下为负
        roll  : 横滚角，右翼向下为正，左翼向下为负

    参数：
        rtk_lat_deg / rtk_lon_deg : RTK天线经纬度（十进制度）
        rtk_height_agl            : RTK天线离地高度（米）
        yaw_deg / pitch_deg / roll_deg : 无人机姿态角（度）
        lever_arm                 : {'forward_m','right_m','down_m'} 偏移字典（米），默认用全局 LEVER_ARM

    返回：
        (gimbal_lat, gimbal_lon, gimbal_height_agl)
    """
    if lever_arm is None:
        lever_arm = LEVER_ARM
    fwd = float(lever_arm.get("forward_m", 0.0))   # 机头方向偏移（米）
    right = float(lever_arm.get("right_m", 0.0))   # 右翼方向偏移（米）
    down = float(lever_arm.get("down_m", 0.0))     # 垂直向下偏移（米）

    # 偏移全为0时直接返回RTK位置（吊舱与天线重合，保持原逻辑）
    if fwd == 0.0 and right == 0.0 and down == 0.0:
        return rtk_lat_deg, rtk_lon_deg, rtk_height_agl

    yaw_rad = math.radians(yaw_deg)
    pitch_rad = math.radians(pitch_deg)
    roll_rad = math.radians(roll_deg)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)

    # 机体系 -> NED 旋转（R = Rz(yaw) * Ry(pitch) * Rx(roll)）
    # N: 北向偏移（米），E: 东向偏移（米），D: 下向偏移（米，向下为正）
    N = (cy * cp) * fwd + (cy * sp * sr - sy * cr) * right + (cy * sp * cr + sy * sr) * down
    E = (sy * cp) * fwd + (sy * sp * sr + cy * cr) * right + (sy * sp * cr - cy * sr) * down
    D = (-sp) * fwd + (cp * sr) * right + (cp * cr) * down

    # 局部切平面近似：EN偏移转为经纬度增量
    R_earth = 6378137.0
    lat_rad = math.radians(rtk_lat_deg)
    delta_lat = (N / R_earth) * (180.0 / math.pi)
    delta_lon = (E / (R_earth * math.cos(lat_rad))) * (180.0 / math.pi)

    gimbal_lat = rtk_lat_deg + delta_lat
    gimbal_lon = rtk_lon_deg + delta_lon
    # D>0 表示吊舱在RTK天线下方，离地高度更低
    gimbal_height = rtk_height_agl - D

    return gimbal_lat, gimbal_lon, gimbal_height


def pixel_to_gps_with_yaw(
    drone_lat_deg,
    drone_lon_deg,
    drone_height_agl,
    target_height_agl,
    focal_length_mm,
    pixel_size_mm,
    image_width,
    image_height,
    pixel_u,
    pixel_v,
    drone_yaw_deg,
    drone_pitch_deg=0.0,
    drone_roll_deg=0.0,
    camera_pitch_deg=0.0
):
    """
    将图像中的像素坐标转换为地理 GPS 坐标（WGS84），考虑无人机完整姿态（yaw/pitch/roll）和相机倾斜角。
    
    参数说明：
    ----------
    drone_lat_deg : float
        无人机纬度（十进制度，北纬为正）
    drone_lon_deg : float
        无人机经度（十进制度，东经为正）
    drone_height_agl : float
        无人机离地高度（米）
    target_height_agl : float
        光伏板中心点离地高度（米）
    focal_length_mm : float
        相机焦距（毫米）
    pixel_size_mm : float
        单个像素物理尺寸（毫米/像素）
    image_width : int
        图像宽度（像素）
    image_height : int
        图像高度（像素）
    pixel_u : float or int
        隐患点在图像中的 x 坐标（列，原点左上角）
    pixel_v : float or int
        隐患点在图像中的 y 坐标（行，原点左上角）
    drone_yaw_deg : float
        无人机偏航角（航向角），单位：度
        - 0° = 正北，90° = 正东，180° = 正南，270° = 正西
    drone_pitch_deg : float
        无人机俯仰角，单位：度
        - 机头向上为正，向下为负
    drone_roll_deg : float
        无人机横滚角，单位：度
        - 右翼向下为正，左翼向下为负
    camera_pitch_deg : float
        相机相对于无人机的倾斜角，朝机头方向倾斜为正（单位：度）
        - 0° = 相机与无人机机体系垂直
        - 10° = 相机朝机头方向倾斜10度（向下俯视时）

    返回值：
    -------
    tuple (target_lat, target_lon)
        目标点的 GPS 坐标（十进制度）
    """
    
    # 高度差
    height_diff = drone_height_agl - target_height_agl
    
    # 将角度转换为弧度
    pitch_rad = math.radians(drone_pitch_deg)
    roll_rad = math.radians(drone_roll_deg)
    yaw_rad = math.radians(drone_yaw_deg)
    cam_pitch_rad = math.radians(camera_pitch_deg)

    # 焦距转换为像素单位
    focal_length_pix = focal_length_mm / pixel_size_mm

    # 主点（图像中心）
    cx = image_width / 2.0
    cy = image_height / 2.0

    # 像素偏移（以图像中心为原点）
    delta_u = pixel_u - cx      # 图像右为正
    delta_v = pixel_v - cy      # 图像下为正

    # 相机坐标系到无人机机体系的转换
    # 相机光轴指向：向下为z轴，右为x轴，前为y轴（考虑相机安装倾角）
    # 完整姿态变换：相机安装角 -> 无人机姿态 -> 地理坐标系
    
    # 第一步：计算有效距离和GSD（考虑综合倾斜角）
    # 综合倾斜角 = 无人机俯仰角 + 相机安装角
    total_pitch_rad = pitch_rad + cam_pitch_rad
    effective_distance = height_diff / math.cos(total_pitch_rad)
    
    # GSD: Ground Sampling Distance (米/像素)
    gsd = effective_distance / focal_length_pix

    # 第二步：像素偏移到机体系坐标（考虑相机安装角）
    # 相机朝机头倾斜时，图像下方对应地面更远点
    offset_x_body = delta_u * gsd  # 机体系右向
    # 实测标定：吊舱垂直朝地时，图像下方对应机尾方向（与机体前向相反），故取负号
    offset_y_body = -delta_v * gsd  # 机体系前向（图像下方为机尾，取负）
    
    # 相机安装角导致的投影中心偏移
    cam_offset = height_diff * math.tan(cam_pitch_rad)
    offset_y_body -= cam_offset

    # 第三步：无人机机体系到地理坐标系（考虑roll影响）
    # 横滚角会导致左右偏移产生纵向分量
    # 完整的3D姿态变换：先roll，再pitch，最后yaw
    
    # 机体系偏移（前、右、下）
    x_body = offset_x_body  # 右
    y_body = offset_y_body  # 前
    z_body = -height_diff   # 下（负号表示向下）
    
    # 考虑横滚角的影响：左右偏移会因roll产生前向分量
    # 当无人机有roll时，机体系的x轴（右）不再水平
    # x_body在水平面上的投影 = x_body * cos(roll)
    # x_body产生的垂直分量 = x_body * sin(roll)
    
    # 修正后的水平偏移（考虑roll）
    ground_offset_x = x_body * math.cos(roll_rad)
    ground_offset_y = y_body + x_body * math.sin(roll_rad) * math.tan(total_pitch_rad)

    # 第四步：将机体系水平偏移旋转到地理坐标系（N-E）
    delta_N = ground_offset_y * math.cos(yaw_rad) - ground_offset_x * math.sin(yaw_rad)  # 北向
    delta_E = ground_offset_y * math.sin(yaw_rad) + ground_offset_x * math.cos(yaw_rad)  # 东向

    # 地球半径（WGS84 赤道半径，单位：米）
    R_earth = 6378137.0

    # 将 EN 偏移转换为经纬度偏移（局部切平面近似）
    lat_rad = math.radians(drone_lat_deg)
    delta_lat_deg = (delta_N / R_earth) * (180.0 / math.pi)
    delta_lon_deg = (delta_E / (R_earth * math.cos(lat_rad))) * (180.0 / math.pi)

    # 计算目标 GPS 坐标
    target_lat = drone_lat_deg + delta_lat_deg
    target_lon = drone_lon_deg + delta_lon_deg

    return target_lat, target_lon

def locatin(capture_ts):
    dt = datetime.fromtimestamp(capture_ts)
    times_image = dt.strftime("%Y-%m-%d %H:%M:%S.%f")


# ================== 摄像头 UTC 时间获取（从 rtsp_pts.py 移植） ==================

def _http_date_poll(host, port):
    """发送 HTTP GET 请求，返回 (parsed_datetime, local_before, local_after) 或 None"""
    try:
        local_before = datetime.now(timezone.utc)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(1)
        s.connect((host, port))
        s.send(f"GET / HTTP/1.0\r\nHost: {host}\r\n\r\n".encode())
        resp = s.recv(1024).decode("utf-8", errors="replace")
        s.close()
        local_after = datetime.now(timezone.utc)
        m = re.search(r"Date:\s*(.+)", resp)
        if not m:
            return None
        dt = parsedate_to_datetime(m.group(1).strip())
        if dt is None:
            return None
        return dt, local_before, local_after
    except Exception:
        return None


def _resolve_tz(cam_date, local_now):
    """判断 HTTP Date 是 UTC 还是 UTC+8，返回校正后的 UTC datetime"""
    cam_utc = cam_date
    cam_cst = cam_date - timedelta(hours=8)
    if abs((cam_cst - local_now).total_seconds()) < abs((cam_utc - local_now).total_seconds()):
        return cam_cst, "UTC+8"
    return cam_utc, "UTC"


def get_camera_utc(host, port=80):
    """获取摄像机 UTC 时间，HTTP Date 边界对齐（<1ms），回退到单次请求"""
    try:
        prev_second = None
        deadline = datetime.now(timezone.utc) + timedelta(seconds=2)
        logs = []

        while datetime.now(timezone.utc) < deadline:
            result = _http_date_poll(host, port)
            if result is None:
                break
            dt, local_before, local_after = result

            if prev_second is not None and dt.second != prev_second:
                cam_utc, tz = _resolve_tz(dt, local_before)
                logs.append((dt, local_before, local_after, True))
                prev_local = logs[-2][1]
                boundary_local = prev_local + (local_before - prev_local) / 2
                offset = (cam_utc.replace(microsecond=0) - boundary_local).total_seconds()
                cam_utc = local_after + timedelta(seconds=offset)
                return cam_utc, f"HTTP(高精度-{tz})"

            logs.append((dt, local_before, local_after, False))
            prev_second = dt.second

        if prev_second is not None:
            cam_utc, tz = _resolve_tz(dt, local_before)
            return cam_utc, f"HTTP(秒级-{tz})"
    except Exception:
        pass

    try:
        result = _http_date_poll(host, port)
        if result is not None:
            dt, local_before, _ = result
            cam_utc, tz = _resolve_tz(dt, local_before)
            return cam_utc, tz
    except Exception:
        pass
    return None, None


def parse_frame_info(line: str):
    """解析 showinfo 滤镜输出的帧信息行"""
    match = re.search(r"pts:\s*(\d+)\s+pts_time:\s*([\d.]+)", line)
    if match:
        return int(match.group(1)), float(match.group(2))
    return None


def match_drone_data(frame_cam_utc, max_diff_ms=500, use_local_time=False):
    """
    从无人机遥测缓冲中查找与帧摄像头时间最接近的数据
    Args:
        frame_cam_utc: 帧的摄像头 UTC 时间 (datetime)
        max_diff_ms: 最大允许时间差（毫秒），超过此值返回 None
        use_local_time: True=用妙算3本地时钟(local_time)匹配（红外帧时间在妙算3系，其时钟无法NTP同步但每条遥测自带local_time，逐条跟踪漂移）
                        False=用ntp_time匹配（可见光帧时间在Jetson系）
    Returns:
        匹配到的数据 dict 或 None
    """
    if frame_cam_utc is None:
        return None
    frame_ms = int(frame_cam_utc.timestamp() * 1000)

    with drone_buffer_lock:
        if not drone_data_buffer:
            return None
        if use_local_time:
            candidates = [d for d in drone_data_buffer if d.get("local_time_ms") is not None]
            if not candidates:
                return None  # 遥测尚未携带local_time（妙算3端未升级），红外无法在妙算3系匹配
            closest = min(candidates, key=lambda d: abs(d["local_time_ms"] - frame_ms))
            diff = abs(closest["local_time_ms"] - frame_ms)
        else:
            closest = min(drone_data_buffer, key=lambda d: abs(d["ntp_time"] - frame_ms))
            diff = abs(closest["ntp_time"] - frame_ms)
        if diff > max_diff_ms:
            return None
        return closest


def parse_rtsp_host(rtsp_url):
    """从 RTSP URL 中提取主机地址"""
    m = re.match(r"rtsp://(?:[^:@]+(?::[^@]+)?@)?([^:/]+)", rtsp_url)
    return m.group(1) if m else None


# ================== 目标追踪器辅助函数 ==================
def calculate_iou(box1, box2):
    """
    计算两个检测框的IoU（交并比）
    box格式: [x1, y1, x2, y2]
    """
    x1_1, y1_1, x2_1, y2_1 = box1
    x1_2, y1_2, x2_2, y2_2 = box2
    
    # 计算交集区域
    inter_x1 = max(x1_1, x1_2)
    inter_y1 = max(y1_1, y1_2)
    inter_x2 = min(x2_1, x2_2)
    inter_y2 = min(y2_1, y2_2)
    
    inter_area = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)
    
    # 计算并集区域
    area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
    area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
    union_area = area1 + area2 - inter_area
    
    if union_area == 0:
        return 0.0
    return inter_area / union_area

# ================== SORT 算法实现 ==================

class KalmanBoxTracker:
    """
    使用卡尔曼滤波器跟踪单个目标的边界框
    状态向量: [x, y, w, h, vx, vy, vw, vh]
    其中 (x,y) 是框中心，(w,h) 是宽高，v* 是对应的速度
    """
    
    def __init__(self, box, class_name):
        # 初始化卡尔曼滤波器
        self.dt = 1.0 / TARGET_FPS  # 时间步长
        
        # 状态转移矩阵 A (8x8)
        self.A = np.array([
            [1, 0, 0, 0, self.dt, 0, 0, 0],
            [0, 1, 0, 0, 0, self.dt, 0, 0],
            [0, 0, 1, 0, 0, 0, self.dt, 0],
            [0, 0, 0, 1, 0, 0, 0, self.dt],
            [0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 1]
        ])
        
        # 观测矩阵 H (4x8) - 只观测位置，不观测速度
        self.H = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0]
        ])
        
        # 过程噪声协方差 Q（速度项加大，快速移动目标时预测更跟手）
        self.Q = np.diag([1, 1, 1, 1, 100, 100, 100, 100]) * 0.01
        
        # 观测噪声协方差 R
        self.R = np.diag([1, 1, 1, 1]) * 1.0
        
        # 状态协方差矩阵 P
        # 位置-速度交叉协方差设为 100：若为纯对角，速度前几帧估计不出来，
        # 预测框会滞后于快速移动的目标，导致 IoU 匹配失败、ID 频繁新建
        self.P = np.array([
            [1000, 0, 0, 0, 100, 0, 0, 0],
            [0, 1000, 0, 0, 0, 100, 0, 0],
            [0, 0, 1000, 0, 0, 0, 100, 0],
            [0, 0, 0, 1000, 0, 0, 0, 100],
            [100, 0, 0, 0, 1000, 0, 0, 0],
            [0, 100, 0, 0, 0, 1000, 0, 0],
            [0, 0, 100, 0, 0, 0, 1000, 0],
            [0, 0, 0, 100, 0, 0, 0, 1000]
        ])
        
        # 初始状态：从检测框提取中心坐标和宽高
        x1, y1, x2, y2 = box
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        w = x2 - x1
        h = y2 - y1
        
        # 初始状态向量：位置 + 速度（速度初始化为0）
        self.x = np.array([cx, cy, w, h, 0, 0, 0, 0])
        
        # 目标类别
        self.class_name = class_name
        
        # 追踪状态
        self.time_since_update = 0  # 自上次更新以来的帧数
        self.hit_streak = 0         # 连续匹配到的帧数
        self.id = None              # 正式目标ID，hit_streak >= SORT_MIN_HITS 后才分配（减少误检导致的ID抖动）
    
    def predict(self):
        """
        卡尔曼滤波预测步骤：根据当前状态预测下一帧状态
        """
        # x = A * x
        self.x = self.A @ self.x
        
        # P = A * P * A^T + Q
        self.P = self.A @ self.P @ self.A.T + self.Q
        
        self.time_since_update += 1
    
    def update(self, box):
        """
        卡尔曼滤波更新步骤：根据新的检测框更新状态
        """
        # 提取观测值（中心坐标和宽高）
        x1, y1, x2, y2 = box
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        w = x2 - x1
        h = y2 - y1
        z = np.array([cx, cy, w, h])
        
        # 计算残差
        y = z - self.H @ self.x
        
        # 计算残差协方差
        S = self.H @ self.P @ self.H.T + self.R
        
        # 计算卡尔曼增益
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # 更新状态
        self.x = self.x + K @ y
        
        # 更新协方差
        I = np.eye(8)
        self.P = (I - K @ self.H) @ self.P
        
        self.time_since_update = 0
        self.hit_streak += 1
    
    def get_state(self):
        """
        获取当前预测的边界框（转换为x1,y1,x2,y2格式）
        """
        cx, cy, w, h = self.x[:4]
        return [cx - w/2, cy - h/2, cx + w/2, cy + h/2]


class SortTracker:
    """
    SORT (Simple Online and Realtime Tracking) 追踪器主类
    使用卡尔曼滤波进行状态预测和匈牙利算法进行数据关联
    """
    
    def __init__(self, id_prefix="target"):
        self.trackers = []  # 活跃的卡尔曼跟踪器列表
        self.id_prefix = id_prefix  # 目标ID前缀（红外流用 target_inf 区分来源）
    
    def update(self, detections, class_names):
        """
        更新跟踪器状态
        
        参数:
            detections: 检测框列表，格式为 [[x1,y1,x2,y2], ...]
            class_names: 对应检测框的类别名称列表
        
        返回:
            matches: 匹配结果列表，格式为 [(detection_idx, tracker), ...]
            unmatched_detections: 未匹配的检测索引列表
        """
        # 1. 对所有现有跟踪器进行预测
        for tracker in self.trackers:
            tracker.predict()
        
        # 2. 获取所有跟踪器的预测框
        tracker_boxes = [t.get_state() for t in self.trackers]
        
        # 3. 构建代价矩阵（基于IoU）
        num_detections = len(detections)
        num_trackers = len(self.trackers)
        
        if num_detections == 0:
            # 没有检测：保留跟踪器继续老化，靠 MAX_AGE 自然淘汰。
            # 不能清空，否则目标只要漏检一帧，跟踪器就全部丢失，下一帧重现时会分配新ID
            self.trackers = [t for t in self.trackers if t.time_since_update <= SORT_MAX_AGE]
            return [], []
        
        if num_trackers == 0:
            # 没有跟踪器，所有检测都是新的
            for i in range(num_detections):
                new_tracker = KalmanBoxTracker(detections[i], class_names[i])
                self.trackers.append(new_tracker)
            return [], list(range(num_detections))
        
        # 计算每个检测-跟踪器对的IoU和中心距离
        iou_matrix = np.zeros((num_detections, num_trackers))
        center_dist_matrix = np.full((num_detections, num_trackers), np.inf)
        for i, det_box in enumerate(detections):
            det_cx = (det_box[0] + det_box[2]) / 2.0
            det_cy = (det_box[1] + det_box[3]) / 2.0
            for j, track_box in enumerate(tracker_boxes):
                # 只有类别相同时才计算IoU
                if class_names[i] != self.trackers[j].class_name:
                    iou_matrix[i, j] = 0.0
                else:
                    iou_matrix[i, j] = calculate_iou(det_box, track_box)
                trk_cx = (track_box[0] + track_box[2]) / 2.0
                trk_cy = (track_box[1] + track_box[3]) / 2.0
                center_dist_matrix[i, j] = math.hypot(det_cx - trk_cx, det_cy - trk_cy)

        # 4. 两阶段匹配：已确认跟踪器优先，未确认跟踪器其次
        #    避免"新建跟踪器IoU天然更高"顶替掉已确认跟踪器，造成ID漂移
        def _match_stage(det_indices, tracker_indices):
            """单阶段匈牙利匹配，返回 (matches, used_rows)"""
            stage_matches = []
            used_rows = set()
            if det_indices and tracker_indices:
                sub_cost = np.zeros((len(det_indices), len(tracker_indices)))
                for a, i in enumerate(det_indices):
                    for b, j in enumerate(tracker_indices):
                        sub_cost[a, b] = 1 - iou_matrix[i, j]  # IoU越大，代价越小
                r_idx, c_idx = linear_sum_assignment(sub_cost)
                for a, b in zip(r_idx, c_idx):
                    i = det_indices[a]
                    j = tracker_indices[b]
                    det_box = detections[i]
                    det_max_side = max(det_box[2] - det_box[0], det_box[3] - det_box[1])
                    # 接受条件：IoU达标，或两者有交集且中心距离在目标自身尺寸范围内
                    # （兜底快速移动目标：预测框错位导致IoU低但实际是同一目标）
                    if (iou_matrix[i, j] > SORT_IOU_THRESHOLD or
                            (iou_matrix[i, j] > 0 and
                             center_dist_matrix[i, j] <= SORT_CENTER_DIST_FACTOR * det_max_side)):
                        stage_matches.append((i, self.trackers[j]))
                        used_rows.add(i)
            return stage_matches, used_rows

        all_det_indices = list(range(num_detections))
        confirmed_indices = [j for j in range(num_trackers)
                             if self.trackers[j].hit_streak >= SORT_MIN_HITS]
        pending_indices = [j for j in range(num_trackers) if j not in confirmed_indices]

        matches = []
        used_rows = set()

        # 阶段1：检测 -> 已确认跟踪器（优先续命，保持ID稳定）
        m1, r1 = _match_stage(all_det_indices, confirmed_indices)
        matches.extend(m1)
        used_rows.update(r1)

        # 阶段2：剩余检测 -> 未确认跟踪器（继续累积确认帧数）
        remaining_dets = [i for i in all_det_indices if i not in used_rows]
        m2, r2 = _match_stage(remaining_dets, pending_indices)
        matches.extend(m2)
        used_rows.update(r2)

        # 5. 更新所有已匹配的跟踪器
        for i, tracker in matches:
            tracker.update(detections[i])
            # 连续确认帧数达标后才分配正式ID
            if tracker.id is None and tracker.hit_streak >= SORT_MIN_HITS:
                tracker.id = generate_target_id(self.id_prefix)

        # 6. 找出未匹配的检测
        unmatched_detections = [i for i in all_det_indices if i not in used_rows]

        # 7. 移除长时间未更新的跟踪器
        # 注意：即使未匹配也要保留跟踪器一段时间（通过MAX_AGE控制）
        self.trackers = [t for t in self.trackers if t.time_since_update <= SORT_MAX_AGE]

        # 8. 为未匹配的检测创建新跟踪器
        for i in unmatched_detections:
            new_tracker = KalmanBoxTracker(detections[i], class_names[i])
            self.trackers.append(new_tracker)

        return matches, unmatched_detections

# ================== 全局 SORT 追踪器实例 ==================
sort_tracker = SortTracker()                            # 可见光目标追踪器
sort_tracker_inf = SortTracker(id_prefix="target_inf")  # 红外目标追踪器（与可见光像素坐标系不同，必须独立跟踪）

# 目标ID计数器锁（可见光/红外线程共用计数器，需保证ID唯一）
target_id_lock = threading.Lock()

# 红外目标收集器（结构同 target_tracker，与可见光隔离）
target_tracker_inf = {}
target_tracker_inf_lock = threading.Lock()

def generate_target_id(prefix="target"):
    """生成唯一的目标ID（线程安全），红外流使用 target_inf 前缀区分"""
    global target_id_counter
    with target_id_lock:
        target_id_counter += 1
        return f"{prefix}_{target_id_counter:04d}"

def match_target(box, class_name, iou_threshold=0.3):
    """
    在追踪器中查找匹配的目标
    返回匹配的目标ID，如果没有匹配则返回None
    注意：已上传的目标也可以匹配（用于识别同一目标的后续检测）
    """
    global target_tracker
    
    with target_tracker_lock:
        best_iou = 0.0
        best_target_id = None
        
        for target_id, data in target_tracker.items():
            # 只匹配相同类别的目标
            if data['class_name'] != class_name:
                continue
            
            # 跳过正在收集中的目标（这些目标还没有上传，继续等待）
            if data['status'] == TargetStatus.COLLECTING:
                continue
            
            # 已上传的目标也可以匹配（用于识别同一目标的后续检测）
            # 计算IoU
            iou = calculate_iou(box, data['last_box'])
            
            if iou > best_iou and iou >= iou_threshold:
                best_iou = iou
                best_target_id = target_id
        
        return best_target_id

def update_target(target_id, box, cx, cy, cam_utc, frame, class_name, current_time,
                  tracker=None, lock=None):
    """
    更新目标信息或创建新目标

    收集逻辑：时间窗口模式
    - 目标第一次出现的帧为时间窗口起点
    - 在接下来的连续5帧时间窗口内，收集所有检测到的目标像素坐标
    - 时间窗口结束后立即触发上传，上传前在 upload_worker 中延迟 1 秒
      匹配无人机遥测数据后计算精确 GPS
    - 已上传的目标再次被检测到时，使用已保存的平均坐标，不重新上传

    参数：
      cx, cy  — 目标中心像素坐标（用于后续 GPS 计算）
      cam_utc — 当前帧的摄像头 UTC 时间（datetime，用于匹配无人机数据）
      tracker / lock — 目标收集器及其锁（默认可见光 target_tracker，红外流传入独立实例）
    """
    global target_tracker, upload_queue

    # 未指定时默认使用可见光目标收集器（保持原有调用兼容）
    if tracker is None:
        tracker = target_tracker
    if lock is None:
        lock = target_tracker_lock

    is_new = False
    ready_to_upload = False
    use_saved_avg = False  # 是否使用已保存的平均坐标

    frame_interval = 1.0 / TARGET_FPS  # 单帧间隔时间（秒）
    time_window = TARGET_CACHE_SIZE * frame_interval  # 时间窗口大小

    # 先复制帧（在锁外执行，避免锁持有时间过长）
    frame_copy = frame.copy() if frame is not None else None

    with lock:
        # 检查目标是否已存在于 tracker 中
        if target_id and target_id in tracker:
            # 更新现有目标
            target = tracker[target_id]
            
            # 更新时间
            target['last_seen'] = current_time
            target['last_box'] = box  # 更新检测框位置，用于后续IoU匹配
            
            if target['status'] == TargetStatus.COLLECTING:
                # 正在收集中的目标，继续收集坐标
                # 检查是否在时间窗口内
                if current_time - target['window_start_time'] < time_window:
                    # 在时间窗口内，继续收集
                    # 添加当前像素坐标
                    target['pixel_coords'].append((cx, cy))
                    target['detected_count'] += 1
                    
                    # 更新平均像素坐标
                    pixel_coords = target['pixel_coords']
                    avg_cx = sum(c[0] for c in pixel_coords) / len(pixel_coords)
                    avg_cy = sum(c[1] for c in pixel_coords) / len(pixel_coords)
                    target['avg_cx'] = avg_cx
                    target['avg_cy'] = avg_cy
                    
                    # print(f"[TRACKER] 目标 {target_id}, 时间窗口内已检测 {target['detected_count']} 次")
            
            elif target['status'] == TargetStatus.UPLOADED:
                # 已上传的目标再次被检测到，使用已保存的平均坐标
                use_saved_avg = True
                #print(f"[TRACKER] 目标 {target_id} 再次检测到，使用已保存的平均坐标 ({target['avg_lat']:.10f}, {target['avg_lon']:.10f})")
        
        else:
            # 创建新目标（目标第一次出现）
            if target_id is None:
                target_id = generate_target_id()
            tracker[target_id] = {
                'first_frame': frame_copy,  # 使用预先复制的帧
                'window_start_time': current_time,  # 时间窗口开始时间（第一帧出现的时间）
                'last_box': box,
                'last_seen': current_time,
                'class_name': class_name,
                'status': TargetStatus.COLLECTING,
                'pixel_coords': [(cx, cy)],  # 时间窗口内收集到的所有像素坐标
                'detected_count': 1,  # 时间窗口内检测到目标的帧数
                'avg_cx': cx,
                'avg_cy': cy,
                'cam_utc': cam_utc  # 第一帧的摄像头UTC时间
            }
            is_new = True
            # print(f"[TRACKER] 新目标: {target_id} ({class_name}), 时间窗口开始")
    
    return (is_new, ready_to_upload, use_saved_avg)

def check_target_timeout(target_id, current_time, tracker=None, lock=None, is_inf=False):
    """
    检查目标是否应该触发上传（时间窗口结束）
    这个函数需要在每一帧（无论是否检测到目标）调用

    tracker / lock : 目标收集器及其锁（默认可见光 target_tracker，红外流传入独立实例）
    is_inf         : 是否红外流任务（upload_worker 据此选择红外相机参数定位）
    """
    global target_tracker, upload_queue

    if tracker is None:
        tracker = target_tracker
    if lock is None:
        lock = target_tracker_lock

    frame_interval = 1.0 / TARGET_FPS
    time_window = TARGET_CACHE_SIZE * frame_interval

    # 先获取目标信息（短时间持有锁）
    target_data = None
    with lock:
        target = tracker.get(target_id)
        if not target:
            return False

        if target['status'] in [TargetStatus.UPLOADED, TargetStatus.EXPIRED]:
            return False

        # 检查时间窗口是否结束
        if current_time - target['window_start_time'] >= time_window:
            # 准备上传数据（不包含GPS，upload_worker延迟计算）
            target_data = {
                'target_id': target_id,
                'class_name': target['class_name'],
                'avg_cx': target['avg_cx'],
                'avg_cy': target['avg_cy'],
                'cam_utc': target['cam_utc'],
                'first_frame': target['first_frame'],
                'detected_count': target['detected_count'],
                'is_inf': is_inf
            }
            target['status'] = TargetStatus.EXPIRED

    # 在锁外处理上传队列（避免锁嵌套）
    if target_data:
        with upload_queue_lock:
            if len(upload_queue) < UPLOAD_QUEUE_MAX_SIZE:
                upload_queue.append(target_data)

        # 更新目标状态为已上传
        with lock:
            target = tracker.get(target_id)
            if target:
                target['status'] = TargetStatus.UPLOADED
                # print(f"[TRACKER] 目标 {target_id} 时间窗口结束，共检测 {target_data['detected_count']} 次，推送上传队列")

        return True

    return False

def cleanup_stale_targets(tracker=None, lock=None):
    """清理超时的目标（默认可见光收集器，红外流传入独立实例）"""
    global target_tracker
    current_time = time.time()
    frame_interval = 1.0 / TARGET_FPS
    time_window = TARGET_CACHE_SIZE * frame_interval

    if tracker is None:
        tracker = target_tracker
    if lock is None:
        lock = target_tracker_lock

    with lock:
        stale_ids = []
        for target_id, data in tracker.items():
            # 清理：已上传且超过 TARGET_TIMEOUT 秒未再检测到的
            if data['status'] == TargetStatus.UPLOADED and current_time - data['last_seen'] > TARGET_TIMEOUT:
                stale_ids.append(target_id)
            # 清理：正在收集但超过时间窗口很久的（防止内存泄漏）
            elif data['status'] == TargetStatus.COLLECTING:
                if current_time - data['window_start_time'] > time_window * 5:
                    stale_ids.append(target_id)
                    # print(f"[TRACKER] 目标 {target_id} 时间窗口超时，已检测 {data['detected_count']} 次，清理")

        for target_id in stale_ids:
            del tracker[target_id]

_LOCATION_LOG_HEADER = (
    "record_time,target_id,matched,cam_utc,ntp_time,"
    "rtk_lat,rtk_lon,rtk_height_m,yaw_deg,pitch_deg,roll_deg,"
    "gimbal_lat,gimbal_lon,gimbal_height_m,"
    "target_lat,target_lon,pixel_u,pixel_v"
)


def write_location_log(target_id, matched_flag, cam_utc, ntp_time_ms,
                       rtk_lat, rtk_lon, rtk_height, yaw_deg, pitch_deg, roll_deg,
                       gimbal_lat, gimbal_lon, gimbal_height,
                       target_lat, target_lon, pixel_u, pixel_v):
    """将一次定位的计算输入输出追加写入CSV日志（location_debug.csv），便于排查定位偏差。

    字段说明：
      record_time    : 记录时间（CST）
      target_id      : 目标ID
      matched        : 1=匹配到遥测数据，0=回退全局位置
      cam_utc        : 帧的摄像头UTC时间（CST）
      ntp_time       : 匹配到的无人机遥测时间（CST，未匹配为空），与cam_utc格式一致
      rtk_*/yaw/pitch/roll : 计算使用的无人机RTK位置与姿态
      gimbal_*       : 杆臂修正后的吊舱位置
      target_lat/lon : 最终隐患定位结果
      pixel_u/v      : 隐患点在图像中的平均像素坐标
    """
    def fmt(v, spec=".8f"):
        try:
            return format(v, spec)
        except (TypeError, ValueError):
            return ""

    def fmt_time_ms(ms):
        """毫秒时间戳转 CST 时间字符串（与cam_utc格式一致），None返回空"""
        if ms is None:
            return ""
        try:
            return datetime.fromtimestamp(ms / 1000.0, tz=timezone.utc).astimezone(TZ_CST).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        except (TypeError, ValueError, OSError):
            return str(ms)

    row = [
        datetime.now(timezone.utc).astimezone(TZ_CST).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
        target_id,
        1 if matched_flag else 0,
        cam_utc.astimezone(TZ_CST).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] if cam_utc else "",
        fmt_time_ms(ntp_time_ms),
        fmt(rtk_lat), fmt(rtk_lon), fmt(rtk_height, ".4f"),
        fmt(yaw_deg, ".4f"), fmt(pitch_deg, ".4f"), fmt(roll_deg, ".4f"),
        fmt(gimbal_lat), fmt(gimbal_lon), fmt(gimbal_height, ".4f"),
        fmt(target_lat), fmt(target_lon),
        fmt(pixel_u, ".1f"), fmt(pixel_v, ".1f"),
    ]
    line = ",".join(str(v) for v in row)

    with LOCATION_LOG_LOCK:
        try:
            # 文件不存在或为空时先写入表头
            if not os.path.exists(LOCATION_LOG_FILE) or os.path.getsize(LOCATION_LOG_FILE) == 0:
                with open(LOCATION_LOG_FILE, "w", encoding="utf-8") as f:
                    f.write(_LOCATION_LOG_HEADER + "\n")
            with open(LOCATION_LOG_FILE, "a", encoding="utf-8") as f:
                f.write(line + "\n")
        except Exception as e:
            print(f"[WARN] 定位日志写入失败: {e}")


def upload_worker():
    """
    上传工作线程：延迟定位策略
    1. 从队列获取检测结果（含像素坐标和摄像头UTC时间）
    2. 等待 1 秒，让无人机遥测数据充分到达
    3. 匹配最近的无人机定位数据
    4. 计算精确 GPS 坐标
    5. 执行上传
    """
    global upload_queue, upload_thread_running
    
    print("[UPLOADER] 上传线程已启动")
    
    while upload_thread_running:
        # 尝试获取上传任务
        task = None
        with upload_queue_lock:
            if upload_queue:
                task = upload_queue.popleft()
        
        if task:
            try:
                # ===== 确保从帧捕获到定位计算至少 2 秒 =====
                elapsed = (datetime.now(timezone.utc) - task['cam_utc']).total_seconds()
                wait = max(0.0, 2.0 - elapsed)
                if wait > 0.01:
                    time.sleep(wait)
                # 如果任务已经超过 2 秒（排队积压），直接处理不等待

                # ===== 打印帧时间与定位到达信息 =====
                # f_cst = task['cam_utc'].astimezone(TZ_CST).strftime("%H:%M:%S.%f")[:-3]
                # now_cst = datetime.now(timezone.utc).astimezone(TZ_CST).strftime("%H:%M:%S.%f")[:-3]
                # print(f"[上传定位] 任务#{task['target_id']} 帧时间={f_cst}  处理时间={now_cst}")

                # ===== 匹配帧时间对应的无人机遥测数据 =====
                # 按任务来源选择杆臂（红外与可见光镜头安装位置不同，各自标定）
                task_lever_arm = LEVER_ARM_INF if task.get('is_inf') else LEVER_ARM
                # 遥测匹配键按相机时间系选择（INF_TIME_SYSTEM）：
                #   "local"=红外源为妙算3的MediaMTX（时钟无法NTP同步），帧时间在妙算3系，用遥测自带的local_time匹配
                #   "ntp"  =红外相机自带HTTP服务且已NTP对时（同可见光），帧时间在Jetson系，用ntp_time匹配
                matched = match_drone_data(
                    task['cam_utc'],
                    use_local_time=bool(task.get('is_inf')) and INF_TIME_SYSTEM == "local"
                )
                if matched:
                    # f_ms = int(task['cam_utc'].timestamp() * 1000)
                    # diff = abs(matched['ntp_time'] - f_ms)
                    # ntp_cst = datetime.fromtimestamp(matched['ntp_time'] / 1000, tz=timezone.utc).astimezone(TZ_CST).strftime("%H:%M:%S.%f")[:-3]
                    # print(f"[上传定位] 匹配成功 帧={f_cst}  定位ntp={ntp_cst}  lat={matched['lat']:.6f} lon={matched['lon']:.6f}  偏差={diff}ms")

                    # RTK天线与镜头安装位置不同：先用姿态修正出镜头位置，再参与隐患定位换算
                    gimbal_lat, gimbal_lon, gimbal_height = compute_gimbal_position(
                        rtk_lat_deg=matched['lat'],
                        rtk_lon_deg=matched['lon'],
                        rtk_height_agl=matched['height'],
                        yaw_deg=matched['yaw'],
                        pitch_deg=matched['pitch'],
                        roll_deg=matched['roll'],
                        lever_arm=task_lever_arm
                    )
                    # 按任务来源选择相机内参（红外/可见光各自使用原生分辨率与像元参数）
                    if task.get('is_inf'):
                        focal_len, pixel_sz, img_w, img_h = FOCAL_LENGTH_INF, PIXEL_SIZE_INF, IMAGE_WIDTH_INF, IMAGE_HEIGHT_INF
                    else:
                        focal_len, pixel_sz, img_w, img_h = FOCAL_LENGTH, PIXEL_SIZE, IMAGE_WIDTH, IMAGE_HEIGHT
                    lat, lon = pixel_to_gps_with_yaw(
                        drone_lat_deg=gimbal_lat,
                        drone_lon_deg=gimbal_lon,
                        drone_height_agl=gimbal_height,
                        target_height_agl=0.0,
                        focal_length_mm=focal_len,
                        pixel_size_mm=pixel_sz,
                        image_width=img_w,
                        image_height=img_h,
                        pixel_u=task['avg_cx'],
                        pixel_v=task['avg_cy'],
                        drone_yaw_deg=matched['yaw'],
                        drone_pitch_deg=0.0,
                        drone_roll_deg=0.0,
                        camera_pitch_deg=0.0
                    )
                    print(f"[UPLOADER] 目标 {task['target_id']} 定位完成: 吊舱=({gimbal_lat:.8f},{gimbal_lon:.8f},{gimbal_height:.2f}m) 隐患=({lat:.10f}, {lon:.10f})")
                    write_location_log(
                        target_id=task['target_id'],
                        matched_flag=True,
                        cam_utc=task['cam_utc'],
                        ntp_time_ms=matched['ntp_time'],
                        rtk_lat=matched['lat'], rtk_lon=matched['lon'], rtk_height=matched['height'],
                        yaw_deg=matched['yaw'], pitch_deg=matched['pitch'], roll_deg=matched['roll'],
                        gimbal_lat=gimbal_lat, gimbal_lon=gimbal_lon, gimbal_height=gimbal_height,
                        target_lat=lat, target_lon=lon,
                        pixel_u=task['avg_cx'], pixel_v=task['avg_cy']
                    )
                else:
                    # 未匹配到遥测：用全局最新位置与姿态修正镜头位置（降级方案）
                    gimbal_lat, gimbal_lon, gimbal_height = compute_gimbal_position(
                        rtk_lat_deg=fused_lat,
                        rtk_lon_deg=fused_lon,
                        rtk_height_agl=height_fusion if height_fusion is not None else 0.0,
                        yaw_deg=yaw_now,
                        pitch_deg=pitch_now,
                        roll_deg=roll_now,
                        lever_arm=task_lever_arm
                    )
                    lat, lon = gimbal_lat, gimbal_lon
                    print(f"[UPLOADER] 目标 {task['target_id']} 未匹配到无人机数据，回退全局位置")
                    write_location_log(
                        target_id=task['target_id'],
                        matched_flag=False,
                        cam_utc=task['cam_utc'],
                        ntp_time_ms=None,
                        rtk_lat=fused_lat, rtk_lon=fused_lon,
                        rtk_height=height_fusion if height_fusion is not None else 0.0,
                        yaw_deg=yaw_now, pitch_deg=pitch_now, roll_deg=roll_now,
                        gimbal_lat=gimbal_lat, gimbal_lon=gimbal_lon, gimbal_height=gimbal_height,
                        target_lat=lat, target_lon=lon,
                        pixel_u=task['avg_cx'], pixel_v=task['avg_cy']
                    )
                
                # ===== 获取时间戳并上传 =====
                capture_ts = task['cam_utc'].timestamp()  # 使用帧的摄像头UTC时间戳
                detected_structs = [{
                    "type": task['class_name'],
                    "x": 0,
                    "y": 0,
                    "w": 0,
                    "h": 0,
                    "conf": 1.0,
                    "lat": lat,
                    "lon": lon
                }]
                
                print(f"[UPLOADER] 上传目标 {task['target_id']}, 坐标: {lat:.10f}, {lon:.10f}")
                trigger_image_upload(detected_structs, task['first_frame'], capture_ts,
                                     target_id=task['target_id'],
                                     is_inf=task.get('is_inf', False))
                
                # 释放图像内存
                task['first_frame'] = None
                
            except Exception as e:
                print(f"[UPLOADER] 上传失败: {str(e)}")
                if task.get('retry_count', 0) < 3:
                    task['retry_count'] = task.get('retry_count', 0) + 1
                    with upload_queue_lock:
                        upload_queue.append(task)
        
        else:
            # 队列为空，短暂休眠
            time.sleep(0.05)
    
    print("[UPLOADER] 上传线程已停止")

def start_upload_thread():
    """启动上传线程"""
    upload_thread = threading.Thread(target=upload_worker, daemon=True)
    upload_thread.start()
    return upload_thread

    


# ================== 进程清理工具 ==================
def terminate_process(proc, tag=""):
    """安全终止子进程：SIGTERM 优雅退出，3秒超时后 SIGKILL 强杀，避免残留 ffmpeg 进程"""
    if proc is None or proc.poll() is not None:
        return
    tag = f"[{tag}] " if tag else ""
    try:
        print(f"{tag}终止 FFmpeg 进程 (PID {proc.pid})...")
        proc.terminate()  # SIGTERM
        try:
            proc.wait(timeout=3)
            print(f"{tag}FFmpeg 进程已退出 (PID {proc.pid})")
        except subprocess.TimeoutExpired:
            print(f"{tag}FFmpeg 进程3秒内未退出，强制结束 (PID {proc.pid})")
            proc.kill()  # SIGKILL
            proc.wait(timeout=3)
    except Exception as e:
        print(f"{tag}终止 FFmpeg 进程异常: {e}")

# ================== 主类 ==================
class MJPEG_TRT_NVENC:
    def __init__(self):
        self.running = True
        # 优化：使用两个队列分离处理流程
        self.inference_queue = deque(maxlen=1)  # 原始帧队列（等待推理）
        self.result_queue = deque(maxlen=1)  # 推理结果队列（等待推流）

        # 推流看门狗状态
        self.last_push_ts = time.time()        # 最近一次成功推帧时间（心跳）
        self.process_thread = None             # Streamer 线程引用（看门狗用于拉起）
        self.ffmpeg_restart_lock = threading.Lock()  # 防止 ffmpeg 被重复重启
        self._restarting = False               # 正在重启 FFmpeg（看门狗据此跳过，避免误杀）

        # MQTT和吊舱连接状态
        self.is_mqtt_connected = False
        self.is_gimbal_connected = False

        # 当前帧检测结果（实例变量，双流并发时不互相覆盖）
        self.detected_structs = []

        # ===== 摄像头 UTC 时间追踪 =====
        self.cam_utc = None          # 摄像头基准 UTC 时间
        self.cam_local_ref = None    # 获取 cam_utc 时的本地 UTC 时间
        self.cam_source = "未知"     # 时间源描述
        self.first_pts_time = None   # 首帧 pts_time（秒）
        self.first_local_time = None # 首帧本地 UTC 时间
        self.current_pts_time = None # 当前帧 pts_time（秒，由 stderr 线程更新）
        self.current_frame_cam_utc = None  # 当前帧的摄像头 UTC 时间（已校准）
        self.matched_drone_data = None    # 匹配到的无人机遥测数据
        self.frame_ntp_diff_ms = None     # 帧与 ntp_time 的偏差毫秒数
        self.pts_lock = threading.Lock()

    # ---------- FFmpeg 拉流 ----------
    def start_ffmpeg_in(self):
        cmd = [
            "ffmpeg",
            "-rtsp_transport", "tcp",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-analyzeduration", "0",
            "-probesize", "32",
            "-c:v", "h264_nvmpi",  # NVDEC 硬件解码（须放在 -i 之前）
            "-i", RTSP_IN,
            "-vf", f"fps={TARGET_FPS},scale={IMAGE_WIDTH}:{IMAGE_HEIGHT},format=bgr24,showinfo",
            "-pix_fmt", "bgr24",
            "-f", "rawvideo",
            "pipe:1"
        ]

        print("[INFO] 启动 FFmpeg 拉流")
        self.ffmpeg_in = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=-1  # 缓冲流：保证 read(帧长) 读满定长帧或遇 EOF
        )

    # ---------- FFmpeg 推流 (Jetson 硬件 H264) ----------
    def start_ffmpeg_out(self):
        cmd = [
            "ffmpeg",
            "-y",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{IMAGE_WIDTH}x{IMAGE_HEIGHT}",
            "-r", str(TARGET_FPS),
            "-i", "pipe:0",
            "-an",
            "-c:v", "h264_nvmpi",  # Jetson 硬件编码
            "-profile:v", "high",
            "-b:v", "4M",
            "-maxrate", "6M",
            "-bufsize", "1M",
            "-g", str(TARGET_FPS),  # 关键帧间隔
            "-bf", "0",  # 禁用 B 帧，降低延迟
            "-pix_fmt", "yuv420p",
            "-colorspace", "bt709",
            "-color_primaries", "bt709",
            "-color_trc", "bt709",
            "-color_range", "pc",
            "-f", "rtsp",
            "-rtsp_transport", "tcp",
            "-muxdelay", "0.1",
            "-muxpreload", "0.1",
            "-nostats",  # 禁用进度统计输出：默认每0.5s向stderr写~120字节，无人消费时会填满64KB管道缓冲导致ffmpeg周期性阻塞
            RTSP_OUT
        ]
        print("[INFO] 启动 FFmpeg 推流 (h264_nvmpi)")
        self.ffmpeg_out = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE,
            stdout=subprocess.DEVNULL  # 丢弃编码器 stdout 刷屏（如 "Opening in BLOCKING MODE"），错误走 stderr 落盘
        )
        # 每次启动推流进程都重新绑定 stderr 读取线程：
        # 若只在 run() 启动一次，进程重启后旧线程仍读死进程的管道，新进程的 stderr 无人消费，
        # 管道缓冲填满后 ffmpeg 阻塞在写日志、停止消费 stdin，表现为约400秒一次的周期性推流停摆
        threading.Thread(target=self.ffmpeg_log, args=(self.ffmpeg_out, "FFMPEG_OUT"), daemon=True).start()

    def _restart_ffmpeg_out(self):
        """重启 FFmpeg 推流进程"""
        if not self.running:
            return  # 正在关闭时不再重启
        print("[INFO] 重启 FFmpeg 推流...")
        self._restarting = True
        try:
            try:
                if self.ffmpeg_out and self.ffmpeg_out.poll() is None:
                    self.ffmpeg_out.stdin.close()
                    self.ffmpeg_out.terminate()
                    self.ffmpeg_out.wait(timeout=3)
            except Exception:
                pass
            self.start_ffmpeg_out()
            self.last_push_ts = time.time()  # 给新进程完整的 STALL_TIMEOUT 窗口推送首帧，避免被看门狗误杀
            time.sleep(0.3)
        finally:
            self._restarting = False

    # ---------- rawvideo 帧读取 ----------
    def reader_thread(self):
        # rawvideo 定长帧：每帧固定 W*H*3 字节，免 MJPEG 编解码（省 CPU）
        frame_size = IMAGE_WIDTH * IMAGE_HEIGHT * 3
        frame_count = 0
        last_log_time = time.time()
        dropped_frames = 0

        while self.running:
            # stdout 为缓冲流，read(n) 会阻塞至读满 n 字节或 EOF
            data = self.ffmpeg_in.stdout.read(frame_size)
            if not data or len(data) < frame_size:
                time.sleep(0.001)
                continue

            # copy() 转可写数组（下游 cv2 绘制需要原地修改）
            frame = np.frombuffer(data, np.uint8).reshape(IMAGE_HEIGHT, IMAGE_WIDTH, 3).copy()
            capture_ts = time.time()

            # 直接追加，deque(maxlen)满时自动丢弃最旧帧
            if len(self.inference_queue) >= self.inference_queue.maxlen:
                dropped_frames += 1
            self.inference_queue.append((frame, capture_ts))
            frame_count += 1

            # 每5秒输出一次队列状态
            now = time.time()
            if now - last_log_time > 5.0:
                logger.info(f"[Reader] 已读取 {frame_count} 帧 | 丢弃 {dropped_frames} 帧 | 推理队列: {len(self.inference_queue)}/{self.inference_queue.maxlen} | 结果队列: {len(self.result_queue)}/{self.result_queue.maxlen}")
                last_log_time = now
                dropped_frames = 0

    # ---------- AI 推理线程（独立线程） ----------
    def inference_thread(self):
        # 检测结果用实例变量（self.detected_structs），避免双流并发时全局变量互相覆盖
        frame_count = 0
        inference_count = 0
        while self.running:
            if not self.inference_queue:
                time.sleep(0.001)
                continue

            frame, capture_ts = self.inference_queue.popleft()
            frame_count += 1

            # ===== 计算当前帧的摄像头 UTC 时间（用 pts_time 秒值直接算） =====
            with self.pts_lock:
                if self.cam_utc is not None and self.first_pts_time is not None and self.current_pts_time is not None:
                    first_cam_utc = self.cam_utc + (self.first_local_time - self.cam_local_ref)
                    delta_sec = self.current_pts_time - self.first_pts_time
                    self.current_frame_cam_utc = first_cam_utc + timedelta(seconds=delta_sec)
                    frame_local_now_cst = datetime.now(timezone.utc).astimezone(TZ_CST).strftime("%H:%M:%S.%f")[:-3]
                    frame_cst_local = self.current_frame_cam_utc.astimezone(TZ_CST).strftime("%H:%M:%S.%f")[:-3]
                    # print(f"[帧时间] 帧UTC={frame_cst_local}  本地到达={frame_local_now_cst}")
                    # 每 30 帧进行一次 2 秒延迟匹配测试
                    # if frame_count % 30 == 0:
                    #     test_cam_utc = self.current_frame_cam_utc
                    #     threading.Thread(target=lambda cu=test_cam_utc: (
                    #         time.sleep(2),
                    #         (lambda m: print(f"[延迟匹配测试] 帧UTC={cu.astimezone(TZ_CST).strftime('%H:%M:%S.%f')[:-3]}  "
                    #                          f"定位ntp={datetime.fromtimestamp(m['ntp_time']/1000,tz=timezone.utc).astimezone(TZ_CST).strftime('%H:%M:%S.%f')[:-3]}  "
                    #                          f"偏差={abs(m['ntp_time']-int(cu.timestamp()*1000))}ms") if m else
                    #                        print(f"[延迟匹配测试] 帧UTC={cu.astimezone(TZ_CST).strftime('%H:%M:%S.%f')[:-3]}  未匹配"))(match_drone_data(cu))),
                    #         daemon=True
                    #     ).start()
                else:
                    self.current_frame_cam_utc = None

            # ===== 匹配当前帧与最近的无人机遥测数据（仅做一次，不阻塞） =====
            self.matched_drone_data = match_drone_data(self.current_frame_cam_utc)
            if self.matched_drone_data:
                frame_ms = int(self.current_frame_cam_utc.timestamp() * 1000)
                self.frame_ntp_diff_ms = abs(self.matched_drone_data["ntp_time"] - frame_ms)
            else:
                self.frame_ntp_diff_ms = None

            # ===== 实时匹配（不做打印，仅用于 upload_worker 使用） =====

            # 获取当前AI状态
            with ai_lock:
                current_work_mode = work_mode
                current_enabled_classes = set(ai_enabled_classes)
                current_enabled_classes.difference_update(set(range(len(COCO_NAMES),len(COCO_NAMES)+len(COCO_NAMES_HW))))
                current_ai_enabled = (len(current_enabled_classes) > 0)
                current_autosave = AUTOSAVE

            # 默认直接输出原始帧
            annotated_frame = frame

            self.detected_structs = []

            # ===== AI 处理 =====
            if current_work_mode == 1 and current_ai_enabled and model is not None:
                try:
                    inference_start = time.time()
                    results = model.predict(
                        source=frame,
                        imgsz=YOLO_IMGSZ,
                        device=0,
                        half=True,
                        conf=CONF_THRES,
                        max_det=10,  # 限制最大检测数，加速NMS
                        verbose=False
                    )

                    inference_time = time.time() - inference_start
                    inference_count += 1

                    # 每60次推理输出一次性能信息，减少日志开销
                    if inference_count % 60 == 0:
                        logger.info(f"[AI] 推理耗时: {inference_time*1000:.1f}ms | 队列长度: {len(self.inference_queue)}")

                    # 直接在原始帧上绘制，避免创建副本
                    if results and results[0].boxes is not None and len(results[0].boxes) > 0:
                        r = results[0]
                        boxes = r.boxes.xyxy.cpu().numpy()
                        clss = r.boxes.cls.cpu().numpy()
                        confs = r.boxes.conf.cpu().numpy()

                        # 限制绘制的目标数量，最多绘制10个
                        max_draw = 10

                        # ===== 收集需要跟踪的检测结果 =====
                        detections = []      # 检测框列表 [x1,y1,x2,y2]
                        class_names = []     # 类别名称列表
                        det_info = []        # 详细信息 [(cls_id, conf, x1, y1, x2, y2, cx, cy), ...]

                        for box, cls_id, conf in zip(boxes, clss, confs):
                            cls_id = int(cls_id)
                            conf = float(conf)

                            if cls_id not in current_enabled_classes:
                                continue
                            if conf < CONF_THRES:
                                continue

                            x1, y1, x2, y2 = map(int, box)
                            w = x2 - x1
                            h = y2 - y1
                            cx = x1 + w // 2
                            cy = y1 + h // 2

                            class_name = COCO_NAMES[cls_id] if cls_id < len(COCO_NAMES) else f"class_{cls_id}"

                            # panel 类别不需要跟踪，直接处理
                            if class_name == "panel":
                                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                label = f"{class_name} {conf:.2f}"
                                cv2.putText(frame, label, (x1, y1 - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                                self.detected_structs.append({
                                    "type": class_name,
                                    "x": cx,
                                    "y": cy,
                                    "w": w,
                                    "h": h,
                                    "conf": round(conf, 3),
                                    "lat": 0.0,
                                    "lon": 0.0
                                })
                                continue

                            # 收集需要跟踪的检测
                            detections.append([x1, y1, x2, y2])
                            class_names.append(class_name)
                            det_info.append((cls_id, conf, x1, y1, x2, y2, cx, cy))

                        # ===== 使用 SORT 算法进行目标跟踪 =====
                        if len(detections) > 0:
                            matches, _ = sort_tracker.update(detections, class_names)
                            
                            # 创建检测索引到跟踪器的映射
                            det_idx_to_tracker = {match[0]: match[1] for match in matches}
                        else:
                            det_idx_to_tracker = {}

                        # ===== 处理每个检测结果 =====
                        count = 0
                        for i, (cls_id, conf, x1, y1, x2, y2, cx, cy) in enumerate(det_info):
                            if count >= max_draw:
                                break

                            class_name = class_names[i]
                            
                            # 获取跟踪器（如果有匹配）
                            tracker = det_idx_to_tracker.get(i)
                            # 仅已确认（连续匹配达到 SORT_MIN_HITS 帧）的跟踪器才分配正式ID
                            # 并参与目标收集/上传，抑制单帧误检造成的ID抖动
                            is_confirmed = tracker is not None and tracker.hit_streak >= SORT_MIN_HITS
                            target_id = tracker.id if is_confirmed else None

                            # ===== 画框 =====
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            label = f"{class_name} {conf:.2f} ID:{target_id}" if target_id else f"{class_name} {conf:.2f}"
                            cv2.putText(frame, label, (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                            # ===== 坐标计算（延迟到 upload_worker 中匹配无人机数据后计算） =====
                            lat, lon = 0.0, 0.0  # 占位，upload_worker中重新计算

                            # ===== 更新目标追踪信息（仅确认后的目标参与收集/上传） =====
                            use_saved_avg = False
                            if is_confirmed:
                                is_new, ready_to_upload, use_saved_avg = update_target(
                                    target_id=target_id,
                                    box=[x1, y1, x2, y2],
                                    cx=cx,
                                    cy=cy,
                                    cam_utc=self.current_frame_cam_utc,
                                    frame=frame,
                                    class_name=class_name,
                                    current_time=time.time()
                                )

                            # 如果目标已上传，使用已保存的平均坐标（占位）
                            if use_saved_avg and target_id is not None:
                                lat, lon = 0.0, 0.0

                            # ===== 构造结构（用于推流显示）=====
                            self.detected_structs.append({
                                "type": class_name,
                                "x": cx,
                                "y": cy,
                                "w": x2 - x1,
                                "h": y2 - y1,
                                "conf": round(conf, 3),
                                "lat": lat,
                                "lon": lon
                            })
                            count += 1
                    
                    # 直接使用原始帧
                    annotated_frame = frame
                except Exception as e:
                    print(f"[ERROR] 推理错误: {str(e)}")

            # ===== 检查所有追踪目标是否应该触发上传 =====
            # 即使当前帧没有检测到目标，也要检查已经收集满5帧的目标是否应该触发上传
            current_time = time.time()
            # 获取目标ID列表（在锁内获取，避免迭代过程中字典被修改）
            target_ids = []
            with target_tracker_lock:
                target_ids = list(target_tracker.keys())
            # 在锁外调用检查函数（避免嵌套锁）
            for target_id in target_ids:
                check_target_timeout(target_id, current_time)

            # ===== 输出到推流队列 =====
            self.result_queue.append(annotated_frame)

            # ===== 定时清理超时目标 =====
            cleanup_stale_targets()

    # ---------- GPS/IMU数据更新线程（独立线程） ----------
    def gps_update_thread(self):
        """独立线程，定时更新GPS/IMU数据，不阻塞推理流程"""
        while self.running:
            global timestamp
            try:
                locatin(timestamp / 1000)
            except Exception as e:
                print(f"[ERROR] GPS更新异常: {e}")
            time.sleep(0.1)  # 10Hz更新，足够用于定位精度要求

    # ---------- 主处理（纯推流） ----------
    def process_loop(self):
        print("[INFO] 开始主处理循环（纯推流）")
        frame_count = 0
        last_log_time = time.time()
        fps_times = deque(maxlen=30)  # 滑动窗口存储最近30帧间隔
        last_frame_time = time.time()

        while self.running:
            try:
                if not self.result_queue:
                    time.sleep(0.001)
                    continue

                frame = self.result_queue.popleft()

                now = time.time()

                # 记录帧间隔时间
                frame_interval = now - last_frame_time
                if frame_interval > 0.5:
                    logger.warning(f"[Streamer] 帧间隔异常: {frame_interval*1000:.0f}ms (卡顿)")
                last_frame_time = now
                fps_times.append(frame_interval)

                # 计算平均FPS（使用滑动窗口）
                if len(fps_times) > 1:
                    avg_interval = sum(fps_times) / len(fps_times)
                    fps = 1.0 / max(avg_interval, 1e-6)
                else:
                    fps = 0.0

                frame_count += 1

                # 每5秒输出一次推流状态
                if now - last_log_time > 5.0:
                    logger.info(f"[Streamer] 已推送 {frame_count} 帧 | 当前FPS: {fps:.1f} | 目标FPS: {TARGET_FPS} | 结果队列: {len(self.result_queue)}/{self.result_queue.maxlen}")
                    last_log_time = now

                # 在帧上叠加FPS
                cv2.putText(frame, f"FPS: {fps:.1f}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # 推流（断连/卡死时自动重启FFmpeg）
                frame_bytes = frame.tobytes()
                while self.running:
                    try:
                        if self.ffmpeg_out is None or self.ffmpeg_out.poll() is not None:
                            print("[WARN] FFmpeg 推流进程已退出，重启中...")
                            with self.ffmpeg_restart_lock:
                                self._restart_ffmpeg_out()
                            continue
                        self.ffmpeg_out.stdin.write(frame_bytes)
                        self.last_push_ts = time.time()  # 推流心跳
                        break
                    except (BrokenPipeError, OSError, ValueError) as e:
                        # 管道被看门狗强杀或自身关闭：稍等后回到循环顶部，由 poll() 统一判定重启。
                        # 此处不直接重启，避免与看门狗双重重启互相关闭对方 stdin（write to closed file）
                        print(f"[ERROR] FFmpeg 推流异常: {e}，稍后重启...")
                        time.sleep(0.2)
            except Exception as e:
                # 兜底：任何异常都不让推流线程静默消失，打印完整堆栈后退出，由看门狗重启线程
                traceback.print_exc()
                print(f"[Streamer] 线程异常退出，等待看门狗重新拉起: {str(e)}")
                break

        print("[Streamer] 主处理循环已退出")

    # ---------- 推流看门狗（检测卡死/线程退出并自动恢复） ----------
    def watchdog_thread(self):
        print("[INFO] 看门狗启动（每2秒检查推流状态）")
        while self.running:
            time.sleep(2)
            # 1) 心跳超时 且 结果队列有积压 且 不在重启中 → 推流卡死（ffmpeg_out 输出端停摆导致 stdin.write 永久阻塞）
            if time.time() - self.last_push_ts > STALL_TIMEOUT and self.result_queue and not self._restarting:
                print(f"[WARN] 推流超过{STALL_TIMEOUT}s无输出且队列积压，判定卡死，强杀 FFmpeg 推流进程...")
                self._force_kill_ffmpeg_out()
            elif time.time() - self.last_push_ts > STALL_TIMEOUT:
                print(f"[WARN] 推流超过{STALL_TIMEOUT}s无输出（队列无积压，可能是读取/推理侧停摆）")
            # 2) Streamer 线程异常退出 → 重新拉起
            if self.process_thread is not None and not self.process_thread.is_alive():
                print("[WARN] Streamer 线程已退出，重新启动...")
                self.process_thread = threading.Thread(target=self.process_loop, daemon=True)
                self.process_thread.start()
        print("[INFO] 看门狗已停止")

    def _force_kill_ffmpeg_out(self):
        """强杀推流 FFmpeg：解除 stdin.write 的永久阻塞（抛 BrokenPipeError 后由推流循环自动重启）"""
        try:
            if self.ffmpeg_out and self.ffmpeg_out.poll() is None:
                print(f"[WARN] 强杀卡死的 FFmpeg 推流进程 (PID {self.ffmpeg_out.pid})")
                # 已判定卡死，直接 SIGKILL。SIGTERM 对 SIGSTOP/挂死的进程无效（信号挂起），还会白白多等 3s 超时
                self.ffmpeg_out.kill()
                try:
                    self.ffmpeg_out.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
            else:
                print("[WARN] ffmpeg_out 已退出/不存在，由推流线程 poll() 分支负责重启")
        except Exception as e:
            print(f"[WARN] 强杀 FFmpeg 推流异常: {e}")
        # 不在此重启：推流线程的 poll() 分支会统一重启，避免双重重启互相关闭对方的 stdin

    # ---------- FFmpeg 拉流 stderr 读取（解析 PTS + 打印日志） ----------
    def _ffmpeg_in_stderr_reader(self):
        """读取 ffmpeg_in 的 stderr，解析 showinfo 获取 PTS，同时打印日志"""
        while self.running:
            line = self.ffmpeg_in.stderr.readline()
            if not line:
                time.sleep(0.1)
                continue
            decoded = line.decode(errors="replace").strip()

            # 解析 showinfo 获取 PTS（不打印 showinfo 行，避免日志刷屏）
            info = parse_frame_info(decoded)
            if info is not None:
                pts, pts_time = info
                with self.pts_lock:
                    self.current_pts_time = pts_time
                    if self.first_pts_time is None:
                        self.first_pts_time = pts_time
                        self.first_local_time = datetime.now(timezone.utc)
                        cam_cst = self.cam_utc.astimezone(TZ_CST) if self.cam_utc else None
                        if cam_cst:
                            print(f"[PTS] 首帧 pts_time={pts_time:.3f}s, 摄像头基准时间={cam_cst.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
            elif not decoded.startswith("[Parsed_showinfo"):
                pass  # 不打印 FFmpeg 日志行

    # ---------- FFmpeg 日志（推流用，仅读取避免管道阻塞） ----------
    def ffmpeg_log(self, p, tag):
        # 将 ffmpeg 推流进程的 stderr 写入日志文件（不占用控制台），用于排查编码器/推流卡死根因
        # 注意：本线程与 p 的生命周期一致，p 退出后本线程结束（由 start_ffmpeg_out 为新进程重新启动）
        log_path = "/tmp/ffmpeg_out_stderr.log"
        try:
            with open(log_path, "a") as f:
                f.write(f"\n===== {tag} 新进程启动 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} =====\n")
                while self.running:
                    line = p.stderr.readline()
                    if not line:
                        if p.poll() is not None:
                            break  # 进程已退出且管道读尽，结束本线程
                        time.sleep(0.1)
                        continue
                    f.write(line.decode(errors="replace"))
                    f.flush()
        except Exception:
            pass  # 日志文件写失败不影响主流程

    # ---------- 启动 ----------
    def run(self):
        # 初始化MQTT
        self.is_mqtt_connected = init_mqtt()

        # 初始化吊舱连接（按当前相机协议自动选择 TCP/UDP）
        self.is_gimbal_connected = init_gimbal()

        # ===== 获取摄像头 UTC 时间 =====
        cam_host = parse_rtsp_host(RTSP_IN)
        if cam_host:
            self.cam_local_ref = datetime.now(timezone.utc)
            print(f"获取摄像机 UTC 时间（{cam_host}:80）...")
            self.cam_utc, self.cam_source = get_camera_utc(cam_host, port=80)
            if self.cam_utc:
                local_cst = self.cam_local_ref.astimezone(TZ_CST)
                cam_cst = self.cam_utc.astimezone(TZ_CST)
                print(f"  时间源: {self.cam_source}")
                print(f"  摄像机 北京时间: {cam_cst.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
                print(f"  本地   北京时间: {local_cst.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
                offset = (self.cam_utc - self.cam_local_ref).total_seconds()
                print(f"  时钟偏差: {offset:+.3f}s")
            else:
                print(f"  HTTP Date 均失败，使用本地 UTC 时间作为参考")
                self.cam_utc = datetime.now(timezone.utc)
                self.cam_source = "本地"
            print("=" * 80)

        self.start_ffmpeg_in()
        self.start_ffmpeg_out()

        # 启动所有线程
        threading.Thread(target=self.reader_thread, daemon=True).start()
        threading.Thread(target=self.inference_thread, daemon=True).start()
        threading.Thread(target=self.gps_update_thread, daemon=True).start()
        self.process_thread = threading.Thread(target=self.process_loop, daemon=True)
        self.process_thread.start()
        threading.Thread(target=self._ffmpeg_in_stderr_reader, daemon=True).start()
        # ffmpeg_out 的 stderr 线程已改由 start_ffmpeg_out 每次启动时绑定（跟随进程重启）
        threading.Thread(target=self.watchdog_thread, daemon=True).start()

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("[INFO] 退出...")
            self.running = False

    def stop(self):
        """停止所有服务并清理资源"""
        self.running = False
        # 终止 FFmpeg 拉流/推流进程，避免残留
        terminate_process(getattr(self, 'ffmpeg_in', None), "FFMPEG_IN")
        terminate_process(getattr(self, 'ffmpeg_out', None), "FFMPEG_OUT")
        # 等待线程池完成任务
        if hasattr(self, 'UPLOAD_EXECUTOR'):
            self.UPLOAD_EXECUTOR.shutdown(wait=True)
        # 关闭MQTT和吊舱连接
        if mqtt_client:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        close_gimbal()

# ================== 红外视频流处理类 ==================
class MJPEG_TRT_NVENC_INF:
    def __init__(self):
        self.running = True
        # 优化：使用两个队列分离处理流程
        self.inference_queue = deque(maxlen=1)  # 原始帧队列（等待推理）
        self.result_queue = deque(maxlen=1)  # 推理结果队列（等待推流）

        # MQTT和吊舱连接状态
        self.is_mqtt_connected = False
        self.is_gimbal_connected = False

        # 当前帧检测结果（实例变量，双流并发时不互相覆盖）
        self.detected_structs = []

        # 推流看门狗状态（红外，与可见光同机制）
        self.last_push_ts = time.time()        # 最近一次成功推帧时间（心跳）
        self.process_thread = None             # Streamer 线程引用（看门狗用于拉起）
        self.ffmpeg_restart_lock = threading.Lock()  # 防止 ffmpeg 被重复重启
        self._restarting = False               # 正在重启 FFmpeg（看门狗据此跳过，避免误杀）

        # ===== 摄像头 UTC 时间追踪（红外，用于目标定位的时间匹配） =====
        self.cam_utc = None          # 摄像头基准 UTC 时间
        self.cam_local_ref = None    # 获取 cam_utc 时的本地 UTC 时间
        self.cam_source = "未知"     # 时间源描述
        self.first_pts_time = None   # 首帧 pts_time（秒）
        self.first_local_time = None # 首帧本地 UTC 时间
        self.current_pts_time = None # 当前帧 pts_time（秒，由 stderr 线程更新）
        self.current_frame_cam_utc = None  # 当前帧的摄像头 UTC 时间（已校准）
        self.pts_lock = threading.Lock()

    # ---------- FFmpeg 拉流（红外） ----------
    def start_ffmpeg_in(self):
        cmd = [
            "ffmpeg",
            "-rtsp_transport", "tcp",
            "-fflags", "nobuffer",
            "-flags", "low_delay",
            "-analyzeduration", "0",
            "-probesize", "32",
            "-c:v", "h264_nvmpi",  # NVDEC 硬件解码（须放在 -i 之前）
            "-i", RTSP_HW_IN,
            # 先裁掉红外源黑边（1440×1080容器中左右各44px纯黑，实测值），再抽帧还原为原生分辨率
            "-vf", (f"crop={INF_CROP}," if INF_CROP else "")
                  + f"fps={TARGET_FPS},scale={IMAGE_WIDTH_INF}:{IMAGE_HEIGHT_INF},format=bgr24,showinfo",
            "-pix_fmt", "bgr24",
            "-f", "rawvideo",
            "pipe:1"
        ]

        print("[INFO] 启动 FFmpeg 拉流（红外）")
        self.ffmpeg_in = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=-1  # 缓冲流：保证 read(帧长) 读满定长帧或遇 EOF
        )

    # ---------- FFmpeg 推流 (Jetson 硬件 H264)（红外） ----------
    def start_ffmpeg_out(self):
        cmd = [
            "ffmpeg",
            "-y",
            "-f", "rawvideo",
            "-pix_fmt", "bgr24",
            "-s", f"{IMAGE_WIDTH_INF}x{IMAGE_HEIGHT_INF}",
            "-r", str(TARGET_FPS),
            "-i", "pipe:0",
            "-an",
            "-c:v", "h264_nvmpi",  # Jetson 硬件编码
            "-profile:v", "high",
            "-b:v", "4M",
            "-maxrate", "6M",
            "-bufsize", "1M",
            "-g", str(TARGET_FPS),  # 关键帧间隔
            "-bf", "0",  # 禁用 B 帧，降低延迟
            "-pix_fmt", "yuv420p",
            "-colorspace", "bt709",
            "-color_primaries", "bt709",
            "-color_trc", "bt709",
            "-color_range", "pc",
            "-f", "rtsp",
            "-rtsp_transport", "tcp",
            "-muxdelay", "0.1",
            "-muxpreload", "0.1",
            "-nostats",  # 禁用进度统计输出，防止管道缓冲填满导致 ffmpeg 周期性阻塞（同可见光）
            RTSP_HW_OUT
        ]
        print("[INFO] 启动 FFmpeg 推流 (h264_nvmpi)（红外）")
        self.ffmpeg_out = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        # 每次启动都重新绑定 stderr 读取线程（跟随进程重启，防止新进程 stderr 无人消费）
        threading.Thread(target=self.ffmpeg_log, args=(self.ffmpeg_out, "FFMPEG_OUT_INF"), daemon=True).start()

    def _restart_ffmpeg_out(self):
        """重启 FFmpeg 推流进程（红外）。调用方需持有 ffmpeg_restart_lock"""
        if not self.running:
            return  # 正在关闭时不再重启
        print("[INFO] 重启 FFmpeg 推流（红外）...")
        self._restarting = True
        try:
            try:
                if self.ffmpeg_out and self.ffmpeg_out.poll() is None:
                    self.ffmpeg_out.stdin.close()
                    self.ffmpeg_out.terminate()
                    self.ffmpeg_out.wait(timeout=3)
            except Exception:
                pass
            self.start_ffmpeg_out()
            self.last_push_ts = time.time()  # 给新进程完整的 STALL_TIMEOUT 窗口推送首帧，避免被看门狗误杀
            time.sleep(0.3)
        finally:
            self._restarting = False

    # ---------- rawvideo 帧读取（红外） ----------
    def reader_thread(self):
        # rawvideo 定长帧：每帧固定 W*H*3 字节，免 MJPEG 编解码（省 CPU）
        frame_size = IMAGE_WIDTH_INF * IMAGE_HEIGHT_INF * 3
        frame_count = 0
        last_log_time = time.time()
        dropped_frames = 0

        while self.running:
            # stdout 为缓冲流，read(n) 会阻塞至读满 n 字节或 EOF
            data = self.ffmpeg_in.stdout.read(frame_size)
            if not data or len(data) < frame_size:
                time.sleep(0.001)
                continue

            # copy() 转可写数组（下游 cv2 绘制需要原地修改）
            frame = np.frombuffer(data, np.uint8).reshape(IMAGE_HEIGHT_INF, IMAGE_WIDTH_INF, 3).copy()
            capture_ts = time.time()

            # 直接追加，deque(maxlen)满时自动丢弃最旧帧
            if len(self.inference_queue) >= self.inference_queue.maxlen:
                dropped_frames += 1
            self.inference_queue.append((frame, capture_ts))
            frame_count += 1

            # 每5秒输出一次队列状态
            now = time.time()
            if now - last_log_time > 5.0:
                logger.info(f"[Reader-INF] 已读取 {frame_count} 帧 | 丢弃 {dropped_frames} 帧 | 推理队列: {len(self.inference_queue)}/{self.inference_queue.maxlen} | 结果队列: {len(self.result_queue)}/{self.result_queue.maxlen}")
                last_log_time = now
                dropped_frames = 0

    # ---------- AI 推理线程（独立线程）（红外） ----------
    def inference_thread(self):
        # 检测结果用实例变量（self.detected_structs），与可见光隔离，双流并发不互相覆盖
        inference_count = 0
        while self.running:
            if not self.inference_queue:
                time.sleep(0.001)
                continue

            frame, capture_ts = self.inference_queue.popleft()

            # ===== 计算当前帧的摄像头 UTC 时间（用 pts_time 秒值直接算，红外） =====
            with self.pts_lock:
                if self.cam_utc is not None and self.first_pts_time is not None and self.current_pts_time is not None:
                    first_cam_utc = self.cam_utc + (self.first_local_time - self.cam_local_ref)
                    delta_sec = self.current_pts_time - self.first_pts_time
                    self.current_frame_cam_utc = first_cam_utc + timedelta(seconds=delta_sec)
                else:
                    self.current_frame_cam_utc = None

            # 获取当前AI状态
            with ai_lock:
                current_work_mode = work_mode
                current_enabled_classes = set(ai_enabled_classes)
                current_enabled_classes.difference_update(set(range(0, len(COCO_NAMES))))
                current_enabled_classes = {x - len(COCO_NAMES) for x in current_enabled_classes}
                current_ai_enabled = (len(current_enabled_classes) > 0)
                current_autosave = AUTOSAVE

            # 默认直接输出原始帧
            annotated_frame = frame

            self.detected_structs = []

            # ===== AI 处理（红外） =====
            if current_work_mode == 1 and current_ai_enabled and model_inf is not None:
                try:
                    inference_start = time.time()
                    results = model_inf.predict(
                        source=frame,
                        imgsz=YOLO_IMGSZ_INF,
                        device=0,
                        half=True,
                        conf=CONF_THRES,
                        max_det=10,  # 限制最大检测数，加速NMS
                        verbose=False
                    )

                    inference_time = time.time() - inference_start
                    inference_count += 1

                    # 每60次推理输出一次性能信息，减少日志开销
                    if inference_count % 60 == 0:
                        logger.info(f"[AI-INF] 推理耗时: {inference_time*1000:.1f}ms | 队列长度: {len(self.inference_queue)}")

                    # 直接在原始帧上绘制，避免创建副本
                    if results and results[0].boxes is not None and len(results[0].boxes) > 0:
                        r = results[0]
                        boxes = r.boxes.xyxy.cpu().numpy()
                        clss = r.boxes.cls.cpu().numpy()
                        confs = r.boxes.conf.cpu().numpy()

                        # ===== 收集需要跟踪的检测结果（红外） =====
                        detections = []      # 检测框列表 [x1,y1,x2,y2]
                        class_names = []     # 类别名称列表
                        det_info = []        # 详细信息 [(cls_id, conf, x1, y1, x2, y2, cx, cy), ...]

                        for box, cls_id, conf in zip(boxes, clss, confs):
                            cls_id = int(cls_id)
                            conf = float(conf)

                            if cls_id not in current_enabled_classes:
                                continue
                            if conf < CONF_THRES:
                                continue

                            x1, y1, x2, y2 = map(int, box)
                            w = x2 - x1
                            h = y2 - y1
                            cx = x1 + w // 2
                            cy = y1 + h // 2

                            class_name = COCO_NAMES_HW[cls_id] if cls_id < len(COCO_NAMES_HW) else f"class_{cls_id}"

                            # panel 类别不需要跟踪，直接处理（与可见光逻辑一致）
                            if class_name == "panel":
                                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                                label = f"{class_name} {conf:.2f}"
                                cv2.putText(frame, label, (x1, y1 - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                                self.detected_structs.append({
                                    "type": class_name,
                                    "x": cx,
                                    "y": cy,
                                    "w": w,
                                    "h": h,
                                    "conf": round(conf, 3),
                                    "lat": 0.0,
                                    "lon": 0.0
                                })
                                continue

                            # 收集需要跟踪的检测（热斑/短路/异常低温等缺陷目标）
                            detections.append([x1, y1, x2, y2])
                            class_names.append(class_name)
                            det_info.append((cls_id, conf, x1, y1, x2, y2, cx, cy))

                        # ===== 使用 SORT 算法进行目标跟踪（红外独立追踪器） =====
                        if len(detections) > 0:
                            matches, _ = sort_tracker_inf.update(detections, class_names)
                            # 创建检测索引到跟踪器的映射
                            det_idx_to_tracker = {match[0]: match[1] for match in matches}
                        else:
                            det_idx_to_tracker = {}

                        # ===== 处理每个检测结果 =====
                        count = 0
                        max_draw = 10
                        for i, (cls_id, conf, x1, y1, x2, y2, cx, cy) in enumerate(det_info):
                            if count >= max_draw:
                                break

                            class_name = class_names[i]

                            # 获取跟踪器（如果有匹配）
                            tracker = det_idx_to_tracker.get(i)
                            # 仅已确认（连续匹配达到 SORT_MIN_HITS 帧）的跟踪器才分配正式ID
                            # 并参与目标收集/上传，抑制单帧误检造成的ID抖动
                            is_confirmed = tracker is not None and tracker.hit_streak >= SORT_MIN_HITS
                            target_id = tracker.id if is_confirmed else None

                            # ===== 画框 =====
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            label = f"{class_name} {conf:.2f} ID:{target_id}" if target_id else f"{class_name} {conf:.2f}"
                            cv2.putText(frame, label, (x1, y1 - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                            # ===== 坐标计算（延迟到 upload_worker 中匹配无人机数据后计算） =====
                            lat, lon = 0.0, 0.0  # 占位，upload_worker中重新计算

                            # ===== 更新目标追踪信息（红外独立收集器，仅确认后的目标参与收集/上传） =====
                            use_saved_avg = False
                            if is_confirmed:
                                is_new, ready_to_upload, use_saved_avg = update_target(
                                    target_id=target_id,
                                    box=[x1, y1, x2, y2],
                                    cx=cx,
                                    cy=cy,
                                    cam_utc=self.current_frame_cam_utc,
                                    frame=frame,
                                    class_name=class_name,
                                    current_time=time.time(),
                                    tracker=target_tracker_inf,
                                    lock=target_tracker_inf_lock
                                )

                            # ===== 构造结构（用于推流显示） =====
                            self.detected_structs.append({
                                "type": class_name,
                                "x": cx,
                                "y": cy,
                                "w": x2 - x1,
                                "h": y2 - y1,
                                "conf": round(conf, 3),
                                "lat": lat,
                                "lon": lon
                            })
                            count += 1

                    # 直接使用原始帧
                    annotated_frame = frame
                except Exception as e:
                    print(f"[ERROR] 推理错误（红外）: {str(e)}")

            # ===== 检查所有追踪目标是否应该触发上传（红外） =====
            # 即使当前帧没有检测到目标，也要检查已经收集满5帧的目标是否应该触发上传
            current_time = time.time()
            # 获取目标ID列表（在锁内获取，避免迭代过程中字典被修改）
            target_ids = []
            with target_tracker_inf_lock:
                target_ids = list(target_tracker_inf.keys())
            # 在锁外调用检查函数（避免嵌套锁）
            for target_id in target_ids:
                check_target_timeout(target_id, current_time,
                                     tracker=target_tracker_inf,
                                     lock=target_tracker_inf_lock,
                                     is_inf=True)

            # ===== 输出到推流队列 =====
            self.result_queue.append(annotated_frame)

            # ===== 定时清理超时目标（红外） =====
            cleanup_stale_targets(tracker=target_tracker_inf, lock=target_tracker_inf_lock)

    # ---------- 主处理（纯推流）（红外） ----------
    def process_loop(self):
        print("[INFO] 开始主处理循环（纯推流）（红外）")
        frame_count = 0
        last_log_time = time.time()
        fps_times = deque(maxlen=30)  # 滑动窗口存储最近30帧间隔
        last_frame_time = time.time()

        while self.running:
            try:
                if not self.result_queue:
                    time.sleep(0.001)
                    continue

                frame = self.result_queue.popleft()

                now = time.time()

                # 记录帧间隔时间
                frame_interval = now - last_frame_time
                if frame_interval > 0.5:
                    logger.warning(f"[Streamer-INF] 帧间隔异常: {frame_interval*1000:.0f}ms (卡顿)")
                last_frame_time = now
                fps_times.append(frame_interval)

                # 计算平均FPS（使用滑动窗口）
                if len(fps_times) > 1:
                    avg_interval = sum(fps_times) / len(fps_times)
                    fps = 1.0 / max(avg_interval, 1e-6)
                else:
                    fps = 0.0

                frame_count += 1

                # 每5秒输出一次推流状态
                if now - last_log_time > 5.0:
                    logger.info(f"[Streamer-INF] 已推送 {frame_count} 帧 | 当前FPS: {fps:.1f} | 目标FPS: {TARGET_FPS} | 结果队列: {len(self.result_queue)}/{self.result_queue.maxlen}")
                    last_log_time = now

                # 在帧上叠加FPS
                cv2.putText(frame, f"FPS: {fps:.1f}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # 推流（断连/卡死时自动重启FFmpeg，与可见光同机制）
                frame_bytes = frame.tobytes()
                while self.running:
                    try:
                        if self.ffmpeg_out is None or self.ffmpeg_out.poll() is not None:
                            print("[WARN] FFmpeg 推流进程已退出（红外），重启中...")
                            with self.ffmpeg_restart_lock:
                                self._restart_ffmpeg_out()
                            continue
                        self.ffmpeg_out.stdin.write(frame_bytes)
                        self.last_push_ts = time.time()  # 推流心跳
                        break
                    except (BrokenPipeError, OSError, ValueError) as e:
                        # 管道被看门狗强杀或自身关闭：稍等后回到循环顶部，由 poll() 统一判定重启。
                        # 此处不直接重启，避免与看门狗双重重启互相关闭对方 stdin（write to closed file）
                        print(f"[ERROR] FFmpeg 推流异常（红外）: {e}，稍后重启...")
                        time.sleep(0.2)
            except Exception as e:
                # 兜底：任何异常都不让推流线程静默消失，打印完整堆栈后退出，由看门狗重启线程
                traceback.print_exc()
                print(f"[Streamer-INF] 线程异常退出，等待看门狗重新拉起: {str(e)}")
                break

        print("[Streamer-INF] 主处理循环已退出")

    # ---------- 推流看门狗（检测卡死/线程退出并自动恢复）（红外） ----------
    def watchdog_thread(self):
        print("[INFO] 看门狗启动（红外，每2秒检查推流状态）")
        while self.running:
            time.sleep(2)
            # 1) 心跳超时 且 结果队列有积压 且 不在重启中 → 推流卡死（ffmpeg_out 输出端停摆导致 stdin.write 永久阻塞）
            if time.time() - self.last_push_ts > STALL_TIMEOUT and self.result_queue and not self._restarting:
                print(f"[WARN] 红外推流超过{STALL_TIMEOUT}s无输出且队列积压，判定卡死，强杀 FFmpeg 推流进程...")
                self._force_kill_ffmpeg_out()
            elif time.time() - self.last_push_ts > STALL_TIMEOUT:
                print(f"[WARN] 红外推流超过{STALL_TIMEOUT}s无输出（队列无积压，可能是读取/推理侧停摆）")
            # 2) Streamer 线程异常退出 → 重新拉起
            if self.process_thread is not None and not self.process_thread.is_alive():
                print("[WARN] 红外 Streamer 线程已退出，重新启动...")
                self.process_thread = threading.Thread(target=self.process_loop, daemon=True)
                self.process_thread.start()
        print("[INFO] 看门狗已停止（红外）")

    def _force_kill_ffmpeg_out(self):
        """强杀推流 FFmpeg（红外）：解除 stdin.write 的永久阻塞（抛 BrokenPipeError 后由推流循环自动重启）"""
        try:
            if self.ffmpeg_out and self.ffmpeg_out.poll() is None:
                print(f"[WARN] 强杀卡死的 FFmpeg 红外推流进程 (PID {self.ffmpeg_out.pid})")
                # 已判定卡死，直接 SIGKILL。SIGTERM 对 SIGSTOP/挂死的进程无效（信号挂起），还会白白多等 3s 超时
                self.ffmpeg_out.kill()
                try:
                    self.ffmpeg_out.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
            else:
                print("[WARN] ffmpeg_out 已退出/不存在（红外），由推流线程 poll() 分支负责重启")
        except Exception as e:
            print(f"[WARN] 强杀 FFmpeg 红外推流异常: {e}")
        # 不在此重启：推流线程的 poll() 分支会统一重启，避免双重重启互相关闭对方的 stdin

    # ---------- FFmpeg 拉流 stderr 读取（解析 PTS + 打印日志）（红外） ----------
    def _ffmpeg_in_stderr_reader(self):
        """读取 ffmpeg_in 的 stderr，解析 showinfo 获取 PTS（红外），用于帧时间校准"""
        while self.running:
            line = self.ffmpeg_in.stderr.readline()
            if not line:
                time.sleep(0.1)
                continue
            decoded = line.decode(errors="replace").strip()

            # 解析 showinfo 获取 PTS（不打印 showinfo 行，避免日志刷屏）
            info = parse_frame_info(decoded)
            if info is not None:
                pts, pts_time = info
                with self.pts_lock:
                    self.current_pts_time = pts_time
                    if self.first_pts_time is None:
                        self.first_pts_time = pts_time
                        self.first_local_time = datetime.now(timezone.utc)
                        cam_cst = self.cam_utc.astimezone(TZ_CST) if self.cam_utc else None
                        if cam_cst:
                            print(f"[PTS-INF] 首帧 pts_time={pts_time:.3f}s, 摄像头基准时间={cam_cst.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")

    # ---------- FFmpeg 日志（红外，仅读取避免管道阻塞） ----------
    def ffmpeg_log(self, p, tag):
        # 仅消费 stderr 防止管道缓冲填满；与 p 生命周期一致，p 退出后线程结束
        while self.running:
            line = p.stderr.readline()
            if not line:
                if p.poll() is not None:
                    break  # 进程已退出且管道读尽，结束本线程
                time.sleep(0.1)
                continue

    # ---------- 启动（红外） ----------
    def run(self):
        # 初始化MQTT
        #self.is_mqtt_connected = init_mqtt()

        # 初始化UDP
        #self.is_udp_connected = init_udp()

        # ===== 获取摄像头 UTC 时间（红外，用于定位时间匹配） =====
        cam_host = parse_rtsp_host(RTSP_HW_IN)
        if cam_host:
            self.cam_local_ref = datetime.now(timezone.utc)
            # 红外源(H30T的MediaMTX)无80端口HTTP服务，但其8889(WebRTC)/8888(HLS)端口响应带标准Date头，
            # 依次尝试 80→8889→8888，取第一个可用的HTTP Date时间源
            cam_utc, cam_source = None, None
            for _port in (80, 8889, 8888):
                print(f"[INF] 获取红外摄像机 UTC 时间（{cam_host}:{_port}）...")
                cam_utc, cam_source = get_camera_utc(cam_host, port=_port)
                if cam_utc:
                    break
            # 红外时间基准保持妙算3时钟系（不平移）：
            # 帧时间与遥测的local_time（妙算3系，逐条自带）直接匹配，两机时钟漂移被逐条遥测自然跟踪
            self.cam_utc, self.cam_source = cam_utc, cam_source
            if self.cam_utc:
                local_cst = self.cam_local_ref.astimezone(TZ_CST)
                cam_cst = self.cam_utc.astimezone(TZ_CST)
                print(f"  时间源: {self.cam_source}")
                print(f"  红外摄像机 北京时间: {cam_cst.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
                print(f"  本地       北京时间: {local_cst.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
                offset = (self.cam_utc - self.cam_local_ref).total_seconds()
                print(f"  时钟偏差: {offset:+.3f}s")
            else:
                print(f"  HTTP Date 均失败，使用本地 UTC 时间作为参考（红外）")
                self.cam_utc = datetime.now(timezone.utc)
                self.cam_source = "本地"
            print("=" * 80)
        else:
            # 无法解析红外相机地址时回退本地时间，保证帧时间链路可用
            self.cam_utc = datetime.now(timezone.utc)
            self.cam_source = "本地"

        self.start_ffmpeg_in()
        self.start_ffmpeg_out()

        # 启动所有线程
        threading.Thread(target=self.reader_thread, daemon=True).start()
        threading.Thread(target=self.inference_thread, daemon=True).start()
        self.process_thread = threading.Thread(target=self.process_loop, daemon=True)
        self.process_thread.start()
        threading.Thread(target=self._ffmpeg_in_stderr_reader, daemon=True).start()
        # ffmpeg_out 的 stderr 线程已改由 start_ffmpeg_out 每次启动时绑定（跟随进程重启）
        threading.Thread(target=self.watchdog_thread, daemon=True).start()

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("[INFO] 退出...（红外）")
            self.running = False

    def stop(self):
        """停止所有服务并清理资源（红外）"""
        self.running = False
        # 终止 FFmpeg 拉流/推流进程，避免残留
        terminate_process(getattr(self, 'ffmpeg_in', None), "FFMPEG_IN_INF")
        terminate_process(getattr(self, 'ffmpeg_out', None), "FFMPEG_OUT_INF")
        # 等待线程池完成任务
        if hasattr(self, 'UPLOAD_EXECUTOR'):
            self.UPLOAD_EXECUTOR.shutdown(wait=True)
        # 关闭MQTT和吊舱连接
        if mqtt_client:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
        close_gimbal()

# ================== 主函数 ==================
if __name__ == "__main__":
    # 创建并运行可见光主处理类（由 ENABLE_VIS_STREAM 开关控制）
    processor = None
    if ENABLE_VIS_STREAM:
        processor = MJPEG_TRT_NVENC()
        print("[INFO] 可见光视频流已启用")
    else:
        print("[INFO] 可见光视频流已禁用（ENABLE_VIS_STREAM=False）")
    # 创建并运行红外视频流处理类（由 ENABLE_INF_STREAM 开关控制）
    processor_inf = None
    if ENABLE_INF_STREAM:
        processor_inf = MJPEG_TRT_NVENC_INF()
        print("[INFO] 红外视频流已启用")
    else:
        print("[INFO] 红外视频流已禁用（ENABLE_INF_STREAM=False）")
    
    # 启动位置计算的MQTT客户端
    location_client = mqtt.Client(client_id="", callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
    location_client.username_pw_set(MQTT_USER, MQTT_PASSWORD)

    # 注册回调函数
    location_client.on_connect = on_connect_location
    location_client.on_message = on_message_location

    # 连接MQTT服务器
    print("🔌 正在连接位置计算MQTT服务器...")
    location_client.connect("192.168.144.125", 1883)  # 服务器地址和端口

    # 开始消息循环
    print("🔄 开始监听无人机位置数据...")
    location_client.loop_start()
    
    # 启动上传线程
    print("[INFO] 启动上传线程...")
    upload_thread = start_upload_thread()
    
    try:
        # 启动处理器
        import threading
        if ENABLE_VIS_STREAM and processor is not None:
            threading.Thread(target=processor.run, daemon=True).start()
        if ENABLE_INF_STREAM and processor_inf is not None:
            threading.Thread(target=processor_inf.run, daemon=True).start()

        # 主线程保持运行
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("[INFO] 退出...")
        # 停止上传线程
        upload_thread_running = False
        upload_thread.join(timeout=2)

        if ENABLE_VIS_STREAM and processor is not None:
            processor.stop()
        if ENABLE_INF_STREAM and processor_inf is not None:
            processor_inf.stop()
        location_client.loop_stop()
        location_client.disconnect()