from config_loader import Config
from can_driver import CANDriver
import serial

cfg = Config("driver_config.yaml")

ser = serial.Serial(
    cfg.get("comm", "port"),
    baudrate=cfg.get("comm", "baudrate"),
    timeout=0.001
)

driver = CANDriver(ser, cfg)

NODE_L = cfg.get("nodes", "left")
NODE_R = cfg.get("nodes", "right")

# init
driver.init_speed_mode(NODE_L)
driver.init_speed_mode(NODE_R)

# lấy config
low_th  = cfg.get("driver", "thresholds", "low")
high_th = cfg.get("driver", "thresholds", "high")

dec_low  = cfg.get("driver", "deceleration", "low_speed")
dec_high = cfg.get("driver", "deceleration", "high_speed")

if vx < low_th and last_mode != "LOW":
    driver.set_deceleration(NODE_R, dec_low)
    last_mode = "LOW"

elif vx > high_th and last_mode != "HIGH":
    driver.set_deceleration(NODE_L, dec_high)
    last_mode = "HIGH"