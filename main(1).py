"""
main.py — 钢球检测 + PID + 步进电机控制 (MaixCam2 一体化, 单文件)

架构: MaixCam2 (视觉+PID+电机驱动) --UART2--> 步进电机
STM32 已移除, 所有逻辑在 MaixCam2 上执行。

按钮:
  START = 启动控制 (设零点 + 启动 PID)
  STOP  = 停止控制 (停电机 + 复位 PID)

接线: 电机驱动器接 UART2 (B0=TX, B1=RX), 115200 8N1
"""
from maix import camera, display, app, image, time, gpio, pinmap, err, uart, touchscreen
import struct
import math


# ================================================================
# 模块1: Motor — Emm_V5.0 步进电机驱动 (从 STM32 Emm_V5.c 迁移)
# ================================================================
_MOTOR_CHECKSUM = 0x6B   # Emm_V5.0 固定校验字节 (非计算)


def _to_bytes(*vals):
    """把多个 int 打包成 bytes (每个当无符号字节)"""
    return bytes(v & 0xFF for v in vals)


class Motor:
    """Emm_V5.0 步进电机封装。

    构造时初始化 UART4 + 引脚复用。所有命令内部包 try/except,
    串口失败时打印错误但不抛异常 (避免主循环崩溃)。
    """

    def __init__(self, addr=1, device="/dev/ttyS4", baud=115200,
                 tx_pin="A21", rx_pin="A22"):
        self.addr = addr
        err.check_raise(pinmap.set_pin_function(tx_pin, "UART4_TX"),
                        f"set {tx_pin} to UART4_TX failed")
        err.check_raise(pinmap.set_pin_function(rx_pin, "UART4_RX"),
                        f"set {rx_pin} to UART4_RX failed")
        self.ser = uart.UART(device, baud)

    # ---------- 底层发送 ----------
    def _send(self, frame):
        try:
            ret = self.ser.write(frame)
            # write 返回 <0 表示错误, 记录但不中断
            if ret is not None and ret < 0:
                print(f"[motor] write err code: {ret}")
        except Exception as e:
            print(f"[motor] write failed: {e}")

    # ---------- 命令 (1:1 对应 STM32 Emm_V5.c) ----------
    def reset_pos_to_zero(self):
        """当前位置清零。 [addr,0x0A,0x6D,0x6B] 4字节"""
        self._send(_to_bytes(self.addr, 0x0A, 0x6D, _MOTOR_CHECKSUM))

    def modify_ctrl_mode(self, svF=False, ctrl_mode=2):
        """设置控制模式。 ctrl_mode: 1=开环, 2=闭环。 6字节"""
        self._send(_to_bytes(self.addr, 0x46, 0x69, int(svF), ctrl_mode, _MOTOR_CHECKSUM))

    def en_control(self, state=True, snF=False):
        """使能/失能电机。 state: True=使能。 6字节"""
        self._send(_to_bytes(self.addr, 0xF3, 0xAB, int(state), int(snF), _MOTOR_CHECKSUM))

    def pos_control(self, dir, vel, acc, clk, raF=True, snF=False):
        """位置控制 (核心)。 13字节。
        dir : 0=CW(抬升右端), 1=CCW(降低右端)
        vel : 转速 RPM (0-5000), 大端2字节
        acc : 加速度 0-255, 0=直接启动
        clk : 目标脉冲数 (无符号32位), 大端4字节
        raF : True=绝对位置, False=相对位置
        snF : True=等待同步触发, False=立即执行
        """
        # 大端打包: addr(1)+func(1)+dir(1)+vel(2,BE)+acc(1)+clk(4,BE)+raF(1)+snF(1)+0x6B(1)
        # acc 只有1字节(0-255), 超过会 struct.pack 报错, 这里做保护
        acc_clamped = max(0, min(255, acc))
        frame = struct.pack(">BBBHB", self.addr, 0xFD, dir, vel, acc_clamped) \
              + struct.pack(">I", clk) \
              + bytes([int(raF), int(snF), _MOTOR_CHECKSUM])
        self._send(frame)

    def stop_now(self, snF=False):
        """紧急停止 (所有模式有效)。 5字节"""
        self._send(_to_bytes(self.addr, 0xFE, 0x98, int(snF), _MOTOR_CHECKSUM))

    def origin_set_o(self, svF=True):
        """设置单圈回零零点。 svF: True=保存到flash。 5字节"""
        self._send(_to_bytes(self.addr, 0x93, 0x88, int(svF), _MOTOR_CHECKSUM))

    # ---------- 高层封装 ----------
    def move_abs(self, target_pulse, vel=3000, acc=0):
        """移动到绝对位置 (累计目标)。
        target_pulse: 有符号累计目标, 正=CW, 负=CCW
        自动转换成 (dir, abs_pulse) 调用 pos_control。
        """
        if target_pulse >= 0:
            dir = 0
            pulse = target_pulse
        else:
            dir = 1
            pulse = -target_pulse
        self.pos_control(dir, vel, acc, pulse, raF=True, snF=False)

    def init_sequence(self):
        """电机初始化序列 (对应 STM32 main.c L206-222)"""
        time.sleep_ms(2000)                            # 等驱动器上电完成
        self.modify_ctrl_mode(svF=False, ctrl_mode=2)  # 闭环
        time.sleep_ms(100)
        self.en_control(state=True, snF=False)         # 使能
        time.sleep_ms(100)
        self.reset_pos_to_zero()                       # 当前位置清零
        time.sleep_ms(100)
        time.sleep_ms(1000)                            # 上电1s后设回零零点
        self.origin_set_o(svF=True)                    # 设单圈回零零点(保存)
        time.sleep_ms(100)
        self.reset_pos_to_zero()                       # 再清零


# ================================================================
# 模块2: PID — 位置式 PID 控制器 (从 STM32 pid.c 迁移)
# ================================================================
class PID:
    """位置式 PID + 抗饱和 + 微分测量 + NaN 保护"""

    def __init__(self, kp, ki, kd, out_min, out_max, int_limit, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = 0.0
        self.integral = 0.0
        self.prev_err = 0.0       # deriv_on_meas=True 时存上次 feedback
        self.out_min = out_min
        self.out_max = out_max
        self.int_limit = int_limit
        self.dt = dt if dt > 0 else 0.05
        self.deriv_on_meas = True   # 微分测量, 避免 setpoint 跳变冲击
        self._last_out = 0.0

    @staticmethod
    def _clamp(v, lo, hi):
        if v > hi:
            return hi
        if v < lo:
            return lo
        return v

    def update(self, feedback):
        """运行一次 PID。 feedback 为当前测量值。返回限幅后的控制量。"""
        if math.isnan(feedback):
            return self._last_out

        err = self.setpoint - feedback

        # 积分 + 抗饱和
        self.integral += err * self.dt
        self.integral = self._clamp(self.integral, -self.int_limit, self.int_limit)

        # 微分
        if self.deriv_on_meas:
            deriv = -(feedback - self.prev_err) / self.dt
            self.prev_err = feedback
        else:
            deriv = (err - self.prev_err) / self.dt
            self.prev_err = err

        # 合成 + 限幅
        u = self.kp * err + self.ki * self.integral + self.kd * deriv
        u = self._clamp(u, self.out_min, self.out_max)
        self._last_out = u
        return u

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0
        self._last_out = 0.0

    def set_setpoint(self, setpoint):
        if setpoint != self.setpoint:
            self.setpoint = setpoint
            self.integral = 0.0


# ================================================================
# 模块3: 主程序 — 视觉 + 控制
# ================================================================
# ---- 视觉配置 ----
LED_PIN       = "B25"
LED_GPIO_ID   = "GPIOB25"

cam_h = 320
cam_w = 240

cam = camera.Camera(cam_h, cam_w, image.Format.FMT_GRAYSCALE)
disp = display.Display()

ROI = [0, 115, 320, 35]
BALL_THRESHOLDS = [[240, 255, -128, 127, -128, 127]]
cam.awb_mode(camera.AwbMode.Manual)
cam.set_wb_gain([0.0682, 0, 0, 0.04897])

AREA_MIN        = 100
AREA_MAX        = 200
ASPECT_RATIO_MIN = 0.7
X_STRIDE        = 2
Y_STRIDE        = 2
frame_count = 0

# --- 零点 (固定 x=160) ---
cx0 = 160
IMG_W, IMG_H = 320, 240

# ---- 电机 + PID 配置 ----
MOTOR_ADDR     = 1
MOTOR_VEL_RPM  = 3000
MOTOR_ACC      = 10          # 加速度 0-255, 0=直接启动。超过255会报错(协议只有1字节)

PID_KP         = 2.75
PID_KI         = 0.65        # 加 I 项: 消除摩擦导致的稳态误差(球停在 err=20)
PID_KD         = 1.25         # 加 D 项: 抑制震荡(原来是0,是震荡主因)
PID_OUT_MIN    = -100.0
PID_OUT_MAX    = 100.0
PID_INT_LIMIT  = 100.0
PID_DT         = 0.0167

TARGET_LIMIT   = 800
K_RETURN       = 30.0        # 回零衰减系数(新公式: 每帧衰减 K_RETURN*dt*target)
                              # dt=0.0167, 每帧衰减比例=K_RETURN*dt≈0.5(目标减半)
                              # 调大→摆杆回水平更积极;调小→摆杆保持倾斜更久
VISION_TIMEOUT_MS = 300

# --- 控制状态 ---
g_target_pulse    = 0
g_motor_running   = 0
g_zero_set        = 0
g_control_started = 0
g_last_frame_tick = 0
g_last_sent_pulse = 0      # 上次发送给电机的目标值
g_last_sent_frame = 0      # 上次发送电机的帧号

# --- 任务模式状态 ---
g_task_mode      = 0       # 0=普通PID, 1=任务模式
g_task_phase     = 0       # 0=未开始, 1=前馈→+5cm, 2=PID稳定@-5cm
g_task_start_tick = 0      # 任务开始时间

# ---- 按钮配置 ----
BTN_W, BTN_H = 70, 24
BTN_MARGIN = 4
BTN1_POS = [IMG_W - 2*BTN_W - BTN_MARGIN*2, IMG_H - BTN_H - BTN_MARGIN, BTN_W, BTN_H]
BTN2_POS = [IMG_W - BTN_W - BTN_MARGIN,     IMG_H - BTN_H - BTN_MARGIN, BTN_W, BTN_H]
BTN3_POS = [BTN_MARGIN, IMG_H - BTN_H - BTN_MARGIN, BTN_W, BTN_H]   # 左下角 TASK

# ---- 任务模式参数 (O→+5cm→-5cm稳定) ----
PX_5CM_NEG = 68          # -5cm对应的像素误差 (球在x=92, 距中心160)
PX_5CM_POS = 60          # +5cm对应的像素误差 (球在x=220, 距中心160), 阶段1目标
PID_CONVERGE_THRESH = 45 # 球离+5cm此距离内: 倾角取反+切阶段2
FF_SLOPE_LEVEL = 60      # 远距离前馈倾角(脉冲), 阶段1: 负值推球往+5cm
FF_SLOPE_NEAR  = 120     # 近距离冲刺倾角(脉冲), 距+5cm≤40像素时切换
FF_NEAR_DIST   = 40      # 冲刺倾角的距离阈值(像素)
TASK_TIMEOUT_MS = 5000   # 任务超时5秒

# 调试开关
DEBUG_DRAW  = True
DEBUG_PRINT = True
PRINT_EVERY = 30

# ============================================================
# 初始化
# ============================================================
err.check_raise(pinmap.set_pin_function(LED_PIN, LED_GPIO_ID), "set pin function failed")
led = gpio.GPIO(LED_GPIO_ID, gpio.Mode.OUT)
led.value(1)

# --- 电机 (UART2, B0/B1) ---
motor = Motor(addr=MOTOR_ADDR)
motor.init_sequence()                         # 闭环+使能+清零+回零零点
g_zero_set = 1

# --- PID ---
pid = PID(PID_KP, PID_KI, PID_KD,
          PID_OUT_MIN, PID_OUT_MAX,
          PID_INT_LIMIT, PID_DT)
pid.set_setpoint(0.0)

# --- 触摸屏 ---
ts = touchscreen.TouchScreen()
BTN1_DISP_POS = image.resize_map_pos(IMG_W, IMG_H, disp.width(), disp.height(),
                                     image.Fit.FIT_CONTAIN,
                                     BTN1_POS[0], BTN1_POS[1], BTN1_POS[2], BTN1_POS[3])
BTN2_DISP_POS = image.resize_map_pos(IMG_W, IMG_H, disp.width(), disp.height(),
                                     image.Fit.FIT_CONTAIN,
                                     BTN2_POS[0], BTN2_POS[1], BTN2_POS[2], BTN2_POS[3])
BTN3_DISP_POS = image.resize_map_pos(IMG_W, IMG_H, disp.width(), disp.height(),
                                     image.Fit.FIT_CONTAIN,
                                     BTN3_POS[0], BTN3_POS[1], BTN3_POS[2], BTN3_POS[3])
touch_pressed_already = False

g_last_frame_tick = time.ticks_ms()

# ============================================================
# 功能函数
# ============================================================
def is_in_button(x, y, btn_pos):
    return (btn_pos[0] < x < btn_pos[0] + btn_pos[2] and
            btn_pos[1] < y < btn_pos[1] + btn_pos[3])

def draw_button(img, btn_pos, label, color):
    img.draw_rect(btn_pos[0], btn_pos[1], btn_pos[2], btn_pos[3], color, 2)
    img.draw_string(btn_pos[0] + 4, btn_pos[1] + 6, label, color, 1)

_otsu_threshold = None   # 缓存 Otsu 阈值, 避免每帧重算
_otsu_frame_cnt = 0
_OTSU_UPDATE_EVERY = 10  # 每 10 帧重算一次阈值

def get_binary_image(img_raw):
    global _otsu_threshold, _otsu_frame_cnt
    _otsu_frame_cnt += 1
    if _otsu_threshold is None or _otsu_frame_cnt % _OTSU_UPDATE_EVERY == 0:
        histogram = img_raw.get_histogram()
        _otsu_threshold = histogram.get_threshold().value()
    img_raw.binary([(_otsu_threshold + 20, 255)], True)
    img_raw.dilate(1, 0)
    return img_raw

def detect_ball(img_raw):
    blobs = img_raw.find_blobs(
        BALL_THRESHOLDS,
        roi=ROI,
        x_stride=X_STRIDE,
        y_stride=Y_STRIDE,
        area_threshold=AREA_MIN,
        pixels_threshold=AREA_MIN,
    )
    if not blobs:
        return None
    best = None
    for b in blobs:
        if b.pixels() > AREA_MAX:
            continue
        ratio = min(b.w(), b.h()) / max(b.w(), b.h())
        if ratio >= ASPECT_RATIO_MIN:
            if best is None or b.pixels() > best.pixels():
                best = b
    return best

def start_control():
    """启动控制: 设零点 + 启动 PID"""
    global g_target_pulse, g_control_started, g_motor_running
    if g_motor_running:
        motor.stop_now()
        g_motor_running = 0
    motor.reset_pos_to_zero()
    pid.reset()
    g_target_pulse = 0
    g_control_started = 1
    print("[ctrl] control STARTED")

def stop_control():
    """停止控制: 停电机 + 复位 PID"""
    global g_motor_running, g_control_started
    if g_motor_running:
        motor.stop_now()
        g_motor_running = 0
    pid.reset()
    g_control_started = 0
    print("[ctrl] control STOPPED")

def start_task():
    """启动任务模式: O→+5cm→-5cm稳定 (2阶段: 前馈+翻转→PID)"""
    global g_task_mode, g_task_phase, g_task_start_tick
    global g_target_pulse, g_motor_running, g_control_started
    if g_motor_running:
        motor.stop_now()
        g_motor_running = 0
    motor.reset_pos_to_zero()
    pid.reset()
    g_target_pulse = 0
    g_control_started = 0
    g_task_mode = 1
    g_task_phase = 1          # 阶段1: 前馈推球往+5cm
    g_task_start_tick = time.ticks_ms()
    print("[task] STARTED: O -> +5cm -> -5cm")

def stop_task():
    """停止任务模式"""
    global g_task_mode, g_task_phase, g_motor_running
    if g_motor_running:
        motor.stop_now()
        g_motor_running = 0
    pid.reset()
    g_task_mode = 0
    g_task_phase = 0
    print("[task] STOPPED")

# ============================================================
# 主循环
# ============================================================
# 注意: img.copy() 每帧必须(二值化会破坏原图), MaixCam GC 正常能回收。
# 卡死重启的主因是串口 write 阻塞 + 循环未捕获异常, 已在下方修复。

while not app.need_exit():
    try:
        img = cam.read()
        if img is None:
            continue
        img_copy = img.copy()
        get_binary_image(img)
        ball = detect_ball(img)

        # --- 触摸按钮 ---
        tx, ty, t_pressed = ts.read()
        if t_pressed and not touch_pressed_already:
            touch_pressed_already = True
            if is_in_button(tx, ty, BTN1_DISP_POS):
                start_control()
            elif is_in_button(tx, ty, BTN2_DISP_POS):
                stop_control()
                stop_task()
            elif is_in_button(tx, ty, BTN3_DISP_POS):
                if ball is not None:
                    start_task()
                else:
                    print("[task] no ball, cannot start task")
        elif not t_pressed:
            touch_pressed_already = False

        # --- 误差计算 ---
        ball_lost = (ball is None)
        error_val: int = 0
        if not ball_lost:
            error_val = ball.cx() - cx0
            error_str = f"ERR: {error_val:+d}"
            g_last_frame_tick = time.ticks_ms()
        else:
            error_str = "ERR: LOST"

        # --- 控制逻辑 ---
        if ball_lost:
            if g_motor_running:
                motor.stop_now()
                g_motor_running = 0
            pid.reset()
        elif g_task_mode:
            # ===== 任务模式: 2阶段 (前馈+翻转→PID) =====
            feedback = -float(error_val)
            if g_task_phase == 1:
                # 阶段1: O→+5cm, 纯前馈, 不用PID
                # 负值=CCW=降低=球往正方向/+5cm滚
                dist_to_target = abs(error_val - PX_5CM_POS)
                if dist_to_target > PID_CONVERGE_THRESH:
                    # 远距离: 两级前馈
                    if dist_to_target > FF_NEAR_DIST:
                        g_target_pulse = -FF_SLOPE_LEVEL    # >40像素: 缓坡-60
                    else:
                        g_target_pulse = -FF_SLOPE_NEAR     # 40~45像素: 冲刺-100
                else:
                    # 球到达取反阈值(≤45): 倾角取反(急刹车+反向推向-5cm), 同时切阶段2
                    g_target_pulse = +FF_SLOPE_NEAR         # 取反: +100
                    g_task_phase = 2
                    pid.reset()
                    print("[task] phase 1->2: +5cm, flip & PID @ -5cm")
            elif g_task_phase == 2:
                # 阶段2: -5cm稳定, 复用普通PID结构(参数完全相同)
                pid.set_setpoint(PX_5CM_NEG)
                u = pid.update(feedback)
                if 0 < error_val <= 8:
                    u += 3.0
                elif -8 <= error_val < 0:
                    u -= 3.0
                g_target_pulse += int(u)
                g_target_pulse -= int(K_RETURN * PID_DT * g_target_pulse)
                if g_target_pulse >  TARGET_LIMIT: g_target_pulse =  TARGET_LIMIT
                if g_target_pulse < -TARGET_LIMIT: g_target_pulse = -TARGET_LIMIT
            motor.move_abs(g_target_pulse, vel=MOTOR_VEL_RPM, acc=MOTOR_ACC)
            g_motor_running = 1
        elif not g_zero_set or not g_control_started:
            pass
        else:
            feedback = -float(error_val)
            u = pid.update(feedback)

            if 0 < error_val <= 8:
                u += 3.0
            elif -8 <= error_val < 0:
                u -= 3.0

            g_target_pulse += int(u)
            g_target_pulse -= int(K_RETURN * PID_DT * g_target_pulse)

            if g_target_pulse >  TARGET_LIMIT: g_target_pulse =  TARGET_LIMIT
            if g_target_pulse < -TARGET_LIMIT: g_target_pulse = -TARGET_LIMIT

            motor.move_abs(g_target_pulse, vel=MOTOR_VEL_RPM, acc=MOTOR_ACC)
            g_motor_running = 1

        # --- 任务超时判断 ---
        if g_task_mode and (time.ticks_ms() - g_task_start_tick) > TASK_TIMEOUT_MS:
            print("[task] TIMEOUT! task failed")
            stop_task()

        # --- 视觉丢帧看门狗 ---
        if (time.ticks_ms() - g_last_frame_tick) > VISION_TIMEOUT_MS:
            if g_motor_running:
                motor.stop_now()
                g_motor_running = 0
            pid.reset()
            g_last_frame_tick = time.ticks_ms() - (VISION_TIMEOUT_MS // 2)

        # --- 绘制 ---
        if DEBUG_DRAW:
            img_copy.draw_rect(ROI[0], ROI[1], ROI[2], ROI[3], image.COLOR_GREEN, 1)
            if ball is not None:
                cx, cy = ball.cx(), ball.cy()
                r = max(ball.w(), ball.h()) // 2
                img_copy.draw_circle(cx, cy, r, image.COLOR_BLUE, 2)
                img_copy.draw_cross(cx, cy, image.COLOR_BLUE, 5, 1)

        ctrl_str = "RUN" if g_control_started else "STOP"
        img_copy.draw_string(IMG_W - 90, 4, error_str, image.COLOR_WHITE, 1)
        img_copy.draw_string(IMG_W - 90, 16, ctrl_str, image.COLOR_WHITE, 1)

        # 任务模式状态显示
        if g_task_mode:
            elapsed = (time.ticks_ms() - g_task_start_tick) / 1000.0
            img_copy.draw_string(IMG_W - 90, 28, f"PH:{g_task_phase} T:{elapsed:.1f}",
                                 image.COLOR_YELLOW, 1)

        draw_button(img_copy, BTN1_POS, "START", image.COLOR_WHITE)
        draw_button(img_copy, BTN2_POS, "STOP", image.COLOR_WHITE)
        draw_button(img_copy, BTN3_POS, "TASK", image.COLOR_WHITE)

        # 中线 + ±5cm目标线
        img_copy.draw_line(160, 0, 160, 240, image.COLOR_GREEN, 1)
        img_copy.draw_line(92, 0, 92, 240, image.COLOR_BLACK, 1)
        img_copy.draw_line(220, 0, 220, 240, image.COLOR_GREEN, 1)
        # 任务模式时红色高亮当前目标点
        if g_task_mode:
            if g_task_phase == 1:
                img_copy.draw_line(220, 0, 220, 240, image.COLOR_RED, 2)  # 目标+5cm
            elif g_task_phase == 2:
                img_copy.draw_line(92, 0, 92, 240, image.COLOR_RED, 2)   # 目标-5cm

        fps = time.fps()
        if fps > 0:
            img_copy.draw_string(8, 28, f"fps: {fps:.1f}", image.COLOR_WHITE, scale=1)

        disp.show(img_copy)
        frame_count += 1

        if DEBUG_PRINT and frame_count % PRINT_EVERY == 0:
            tgt_str = f"TGT:{g_target_pulse}"
            print(f"time: {1000/fps:.02f}ms, fps: {fps:.02f}, {error_str}, {ctrl_str}, {tgt_str}")

    except Exception as e:
        # 捕获单帧异常, 避免程序崩溃重启。打印错误后继续下一帧
        print(f"[loop] frame error: {e}")
        # 如果是电机相关错误, 停电机保安全
        try:
            if g_motor_running:
                motor.stop_now()
                g_motor_running = 0
        except:
            pass

led.value(0)
