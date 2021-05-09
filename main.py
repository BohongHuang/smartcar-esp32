from machine import Pin, PWM, UART
from time import sleep, time, sleep_us, sleep_ms, ticks_us, ticks_ms
from ble_uart_peripheral import BLEUART
import bluetooth

pin_back_right_neg = Pin(4)
pin_back_right_pos = Pin(15)
pin_back_left_neg = Pin(5)
pin_back_left_pos = Pin(27)
pin_front_right_neg = Pin(21)
pin_front_right_pos = Pin(22)
pin_front_left_neg = Pin(32)
pin_front_left_pos = Pin(33)
pin_led = Pin(2, Pin.OUT)
pin_dist_sensor_trig = Pin(23, Pin.OUT)
pin_dist_sensor_echo = Pin(34, Pin.IN, Pin.PULL_DOWN)

pwm_freq = 100000

pwm_front_left_pos = PWM(pin_front_left_pos)
pwm_front_left_neg = PWM(pin_front_left_neg)
pwm_front_right_pos = PWM(pin_front_right_pos)
pwm_front_right_neg = PWM(pin_front_right_neg)
pwm_back_left_pos = PWM(pin_back_left_pos)
pwm_back_left_neg = PWM(pin_back_left_neg)
pwm_back_right_pos = PWM(pin_back_right_pos)
pwm_back_right_neg = PWM(pin_back_right_neg)

pwms = [
    pwm_front_left_pos,
    pwm_front_left_neg,
    pwm_front_right_pos,
    pwm_front_right_neg,
    pwm_back_left_pos,
    pwm_back_left_neg,
    pwm_back_right_pos,
    pwm_back_right_neg,
]
for pwm in pwms:
    pwm.freq(pwm_freq)
    pwm.duty(0)
pos = [
    pwm_front_left_pos,
    pwm_front_right_pos,
    pwm_back_left_pos,
    pwm_back_right_pos,
]
neg = [
    pwm_front_left_neg,
    pwm_front_right_neg,
    pwm_back_left_neg,
    pwm_back_right_neg,
]
left_pos = [
    pwm_front_left_pos,
    pwm_back_left_pos
]
left_neg = [
    pwm_front_left_neg,
    pwm_back_left_neg,
]
right_pos = [
    pwm_front_right_pos,
    pwm_back_right_pos
]
right_neg = [
    pwm_front_right_neg,
    pwm_back_right_neg
]

dist_sensor_trig_us = 0
dist_cm = 1000.0

#下降沿外部中断函数
def on_dist_sensor_echo_low(pin):
    global dist_sensor_trig_us, dist_cm
    if dist_sensor_trig_us > 0:
        dist_sensor_echo_us = ticks_us()
        delta = dist_sensor_echo_us - dist_sensor_trig_us
        dist_cm = float(delta) / 58.0
        dist_sensor_trig_us = 0

#上升沿外部中断函数
def on_dist_sensor_echo_high(pin):
    global dist_sensor_trig_us
    dist_sensor_trig_us = ticks_us()
    pin_dist_sensor_echo.irq(trigger=Pin.IRQ_FALLING, handler=on_dist_sensor_echo_low)


#触发距离测量
def dist_sensor_trig():
    global dist_sensor_trig_us
    # dist_sensor_trig_us = ticks_us()
    pin_dist_sensor_echo.irq(trigger=Pin.IRQ_RISING, handler=on_dist_sensor_echo_high)
    pin_dist_sensor_trig.on()
    sleep_us(10)
    pin_dist_sensor_trig.off()


# PWM占空比低时，电机可能不转动，给予速度一个偏移量，使得speed较小的情况下也能让电机转动。
speed_offset = 0.55


# 控制小车移动的函数。
# dir是方向盘。
# speed的正负决定前进或者后退。

def move(dir, speed):
    speed *= (1 - speed_offset)
    if speed > 0:
        speed += speed_offset
    elif speed < 0:
        speed -= speed_offset

    duty_total = speed * 1023

    percentage_left = dir - (-1.0)
    percentage_right = 1.0 - dir

    duty_left = min(percentage_left, 1.0) * duty_total
    duty_right = min(percentage_right, 1.0) * duty_total

    left = left_pos if duty_left > 0 else left_neg
    right = right_pos if duty_right > 0 else right_neg

    for pwm in pwms:
        pwm.duty(0)
    for pwm in left:
        pwm.duty(int(abs(duty_left)))
    for pwm in right:
        pwm.duty(int(abs(duty_right)))


# 状态机
class StateMachine:
    state = None

    def __init__(self):
        self.state = Free(self)

    def set_state(self, to_state):
        from_state = self.state
        self.state.exit(to_state)
        self.state = to_state
        to_state.enter(from_state)

    def update(self, delta):
        self.state.update(delta)


# 状态抽象

class State:
    state_machine = None
    name = 'State'

    def __init__(self, state_machine):
        self.state_machine = state_machine

    def update(self, delta):
        pass

    def enter(self, state_from):
        pass

    def exit(self, state_to):
        pass


# 有时长的状态。经过该时长后，自动调用on_finish()方法。
class DurationState(State):
    remainderTime = 0.5

    def on_finish(self):
        pass

    def update(self, delta):
        self.remainderTime -= delta
        if self.remainderTime <= 0.0:
            self.on_finish()


class BacktrackState(State):
    from_state = None


# 有时长且自动返回的状态。时长结束后自动返回到进入该状态前的状态。
class DurationBacktrackState(DurationState, BacktrackState):
    def __init__(self, state_machine, from_state, duration):
        State.__init__(self, state_machine)
        self.from_state = from_state
        self.remainderTime = duration

    def on_finish(self):
        self.state_machine.set_state(self.from_state)


# 序列状态。每次进入该状态时，进入序列中的下一个状态。
class SequenceState(State):
    sequence = []

    def on_finish(self):
        pass

    def enter(self, state_from):
        if len(self.sequence) > 0:
            self.state_machine.set_state(self.sequence[0])
            del self.sequence[0]
        else:
            self.on_finish()


# 具体状态实现

# 后退
class Back(DurationBacktrackState):
    name = 'Back'

    def enter(self, state_from):
        move(0.0, -0.8)


# 右转
class Rotate(DurationBacktrackState):
    name = 'Rotate'

    def enter(self, state_from):
        move(2.0, 0.8)


# 尝试另一个方向（后退0.5s+右转0.5s）
class TryDirection(SequenceState, BacktrackState):
    name = 'TryDirection'

    def __init__(self, state_machine, from_state):
        super().__init__(state_machine)
        self.from_state = from_state
        self.sequence = [Back(self.state_machine, self, 0.5), Rotate(self.state_machine, self, 0.5)]

    def on_finish(self):
        self.state_machine.set_state(self.from_state)


# 默认状态（小车持续前进，直到距离传感器测量的距离小于20cm，进入TryDirection状态)
class Free(State):
    name = 'Free'

    def enter(self, state_from):
        move(0, 0.8)

    def update(self, delta):
        if dist_cm < 20.0:
            self.state_machine.set_state(TryDirection(self.state_machine, self))


def map_bound(x, ibound, fbound):  # 把[0, ibound]的整数映射成[-fbound, fbound]的浮点数
    if x == ibound // 2:
        return 0.0
    elif x == 0x00:
        return -fbound
    elif x == ibound:
        return fbound
    else:
        return (x - ibound / 2.0) / (ibound / (fbound * 2))


def decode_control_data(buf):  # 解码串口接收的数据
    val_speed = buf[0] & 0x0f
    val_dir = (buf[0] >> 4) & 0x0f
    return map_bound(val_dir, 0x0f, 2.0), map_bound(val_speed, 0x0f, 1.0)


# 受控状态，从串口获取移动指令。
class UARTControlled(State):
    # 当前方向
    dir = 0.0
    # 当前速度
    speed = 0.0
    # 上一次接收数据的时刻
    ticks_ms = 0
    # 接收的数据过期时长（防止蓝牙信号不佳或断开连接导致失控）
    expire_time_ms = 250

    # 主动读取串口数据
    def read_data(self):
        pass

    def update(self, delta):
        received = self.read_data()
        if received and len(received) > 0:
            dir, speed = decode_control_data(received)
            self.dir = dir
            self.speed = speed
            self.ticks_ms = ticks_ms()
        elif ticks_ms() - self.ticks_ms > self.expire_time_ms:
            self.dir = 0
            self.speed = 0
        print(str(ticks_ms() - self.ticks_ms))
        # print('dir: ' + str(self.dir) + ' speed: ' + str(self.speed))
        move(self.dir, self.speed)


# 有线串口控制
class WiredUARTControlled(UARTControlled):
    name = 'WiredUARTControlled'
    uart_wired = UART(2, 115200)

    def read_data(self):
        return self.uart_wired.read(1)

    def enter(self, state_from):
        self.uart_wired.init(115200, bits=8, parity=None, stop=1)


ble = bluetooth.BLE()


# 蓝牙（BLE）串口控制
class BLEUARTControlled(UARTControlled):
    name = 'BLEUARTControlled'
    uart_ble = BLEUART(ble=ble, name="SmartCar", rxbuf=1)

    def enter(self, state_from):
        def on_receive_data():
            data = self.uart_ble.read()
            dir, speed = decode_control_data(data)
            self.dir = dir
            self.speed = speed
            self.ticks_ms = ticks_ms()

        self.uart_ble.irq(on_receive_data)

    def read_data(self):
        return None  # self.uart_ble.read(max(self.uart_ble.any() // 2, 1))


# 刷新状态的间隔时间
delta_time = 1.0 / 10.0


def main():
    state_machine = StateMachine()
    state_controlled = BLEUARTControlled(state_machine)
    state_machine.set_state(state_controlled)
    while True:
        dist_sensor_trig()
        state_machine.update(delta_time)
        sleep(delta_time)


main()

