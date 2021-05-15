from machine import Pin, PWM, UART
from time import sleep, time, sleep_us, sleep_ms, ticks_us, ticks_ms
from ble_uart_peripheral import BLEUART
import bluetooth

CMD_IDLE = 0x00
CMD_BLE_CONTROL = 0x01
CMD_AVOID = 0x02

pin_back_right_neg = Pin(4)
pin_back_right_pos = Pin(23)
pin_back_left_neg = Pin(26)
pin_back_left_pos = Pin(27)
pin_front_right_neg = Pin(21)
pin_front_right_pos = Pin(22)
pin_front_left_neg = Pin(32)
pin_front_left_pos = Pin(33)
pin_dist_sensor_trig = Pin(15, Pin.OUT)
pin_dist_sensor_echo_center = Pin(34, Pin.IN)
pin_dist_sensor_echo_left = Pin(5, Pin.IN)
pin_dist_sensor_echo_right = Pin(25, Pin.IN)
pin_onboard_led = Pin(2, Pin.OUT)

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
dist_sensor_trig_us_left = 0
dist_sensor_trig_us_right = 0
dist_sensor_trig_us_center = 0
dist_cm_left = 1000.0
dist_cm_center = 1000.0
dist_cm_right = 1000.0


# 下降沿外部中断函数
def on_dist_sensor_echo_low_center(pin):
    global dist_sensor_trig_us_center, dist_cm_center
    if dist_sensor_trig_us_center > 0:
        dist_sensor_echo_us = ticks_us()
        delta = dist_sensor_echo_us - dist_sensor_trig_us_center
        dist_cm_center = float(delta) / 58.0
        dist_sensor_trig_us_center = 0


# 上升沿外部中断函数
def on_dist_sensor_echo_high_center(pin):
    global dist_sensor_trig_us_center
    dist_sensor_trig_us_center = ticks_us()
    pin_dist_sensor_echo_center.irq(trigger=Pin.IRQ_FALLING, handler=on_dist_sensor_echo_low_center)


# 下降沿外部中断函数
def on_dist_sensor_echo_low_left(pin):
    global dist_sensor_trig_us_left, dist_cm_left
    if dist_sensor_trig_us_left > 0:
        dist_sensor_echo_us = ticks_us()
        delta = dist_sensor_echo_us - dist_sensor_trig_us_left
        dist_cm_left = float(delta) / 58.0
        dist_sensor_trig_us_left = 0


# 上升沿外部中断函数
def on_dist_sensor_echo_high_left(pin):
    global dist_sensor_trig_us_left
    dist_sensor_trig_us_left = ticks_us()
    pin_dist_sensor_echo_left.irq(trigger=Pin.IRQ_FALLING, handler=on_dist_sensor_echo_low_left)


# 下降沿外部中断函数
def on_dist_sensor_echo_low_right(pin):
    global dist_sensor_trig_us_right, dist_cm_right
    if dist_sensor_trig_us_right > 0:
        dist_sensor_echo_us = ticks_us()
        delta = dist_sensor_echo_us - dist_sensor_trig_us_right
        dist_cm_right = float(delta) / 58.0
        dist_sensor_trig_us_right = 0


# 上升沿外部中断函数
def on_dist_sensor_echo_high_right(pin):
    global dist_sensor_trig_us_right
    dist_sensor_trig_us_right = ticks_us()
    pin_dist_sensor_echo_right.irq(trigger=Pin.IRQ_FALLING, handler=on_dist_sensor_echo_low_right)


# 触发距离测量
def dist_sensor_trig():
    global dist_sensor_trig_us_center
    # dist_sensor_trig_us = ticks_us()
    pin_dist_sensor_echo_left.irq(trigger=Pin.IRQ_RISING, handler=on_dist_sensor_echo_high_left)
    pin_dist_sensor_echo_center.irq(trigger=Pin.IRQ_RISING, handler=on_dist_sensor_echo_high_center)
    pin_dist_sensor_echo_right.irq(trigger=Pin.IRQ_RISING, handler=on_dist_sensor_echo_high_right)
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
        self.state = Avoidance(self)

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
        move(0.0, -0.25)


# 右转
class Rotate(DurationBacktrackState):
    name = 'Rotate'
    dir = 2.0

    def __init__(self, state_machine, from_state, dir, duration):
        super().__init__(state_machine, from_state, duration)
        self.dir = dir

    def enter(self, state_from):
        move(self.dir, 0.25)


# 尝试另一个方向（后退0.5s+右转0.5s）
class TryDirection(SequenceState, BacktrackState):
    name = 'TryDirection'

    def __init__(self, state_machine, from_state, dir):
        super().__init__(state_machine)
        self.from_state = from_state
        self.sequence = [Back(self.state_machine, self, 0.25), Rotate(self.state_machine, self, dir, 0.25)]

    def on_finish(self):
        self.state_machine.set_state(self.from_state)


# 默认状态（小车持续前进，直到距离传感器测量的距离小于20cm，进入TryDirection状态)
class Avoidance(State):
    name = 'Free'

    def enter(self, state_from):
        move(0, 0.25)

    def update(self, delta):
        if min(dist_cm_left, dist_cm_center, dist_cm_right) < 15.0:
            self.state_machine.set_state(
                TryDirection(self.state_machine, self, 2.0 if dist_cm_left < dist_cm_right else -2.0))




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
uart_ble = BLEUART(ble=ble, name="SmartCar", rxbuf=2)



def handle_command(cmd, state_machine):
    if cmd == CMD_IDLE:
        state_machine.set_state(Idle(state_machine))
    elif cmd == CMD_BLE_CONTROL:
        state_machine.set_state(BLEUARTControlled(state_machine))
    elif cmd == CMD_AVOID:
        state_machine.set_state(Avoidance(state_machine))


# 蓝牙（BLE）串口控制
class BLEUARTControlled(UARTControlled):
    name = 'BLEUARTControlled'


    def enter(self, state_from):
        def on_receive_data():
            data = uart_ble.read()
            if data and len(data) > 0:
                if data[0] == CMD_BLE_CONTROL:
                    data = data[1:]
                    if len(data) == 1:
                        dir, speed = decode_control_data(data)
                        self.dir = dir
                        self.speed = speed
                        self.ticks_ms = ticks_ms()
                else:
                    handle_command(data[0], self.state_machine)


        uart_ble.irq(on_receive_data)

    def read_data(self):
        return None  # self.uart_ble.read(max(self.uart_ble.any() // 2, 1))


class Idle(State):
    name = "Idle"
    onboard_led_anim_time = 0.0
    onboard_led_anim_duration = 1.0
    def enter(self, state_from):
        def on_receive_command():
            cmd = uart_ble.read()
            if cmd and len(cmd) > 0:
                handle_command(cmd[0], self.state_machine)
        uart_ble.irq(on_receive_command)
    def exit(self, state_to):
        pin_onboard_led.on()
    def update(self, delta):
        move(0.0, 0.0)
        self.onboard_led_anim_time += delta

        if self.onboard_led_anim_time >= self.onboard_led_anim_duration:
            self.onboard_led_anim_time -= self.onboard_led_anim_time
        if self.onboard_led_anim_time < self.onboard_led_anim_duration / 2.0:
            pin_onboard_led.off()
        else:
            pin_onboard_led.on()

# 刷新状态的间隔时间
delta_time = 1.0 / 10.0

def main():
    state_machine = StateMachine()
    state_machine.set_state(Idle(state_machine))
    while True:
        # print('left: ' + str(dist_cm_left) + ' center: ' + str(dist_cm_center) + ' right: ' + str(dist_cm_right))
        state_machine.update(delta_time)
        dist_sensor_trig()  # 确保中断是在sleep中触发的
        sleep(delta_time)


main()
