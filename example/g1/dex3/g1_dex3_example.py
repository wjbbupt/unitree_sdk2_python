import sys
import time
import math
import termios
import fcntl
import os
import threading
from dataclasses import dataclass

import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.utils.thread import RecurrentThread

from unitree_sdk2py.idl.default  import (
    unitree_hg_msg_dds__HandCmd_,
    unitree_hg_msg_dds__HandState_,
)
# 导入IDL消息类型
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import (
    HandCmd_ as HGHandCmd_,
    HandState_ as HGHandState_,
)

MOTOR_MAX = 7
SENSOR_MAX = 9

class State:
    INIT = 0
    ROTATE = 1
    GRIP = 2
    STOP = 3
    PRINT = 4

# stateToString Method
def state_to_string(state):
    return {
        State.INIT: "INIT",
        State.ROTATE: "ROTATE",
        State.GRIP: "GRIP",
        State.STOP: "STOP",
        State.PRINT: "PRINT",
    }.get(state, "UNKNOWN")

#set URDF Limits
MAX_LIMITS_LEFT = [1.05, 1.05, 1.75, 0, 0, 0, 0]  # set max motor value
MIN_LIMITS_LEFT = [-1.05, -0.724, 0, -1.57, -1.75, -1.57, -1.75]
MAX_LIMITS_RIGHT = [1.05, 0.742, 0, 1.57, 1.75, 1.57, 1.75]
MIN_LIMITS_RIGHT = [-1.05, -1.05, -1.75, 0, 0, 0, 0]

@dataclass
class RISMode:
    id: int = 0
    status: int = 0
    timeout: int = 0

    def to_byte(self):
        mode = 0
        mode |= (self.id & 0x0F)
        mode |= (self.status & 0x07) << 4
        mode |= (self.timeout & 0x01) << 7
        return mode

class Dex3HandController:
    def __init__(self, hand_side: str, network_interface: str):
        self.hand_side = hand_side.upper()
        self.hand_id = 0 if self.hand_side == 'L' else 1

        if self.hand_id == 0:
            self.dds_namespace = "rt/dex3/left"
            self.sub_namespace = "rt/dex3/left/state"
        else:
            self.dds_namespace = "rt/dex3/right"
            self.sub_namespace = "rt/dex3/right/state"

        self.network_interface = network_interface

        self.current_state = State.INIT
        self._state_lock =  threading.Lock()

        self.msg = unitree_hg_msg_dds__HandCmd_()
        self.state = unitree_hg_msg_dds__HandState_()

        self.handcmd_publisher = None
        self.handstate_subscriber = None

        self._rotate_count = 1
        self._rotate_dir = 1

        self._input_thread = None
        self._running = False

    def _acquire_lock(self):
        self._state_lock.acquire()

    def _release_lock(self):
        self._state_lock.release()

    def Init(self):
        ChannelFactoryInitialize(0, self.network_interface)

        self.handcmd_publisher = ChannelPublisher(self.dds_namespace + "/cmd", HGHandCmd_)
        self.handcmd_publisher.Init()

        self.handstate_subscriber = ChannelSubscriber(self.sub_namespace, HGHandState_)
        self.handstate_subscriber.Init(self._state_handler, 1)

    def Start(self):
        self._running = True

        self._input_thread = RecurrentThread(interval=0.1, target=self._user_input_loop, name="UserInputThread")
        self._input_thread.Start()

        last_state = None

        while self._running:
            self._acquire_lock()
            cur_state = self.current_state
            self._release_lock()

            if cur_state != last_state:
                print(f"\n--- Current State: {state_to_string(cur_state)} ---")
                print("Commands:")
                print("  r - Rotate")
                print("  g - Grip")
                print("  p - Print_state")
                print("  q - Quit")
                print("  s - Stop")
                last_state = cur_state

            if cur_state == State.INIT:
                print("Initializing...")
                self._acquire_lock()
                self.current_state = State.ROTATE
                self._release_lock()

            elif cur_state == State.ROTATE:
                self.rotate_motors(self.hand_id == 0)

            elif cur_state == State.GRIP:
                self.grip_hand(self.hand_id == 0)

            elif cur_state == State.STOP:
                self.stop_motors()

            elif cur_state == State.PRINT:
                self.print_state(self.hand_id == 0)

            else:
                print("Invalid state!")
                self._running = False

            time.sleep(0.01)

        self._input_thread.Wait()

    def _state_handler(self, msg: HGHandState_):
        self.state = msg
    
    # this method can send kp and kd to motors
    def rotate_motors(self, is_left_hand: bool):
        max_limits = MAX_LIMITS_LEFT if is_left_hand else MAX_LIMITS_RIGHT
        min_limits = MIN_LIMITS_LEFT if is_left_hand else MIN_LIMITS_RIGHT

        for i in range(MOTOR_MAX):
            ris_mode = RISMode(i, 0x01, 0)
            mode = ris_mode.to_byte()

            motor_cmd = self.msg.motor_cmd[i]
            motor_cmd.mode = mode
            motor_cmd.tau = 0
            motor_cmd.kp = 0.5
            motor_cmd.kd = 0.1

            range_ = max_limits[i] - min_limits[i]
            mid = (max_limits[i] + min_limits[i]) / 2.0
            amplitude = range_ / 2.0

            q = mid + amplitude * math.sin(self._rotate_count / 20000.0 * math.pi)
            motor_cmd.q = q

        self.handcmd_publisher.Write(self.msg)

        self._rotate_count += self._rotate_dir
        if self._rotate_count >= 10000:
            self._rotate_dir = -1
        elif self._rotate_count <= -10000:
            self._rotate_dir = 1

        time.sleep(0.0001)

    # this method can send kp and kd to motors
    def grip_hand(self, is_left_hand: bool):
        max_limits = MAX_LIMITS_LEFT if is_left_hand else MAX_LIMITS_RIGHT
        min_limits = MIN_LIMITS_LEFT if is_left_hand else MIN_LIMITS_RIGHT

        for i in range(MOTOR_MAX):
            ris_mode = RISMode(i, 0x01, 0)
            mode = ris_mode.to_byte()

            motor_cmd = self.msg.motor_cmd[i]
            motor_cmd.mode = mode
            motor_cmd.tau = 0

            mid = (max_limits[i] + min_limits[i]) / 2.0
            motor_cmd.q = mid
            motor_cmd.dq = 0
            motor_cmd.kp = 1.5
            motor_cmd.kd = 0.1

        self.handcmd_publisher.Write(self.msg)
        time.sleep(1)

    #this method can send dynamic position to motors
    def stop_motors(self):
        for i in range(MOTOR_MAX):
            ris_mode = RISMode(i, 0x01, 1)
            mode = ris_mode.to_byte()

            motor_cmd = self.msg.motor_cmd[i]
            motor_cmd.mode = mode
            motor_cmd.tau = 0
            motor_cmd.dq = 0
            motor_cmd.kp = 0
            motor_cmd.kd = 0
            motor_cmd.q = 0

        self.handcmd_publisher.Write(self.msg)
        time.sleep(1)

    #this method can subscribe dds and show the position for now
    def print_state(self, is_left_hand: bool):
        max_limits = MAX_LIMITS_LEFT if is_left_hand else MAX_LIMITS_RIGHT
        min_limits = MIN_LIMITS_LEFT if is_left_hand else MIN_LIMITS_RIGHT

        q = np.zeros(MOTOR_MAX, dtype=np.float32)
        if len(self.state.motor_state) < MOTOR_MAX:
            print("Waiting for valid motor_state data...")
            return
        for i in range(MOTOR_MAX):
            q[i] = self.state.motor_state[i].q
            q[i] = (q[i] - min_limits[i]) / (max_limits[i] - min_limits[i])
            q[i] = np.clip(q[i], 0.0, 1.0)

        print("\033[2J\033[H", end='')
        print("-- Hand State --")
        print("--- Current State: Test ---")
        print("Commands:")
        print("  r - Rotate")
        print("  g - Grip")
        print("  t - Test")
        print("  q - Quit")

        if is_left_hand:
            print(" L:", q)
        else:
            print(" R:", q)

        time.sleep(0.1)

    def _user_input_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            new_settings = termios.tcgetattr(fd)
            new_settings[3] &= ~(termios.ICANON | termios.ECHO)
            termios.tcsetattr(fd, termios.TCSANOW, new_settings)

            old_flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, old_flags | os.O_NONBLOCK)

            try:
                c = sys.stdin.read(1)
                if c == '':
                    return
                c = c.lower()
                self._acquire_lock()
                if c == 'q':
                    print("Exiting...")
                    self.current_state = State.STOP
                    self._running = False
                elif c == 'r':
                    self.current_state = State.ROTATE
                elif c == 'g':
                    self.current_state = State.GRIP
                elif c == 'p':
                    self.current_state = State.PRINT
                elif c == 's':
                    self.current_state = State.STOP
                self._release_lock()
            except IOError:
                pass
            finally:
                fcntl.fcntl(fd, fcntl.F_SETFL, old_flags)
        finally:
            termios.tcsetattr(fd, termios.TCSANOW, old_settings)

def main():
    print(" --- Unitree Robotics --- ")
    print("     Dex3 Hand Example      \n")

    input_hand = input("Please input the hand id (L for left hand, R for right hand): ").strip().upper()

    if input_hand not in ('L', 'R'):
        print("Invalid hand id. Please input 'L' or 'R'.")
        return -1

    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} networkInterface")
        return -1

    controller = Dex3HandController(input_hand, sys.argv[1])
    controller.Init()
    controller.Start()

if __name__ == "__main__":
    main()