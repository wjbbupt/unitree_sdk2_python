import sys
import time
import threading
import yaml
from enum import Enum
from dataclasses import dataclass, field
from typing import List

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_, MotorCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

# ==== Constants & Enums ====
G1_NUM_MOTOR = 29

class MotorType(Enum):
    GearboxS = 0
    GearboxM = 1
    GearboxL = 2

# Mirror of C++ structs for intermediate storage
@dataclass
class ImuState:
    rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    omega: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])

@dataclass
class MotorCommand:
    q_target: List[float] = field(default_factory=lambda: [0.0] * G1_NUM_MOTOR)
    dq_target: List[float] = field(default_factory=lambda: [0.0] * G1_NUM_MOTOR)
    kp: List[float]       = field(default_factory=lambda: [0.0] * G1_NUM_MOTOR)
    kd: List[float]       = field(default_factory=lambda: [0.0] * G1_NUM_MOTOR)
    tau_ff: List[float]   = field(default_factory=lambda: [0.0] * G1_NUM_MOTOR)

@dataclass
class MotorState:
    q: List[float]  = field(default_factory=lambda: [0.0] * G1_NUM_MOTOR)
    dq: List[float] = field(default_factory=lambda: [0.0] * G1_NUM_MOTOR)


# Motor type array copied from C++
G1MotorType = [
    MotorType.GearboxM, MotorType.GearboxM, MotorType.GearboxM, MotorType.GearboxL, MotorType.GearboxS, MotorType.GearboxS,
    MotorType.GearboxM, MotorType.GearboxM, MotorType.GearboxM, MotorType.GearboxL, MotorType.GearboxS, MotorType.GearboxS,
    MotorType.GearboxM, MotorType.GearboxS, MotorType.GearboxS
] + [MotorType.GearboxS] * (G1_NUM_MOTOR - 15)

class G1JointIndex:
    LeftShoulderPitch = 15

class Mode(Enum):
    PR = 0
    AB = 1

# Kp/Kd lookup

def GetMotorKp(idx: int) -> float:
    m = G1MotorType[idx]
    if m in (MotorType.GearboxS, MotorType.GearboxM): 
        return 40.0
    if m == MotorType.GearboxL: 
        return 100.0
    return 0.0

def GetMotorKd(idx: int) -> float:
    return 1.0

# Service mapping

def queryServiceName(form: str, name: str) -> str:
    if form == "0":
        return {"normal":"sport_mode",
                "ai":"ai_sport",
                "advanced":"advanced_sport"
                }.get(name, "")
    return { "ai-w":"wheeled_sport(go2W)",
             "normal-w":"wheeled_sport(b2W)"
           }.get(name, "")

class Custom:
    def __init__(self):
        self.control_dt = 0.002
        self.duration = 3.0
        self.time = 0.0

        # Internal buffers
        self._motor_state = MotorState()
        self._motor_cmd = MotorCommand()
        self._imu_state  = ImuState()

        self.mode_machine = 0
        self._cmd_lock = threading.Lock()

        # CRC helper
        self.crc = CRC()

        # Trajectory frames
        self.motion_frames = []

        # DDS cmd template
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()

    def Init(self):
        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()
        while True:
            status, info = self.msc.CheckMode()
            if status != 0:
                print(f"CheckMode failed: {status}")
            else:
                name = info.get("name", "")
                form = info.get("form", "")
                if not name:
                    print("Service deactivated.")
                    break
                print(f"Service active: {queryServiceName(form,name)}, releasing")
                self.msc.ReleaseMode()
            time.sleep(1.0)

    def Start(self):
        self.load_behavior_library("behavior_lib/motion.seq")
        self.pub = ChannelPublisher("rt/lowcmd", LowCmd_); self.pub.InitChannel()
        self.sub = ChannelSubscriber("rt/lowstate", LowState_);
        self.sub.InitChannel(self.LowStateHandler, 1)

        self.control_thread = RecurrentThread(self.control_dt, self.Control, name="control")
        self.writer_thread  = RecurrentThread(self.control_dt, self.LowCommandWriter, name="command_writer")
        time.sleep(0.2)
        self.control_thread.Start(); self.writer_thread.Start()

    def load_behavior_library(self, path: str):
        motion = yaml.safe_load(open(path, 'r'))
        component = motion["components"][1]

        content = component.get("content", "")
        num_parts = component.get("num_parts", 0)

        print(f"BehaviorName: {path.split('/')[-1]}")
        print(f"{content} with {num_parts}")

        frames = component.get("frames", [])
        self.motion_frames = []
        for frame in frames:
            frame_data = []
            for element in frame:
                frame_data.append(float(element))
            self.motion_frames.append(frame_data)

        print(f"{len(self.motion_frames)} knots with {len(self.motion_frames[0]) if self.motion_frames else 0} DOF")

    def LowStateHandler(self, msg: LowState_):
        if msg.crc != self.crc.Crc(msg): return print("LowState CRC Error")
        # fill MotorState
        for i in range(G1_NUM_MOTOR):
            self._motor_state.q[i]  = msg.motor_state[i].q
            self._motor_state.dq[i] = msg.motor_state[i].dq
        # fill IMU
        self._imu_state.rpy   = list(msg.imu_state.rpy)
        self._imu_state.omega = list(msg.imu_state.gyroscope)
        if self.mode_machine != msg.mode_machine:
            print(f"[Sync] mode_machine = {msg.mode_machine}")
            self.mode_machine = msg.mode_machine

    def Control(self):
        self.time += self.control_dtl
        with self._cmd_lock:
            if self.time < self.duration:
                ratio = min(self.time/self.duration,1.0)
                for i in range(G1_NUM_MOTOR):
                    q_cur = self._motor_state.q[i]
                    self._motor_cmd.q_target[i]  = (0-q_cur)*ratio + q_cur
                    self._motor_cmd.dq_target[i] = 0.0
                    self._motor_cmd.tau_ff[i]    = 0.0
                    self._motor_cmd.kp[i]        = GetMotorKp(i)
                    self._motor_cmd.kd[i]        = GetMotorKd(i)
            else:
                idx = int((self.time-self.duration)/self.control_dt)
                if idx>=len(self.motion_frames): idx=len(self.motion_frames)-1; self.time=0.0
                if idx%100==0: print(f"Frame Index: {idx}")
                for i in range(G1_NUM_MOTOR):
                    self._motor_cmd.q_target[i]  = (self.motion_frames[idx][i-G1JointIndex.LeftShoulderPitch]
                                                    if i>=G1JointIndex.LeftShoulderPitch else 0.0)
                    self._motor_cmd.dq_target[i] = 0.0
                    self._motor_cmd.tau_ff[i]    = 0.0
                    self._motor_cmd.kp[i]        = GetMotorKp(i)
                    self._motor_cmd.kd[i]        = GetMotorKd(i)

    def LowCommandWriter(self):
        with self._cmd_lock:
            cmd = unitree_hg_msg_dds__LowCmd_()
            cmd.mode_pr      = Mode.PR.value
            cmd.mode_machine = self.mode_machine
            # map MotorCommand -> DDS MotorCmd_
            for i in range(G1_NUM_MOTOR):
                cmd.motor_cmd[i].mode = 1
                cmd.motor_cmd[i].q    = self._motor_cmd.q_target[i]
                cmd.motor_cmd[i].dq   = self._motor_cmd.dq_target[i]
                cmd.motor_cmd[i].tau  = self._motor_cmd.tau_ff[i]
                cmd.motor_cmd[i].kp   = self._motor_cmd.kp[i]
                cmd.motor_cmd[i].kd   = self._motor_cmd.kd[i]
            cmd.crc = self.crc.Crc(cmd)
        self.pub.Write(cmd)

if __name__ == "__main__":
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()
    while True:        
        time.sleep(1)