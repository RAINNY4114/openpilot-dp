import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:car.CarControl.Actuators.LongControlState.off+1]
LongCtrlState = car.CarControl.Actuators.LongControlState

def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
    stopping_condition = should_stop
    starting_condition = (not should_stop and not cruise_standstill and not brake_pressed)
    started_condition = v_ego > CP.vEgoStarting

    if not active:
        return LongCtrlState.off

    if long_control_state == LongCtrlState.off:
        if not starting_condition:
            return LongCtrlState.stopping
        return LongCtrlState.starting if CP.startingState else LongCtrlState.pid

    if long_control_state == LongCtrlState.stopping:
        if starting_condition:
            return LongCtrlState.starting if CP.startingState else LongCtrlState.pid

    if long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
        if stopping_condition:
            return LongCtrlState.stopping
        if started_condition:
            return LongCtrlState.pid

    return long_control_state


class LongControl:
    def __init__(self, CP):
        self.CP = CP
        self.long_control_state = LongCtrlState.off
        self.pid = PIDController(
            (CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
            (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
            rate=1 / DT_CTRL
        )
        self.last_output_accel = 0.0
        self.aggressiveness = 0.4  # 初始偏舒适
        self.accel_history = []

    def reset(self):
        self.pid.reset()

    # 自适应驾驶人格
    def update_personality(self, v_ego, a_target, lead_dist):
        target_factor = np.clip(abs(a_target) / 2.0, 0.0, 1.0)
        dist_factor = 0.0
        if lead_dist is not None:
            dist_factor = np.clip((lead_dist - 10.0) / 40.0, 0.0, 1.0)
        desired = 0.3 + 0.4 * target_factor + 0.3 * dist_factor
        self.aggressiveness += (desired - self.aggressiveness) * 0.02

    # 动态加减速率
    def dynamic_accel_rate(self, v_ego, lead_dist):
        a = self.aggressiveness

        # 舒适端和运动端
        accel_soft, decel_soft = 1.2, 3.0
        accel_sport, decel_sport = 2.2, 4.2

        # 市区加速增强
        if v_ego < 15.0:
            a *= 1.25
        elif v_ego < 30.0:
            a *= 1.15

        max_accel_rate = accel_soft * (1 - a) + accel_sport * a
        max_decel_rate = decel_soft * (1 - a) + decel_sport * a

        # 前车近时提高制动
        if lead_dist is not None and lead_dist < 15:
            max_accel_rate *= 0.7
            max_decel_rate *= 1.6

        return max_accel_rate, max_decel_rate

    # 人类化油门曲线
    def humanize_accel(self, accel):
        if accel > 0:
            compress = 0.20 * (1 - self.aggressiveness)
            accel = accel * (1 - compress * np.tanh(accel))
        return accel

    # 主更新
    def update(self, active, CS, a_target, should_stop, accel_limits):
        self.pid.neg_limit = accel_limits[0]
        self.pid.pos_limit = accel_limits[1]

        self.long_control_state = long_control_state_trans(
            self.CP, active, self.long_control_state, CS.vEgo,
            should_stop, CS.brakePressed,
            CS.cruiseState.standstill
        )

        if self.long_control_state == LongCtrlState.off:
            self.reset()
            return 0.0

        elif self.long_control_state == LongCtrlState.stopping:
            output_accel = self.last_output_accel
            if output_accel > self.CP.stopAccel:
                output_accel = min(output_accel, 0.0)
                output_accel -= self.CP.stoppingDecelRate * DT_CTRL
            self.reset()

        elif self.long_control_state == LongCtrlState.starting:
            output_accel = min(self.CP.startAccel, 0.95)
            self.reset()

        else:
            error = a_target - CS.aEgo
            output_accel = self.pid.update(error, speed=CS.vEgo, feedforward=a_target)

        # 前车距离读取
        lead_dist = None
        if hasattr(CS, "leadOne") and CS.leadOne.status:
            lead_dist = CS.leadOne.dRel

        # 更新驾驶人格
        self.update_personality(CS.vEgo, a_target, lead_dist)

        # 人类化油门
        output_accel = self.humanize_accel(output_accel)

        # 紧急制动直通
        if output_accel < -2.2:
            self.last_output_accel = output_accel
            return np.clip(output_accel, accel_limits[0], accel_limits[1])

        # 动态 accel_rate
        max_accel_rate, max_decel_rate = self.dynamic_accel_rate(CS.vEgo, lead_dist)
        delta = output_accel - self.last_output_accel
        if delta > 0:
            delta = min(delta, max_accel_rate * DT_CTRL)
        else:
            delta = max(delta, -max_decel_rate * DT_CTRL)

        smoothed_accel = self.last_output_accel + delta

        # 防震荡
        if abs(smoothed_accel) < 0.05:
            smoothed_accel = 0.0

        self.last_output_accel = np.clip(smoothed_accel, accel_limits[0], accel_limits[1])
        return self.last_output_accel
