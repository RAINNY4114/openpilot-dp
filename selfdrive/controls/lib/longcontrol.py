import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
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
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0

  def reset(self):
    self.pid.reset()

  # ===============================
  # 1️⃣ 根据车速 + 前车距离动态调节 accel_rate
  # ===============================
  def dynamic_accel_rate(self, v_ego, lead_dist):

    # 速度影响（高速更保守）
    speed_bp = [0., 15., 30.]
    accel_speed_vals = [1.1, 0.9, 0.7]
    decel_speed_vals = [1.0, 0.85, 0.75]

    max_accel_rate = np.interp(v_ego, speed_bp, accel_speed_vals)
    max_decel_rate = np.interp(v_ego, speed_bp, decel_speed_vals)

    # 前车距离影响（距离近 → 更柔）
    if lead_dist is not None:
      dist_bp = [5., 15., 40.]
      dist_factor_vals = [0.6, 0.85, 1.1]
      dist_factor = np.interp(lead_dist, dist_bp, dist_factor_vals)
      max_accel_rate *= dist_factor

    return max_accel_rate, max_decel_rate

  # ===============================
  # 2️⃣ 模拟人类油门非线性
  # ===============================
  def humanize_accel(self, accel):

    # 正加速度压缩（模拟渐进踩油门）
    if accel > 0:
      accel = accel * (1 - 0.25 * np.tanh(accel))

    return accel

  def update(self, active, CS, a_target, should_stop, accel_limits):

    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(
      self.CP, active, self.long_control_state, CS.vEgo,
      should_stop, CS.brakePressed,
      CS.cruiseState.standstill)

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = min(self.CP.startAccel, 0.85)
      self.reset()

    else:
      # 城市更灵敏，高速自动收敛
      speed_factor = np.interp(CS.vEgo, [0., 20., 30.], [1.08, 1.04, 1.02])
      error = (a_target - CS.aEgo) * speed_factor
      output_accel = self.pid.update(error,
                                     speed=CS.vEgo,
                                     feedforward=a_target)

    # 人类油门曲线
    output_accel = self.humanize_accel(output_accel)

    # 读取前车距离（如果存在）
    lead_dist = None
    if hasattr(CS, "leadOne") and CS.leadOne.status:
      lead_dist = CS.leadOne.dRel

    max_accel_rate, max_decel_rate = self.dynamic_accel_rate(CS.vEgo, lead_dist)

    delta = output_accel - self.last_output_accel

    if delta > 0:
      delta = min(delta, 0.7 * max_accel_rate * DT_CTRL)
    else:
      delta = max(delta, -max_decel_rate * DT_CTRL)

    smoothed_accel = self.last_output_accel + delta

    # 防止 0 附近震荡（Ford 必需）
    if abs(smoothed_accel) < 0.02:
      smoothed_accel = 0.0

    self.last_output_accel = np.clip(smoothed_accel,
                                     accel_limits[0],
                                     accel_limits[1])

    return self.last_output_accel
