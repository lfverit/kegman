from selfdrive.controls.lib.pid import LatPIDController
from selfdrive.controls.lib.drive_helpers import get_steer_max
from cereal import car
from cereal import log
from selfdrive.kegman_conf import kegman_conf
from common.numpy_fast import interp


class LatControlPID():
  def __init__(self, CP):
    self.kegman = kegman_conf(CP)
    self.deadzone = float(self.kegman.conf['deadzone'])
    self.pid = LatPIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                                (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                                (CP.lateralTuning.pid.kdBP, CP.lateralTuning.pid.kdV),
                                k_f=CP.lateralTuning.pid.kf, pos_limit=1.0, neg_limit=-1.0,
                                sat_limit=CP.steerLimitTimer)
    self.angle_steers_des = 0.
    self.angle_steers_des_last = 0.
    self.mpc_frame = 0

    self.angle_steer_rate = [0.3, 0.8, 1.0]
    self.angleBP = [10., 25., 27.0]
    self.angle_steer_new = 0.0


  def reset(self):
    self.pid.reset()

  def live_tune(self, CP):
    self.mpc_frame += 1
    if self.mpc_frame % 300 == 0:
      # live tuning through /data/openpilot/tune.py overrides interface.py settings
      self.kegman = kegman_conf()
      if self.kegman.conf['tuneGernby'] == "1":
        #self.steerKpV = [float(self.kegman.conf['Kp'])]
        #self.steerKiV = [float(self.kegman.conf['Ki'])]
        #self.steerKf = float(self.kegman.conf['Kf'])
        #self.pid = PIController((CP.lateralTuning.pid.kpBP, self.steerKpV),
        #                    (CP.lateralTuning.pid.kiBP, self.steerKiV),
        #                    k_f=self.steerKf, pos_limit=1.0)
        self.deadzone = float(self.kegman.conf['deadzone'])

      self.mpc_frame = 0


  def update(self, active, CS, CP, path_plan):
    self.live_tune(CP)

    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steerAngle = float(CS.steeringAngle)
    pid_log.steerRate = float(CS.steeringRate)

    if CS.vEgo < 0.3 or not active:
      output_steer = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      self.angle_steers_des = path_plan.angleSteers  # get from MPC/PathPlanner
      self.angle_steer_new = interp(CS.vEgo, self.angleBP, self.angle_steer_rate)
      check_pingpong = abs(self.angle_steers_des - self.angle_steers_des_last) > 3.0
      if check_pingpong:
        self.angle_steers_des = path_plan.angleSteers * self.angle_steer_new

      steers_max = get_steer_max(CP, CS.vEgo)
      self.pid.pos_limit = steers_max
      self.pid.neg_limit = -steers_max
      steer_feedforward = self.angle_steers_des   # feedforward desired angle
      self.angle_steers_des_last = self.angle_steers_des
      if CP.steerControlType == car.CarParams.SteerControlType.torque:
        # TODO: feedforward something based on path_plan.rateSteers
        steer_feedforward -= path_plan.angleOffset   # subtract the offset, since it does not contribute to resistive torque
        #steer_feedforward *= CS.vEgo**2  # proportional to realigning tire momentum (~ lateral accel)
        _c1, _c2, _c3 = [0.35189607550172824, 7.506201251644202, 69.226826411091]
        steer_feedforward *= _c1 * CS.vEgo ** 2 + _c2 * CS.vEgo + _c3

      deadzone = self.deadzone

      check_saturation = (CS.vEgo > 10) and not CS.steeringRateLimited and not CS.steeringPressed
      output_steer = self.pid.update(self.angle_steers_des, CS.steeringAngle, check_saturation=check_saturation, override=CS.steeringPressed,
                                     feedforward=steer_feedforward, speed=CS.vEgo, deadzone=deadzone)
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = bool(self.pid.saturated)

    return output_steer, float(self.angle_steers_des), pid_log
