from cereal import car
from common.numpy_fast import mean
from selfdrive.config import Conversions as CV
from opendbc.can.can_define import CANDefine
from opendbc.can.parser import CANParser
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.gm.values import DBC, CAR, AccState, CanBus, \
                                    CruiseButtons, STEER_THRESHOLD


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["ECMPRDNL"]["PRNDL"]

    self.prev_distance_button = 0
    self.prev_lka_button = 0
    self.lka_button = 0
    self.distance_button = 0
    self.follow_level = 2
    self.lkMode = True
    self.autoHold = False  ## testing ...
    self.autoHoldActive = False
    self.autoHoldActivated = False
    self.engineRPM = 0
    self.prev_autohold_button = 0
    self.autohold_button = 0
    self.regenPaddlePressed = 0

  def update(self, pt_cp):
    ret = car.CarState.new_message()

    self.prev_cruise_buttons = self.cruise_buttons
    self.cruise_buttons = pt_cp.vl["ASCMSteeringButton"]['ACCButtons']
    #self.prev_lka_button = self.lka_button
    #self.lka_button = pt_cp.vl["ASCMSteeringButton"]["LKAButton"]
    self.prev_autohold_button = self.autohold_button
    self.autohold_button = pt_cp.vl["ASCMSteeringButton"]["LKAButton"]
    self.prev_distance_button = self.distance_button
    self.distance_button = pt_cp.vl["ASCMSteeringButton"]["DistanceButton"]

    ret.wheelSpeeds.fl = pt_cp.vl["EBCMWheelSpdFront"]['FLWheelSpd'] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = pt_cp.vl["EBCMWheelSpdFront"]['FRWheelSpd'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = pt_cp.vl["EBCMWheelSpdRear"]['RLWheelSpd'] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = pt_cp.vl["EBCMWheelSpdRear"]['RRWheelSpd'] * CV.KPH_TO_MS
    #ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr])
    ret.vEgoRaw = mean([ret.wheelSpeeds.fl, ret.wheelSpeeds.fr, ret.wheelSpeeds.rl, ret.wheelSpeeds.rr]) * 1.02
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vEgoRaw < 0.01

    self.angle_steers = pt_cp.vl["PSCMSteeringAngle"]['SteeringWheelAngle']
    self.gear_shifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]['PRNDL'], None))
    self.user_brake = pt_cp.vl["EBCMBrakePedalPosition"]['BrakePedalPosition']
    ret.steeringAngle = pt_cp.vl["PSCMSteeringAngle"]['SteeringWheelAngle']
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(pt_cp.vl["ECMPRDNL"]['PRNDL'], None))
    ret.brake = pt_cp.vl["EBCMBrakePedalPosition"]['BrakePedalPosition'] / 0xd0
    # Brake pedal's potentiometer returns near-zero reading even when pedal is not pressed.
    if ret.brake < 10/0xd0:
      ret.brake = 0.
    #print("ret.brake %f", ret.brake)

    ret.gas = pt_cp.vl["AcceleratorPedal"]['AcceleratorPedal'] / 254.
    ret.gasPressed = ret.gas > 1e-5

    ret.steeringTorque = pt_cp.vl["PSCMStatus"]['LKADriverAppldTrq']
    ret.steeringTorqueEps = pt_cp.vl["PSCMStatus"]['LKATotalTorqueDelivered']
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # 1 - open, 0 - closed
    ret.doorOpen = (pt_cp.vl["BCMDoorBeltStatus"]['FrontLeftDoor'] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]['FrontRightDoor'] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]['RearLeftDoor'] == 1 or
                    pt_cp.vl["BCMDoorBeltStatus"]['RearRightDoor'] == 1)

    # 1 - latched
    ret.seatbeltUnlatched = pt_cp.vl["BCMDoorBeltStatus"]['LeftSeatBelt'] == 0
    ret.leftBlinker = pt_cp.vl["BCMTurnSignals"]['TurnSignals'] == 1
    ret.rightBlinker = pt_cp.vl["BCMTurnSignals"]['TurnSignals'] == 2

    self.park_brake = pt_cp.vl["EPBStatus"]['EPBClosed']
    ret.cruiseState.available = bool(pt_cp.vl["ECMEngineStatus"]['CruiseMainOn'])
    ret.espDisabled = pt_cp.vl["ESPStatus"]['TractionControlOn'] != 1
    self.pcm_acc_status = pt_cp.vl["AcceleratorPedal2"]['CruiseState']

    ret.brakePressed = ret.brake > 1e-5
    # Regen braking is braking
    if self.car_fingerprint == CAR.VOLT:
      ret.brakePressed = ret.brakePressed or bool(pt_cp.vl["EBCMRegenPaddle"]['RegenPaddle'])
      self.regenPaddlePressed = bool(pt_cp.vl["EBCMRegenPaddle"]['RegenPaddle'])
      ##print("self.regenPaddlePressed = ", self.regenPaddlePressed);

    ret.cruiseState.enabled = self.pcm_acc_status != AccState.OFF

    brake_light_enable = True
    if self.car_fingerprint == CAR.BOLT:
      if ret.aEgo < -1.3:
        brake_light_enable = True

    ret.brakeLights = ret.brakePressed or self.regenPaddlePressed or brake_light_enable

    #ret.cruiseState.available = self.main_on
    #ret.cruiseState.enabled = self.pcm_acc_status != 0
    ret.cruiseState.standstill = False

    # 0 - inactive, 1 - active, 2 - temporary limited, 3 - failed
    self.lkas_status = pt_cp.vl["PSCMStatus"]['LKATorqueDeliveredStatus']
    ret.steerWarning = self.lkas_status not in [0, 1]

    ret.steeringTorqueEps = pt_cp.vl["PSCMStatus"]['LKATorqueDelivered']
    self.engineRPM = pt_cp.vl["ECMEngineStatus"]['EngineRPM']

    ret.autoHoldActivated = self.autoHoldActivated

    return ret


  def get_follow_level(self):
    return self.follow_level


  @staticmethod
  def get_can_parser(CP):
    # this function generates lists for signal, messages and initial values
    signals = [
      # sig_name, sig_address, default
      ("BrakePedalPosition", "EBCMBrakePedalPosition", 0),
      ("FrontLeftDoor", "BCMDoorBeltStatus", 0),
      ("FrontRightDoor", "BCMDoorBeltStatus", 0),
      ("RearLeftDoor", "BCMDoorBeltStatus", 0),
      ("RearRightDoor", "BCMDoorBeltStatus", 0),
      ("LeftSeatBelt", "BCMDoorBeltStatus", 0),
      ("RightSeatBelt", "BCMDoorBeltStatus", 0),
      ("TurnSignals", "BCMTurnSignals", 0),
      ("AcceleratorPedal", "AcceleratorPedal", 0),
      ("CruiseState", "AcceleratorPedal2", 0),
      ("ACCButtons", "ASCMSteeringButton", CruiseButtons.UNPRESS),
      ("SteeringWheelAngle", "PSCMSteeringAngle", 0),
      ("FLWheelSpd", "EBCMWheelSpdFront", 0),
      ("FRWheelSpd", "EBCMWheelSpdFront", 0),
      ("RLWheelSpd", "EBCMWheelSpdRear", 0),
      ("RRWheelSpd", "EBCMWheelSpdRear", 0),
      ("PRNDL", "ECMPRDNL", 0),
      ("LKADriverAppldTrq", "PSCMStatus", 0),
      ("LKATorqueDeliveredStatus", "PSCMStatus", 0),
      ("TractionControlOn", "ESPStatus", 0),
      ("EPBClosed", "EPBStatus", 0),
      ("CruiseMainOn", "ECMEngineStatus", 0),
      ("LKAButton", "ASCMSteeringButton", 0),
      ("DistanceButton", "ASCMSteeringButton", 0),
      ("LKATorqueDelivered", "PSCMStatus", 0),
      ("EngineRPM", "ECMEngineStatus", 0),
      ("ACCCmdActive", "ASCMActiveCruiseControlStatus", 0),
      ("LKATotalTorqueDelivered", "PSCMStatus", 0),
    ]

    if CP.carFingerprint == CAR.VOLT or CP.carFingerprint == CAR.BOLT:
      signals += [
        ("RegenPaddle", "EBCMRegenPaddle", 0),
      ]

    return CANParser(DBC[CP.carFingerprint]['pt'], signals, [], CanBus.POWERTRAIN)
