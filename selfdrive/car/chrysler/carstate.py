from cereal import car
from common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from selfdrive.car.chrysler.chryslerlonghelper import SET_SPEED_MIN
from selfdrive.car.interfaces import CarStateBase
from selfdrive.car.chrysler.values import DBC, STEER_THRESHOLD, RAM_CARS
from common.params import Params


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    if CP.carFingerprint in RAM_CARS:
      self.shifter_values = can_define.dv["Transmission_Status"]["Gear_State"]
    else:
      self.shifter_values = can_define.dv["GEAR"]["PRNDL"]
    self.acc_on_button = False
    self.veh_on_timer = 0
    self.axle_torq = 0

  def update(self, cp, cp_cam):

    ret = car.CarState.new_message()

    # lock info
    ret.doorOpen = any([cp.vl["BCM_1"]["DOOR_OPEN_FL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_FR"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["ORC_1"]["SEATBELT_DRIVER_UNLATCHED"] == 1

    ret.brakePressed = cp.vl["ESP_1"]["BRAKE_PEDAL"] == 1  # driver-only
    ret.brake = cp.vl["ESP_8"]["BRAKE_VAL_TOTAL"]
    # ret.brakeLights = bool(cp.vl["ESP_1"]["BRAKE_LIGHT"])
    ret.gas = cp.vl["ACCEL_GAS_22F"]["GAS_PEDAL_POS"]
    ret.gasPressed = ret.gas > 1e-5

    ret.espDisabled = (cp.vl["TRACTION_BUTTON"]["TRACTION_OFF"] == 1)

    ret.wheelSpeeds.fl = cp.vl["ESP_6"]["WHEEL_SPEED_FL"]
    ret.wheelSpeeds.rr = cp.vl["ESP_6"]["WHEEL_SPEED_RR"]
    ret.wheelSpeeds.rl = cp.vl["ESP_6"]["WHEEL_SPEED_RL"]
    ret.wheelSpeeds.fr = cp.vl["ESP_6"]["WHEEL_SPEED_FR"]
    ret.vEgoRaw = cp.vl["ESP_8"]["VEHICLE_SPEED_KPH"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = bool(cp.vl["ESP_8"]["STANDSTILL"])
    self.long_accel = cp.vl["INERTIAL_SENSOR"]["LONG_ACCEL"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["GEAR"]["PRNDL"], None))

    # button presses
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(200, cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 1,
                                                                       cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 2)

    # steering wheel
    ret.steeringAngleDeg = cp.vl["STEERING"]["STEERING_ANGLE"] + cp.vl["STEERING"]["STEERING_ANGLE_HP"]
    ret.steeringRateDeg = cp.vl["STEERING"]["STEERING_RATE"]

    self.acc_on_button_prev = self.acc_on_button
    self.acc_on_button = bool(cp.vl["WHEEL_BUTTONS"]["ACC_BUTTON_ON"])
    self.reg_cc_on_button = bool(cp.vl["WHEEL_BUTTONS"]["REG_CC_BUTTON_ON"])

    ret.cruiseState.enabled = bool(cp.vl["ACC_2"]["ACC_ENABLED"])  # ACC is green.
    ret.cruiseState.available = bool(cp.vl["ACC_2"]["ACC_AVAILABLE"])
    ret.cruiseState.speed = max(cp.vl["DASHBOARD"]["ACC_SET_SPEED_MPH"] * CV.MPH_TO_MS, SET_SPEED_MIN)
    # CRUISE_STATE is a three bit msg, 0 is off, 1 and 2 are Non-ACC mode, 3 and 4 are ACC mode, find if there are other states too
    ret.cruiseState.nonAdaptive = cp.vl["DASHBOARD"]["CRUISE_STATE"] in (1, 2)


    self.desiredExperimentalToggleStatus = False
    if cp.vl["DASHBOARD"]["CRUISE_ICON"] in (2, 8, 12):
      ret.cruiseState.followSettings = 1
    elif cp.vl["DASHBOARD"]["CRUISE_ICON"] in (3, 9, 13):
      ret.cruiseState.followSettings = 2
    elif cp.vl["DASHBOARD"]["CRUISE_ICON"] in (4, 10, 14):
      ret.cruiseState.followSettings = 3
    else:
      ret.cruiseState.followSettings = 4
      self.desiredExperimentalToggleStatus = True

    if self.desiredExperimentalToggleStatus != Params().get_bool('ExperimentalMode'):
      Params().put_bool("ExperimentalMode", self.desiredExperimentalToggleStatus)
    if self.desiredExperimentalToggleStatus != Params().get_bool('ExperimentalLongitudinalEnabled'):
      Params().put_bool("ExperimentalLongitudinalEnabled", self.desiredExperimentalToggleStatus)

    ret.steeringTorque = cp.vl["EPS_2"]["TORQUE_DRIVER"]/4
    ret.steeringTorqueEps = cp.vl["EPS_2"]["TORQUE_MOTOR"]/4 if Params().get_bool("ChryslerMangoLat") else cp.vl["EPS_2"]["TORQUE_MOTOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD/4
    # cruise state
    #cp_cruise = cp_cam if self.CP.carFingerprint in RAM_CARS else cp

    self.steerFaultPermanent = cp.vl["EPS_2"]["LKAS_STEER_FAULT"] == 4
    self.apaFault = cp.vl["EPS_2"]["APA_STEER_FAULT"] == 1
    self.apasteerOn = cp.vl["EPS_2"]["APA_ACTIVE"] == 1

    ret.genericToggle = bool(cp.vl["STEERING_LEVERS"]["HIGH_BEAM_FLASH"])

    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["BLIND_SPOT_WARNINGS"]["BLIND_SPOT_LEFT"] == 1
      ret.rightBlindspot = cp.vl["BLIND_SPOT_WARNINGS"]["BLIND_SPOT_RIGHT"] == 1

    self.lkas_counter = cp_cam.vl["LKAS_COMMAND"]["COUNTER"]
    self.lkas_status_ok = cp_cam.vl["LKAS_HEARTBIT"]["LKAS_BUTTON_LED"]
    self.apa_steer_status = cp.vl["AUTO_PARK_REQUEST"]["APA_STEER_ACT"] == 1
    if self.CP.enablehybridEcu:
       if cp.vl["HYBRID_ECU"]["VEH_ON"] == 1:
         self.veh_on_timer += 1
       else:
         self.veh_on_timer = 0
       self.veh_on = self.veh_on_timer >= 50
       self.axle_torq = cp.vl["AXLE_TORQ"]["AXLE_TORQ"]
       self.axle_torq_max = cp.vl["AXLE_TORQ"]["AXLE_TORQ_MAX"]
       self.axle_torq_min = cp.vl["AXLE_TORQ"]["AXLE_TORQ_MIN"]
       self.hybrid_power_meter = cp.vl["HEV_HMI"]["ELEC_MODE_PERCENT"]
    else:
      self.veh_on_timer += 1
      self.veh_on = self.veh_on_timer >= 200
      self.axle_torq_min = 20.
      self.axle_torq_max = 300.
      self.hybrid_power_meter = 1

    self.acc_hold = bool(cp.vl["ACC_2"]["ACC_STOP"])
    self.lead_dist = cp.vl["DASHBOARD"]["LEAD_DIST"]
    self.wheel_button_counter = cp.vl["WHEEL_BUTTONS"]["COUNTER"]

    self.tcs_active = bool(cp.vl["ESC_ACC_COPY"]["TCS_ACTIVE"])

    self.acc_cancel_button = bool(cp.vl["WHEEL_BUTTONS"]["ACC_CANCEL"]) or self.reg_cc_on_button or self.tcs_active
    self.acc_resume_button = bool(cp.vl["WHEEL_BUTTONS"]["ACC_RESUME"])
    self.acc_setplus_button = bool(cp.vl["WHEEL_BUTTONS"]["ACC_SPEED_INC"])
    self.acc_setminus_button = bool(cp.vl["WHEEL_BUTTONS"]["ACC_SPEED_DEC"])
    self.acc_followdec_button = bool(cp.vl["WHEEL_BUTTONS"]["ACC_FOLLOW_DEC"])
    self.acc_followinc_button = bool(cp.vl["WHEEL_BUTTONS"]["ACC_FOLLOW_INC"])

    self.acc_button_pressed = self.acc_cancel_button or self.acc_resume_button or self.acc_setplus_button or \
                              self.acc_setminus_button or self.acc_followdec_button or self.acc_followinc_button

    ret.accgasOverride = bool(cp.vl["ACCEL_RELATED_120"]["ACC_OVERRIDE"])
    self.accbrakeFaulted = ((cp.vl["ESP_1"]["ACC_BRAKE_FAIL"]) > 0) or ((cp.vl["ACC_ERROR"]["ACC_ERROR"]) > 0)
    self.accengFaulted = (cp.vl["ACCEL_RELATED_120"]["ACC_ENG_OK"]) == 0

    return ret

  @staticmethod
  def get_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("PRNDL", "GEAR"),
      ("DOOR_OPEN_FL", "BCM_1"),
      ("DOOR_OPEN_FR", "BCM_1"),
      ("DOOR_OPEN_RL", "BCM_1"),
      ("DOOR_OPEN_RR", "BCM_1"),
      ("BRAKE_PEDAL", "ESP_1"),
      ("GAS_PEDAL_POS", "ACCEL_GAS_22F"),
      ("WHEEL_SPEED_FL", "ESP_6"),
      ("WHEEL_SPEED_RR", "ESP_6"),
      ("WHEEL_SPEED_RL", "ESP_6"),
      ("WHEEL_SPEED_FR", "ESP_6"),
      ("STEERING_ANGLE", "STEERING"),
      ("STEERING_ANGLE_HP", "STEERING"),
      ("STEERING_RATE", "STEERING"),
      ("TURN_SIGNALS", "STEERING_LEVERS"),
      ("ACC_ENABLED", "ACC_2"),
      ("ACC_AVAILABLE", "ACC_2"),
      ("HIGH_BEAM_FLASH", "STEERING_LEVERS"),
      ("ACC_SET_SPEED_MPH", "DASHBOARD"),
      ("LEAD_DIST", "DASHBOARD"),
      ("CRUISE_STATE", "DASHBOARD"),
      ("TORQUE_DRIVER", "EPS_2"),
      ("DRIVER_TAKEOVER", "EPS_2"),
      ("TORQUE_MOTOR", "EPS_2"),
      ("LKAS_STEER_FAULT", "EPS_2"),
      ("COUNTER", "EPS_2"),
      ("TRACTION_OFF", "TRACTION_BUTTON"),
      ("SEATBELT_DRIVER_UNLATCHED", "ORC_1"),
      ("APA_ACTIVE", "EPS_2"),
      ("APA_STEER_FAULT", "EPS_2"),
      ("ACC_STOP", "ACC_2"),
      ("BLIND_SPOT_RIGHT", "BLIND_SPOT_WARNINGS"),
      ("BLIND_SPOT_LEFT", "BLIND_SPOT_WARNINGS"),
      ("COUNTER", "WHEEL_BUTTONS"),
      ("ACC_RESUME", "WHEEL_BUTTONS"),
      ("ACC_CANCEL", "WHEEL_BUTTONS"),
      ("ACC_SPEED_INC", "WHEEL_BUTTONS"),
      ("ACC_SPEED_DEC", "WHEEL_BUTTONS"),
      ("ACC_FOLLOW_INC", "WHEEL_BUTTONS"),
      ("ACC_FOLLOW_DEC", "WHEEL_BUTTONS"),
      ("ACC_BUTTON_ON", "WHEEL_BUTTONS"),
      ("REG_CC_BUTTON_ON", "WHEEL_BUTTONS"),
      ("CRUISE_ICON", "DASHBOARD"),
      ("STANDSTILL", "ESP_8"),
      ("BRAKE_VAL_TOTAL", "ESP_8"),
      ("VEHICLE_SPEED_KPH", "ESP_8"),
      ("BRAKE_LIGHT", "ESP_1"),
      ("APA_STEER_ACT", "AUTO_PARK_REQUEST"),
      ("ACC_OVERRIDE", "ACCEL_RELATED_120"),
      ("ACC_BRAKE_FAIL", "ESP_1"),
      ("ACC_ENG_OK", "ACCEL_RELATED_120"),
      ("ACC_ERROR", "ACC_ERROR"),
      ("LONG_ACCEL", "INERTIAL_SENSOR"),
      ("TCS_ACTIVE", "ESC_ACC_COPY"),
    ]

    checks = [
      # sig_address, frequency
      ("ESP_1", 50),
      ("EPS_2", 100),
      ("SPEED_1", 100),
      ("ESP_6", 50),
      ("STEERING", 100),
      ("ACC_2", 50),
      ("GEAR", 50),
      ("ACCEL_GAS_134", 50),
      ("DASHBOARD", 15),
      ("STEERING_LEVERS", 10),
      ("ORC_1", 2),
      ("BCM_1", 1),
      ("TRACTION_BUTTON", 1),
      ("BLIND_SPOT_WARNINGS", 2),
      ("ESP_8", 50),
      ("AUTO_PARK_REQUEST", 50),
      ("WHEEL_BUTTONS", 1),
      ("ACCEL_GAS_22F", 50),
      ("ACCEL_RELATED_120", 50),
      ("ACC_ERROR", 0),
      ("INERTIAL_SENSOR", 50),
      ("ESC_ACC_COPY", 50),
    ]

    if CP.enablehybridEcu:
      signals += [
        ("VEH_ON", "HYBRID_ECU", 0),
        ("AXLE_TORQ", "AXLE_TORQ", 0),
        ("AXLE_TORQ_MIN", "AXLE_TORQ", 0),
        ("AXLE_TORQ_MAX", "AXLE_TORQ", 0),
        ("ELEC_MODE_PERCENT", "HEV_HMI", 0),
      ]
      checks += [
        ("HYBRID_ECU", 1),
        ("AXLE_TORQ", 100),
        ("HEV_HMI", 10),
      ]

    if CP.enableBsm:
      signals += [
        ("BLIND_SPOT_RIGHT", "BLIND_SPOT_WARNINGS"),
        ("BLIND_SPOT_LEFT", "BLIND_SPOT_WARNINGS"),
      ]
      checks.append(("BLIND_SPOT_WARNINGS", 2))

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    signals = [
      # sig_name, sig_address
      ("COUNTER", "LKAS_COMMAND"),
      ("LKAS_BUTTON_LED", "LKAS_HEARTBIT")
    ]
    checks = [
      ("LKAS_COMMAND", 100),
      ("LKAS_HEARTBIT", 10),
      ("LKAS_HUD", 4),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], signals, checks, 2)
