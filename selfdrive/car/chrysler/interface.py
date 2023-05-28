#!/usr/bin/env python3
from cereal import car
from panda import Panda
from selfdrive.car import STD_CARGO_KG, get_safety_config
from selfdrive.car.chrysler.values import CAR, RAM_HD, RAM_DT, RAM_CARS, ChryslerFlags
from selfdrive.car.interfaces import CarInterfaceBase
from common.params import Params

class CarInterface(CarInterfaceBase):
  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "chrysler"
    ret.dashcamOnly = candidate in RAM_HD

    # radar parsing needs some work, see https://github.com/commaai/openpilot/issues/26842
    ret.radarUnavailable = False # DBC[candidate]['radar'] is None
    ret.steerActuatorDelay = 0.1
    ret.steerLimitTimer = 0.4

    # safety config
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.chrysler)]
    if candidate in RAM_HD:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_CHRYSLER_RAM_HD
    elif candidate in RAM_DT:
      ret.safetyConfigs[0].safetyParam |= Panda.FLAG_CHRYSLER_RAM_DT

    ret.minSteerSpeed = 3.8  # m/s

    ret.lateralTuning.pid.kpBP = [0., 10., 35.]
    ret.lateralTuning.pid.kpV = [0.02, 0.02, 0.02]

    ret.lateralTuning.pid.kiBP = [0., 15., 30.]
    ret.lateralTuning.pid.kiV = [0.003, 0.003, 0.004]

    ret.lateralTuning.pid.kf = 0.00002   # full torque for 10 deg at 80mph means 0.00007818594

    ret.experimentalLongitudinalAvailable = Params().get_bool('ChryslerMangoLong')
    ret.openpilotLongitudinalControl = Params().get_bool('ChryslerMangoLong')

    # Long tuning Params -  make individual params for cars, baseline Pacifica Hybrid
    ret.longitudinalTuning.kpBP = [0., 6., 10., 35.]
    ret.longitudinalTuning.kpV = [.4, .6, 0.5, .2]
    ret.longitudinalTuning.kiBP = [0., 30.]
    ret.longitudinalTuning.kiV = [.001, .001]
    ret.stoppingControl = True
    ret.stoppingDecelRate = 0.2

    if candidate in (CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020, CAR.JEEP_CHEROKEE_2019):
      # TODO: allow 2019 cars to steer down to 13 m/s if already engaged.
      ret.minSteerSpeed = 17.5  if not Params().get_bool('ChryslerMangoLat') and not Params().get_bool('LkasFullRangeAvailable') else 0 # m/s 17 on the way up, 13 on the way down once engaged.
    
    # Chrysler
    if candidate in (CAR.PACIFICA_2017_HYBRID, CAR.PACIFICA_2018, CAR.PACIFICA_2018_HYBRID, CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020):
      ret.wheelbase = 3.089  # in meters for Pacifica Hybrid 2017
      ret.steerRatio = 16.2  # Pacifica Hybrid 2017
      ret.mass = 2242. + STD_CARGO_KG  # kg curb weight Pacifica Hybrid 2017     
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    # Jeep
    elif candidate in (CAR.JEEP_CHEROKEE, CAR.JEEP_CHEROKEE_2019):
      ret.mass = 1778 + STD_CARGO_KG
      ret.wheelbase = 2.71
      ret.steerRatio = 16.7
      ret.steerActuatorDelay = 0.2

      ret.lateralTuning.init('pid')
      ret.lateralTuning.pid.kpBP, ret.lateralTuning.pid.kiBP = [[9., 20.], [9., 20.]]
      ret.lateralTuning.pid.kpV, ret.lateralTuning.pid.kiV = [[0.15, 0.30], [0.03, 0.05]]
      ret.lateralTuning.pid.kf = 0.00006

    # Ram
    elif candidate == CAR.RAM_1500:
      ret.steerActuatorDelay = 0.2
      ret.wheelbase = 3.88
      ret.steerRatio = 16.3
      ret.mass = 2493. + STD_CARGO_KG
      ret.minSteerSpeed = 14.5
      # Older EPS FW allow steer to zero
      if any(fw.ecu == 'eps' and fw.fwVersion[:4] <= b"6831" for fw in car_fw):
        ret.minSteerSpeed = 0.

    elif candidate == CAR.RAM_HD:
      ret.steerActuatorDelay = 0.2
      ret.wheelbase = 3.785
      ret.steerRatio = 15.61
      ret.mass = 3405. + STD_CARGO_KG
      ret.minSteerSpeed = 16
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, 1.0, False)

    else:
      raise ValueError(f"Unsupported car: {candidate}")

    ret.centerToFront = ret.wheelbase * 0.44
    ret.enableBsm = 720 in fingerprint[0]
    ret.enablehybridEcu = 655 in fingerprint[0] or 291 in fingerprint[0]

    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)


    ret.steerFaultPermanent = self.CC.steerErrorMod
    ret.hightorqUnavailable = self.CC.hightorqUnavailable

    # events
    events = self.create_common_events(ret, extra_gears=[car.CarState.GearShifter.low])


    if ret.vEgo < self.CP.minSteerSpeed and not Params().get_bool('ChryslerMangoLat') and not Params().get_bool('LkasFullRangeAvailable'):
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    if self.CC.acc_enabled and (self.CS.accbrakeFaulted or self.CS.accengFaulted):
      events.add(car.CarEvent.EventName.accFaulted)

    ret.events = events.to_msg()

    return ret

  def apply(self, c, now_nanos):
    return self.CC.update(c, self.CS, now_nanos)
