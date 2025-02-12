from common.conversions import Conversions as CV
from common.numpy_fast import clip


SET_SPEED_MIN = 5 * CV.MPH_TO_MS
SET_SPEED_MAX = 120 * CV.MPH_TO_MS
LONG_PRESS_TIME = 50  # 500msec
SHORT_PRESS_STEP = 1
LONG_PRESS_STEP = 5
# Accel Hard limits
ACCEL_HYST_GAP = 0.0  # don't change accel command for small oscillations within this value
ACCEL_MAX = 2. # m/s2
ACCEL_MIN = -3.8  # m/s2
ACCEL_SCALE = 1.

DEFAULT_DECEL = 4.0 # m/s2
START_BRAKE_THRESHOLD = -0.00001 # m/s2
STOP_BRAKE_THRESHOLD = 0.1 # m/s2
START_GAS_THRESHOLD = 0.00001 # m/s2
STOP_GAS_THRESHOLD = 0.0 # m/s2

CHIME_TIME = 8
CHIME_GAP_TIME = 5

def setspeedlogic(set_speed, acc_enabled, acc_enabled_prev, setplus, setminus, resbut, timer, ressetspeed, short_press, vego, gas_set, gas, gas_timer):

    set_speed = int(round((set_speed * CV.MS_TO_MPH), 0))
    vego = int(round((vego * CV.MS_TO_MPH), 0))

    if not acc_enabled and acc_enabled_prev:
      ressetspeed = set_speed

    if not gas or (gas_timer > 200 and setminus):
      gas_set = False
      gas_timer = 0
    elif gas_set:
      gas_timer += 1

    if acc_enabled_prev and acc_enabled:
      if setplus:
        if not short_press:
          if gas and not gas_set:
            if set_speed < vego:
              set_speed = vego
            else:
              set_speed += SHORT_PRESS_STEP
            gas_set = True
          else:
            set_speed += SHORT_PRESS_STEP
          short_press = True
        elif timer % LONG_PRESS_TIME == 0:
            set_speed += (LONG_PRESS_STEP - set_speed % LONG_PRESS_STEP)
        timer += 1
      elif setminus:
        if not short_press:
          if gas and not gas_set:
            set_speed = vego
            gas_set = True
          else:
            set_speed -= SHORT_PRESS_STEP
          short_press = True
        elif timer % LONG_PRESS_TIME == 0:
          if set_speed % LONG_PRESS_STEP > 0:
            set_speed += (LONG_PRESS_STEP - set_speed % LONG_PRESS_STEP)
          set_speed -= LONG_PRESS_STEP
        timer += 1
      else:
        timer = 0
        short_press = False
    elif acc_enabled and not short_press:
      if resbut:
        set_speed = ressetspeed
      else:
        set_speed = vego
      short_press = True
      timer += 1
    else:
      short_press = False
      timer = 0

    set_speed = set_speed * CV.MPH_TO_MS
    set_speed = clip(set_speed, SET_SPEED_MIN, SET_SPEED_MAX)

    return set_speed, short_press, timer, gas_set, ressetspeed, gas_timer


def cruiseiconlogic(acc_enabled, acc_available, has_lead, dist_dec, dist_inc, follow_set, follow_set_prev):

    if follow_set == follow_set_prev:
        follow_set -= dist_dec
        follow_set += dist_inc
        follow_set = clip(follow_set, 1, 4)
    if not (dist_dec or dist_inc):
        follow_set_prev = follow_set

    if acc_enabled:
      cruise_state = 4  # ACC engaged
      if has_lead:
        cruise_icon = follow_set + 11  # ACC green icon with lead
      else:
        cruise_icon = follow_set + 7  # ACC green icon with no lead
    else:
      if acc_available:
        cruise_state = 3 # ACC on
        cruise_icon = follow_set + 1 # ACC white icon
      else:
        cruise_state = 0
        cruise_icon = 0

    return cruise_state, cruise_icon, follow_set, follow_set_prev

def accel_hysteresis(accel, accel_steady):

  # for small accel oscillations within ACCEL_HYST_GAP, don't change the accel command
  if accel > accel_steady + ACCEL_HYST_GAP:
    accel_steady = accel - ACCEL_HYST_GAP
  elif accel < accel_steady - ACCEL_HYST_GAP:
    accel_steady = accel + ACCEL_HYST_GAP
  accel = accel_steady

  return accel, accel_steady

def accel_rate_limit(accel_lim, accel_lim_prev, smoothing_factor=0.4):
    """
    Smooths the changes in acceleration using a low-pass filter.

    Parameters:
    accel_lim (float): The current desired acceleration.
    accel_lim_prev (float): The previous acceleration value.
    smoothing_factor (float): A factor between 0 and 1 that controls the smoothing.
                              Closer to 0 means more smoothing (slower response),
                              closer to 1 means less smoothing (faster response).

    Returns:
    float: The new smoothed acceleration value.
    """
    # Ensure smoothing factor is within the valid range
    if smoothing_factor < 0 or smoothing_factor > 1:
      # raise ValueError("Smoothing factor must be between 0 and 1.")
      return accel_lim

    # Apply the smoothing filter
    accel_smoothed = smoothing_factor * accel_lim + (1 - smoothing_factor) * accel_lim_prev
    return accel_smoothed

def cluster_chime(chime_val, enabled, enabled_prev, chime_timer, gap_timer, mute):

  if not enabled_prev and enabled:
    chime_val = 4
    chime_timer = CHIME_TIME
    gap_timer = 0
  elif enabled_prev and not enabled and not mute:
    chime_val = 7
    chime_timer = CHIME_TIME
    gap_timer = CHIME_GAP_TIME

  if chime_timer > 0:
    chime_timer -= 1
  elif gap_timer > 0:
    gap_timer -= 1
    if gap_timer == 0:
      chime_timer = CHIME_TIME

  return chime_val, chime_timer, gap_timer
