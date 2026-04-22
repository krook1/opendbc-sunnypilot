import math
from opendbc.can import CANDefine
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.jlr.values import DBC, CanBus, CarControllerParams


class CarState(CarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)

    can_define = CANDefine(DBC[CP.carFingerprint]["radar"])
    # Use CarStateBase.out / out_sp as rolling previous-state buffers
    self.shifter_values = can_define.dv["GearPRND"]["PRND"]
    self.params = CarControllerParams()
    self.wheelbase = CP.wheelbase
    self.lkas_button = 0
    self.frame = 0

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    # External panda is index 1 -> buses 4-7. Use bus 4 for main traffic.
    cp_main = CANParser("rr_evoque_2021",
      [("WheelSpeedFront", float("nan")),
       ("WheelSpeedRear", float("nan")),
       ("Info02", float("nan")),
       ("SWM_Angle", float("nan")),
       ("Eps01", float("nan")),
       ("GearPRND", float("nan"))],
       bus=1)
    # One-time DBC config; avoid doing this in the control loop
    #cp_main.dbc.name_to_msg["WheelSpeedFront"].ignore_checksum = True
    #cp_main.dbc.name_to_msg["WheelSpeedFront"].ignore_counter = True
    #cp_main.dbc.name_to_msg["WheelSpeedRear"].ignore_checksum = True
    #cp_main.dbc.name_to_msg["WheelSpeedRear"].ignore_counter = True
    #cp_main.dbc.name_to_msg["SWM_Angle"].ignore_checksum = True
    #cp_main.dbc.name_to_msg["SWM_Angle"].ignore_counter = True
    # Yaw and driver torque parsing; ignore checks for now
    #cp_main.dbc.name_to_msg["Eps01"].ignore_checksum = True
    #cp_main.dbc.name_to_msg["Eps01"].ignore_counter = True
    # Gear switch parsing uses cycle base 3; ignore checks
    #cp_main.dbc.name_to_msg["GearPRND"].ignore_checksum = True
    #cp_main.dbc.name_to_msg["GearPRND"].ignore_counter = True

    cp_sas = CANParser("rr_evoque_2021", [("LKAS_OP_TO_FLEXRAY", float("nan"))], bus=5)
    # ACC RX is currently synthetic; ignore checks until real CRC/counter implemented
    cp_sas.dbc.name_to_msg["LKAS_OP_TO_FLEXRAY"].ignore_checksum = True
    cp_sas.dbc.name_to_msg["LKAS_OP_TO_FLEXRAY"].ignore_counter = True
    return {
      Bus.main: cp_main,
      Bus.adas: cp_sas,
    }

  def _demux_last(self, cp: CANParser, msg: str, cc_sig: str, val_sig: str, cycle_base: int) -> tuple[bool, float]:
    cc_list = cp.vl_all[msg].get(cc_sig, [])
    val_list = cp.vl_all[msg].get(val_sig, [])
    for i in range(len(cc_list) - 1, -1, -1):
      if int(cc_list[i]) == cycle_base:
        return True, float(val_list[i])
    return False, 0.0

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.main]
    cp_sas = can_parsers[Bus.adas]
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # Previous state snapshot (avoids extra allocations and getattr fallback)
    prev = self.out

    # fl = cp.vl["wheel_speed"].get("FL", 0.0)
    # fr = cp.vl["wheel_speed"].get("FR", 0.0)
    # rl = cp.vl["wheel_speed"].get("RL", 0.0)
    # rr = cp.vl["wheel_speed"].get("RR", 0.0)
    # self.parse_wheel_speeds(ret, fl, fr, rl, rr, CV.KPH_TO_MS)

    # Batch demux using helper; BMW DBC uses fixed cycle codes
    #veh_found, veh_speed_kph = self._demux_last(cp, "vehicle_speed", "cycle_count", "veh_speed", cycle_base=3)

    #if veh_found:
      #ret.vEgoRaw = veh_speed_kph * CV.KPH_TO_MS
      #ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
      #ret.vEgoCluster = ret.vEgoRaw
    #else:
      #ret.vEgoRaw = prev.vEgoRaw
      #ret.vEgo = prev.vEgo
      #ret.aEgo = prev.aEgo
      #ret.vEgoCluster = float(prev.vEgoCluster)

    #xxself.parse_wheel_speeds(ret,
      #xxcp.vl["WheelSpeedFront"]["SpeedLeft"],
      #xxcp.vl["WheelSpeedFront"]["SpeedRight"],
      #xxcp.vl["WheelSpeedRear"]["SpeedLeft"],
      #xxcp.vl["WheelSpeedRear"]["SpeedRight"],
      #xx)
    self.parse_wheel_speeds(ret, 0, 0, 0, 0,)

    # Steering angle: choose cycle_count == 0 if present
    #eps_found, eps_angle = self._demux_last(cp, "EPS_Angle", "cycle_count", "steering_angle", cycle_base=0)
    #if eps_found:
      #ret.steeringAngleDeg = float(eps_angle)
    #else:
      #ret.steeringAngleDeg = float(prev.steeringAngleDeg)
    #xxret.steeringAngleDeg = cp.vl["SWM_Angle"]["SteerAngle"]
    #xxret.steeringRateDeg = cp.vl["SWM_Angle"]["SteerRate"]  # TODO
    ret.steeringAngleDeg = 0
    ret.steeringRateDeg = 0

    ret.standstill = ret.vEgoRaw < 0.01

    # Gear parsing via demux (cycle base 3). If cycle_count is absent, fallback to mux field name.
    #gear_found, gear_val = self._demux_last(cp, "maybe_gear_switch", "cycle_count", "GEAR", cycle_base=3)

    #if gear_found:
      #gear_int = int(gear_val)
      #if gear_int == 4:
        #ret.gearShifter = structs.CarState.GearShifter.drive
      #elif gear_int == 5:
        #ret.gearShifter = structs.CarState.GearShifter.reverse
      #elif gear_int == 2:
        ## "P or N" -> disambiguate with P_locked (1=Park, 0=Neutral)
        #p_found, p_locked_val = self._demux_last(cp, "maybe_gear_switch", "cycle_count", "P_locked", cycle_base=3)
        #if not p_found:
          #p_found, p_locked_val = self._demux_last(cp, "maybe_gear_switch", "NEW_SIGNAL_1", "P_locked", cycle_base=3)
        #p_locked_int = int(p_locked_val) if p_found else 0
        #ret.gearShifter = structs.CarState.GearShifter.park if p_locked_int == 1 else structs.CarState.GearShifter.neutral
      #else:
        ## Unknown mapping, keep previous
        #ret.gearShifter = prev.gearShifter
    #else:
      ## Fallback to previous if demux not available
      #ret.gearShifter = prev.gearShifter

    #xxgear = cp.vl["GearPRND"]["PRND"]
    gear = 3
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))
    #if ret.gearShifter != structs.CarState.GearShifter.drive :
    #self.frame += 1
    #if(self.frame % 100 == 0) :
       #carlog.warning(f"gear data: {ret.gearShifter}")

    # ACC assist_mode demux with cycle base 1
    #acc_found, acc_assist_mode = self._demux_last(cp_sas, "LKAS_OP_TO_FLEXRAY", "cycle_count", "assist_mode", cycle_base=1)
    #ret.cruiseState.enabled = bool(int(acc_assist_mode)) if acc_found else bool(prev.cruiseState.enabled)
    #ret.cruiseState.available = True

    #xxret.cruiseState.enabled = cp.vl["CruiseInfo"]["CruiseOn"] == 1
    ret.cruiseState.enabled = True
    ret.cruiseState.speed = ret.vEgoRaw
    ret.cruiseState.nonAdaptive = False
    ret.cruiseState.standstill = False

    prev_lkas_button = self.lkas_button
    #xxself.lkas_button = bool(cp.vl["LKAS_BTN"]["LKAS_Btn_on"])
    self.lkas_button = True
    ret.cruiseState.available = self.lkas_button

    # Yaw rate (deg/s -> rad/s), demux cycle base 0
    #yaw_found, yaw_deg_s = self._demux_last(cp, "NEW_MSG_38", "cycle_count", "yaw", cycle_base=0)
    #if yaw_found:
      #ret.yawRate = float(yaw_deg_s) * CV.DEG_TO_RAD
    #else:
      #ret.yawRate = float(prev.yawRate)

    steer_rad = math.radians(ret.steeringAngleDeg)
    yawrate_rad_s = math.tan(steer_rad) * ret.vEgo / self.wheelbase

    ret.yawRate = math.degrees(yawrate_rad_s)

    # Driver steering torque (native units from CAN)
    #steering_torque_found, steering_torque = self._demux_last(cp, "steer_torque", "cycle_count", "driver_steer_torque", cycle_base=0)
    #if steering_torque_found:
      #ret.steeringTorque = float(steering_torque)
    #else:
      #ret.steeringTorque = float(prev.steeringTorque)

    # TODO torq TorqEPS Pressed
    #xxret.steeringTorque = cp.vl["SWM_Torque"]["TorqueDriver"]
    #xxret.steeringTorqueEps = cp.vl["PSCM_Out"]["AngleTorque"]
    ret.steeringTorque = 0
    ret.steeringTorqueEps = 0
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 150, 5)
    #TODO Fix
    #ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_THRESHOLD, 5)

    return ret, ret_sp


