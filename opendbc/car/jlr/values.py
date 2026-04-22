from dataclasses import dataclass, field
from opendbc.car.docs_definitions import CarDocs

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.lateral import AngleSteeringLimits, ISO_LATERAL_ACCEL

class CarControllerParams:
  ACCEL_MAX = 2.0 # m/s
  ACCEL_MIN = -3.5 # m/s
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    90,  # deg
    ([0., 5., 25.], [2.5, 1.5, 0.2]),
    ([0., 5., 25.], [5., 2.0, 0.3]),
  )

  STEER_DELTA_UP = 3
  STEER_DELTA_DOWN = 5
  STEER_DRIVER_ALLOWANCE = 50
  STEER_DRIVER_MULTIPLIER = 2
  STEER_DRIVER_FACTOR = 1
  STEER_THRESHOLD = 150
  STEER_STEP = 1  # 100 Hz
  STEER_MAX = 500

  STEER_DRIVER_ALLOWANCE = 200
  STEER_DRIVER_MULTIPLIER = 2
  STEER_THRESHOLD = 50
  STEER_STEP = 2  # 50 Hz

class CanBus:
  UNDERBODY = 1
  CAN2FLEXRAY = 5
  CAM = 2



@dataclass(frozen=True, kw_only=True)
class JLRCarSpecs(CarSpecs):
  mass: float = 2000.
  wheelbase: float = 3.105
  steerRatio: float = 16.3
  centerToFrontRatio: float = 0.5


@dataclass
class JLRPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.radar: 'rr_evoque_2021'})

@dataclass
class JLRCarDocs(CarDocs):
  name: str = "JLR EVA2"
  package: str = "All"
class CAR(Platforms):
  JLR_EVA2 = JLRPlatformConfig(
    [JLRCarDocs()],
    JLRCarSpecs(),
  )

DBC = CAR.create_dbc_map()

# Lateral limits and controller parameters for JLR angle control
class CarControllerParams:
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    # Assume EPAS faults above this angle; tune with testing
    360,  # deg
    # JLR uses vehicle-model limiting; rate tables unused here
    ([], []),
    ([], []),

    # Vehicle model-based limits (start conservative; adjust after road test)
    MAX_LATERAL_ACCEL=ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * 0.04),  # ~3.4-3.5 m/s^2
    MAX_LATERAL_JERK=3.0 + (ACCELERATION_DUE_TO_GRAVITY * 0.04),                 # ~3.4-3.5 m/s^3

    # prevent EPS faults and improve low-speed comfort
    MAX_ANGLE_RATE=5,  # deg/20ms frame
  )

  # Angle command is sent every other frame (~50 Hz when DT_CTRL=100 Hz)
  STEER_STEP = 2

