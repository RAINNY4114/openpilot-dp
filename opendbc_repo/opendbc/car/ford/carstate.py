from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import DBC, CarControllerParams, FordFlags
from opendbc.car.interfaces import CarStateBase


ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

    # ==============================================================
    # Gear selector
    #
    # Older Ford/Lincoln:
    #   PowertrainData_10 / TrnRng_D_Rq
    #
    # ALT_STEER_ANGLE platform:
    #   TransGearData / GearLvrPos_D_Actl
    # ==============================================================

    if CP.transmissionType == TransmissionType.automatic:
      if CP.flags & FordFlags.ALT_STEER_ANGLE:
        self.shifter_values = (
          can_define.dv["TransGearData"]
          ["GearLvrPos_D_Actl"]
        )
      else:
        self.shifter_values = (
          can_define.dv["PowertrainData_10"]
          ["TrnRng_D_Rq"]
        )

    self.distance_button = 0
    self.lc_button = 0

    # Used by ALT_STEER_ANGLE Ford/Lincoln platforms.
    self.steering_angle_offset_deg = 0.0


  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()

    # ==============================================================
    # Steering sensor validity
    #
    # ALT_STEER_ANGLE:
    #   ParkAid_Data / EPASExtAngleStatReq
    #
    # Older Ford/Lincoln:
    #   SteeringPinion_Data / StePinCompAnEst_D_Qf
    #
    # The older Ford/Lincoln check is especially important during
    # startup because the steering pinion offset can recalibrate.
    # ==============================================================

    if self.CP.flags & FordFlags.ALT_STEER_ANGLE:
      ret.vehicleSensorsInvalid = (
        cp.vl["ParkAid_Data"]
        ["EPASExtAngleStatReq"] != 0
      )
    else:
      ret.vehicleSensorsInvalid = (
        cp.vl["SteeringPinion_Data"]
        ["StePinCompAnEst_D_Qf"] != 3
      )

    # ==============================================================
    # Vehicle speed
    # ==============================================================

    ret.vEgoRaw = (
      cp.vl["BrakeSysFeatures"]
      ["Veh_V_ActlBrk"] *
      CV.KPH_TO_MS
    )

    ret.vEgo, ret.aEgo = self.update_speed_kf(
      ret.vEgoRaw
    )

    ret.yawRate = (
      cp.vl["Yaw_Data_FD1"]
      ["VehYaw_W_Actl"]
    )

    ret.standstill = (
      cp.vl["DesiredTorqBrk"]
      ["VehStop_D_Stat"] == 1
    )

    # ==============================================================
    # Gas pedal
    # ==============================================================

    ret.gasPressed = (
      cp.vl["EngVehicleSpThrottle"]
      ["ApedPos_Pc_ActlArb"] / 100.0
      > 1e-6
    )

    # ==============================================================
    # Brake pedal
    # ==============================================================

    # Keep DragonPilot's original brake torque calculation.
    ret.brake = (
      cp.vl["BrakeSnData_4"]
      ["BrkTot_Tq_Actl"] / 32756.0
    )

    ret.brakePressed = (
      cp.vl["EngBrakeData"]
      ["BpedDrvAppl_D_Actl"] == 2
    )

    ret.parkingBrake = (
      cp.vl["DesiredTorqBrk"]
      ["PrkBrkStatus"] in (1, 2)
    )

    # ==============================================================
    # Steering angle
    #
    # ALT_STEER_ANGLE:
    #   SteeringPinion_Data_Alt
    #   ParkAid_Data
    #
    # Older Ford/Lincoln:
    #   SteeringPinion_Data
    # ==============================================================

    if self.CP.flags & FordFlags.ALT_STEER_ANGLE:
      steering_angle_init = (
        cp.vl["SteeringPinion_Data_Alt"]
        ["StePinRelInit_An_Sns"]
      )

      steering_angle_est = (
        cp.vl["ParkAid_Data"]
        ["ExtSteeringAngleReq2"]
      )

      self.steering_angle_offset_deg = (
        steering_angle_est -
        steering_angle_init
      )

      ret.steeringAngleDeg = (
        steering_angle_init +
        self.steering_angle_offset_deg
      )

    else:
      # ============================================================
      # Old Ford/Lincoln steering angle
      # ============================================================

      ret.steeringAngleDeg = (
        cp.vl["SteeringPinion_Data"]
        ["StePinComp_An_Est"]
      )

    ret.steeringTorque = (
      cp.vl["EPAS_INFO"]
      ["SteeringColumnTorque"]
    )

    ret.steeringPressed = (
      self.update_steering_pressed(
        abs(ret.steeringTorque) >
        CarControllerParams.STEER_DRIVER_ALLOWANCE,
        5,
      )
    )

    ret.steerFaultTemporary = (
      cp.vl["EPAS_INFO"]
      ["EPAS_Failure"] == 1
    )

    ret.steerFaultPermanent = (
      cp.vl["EPAS_INFO"]
      ["EPAS_Failure"] in (2, 3)
    )

    ret.espDisabled = (
      cp.vl["Cluster_Info1_FD1"]
      ["DrvSlipCtlMde_D_Rq"] != 0
    )

    # CAN-FD-only steering status.
    #
    # Do NOT access this message on old non-CAN-FD vehicles.
    if self.CP.flags & FordFlags.CANFD:
      ret.steerFaultTemporary |= (
        cp.vl["Lane_Assist_Data3_FD1"]
        ["LatCtlSte_D_Stat"]
        not in (1, 2, 3)
      )

    # ==============================================================
    # Cruise control speed
    #
    # Older Ford/Lincoln:
    #   INSTRUMENT_PANEL / METRIC_UNITS
    #
    # CAN-FD:
    #   preserve existing DP behavior
    # ==============================================================

    is_metric = (
      cp.vl["INSTRUMENT_PANEL"]
      ["METRIC_UNITS"] == 1
      if not self.CP.flags & FordFlags.CANFD
      else False
    )

    ret.cruiseState.speed = (
      cp.vl["EngBrakeData"]
      ["Veh_V_DsplyCcSet"] *
      (
        CV.KPH_TO_MS
        if is_metric
        else CV.MPH_TO_MS
      )
    )

    ret.cruiseState.enabled = (
      cp.vl["EngBrakeData"]
      ["CcStat_D_Actl"] in (4, 5)
    )

    ret.cruiseState.available = (
      cp.vl["EngBrakeData"]
      ["CcStat_D_Actl"] in (3, 4, 5)
    )

    ret.cruiseState.nonAdaptive = (
      cp.vl["Cluster_Info1_FD1"]
      ["AccEnbl_B_RqDrv"] == 0
    )

    ret.cruiseState.standstill = (
      cp.vl["EngBrakeData"]
      ["AccStopMde_D_Rq"] == 3
    )

    ret.accFaulted = (
      cp.vl["EngBrakeData"]
      ["CcStat_D_Actl"] in (1, 2)
    )

    # Stock camera ACC denial.
    if not self.CP.openpilotLongitudinalControl:
      ret.accFaulted = (
        ret.accFaulted or
        cp_cam.vl["ACCDATA"]
        ["CmbbDeny_B_Actl"] == 1
      )

    # ==============================================================
    # Gear
    # ==============================================================

    if self.CP.transmissionType == TransmissionType.automatic:

      if self.CP.flags & FordFlags.ALT_STEER_ANGLE:
        gear = self.shifter_values.get(
          cp.vl["TransGearData"]
          ["GearLvrPos_D_Actl"]
        )
      else:
        # Old Ford/Lincoln
        gear = self.shifter_values.get(
          cp.vl["PowertrainData_10"]
          ["TrnRng_D_Rq"]
        )

      ret.gearShifter = (
        self.parse_gear_shifter(gear)
      )

    elif self.CP.transmissionType == TransmissionType.manual:

      if bool(
        cp.vl["BCM_Lamp_Stat_FD1"]
        ["RvrseLghtOn_B_Stat"]
      ):
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # ==============================================================
    # Stock safety systems
    # ==============================================================

    ret.stockFcw = bool(
      cp_cam.vl["ACCDATA_3"]
      ["FcwVisblWarn_B_Rq"]
    )

    ret.stockAeb = bool(
      cp_cam.vl["ACCDATA_2"]
      ["CmbbBrkDecel_B_Rq"]
    )

    # ==============================================================
    # Steering wheel buttons
    # ==============================================================

    ret.leftBlinker = (
      cp.vl["Steering_Data_FD1"]
      ["TurnLghtSwtch_D_Stat"] == 1
    )

    ret.rightBlinker = (
      cp.vl["Steering_Data_FD1"]
      ["TurnLghtSwtch_D_Stat"] == 2
    )

    # Keep original DP TJA handling.
    ret.genericToggle = bool(
      cp.vl["Steering_Data_FD1"]
      ["TjaButtnOnOffPress"]
    )

    prev_distance_button = self.distance_button
    prev_lc_button = self.lc_button

    self.distance_button = (
      cp.vl["Steering_Data_FD1"]
      ["AccButtnGapTogglePress"]
    )

    self.lc_button = bool(
      cp.vl["Steering_Data_FD1"]
      ["TjaButtnOnOffPress"]
    )

    # ==============================================================
    # Door / seatbelt
    # ==============================================================

    ret.doorOpen = any([
      cp.vl["BodyInfo_3_FD1"]
      ["DrStatDrv_B_Actl"],

      cp.vl["BodyInfo_3_FD1"]
      ["DrStatPsngr_B_Actl"],

      cp.vl["BodyInfo_3_FD1"]
      ["DrStatRl_B_Actl"],

      cp.vl["BodyInfo_3_FD1"]
      ["DrStatRr_B_Actl"],
    ])

    ret.seatbeltUnlatched = (
      cp.vl["RCMStatusMessage2_FD1"]
      ["FirstRowBuckleDriver"] == 2
    )

    # ==============================================================
    # Blind spot monitoring
    #
    # Old Ford/Lincoln:
    #   main / PT bus
    #
    # CAN-FD:
    #   camera bus
    # ==============================================================

    if self.CP.enableBsm:

      cp_bsm = (
        cp_cam
        if self.CP.flags & FordFlags.CANFD
        else cp
      )

      ret.leftBlindspot = (
        cp_bsm.vl["Side_Detect_L_Stat"]
        ["SodDetctLeft_D_Stat"] != 0
      )

      ret.rightBlindspot = (
        cp_bsm.vl["Side_Detect_R_Stat"]
        ["SodDetctRight_D_Stat"] != 0
      )

    # ==============================================================
    # Stock CAN values used by DragonPilot controller
    # ==============================================================

    self.buttons_stock_values = (
      cp.vl["Steering_Data_FD1"]
    )

    self.acc_tja_status_stock_values = (
      cp_cam.vl["ACCDATA_3"]
    )

    self.lkas_status_stock_values = (
      cp_cam.vl["IPMA_Data"]
    )

    # ==============================================================
    # Button events
    # ==============================================================

    ret.buttonEvents = [
      *create_button_events(
        self.distance_button,
        prev_distance_button,
        {
          1: ButtonType.gapAdjustCruise,
        },
      ),

      *create_button_events(
        self.lc_button,
        prev_lc_button,
        {
          1: ButtonType.lkas,
        },
      ),
    ]

    # ==============================================================
    # DragonPilot ALKA
    #
    # Keep original DP behavior.
    # ==============================================================

    self.lkas_on = (
      ret.cruiseState.available
    )

    return ret


  @staticmethod
  def get_can_parsers(CP):
    """
    Ford/Lincoln CAN parser compatibility.

    Main purpose:
      1. Support older non-CAN-FD Ford/Lincoln.
      2. Support ALT_STEER_ANGLE platforms.
      3. Select the correct steering-angle message.
      4. Select the correct transmission message.
      5. Select correct BSM bus.
      6. Load INSTRUMENT_PANEL only for older platforms.
    """

    # ==============================================================
    # Main / powertrain CAN bus
    # ==============================================================

    pt_messages = [
      ("BrakeSysFeatures", 50),
      ("Yaw_Data_FD1", 100),
      ("DesiredTorqBrk", 50),
      ("EngVehicleSpThrottle", 100),
      ("EngBrakeData", 10),
      ("Cluster_Info1_FD1", 10),
      ("EPAS_INFO", 50),
      ("Steering_Data_FD1", 10),
      ("BodyInfo_3_FD1", 2),
      ("RCMStatusMessage2_FD1", 10),
      ("BCM_Lamp_Stat_FD1", float("nan")),

      # Original DP brake torque signal.
      ("BrakeSnData_4", 50),
    ]

    # ==============================================================
    # Steering-angle message
    #
    # IMPORTANT:
    #
    # Old Ford/Lincoln:
    #   SteeringPinion_Data
    #
    # ALT_STEER_ANGLE:
    #   SteeringPinion_Data_Alt
    #   ParkAid_Data
    # ==============================================================

    if CP.flags & FordFlags.ALT_STEER_ANGLE:

      pt_messages += [
        ("SteeringPinion_Data_Alt", 100),
        ("ParkAid_Data", 50),
      ]

    else:

      pt_messages += [
        ("SteeringPinion_Data", 100),
      ]

    # ==============================================================
    # Transmission
    # ==============================================================

    if CP.transmissionType == TransmissionType.automatic:

      if CP.flags & FordFlags.ALT_STEER_ANGLE:

        pt_messages += [
          ("TransGearData", 10),
        ]

      else:

        pt_messages += [
          ("PowertrainData_10", 10),
        ]

    elif CP.transmissionType == TransmissionType.manual:

      pt_messages += [
        ("Engine_Clutch_Data", 33),
      ]

    # ==============================================================
    # CAN-FD / old-platform separation
    # ==============================================================

    if CP.flags & FordFlags.CANFD:

      # CAN-FD steering status.
      pt_messages += [
        ("Lane_Assist_Data3_FD1", 33),
      ]

    else:

      # ============================================================
      # IMPORTANT FOR OLD FORD/LINCOLN
      #
      # Old vehicles use the instrument cluster's unit setting
      # to determine whether cruise speed is MPH or KPH.
      # ============================================================

      pt_messages += [
        ("INSTRUMENT_PANEL", 1),
      ]

    # ==============================================================
    # BSM
    #
    # Old Ford/Lincoln:
    #   BSM is on main bus.
    #
    # CAN-FD:
    #   BSM is on camera bus.
    # ==============================================================

    if (
      CP.enableBsm and
      not (CP.flags & FordFlags.CANFD)
    ):

      pt_messages += [
        ("Side_Detect_L_Stat", 5),
        ("Side_Detect_R_Stat", 5),
      ]

    # ==============================================================
    # Camera CAN bus
    # ==============================================================

    cam_messages = [
      ("ACCDATA", 50),
      ("ACCDATA_2", 50),
      ("ACCDATA_3", 5),
      ("IPMA_Data", 1),
    ]

    # ==============================================================
    # Traffic / IPMA
    # ==============================================================

    if CP.flags & FordFlags.CANFD:

      cam_messages += [
        ("Traffic_RecognitnData", 1),
        ("IPMA_Data2", 1),
      ]

    else:

      # Q3 / older Ford camera compatibility.
      #
      # float("nan") makes this message optional.
      cam_messages += [
        ("Traffic_RecognitnData", float("nan")),
      ]

    # ==============================================================
    # CAN-FD BSM
    #
    # CAN-FD BSM is on camera bus.
    # ==============================================================

    if (
      CP.enableBsm and
      CP.flags & FordFlags.CANFD
    ):

      cam_messages += [
        ("Side_Detect_L_Stat", 5),
        ("Side_Detect_R_Stat", 5),
      ]

    return {
      Bus.pt: CANParser(
        DBC[CP.carFingerprint][Bus.pt],
        pt_messages,
        CanBus(CP).main,
      ),

      Bus.cam: CANParser(
        DBC[CP.carFingerprint][Bus.pt],
        cam_messages,
        CanBus(CP).camera,
      ),
    }
