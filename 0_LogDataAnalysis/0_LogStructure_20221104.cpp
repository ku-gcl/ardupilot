// There is no LoggerMessage
// FILE
// VER: Not logged in fixed-arm airplane


// ##AP_AHRS/LogStructure.h
// @LoggerMessage: AHR2
// @Description: Backup AHRS data
// @Field: TimeUS: Time since system startup
// @Field: Roll: Estimated roll
// @Field: Pitch: Estimated pitch
// @Field: Yaw: Estimated yaw
// @Field: Alt: Estimated altitude
// @Field: Lat: Estimated latitude
// @Field: Lng: Estimated longitude
// @Field: Q1: Estimated attitude quaternion component 1
// @Field: Q2: Estimated attitude quaternion component 2
// @Field: Q3: Estimated attitude quaternion component 3
// @Field: Q4: Estimated attitude quaternion component 4

// @LoggerMessage: AOA
// @Description: Angle of attack and Side Slip Angle values
// @Field: TimeUS: Time since system startup
// @Field: AOA: Angle of Attack calculated from airspeed, wind vector,velocity vector 
// @Field: SSA: Side Slip Angle calculated from airspeed, wind vector,velocity vector

// @LoggerMessage: ATT
// @Description: Canonical vehicle attitude
// @Field: TimeUS: Time since system startup
// @Field: DesRoll: vehicle desired roll
// @Field: Roll: achieved vehicle roll
// @Field: DesPitch: vehicle desired pitch
// @Field: Pitch: achieved vehicle pitch
// @Field: DesYaw: vehicle desired yaw
// @Field: Yaw: achieved vehicle yaw
// @Field: ErrRP: lowest estimated gyro drift error
// @Field: ErrYaw: difference between measured yaw and DCM yaw estimate
// @Field: AEKF: active EKF type

// @LoggerMessage: ORGN
// @Description: Vehicle navigation origin or other notable position
// @Field: TimeUS: Time since system startup
// @Field: Type: Position type
// @Field: Lat: Position latitude
// @Field: Lng: Position longitude
// @Field: Alt: Position altitude

// @LoggerMessage: POS
// @Description: Canonical vehicle position
// @Field: TimeUS: Time since system startup
// @Field: Lat: Canonical vehicle latitude
// @Field: Lng: Canonical vehicle longitude
// @Field: Alt: Canonical vehicle altitude
// @Field: RelHomeAlt: Canonical vehicle altitude relative to home
// @Field: RelOriginAlt: Canonical vehicle altitude relative to navigation origin

// @LoggerMessage: RATE
// @Description: Desired and achieved vehicle attitude rates. Not logged in Fixed Wing Plane modes.
// @Field: TimeUS: Time since system startup
// @Field: RDes: vehicle desired roll rate
// @Field: R: achieved vehicle roll rate
// @Field: ROut: normalized output for Roll
// @Field: PDes: vehicle desired pitch rate
// @Field: P: vehicle pitch rate
// @Field: POut: normalized output for Pitch
// @Field: Y: achieved vehicle yaw rate
// @Field: YOut: normalized output for Yaw
// @Field: YDes: vehicle desired yaw rate
// @Field: ADes: desired vehicle vertical acceleration
// @Field: A: achieved vehicle vertical acceleration
// @Field: AOut: percentage of vertical thrust output current being used

// @LoggerMessage: VSTB
// @Description: Log message for video stabilisation software such as Gyroflow
// @Field: TimeUS: Time since system startup
// @Field: GyrX: measured rotation rate about X axis
// @Field: GyrY: measured rotation rate about Y axis
// @Field: GyrZ: measured rotation rate about Z axis
// @Field: AccX: acceleration along X axis
// @Field: AccY: acceleration along Y axis
// @Field: AccZ: acceleration along Z axis
// @Field: Q1: Estimated attitude quaternion component 1
// @Field: Q2: Estimated attitude quaternion component 2
// @Field: Q3: Estimated attitude quaternion component 3
// @Field: Q4: Estimated attitude quaternion component 4

// ##AP_Baro/LogStructure.h
// @LoggerMessage: BARO
// @Description: Gathered Barometer data
// @Field: TimeUS: Time since system startup
// @Field: I: barometer sensor instance number
// @Field: Alt: calculated altitude
// @Field: Press: measured atmospheric pressure
// @Field: Temp: measured atmospheric temperature
// @Field: CRt: derived climb rate from primary barometer
// @Field: SMS: time last sample was taken
// @Field: Offset: raw adjustment of barometer altitude, zeroed on calibration, possibly set by GCS
// @Field: GndTemp: temperature on ground, specified by parameter or measured while on ground
// @Field: Health: true if barometer is considered healthy

// ##AP_BattMonitor/LogStructure.h
// @LoggerMessage: BAT
// @Description: Gathered battery data
// @Field: TimeUS: Time since system startup
// @Field: Instance: battery instance number
// @Field: Volt: measured voltage
// @Field: VoltR: estimated resting voltage
// @Field: Curr: measured current
// @Field: CurrTot: consumed Ah, current * time
// @Field: EnrgTot: consumed Wh, energy this battery has expended
// @Field: Temp: measured temperature
// @Field: Res: estimated battery resistance
// @Field: RemPct: remaining percentage

// ##AC_AttitudeControl.cpp
// @LoggerMessage: CTRL
// @Description: Attitude Control oscillation monitor diagnostics
// @Field: TimeUS: Time since system startup
// @Field: RMSRollP: LPF Root-Mean-Squared Roll Rate controller P gain
// @Field: RMSRollD: LPF Root-Mean-Squared Roll rate controller D gain
// @Field: RMSPitchP: LPF Root-Mean-Squared Pitch Rate controller P gain
// @Field: RMSPitchD: LPF Root-Mean-Squared Pitch Rate controller D gain
// @Field: RMSYaw: LPF Root-Mean-Squared Yaw Rate controller P+D gain

// ##ArduCopter/Log.cpp
// @LoggerMessage: CTUN
// @Description: Control Tuning information
// @Field: TimeUS: Time since system startup
// @Field: ThI: throttle input
// @Field: ABst: angle boost
// @Field: ThO: throttle output
// @Field: ThH: calculated hover throttle
// @Field: DAlt: desired altitude
// @Field: Alt: achieved altitude
// @Field: BAlt: barometric altitude
// @Field: DSAlt: desired rangefinder altitude
// @Field: SAlt: achieved rangefinder altitude
// @Field: TAlt: terrain altitude
// @Field: DCRt: desired climb rate
// @Field: CRt: climb rate

// @LoggerMessage: D16
// @Description: Generic 16-bit-signed-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DU16
// @Description: Generic 16-bit-unsigned-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: D32
// @Description: Generic 32-bit-signed-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DFLT
// @Description: Generic float storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DU32
// @Description: Generic 32-bit-unsigned-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: PTUN
// @Description: Parameter Tuning information
// @URL: https://ardupilot.org/copter/docs/tuning.html#in-flight-tuning
// @Field: TimeUS: Time since system startup
// @Field: Param: Parameter being tuned
// @Field: TunVal: Normalized value used inside tuning() function
// @Field: TunMin: Tuning minimum limit
// @Field: TunMax: Tuning maximum limit

// ##AP_InertialSensor/LogStructure.h
// @LoggerMessage: ACC
// @Description: IMU accelerometer data
// @Field: TimeUS: Time since system startup
// @Field: I: accelerometer sensor instance number
// @Field: SampleUS: time since system startup this sample was taken
// @Field: AccX: acceleration along X axis
// @Field: AccY: acceleration along Y axis
// @Field: AccZ: acceleration along Z axis

// @LoggerMessage: GYR
// @Description: IMU gyroscope data
// @Field: TimeUS: Time since system startup
// @Field: I: gyroscope sensor instance number
// @Field: SampleUS: time since system startup this sample was taken
// @Field: GyrX: measured rotation rate about X axis
// @Field: GyrY: measured rotation rate about Y axis
// @Field: GyrZ: measured rotation rate about Z axis

// @LoggerMessage: IMU
// @Description: Inertial Measurement Unit data
// @Field: TimeUS: Time since system startup
// @Field: I: IMU sensor instance number
// @Field: GyrX: measured rotation rate about X axis
// @Field: GyrY: measured rotation rate about Y axis
// @Field: GyrZ: measured rotation rate about Z axis
// @Field: AccX: acceleration along X axis
// @Field: AccY: acceleration along Y axis
// @Field: AccZ: acceleration along Z axis
// @Field: EG: gyroscope error count
// @Field: EA: accelerometer error count
// @Field: T: IMU temperature
// @Field: GH: gyroscope health
// @Field: AH: accelerometer health
// @Field: GHz: gyroscope measurement rate
// @Field: AHz: accelerometer measurement rate

// @LoggerMessage: VIBE
// @Description: Processed (acceleration) vibration information
// @Field: TimeUS: Time since system startup
// @Field: IMU: Vibration instance number
// @Field: VibeX: Primary accelerometer filtered vibration, x-axis
// @Field: VibeY: Primary accelerometer filtered vibration, y-axis
// @Field: VibeZ: Primary accelerometer filtered vibration, z-axis
// @Field: Clip: Number of clipping events on 1st accelerometer

// ##AP_IOMCU/AP_IOMCU.cpp
// @LoggerMessage: IOMC
// @Description: IOMCU diagnostic information
// @Field: TimeUS: Time since system startup
// @Field: RSErr: Status Read error count (zeroed on successful read)
// @Field: Mem: Free memory
// @Field: TS: IOMCU uptime
// @Field: NPkt: Number of packets received by IOMCU
// @Field: Nerr: Protocol failures on MCU side
// @Field: Nerr2: Reported number of failures on IOMCU side
// @Field: NDel: Number of delayed packets received by MCU

// ##AP_NavEKF3/LogStructure.h
// @LoggerMessage: XKF0
// @Description: EKF3 beacon sensor diagnostics
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: ID: Beacon sensor ID
// @Field: rng: Beacon range
// @Field: innov: Beacon range innovation
// @Field: SIV: sqrt of beacon range innovation variance
// @Field: TR: Beacon range innovation consistency test ratio
// @Field: BPN: Beacon north position
// @Field: BPE: Beacon east position
// @Field: BPD: Beacon down position
// @Field: OFH: High estimate of vertical position offset of beacons rel to EKF origin
// @Field: OFL: Low estimate of vertical position offset of beacons rel to EKF origin
// @Field: OFN: North position of receiver rel to EKF origin
// @Field: OFE: East position of receiver rel to EKF origin
// @Field: OFD: Down position of receiver rel to EKF origin

// @LoggerMessage: XKF1
// @Description: EKF3 estimator outputs
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: Roll: Estimated roll
// @Field: Pitch: Estimated pitch
// @Field: Yaw: Estimated yaw
// @Field: VN: Estimated velocity (North component)
// @Field: VE: Estimated velocity (East component)
// @Field: VD: Estimated velocity (Down component)
// @Field: dPD: Filtered derivative of vertical position (down)
// @Field: PN: Estimated distance from origin (North component)
// @Field: PE: Estimated distance from origin (East component)
// @Field: PD: Estimated distance from origin (Down component)
// @Field: GX: Estimated gyro bias, X axis
// @Field: GY: Estimated gyro bias, Y axis
// @Field: GZ: Estimated gyro bias, Z axis
// @Field: OH: Height of origin above WGS-84

// @LoggerMessage: XKF2
// @Description: EKF3 estimator secondary outputs
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: AX: Estimated accelerometer X bias
// @Field: AY: Estimated accelerometer Y bias
// @Field: AZ: Estimated accelerometer Z bias
// @Field: VWN: Estimated wind velocity (North component)
// @Field: VWE: Estimated wind velocity (East component)
// @Field: MN: Magnetic field strength (North component)
// @Field: ME: Magnetic field strength (East component)
// @Field: MD: Magnetic field strength (Down component)
// @Field: MX: Magnetic field strength (body X-axis)
// @Field: MY: Magnetic field strength (body Y-axis)
// @Field: MZ: Magnetic field strength (body Z-axis)
// @Field: IDX: Innovation in vehicle drag acceleration (X-axis component)
// @Field: IDY: Innovation in vehicle drag acceleration (Y-axis component)
// @Field: IS: Innovation in vehicle sideslip

// @LoggerMessage: XKF3
// @Description: EKF3 innovations
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: IVN: Innovation in velocity (North component)
// @Field: IVE: Innovation in velocity (East component)
// @Field: IVD: Innovation in velocity (Down component)
// @Field: IPN: Innovation in position (North component)
// @Field: IPE: Innovation in position (East component)
// @Field: IPD: Innovation in position (Down component)
// @Field: IMX: Innovation in magnetic field strength (X-axis component)
// @Field: IMY: Innovation in magnetic field strength (Y-axis component)
// @Field: IMZ: Innovation in magnetic field strength (Z-axis component)
// @Field: IYAW: Innovation in vehicle yaw
// @Field: IVT: Innovation in true-airspeed
// @Field: RErr: Accumulated relative error of this core with respect to active primary core
// @Field: ErSc: A consolidated error score where higher numbers are less healthy

// @LoggerMessage: XKF4
// @Description: EKF3 variances.  SV, SP, SH and SM are probably best described as 'Squared Innovation Test Ratios' where values <1 tells us the measurement was accepted and >1 tells us it was rejected. They represent the square of the (innovation / maximum allowed innovation) where the innovation is the difference between predicted and measured value and the maximum allowed innovation is determined from the uncertainty of the measurement, uncertainty of the prediction and scaled using the number of standard deviations set by the innovation gate parameter for that measurement, eg EK3_MAG_I_GATE, EK3_HGT_I_GATE, etc
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: SV: Square root of the velocity variance
// @Field: SP: Square root of the position variance
// @Field: SH: Square root of the height variance
// @Field: SM: Magnetic field variance
// @Field: SVT: Square root of the total airspeed variance
// @Field: errRP: Filtered error in roll/pitch estimate
// @Field: OFN: Most recent position reset (North component)
// @Field: OFE: Most recent position reset (East component)
// @Field: FS: Filter fault status
// @Field: TS: Filter timeout status bitmask (0:position measurement, 1:velocity measurement, 2:height measurement, 3:magnetometer measurement, 4:airspeed measurement, 5:drag measurement)
// @Field: SS: Filter solution status
// @Field: GPS: Filter GPS status
// @Field: PI: Primary core index

// @LoggerMessage: XKF5
// @Description: EKF3 Sensor innovations (primary core) and general dumping ground
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: NI: Normalised flow variance
// @Field: FIX: Optical flow LOS rate vector innovations from the main nav filter (X-axis)
// @Field: FIY: Optical flow LOS rate vector innovations from the main nav filter (Y-axis)
// @Field: AFI: Optical flow LOS rate innovation from terrain offset estimator
// @Field: HAGL: Height above ground level
// @Field: offset: Estimated vertical position of the terrain relative to the nav filter zero datum
// @Field: RI: Range finder innovations
// @Field: rng: Measured range
// @Field: Herr: Filter ground offset state error
// @Field: eAng: Magnitude of angular error
// @Field: eVel: Magnitude of velocity error
// @Field: ePos: Magnitude of position error

// @LoggerMessage: XKFD
// @Description: EKF3 Body Frame Odometry errors
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: IX: Innovation in velocity (X-axis)
// @Field: IY: Innovation in velocity (Y-axis)
// @Field: IZ: Innovation in velocity (Z-axis)
// @Field: IVX: Variance in velocity (X-axis)
// @Field: IVY: Variance in velocity (Y-axis)
// @Field: IVZ: Variance in velocity (Z-axis)

// @LoggerMessage: XKT
// @Description: EKF3 timing information
// @Field: TimeUS: Time since system startup
// @Field: C: EKF core this message instance applies to
// @Field: Cnt: count of samples used to create this message
// @Field: IMUMin: smallest IMU sample interval
// @Field: IMUMax: largest IMU sample interval
// @Field: EKFMin: low-passed achieved average time step rate for the EKF (minimum)
// @Field: EKFMax: low-passed achieved average time step rate for the EKF (maximum)
// @Field: AngMin: accumulated measurement time interval for the delta angle (minimum)
// @Field: AngMax: accumulated measurement time interval for the delta angle (maximum)
// @Field: VMin: accumulated measurement time interval for the delta velocity (minimum)
// @Field: VMax: accumulated measurement time interval for the delta velocity (maximum)

// @LoggerMessage: XKFM
// @Description: EKF3 diagnostic data for on-ground-and-not-moving check
// @Field: TimeUS: Time since system startup
// @Field: C: EKF core this message instance applies to
// @Field: OGNM: True of on ground and not moving
// @Field: GLR: Gyroscope length ratio
// @Field: ALR: Accelerometer length ratio
// @Field: GDR: Gyroscope rate of change ratio
// @Field: ADR: Accelerometer rate of change ratio

// @LoggerMessage: XKQ
// @Description: EKF3 quaternion defining the rotation from NED to XYZ (autopilot) axes
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: Q1: Quaternion a term
// @Field: Q2: Quaternion b term
// @Field: Q3: Quaternion c term
// @Field: Q4: Quaternion d term

// @LoggerMessage: XKFS
// @Description: EKF3 sensor selection
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: MI: compass selection index
// @Field: BI: barometer selection index
// @Field: GI: GPS selection index
// @Field: AI: airspeed selection index
// @Field: SS: Source Set (primary=0/secondary=1/tertiary=2)

// @LoggerMessage: XKTV
// @Description: EKF3 Yaw Estimator States
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: TVS: Tilt Error Variance from symbolic equations (rad^2)
// @Field: TVD: Tilt Error Variance from difference method (rad^2)

// @LoggerMessage: XKV1
// @Description: EKF3 State variances (primary core)
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: V00: Variance for state 0
// @Field: V01: Variance for state 1
// @Field: V02: Variance for state 2
// @Field: V03: Variance for state 3
// @Field: V04: Variance for state 4
// @Field: V05: Variance for state 5
// @Field: V06: Variance for state 6
// @Field: V07: Variance for state 7
// @Field: V08: Variance for state 8
// @Field: V09: Variance for state 9
// @Field: V10: Variance for state 10
// @Field: V11: Variance for state 11

// @LoggerMessage: XKV2
// @Description: more EKF3 State Variances (primary core)
// @Field: TimeUS: Time since system startup
// @Field: C: EKF3 core this data is for
// @Field: V12: Variance for state 12
// @Field: V13: Variance for state 13
// @Field: V14: Variance for state 14
// @Field: V15: Variance for state 15
// @Field: V16: Variance for state 16
// @Field: V17: Variance for state 17
// @Field: V18: Variance for state 18
// @Field: V19: Variance for state 19
// @Field: V20: Variance for state 20
// @Field: V21: Variance for state 21
// @Field: V22: Variance for state 22
// @Field: V23: Variance for state 23

// ##AP_Logger/LogStructure.h
// @LoggerMessage: ADSB
// @Description: Automatic Dependent Serveillance - Broadcast detected vehicle information
// @Field: TimeUS: Time since system startup
// @Field: ICAO_address: Transponder address
// @Field: Lat: Vehicle latitude
// @Field: Lng: Vehicle longitude
// @Field: Alt: Vehicle altitude
// @Field: Heading: Vehicle heading
// @Field: Hor_vel: Vehicle horizontal velocity
// @Field: Ver_vel: Vehicle vertical velocity
// @Field: Squark: Transponder squawk code

// @LoggerMessage: ARM
// @Description: Arming status changes
// @Field: TimeUS: Time since system startup
// @Field: ArmState: true if vehicle is now armed
// @Field: ArmChecks: arming bitmask at time of arming
// @Field: Forced: true if arm/disarm was forced
// @Field: Method: method used for arming

// @LoggerMessage: ARSP
// @Description: Airspeed sensor data
// @Field: TimeUS: Time since system startup
// @Field: I: Airspeed sensor instance number
// @Field: Airspeed: Current airspeed
// @Field: DiffPress: Pressure difference between static and dynamic port
// @Field: Temp: Temperature used for calculation
// @Field: RawPress: Raw pressure less offset
// @Field: Offset: Offset from parameter
// @Field: U: True if sensor is being used
// @Field: H: True if sensor is healthy
// @Field: Hp: Probability sensor is healthy
// @Field: TR: innovation test ratio
// @Field: Pri: True if sensor is the primary sensor

// @LoggerMessage: CMD
// @Description: Executed mission command information
// @Field: TimeUS: Time since system startup
// @Field: CTot: Total number of mission commands
// @Field: CNum: This command's offset in mission
// @Field: CId: Command type
// @Field: Prm1: Parameter 1
// @Field: Prm2: Parameter 2
// @Field: Prm3: Parameter 3
// @Field: Prm4: Parameter 4
// @Field: Lat: Command latitude
// @Field: Lng: Command longitude
// @Field: Alt: Command altitude
// @Field: Frame: Frame used for position

// @LoggerMessage: CSRV
// @Description: Servo feedback data
// @Field: TimeUS: Time since system startup
// @Field: Id: Servo number this data relates to
// @Field: Pos: Current servo position
// @Field: Force: Force being applied
// @Field: Speed: Current servo movement speed
// @Field: Pow: Amount of rated power being applied

// @LoggerMessage: DMS
// @Description: DataFlash-Over-MAVLink statistics
// @Field: TimeUS: Time since system startup
// @Field: N: Current block number
// @Field: Dp: Number of times we rejected a write to the backend
// @Field: RT: Number of blocks sent from the retry queue
// @Field: RS: Number of resends of unacknowledged data made
// @Field: Fa: Average number of blocks on the free list
// @Field: Fmn: Minimum number of blocks on the free list
// @Field: Fmx: Maximum number of blocks on the free list
// @Field: Pa: Average number of blocks on the pending list
// @Field: Pmn: Minimum number of blocks on the pending list
// @Field: Pmx: Maximum number of blocks on the pending list
// @Field: Sa: Average number of blocks on the sent list
// @Field: Smn: Minimum number of blocks on the sent list
// @Field: Smx: Maximum number of blocks on the sent list

// @LoggerMessage: DSF
// @Description: Onboard logging statistics
// @Field: TimeUS: Time since system startup
// @Field: Dp: Number of times we rejected a write to the backend
// @Field: Blk: Current block number
// @Field: Bytes: Current write offset
// @Field: FMn: Minimum free space in write buffer in last time period
// @Field: FMx: Maximum free space in write buffer in last time period
// @Field: FAv: Average free space in write buffer in last time period

// @LoggerMessage: DSTL
// @Description: Deepstall Landing data
// @Field: TimeUS: Time since system startup
// @Field: Stg: Deepstall landing stage
// @Field: THdg: Target heading
// @Field: Lat: Landing point latitude
// @Field: Lng: Landing point longitude
// @Field: Alt: Landing point altitude
// @Field: XT: Crosstrack error
// @Field: Travel: Expected travel distance vehicle will travel from this point
// @Field: L1I: L1 controller crosstrack integrator value
// @Field: Loiter: wind estimate loiter angle flown
// @Field: Des: Deepstall steering PID desired value
// @Field: P: Deepstall steering PID Proportional response component
// @Field: I: Deepstall steering PID Integral response component
// @Field: D: Deepstall steering PID Derivative response component

// @LoggerMessage: ERR
// @Description: Specifically coded error messages
// @Field: TimeUS: Time since system startup
// @Field: Subsys: Subsystem in which the error occurred
// @Field: ECode: Subsystem-specific error code

// @LoggerMessage: EV
// @Description: Specifically coded event messages
// @Field: TimeUS: Time since system startup
// @Field: Id: Event identifier

// @LoggerMessage: FMT
// @Description: Message defining the format of messages in this file
// @URL: https://ardupilot.org/dev/docs/code-overview-adding-a-new-log-message.html
// @Field: Type: unique-to-this-log identifier for message being defined
// @Field: Length: the number of bytes taken up by this message (including all headers)
// @Field: Name: name of the message being defined
// @Field: Format: character string defining the C-storage-type of the fields in this message
// @Field: Columns: the labels of the message being defined

// @LoggerMessage: FMTU
// @Description: Message defining units and multipliers used for fields of other messages
// @Field: TimeUS: Time since system startup
// @Field: FmtType: numeric reference to associated FMT message
// @Field: UnitIds: each character refers to a UNIT message.  The unit at an offset corresponds to the field at the same offset in FMT.Format
// @Field: MultIds: each character refers to a MULT message.  The multiplier at an offset corresponds to the field at the same offset in FMT.Format

// @LoggerMessage: LGR
// @Description: Landing gear information
// @Field: TimeUS: Time since system startup
// @Field: LandingGear: Current landing gear state
// @Field: WeightOnWheels: True if there is weight on wheels

// @LoggerMessage: MAG
// @Description: Information received from compasses
// @Field: TimeUS: Time since system startup
// @Field: I: magnetometer sensor instance number
// @Field: MagX: magnetic field strength in body frame
// @Field: MagY: magnetic field strength in body frame
// @Field: MagZ: magnetic field strength in body frame
// @Field: OfsX: magnetic field offset in body frame
// @Field: OfsY: magnetic field offset in body frame
// @Field: OfsZ: magnetic field offset in body frame
// @Field: MOX: motor interference magnetic field offset in body frame
// @Field: MOY: motor interference magnetic field offset in body frame
// @Field: MOZ: motor interference magnetic field offset in body frame
// @Field: Health: true if the compass is considered healthy
// @Field: S: time measurement was taken

// @LoggerMessage: MAV
// @Description: GCS MAVLink link statistics
// @Field: TimeUS: Time since system startup
// @Field: chan: mavlink channel number
// @Field: txp: transmitted packet count
// @Field: rxp: received packet count
// @Field: rxdp: perceived number of packets we never received
// @Field: flags: compact representation of some stage of the channel
// @Field: ss: stream slowdown is the number of ms being added to each message to fit within bandwidth
// @Field: tf: times buffer was full when a message was going to be sent

// @LoggerMessage: MAVC
// @Description: MAVLink command we have just executed
// @Field: TimeUS: Time since system startup
// @Field: TS: target system for command
// @Field: TC: target component for command
// @Field: SS: source system for command
// @Field: SC: source component for command
// @Field: Fr: command frame
// @Field: Cmd: mavlink command enum value
// @Field: P1: first parameter from mavlink packet
// @Field: P2: second parameter from mavlink packet
// @Field: P3: third parameter from mavlink packet
// @Field: P4: fourth parameter from mavlink packet
// @Field: X: X coordinate from mavlink packet
// @Field: Y: Y coordinate from mavlink packet
// @Field: Z: Z coordinate from mavlink packet
// @Field: Res: command result being returned from autopilot
// @Field: WL: true if this command arrived via a COMMAND_LONG rather than COMMAND_INT

// @LoggerMessage: MODE
// @Description: vehicle control mode information
// @Field: TimeUS: Time since system startup
// @Field: Mode: vehicle-specific mode number
// @Field: ModeNum: alias for Mode
// @Field: Rsn: reason for entering this mode; enumeration value

// @LoggerMessage: MSG
// @Description: Textual messages
// @Field: TimeUS: Time since system startup
// @Field: Message: message text

// @LoggerMessage: MULT
// @Description: Message mapping from single character to numeric multiplier
// @Field: TimeUS: Time since system startup
// @Field: Id: character referenced by FMTU
// @Field: Mult: numeric multiplier

// @LoggerMessage: OF
// @Description: Optical flow sensor data
// @Field: TimeUS: Time since system startup
// @Field: Qual: Estimated sensor data quality
// @Field: flowX: Sensor flow rate, X-axis
// @Field: flowY: Sensor flow rate,Y-axis
// @Field: bodyX: derived velocity, X-axis
// @Field: bodyY: derived velocity, Y-axis

// @LoggerMessage: PARM
// @Description: parameter value
// @Field: TimeUS: Time since system startup
// @Field: Name: parameter name
// @Field: Value: parameter value
// @Field: Default: default parameter value for this board and config

// @LoggerMessage: PIDR,PIDP,PIDY,PIDA,PIDS,PIDN,PIDE
// @Description: Proportional/Integral/Derivative gain values for Roll/Pitch/Yaw/Altitude/Steering
// @Field: TimeUS: Time since system startup
// @Field: Tar: desired value
// @Field: Act: achieved value
// @Field: Err: error between target and achieved
// @Field: P: proportional part of PID
// @Field: I: integral part of PID
// @Field: D: derivative part of PID
// @Field: FF: controller feed-forward portion of response
// @Field: Dmod: scaler applied to D gain to reduce limit cycling
// @Field: SRate: slew rate used in slew limiter
// @Field: Limit: 1 if I term is limited due to output saturation

// @LoggerMessage: PM
// @Description: autopilot system performance and general data dumping ground
// @Field: TimeUS: Time since system startup
// @Field: NLon: Number of long loops detected
// @Field: NLoop: Number of measurement loops for this message
// @Field: MaxT: Maximum loop time
// @Field: Mem: Free memory available
// @Field: Load: System processor load
// @Field: IntE: Internal error mask; which internal errors have been detected
// @Field: ErrL: Internal error line number; last line number on which a internal error was detected
// @Field: ErrC: Internal error count; how many internal errors have been detected
// @Field: SPIC: Number of SPI transactions processed
// @Field: I2CC: Number of i2c transactions processed
// @Field: I2CI: Number of i2c interrupts serviced
// @Field: Ex: number of microseconds being added to each loop to address scheduler overruns

// @LoggerMessage: POWR
// @Description: System power information
// @Field: TimeUS: Time since system startup
// @Field: Vcc: Flight board voltage
// @Field: VServo: Servo rail voltage
// @Field: Flags: System power flags
// @Field: AccFlags: Accumulated System power flags; all flags which have ever been set
// @Field: Safety: Hardware Safety Switch status
// @Field: MTemp: MCU Temperature
// @Field: MVolt: MCU Voltage
// @Field: MVmin: MCU Voltage min
// @Field: MVmax: MCU Voltage max

// @LoggerMessage: RAD
// @Description: Telemetry radio statistics
// @Field: TimeUS: Time since system startup
// @Field: RSSI: RSSI
// @Field: RemRSSI: RSSI reported from remote radio
// @Field: TxBuf: number of bytes in radio ready to be sent
// @Field: Noise: local noise floor
// @Field: RemNoise: local noise floor reported from remote radio
// @Field: RxErrors: damaged packet count
// @Field: Fixed: fixed damaged packet count

// @LoggerMessage: RALY
// @Description: Rally point information
// @Field: TimeUS: Time since system startup
// @Field: Tot: total number of rally points onboard
// @Field: Seq: this rally point's sequence number
// @Field: Lat: latitude of rally point
// @Field: Lng: longitude of rally point
// @Field: Alt: altitude of rally point

// @LoggerMessage: RCI2
// @Description: (More) RC input channels to vehicle
// @Field: TimeUS: Time since system startup
// @Field: C15: channel 15 input
// @Field: C16: channel 16 input
// @Field: OMask: bitmask of RC channels being overridden by mavlink input

// @LoggerMessage: RCIN
// @Description: RC input channels to vehicle
// @Field: TimeUS: Time since system startup
// @Field: C1: channel 1 input
// @Field: C2: channel 2 input
// @Field: C3: channel 3 input
// @Field: C4: channel 4 input
// @Field: C5: channel 5 input
// @Field: C6: channel 6 input
// @Field: C7: channel 7 input
// @Field: C8: channel 8 input
// @Field: C9: channel 9 input
// @Field: C10: channel 10 input
// @Field: C11: channel 11 input
// @Field: C12: channel 12 input
// @Field: C13: channel 13 input
// @Field: C14: channel 14 input

// @LoggerMessage: RCOU
// @Description: Servo channel output values 1 to 14
// @Field: TimeUS: Time since system startup
// @Field: C1: channel 1 output
// @Field: C2: channel 2 output
// @Field: C3: channel 3 output
// @Field: C4: channel 4 output
// @Field: C5: channel 5 output
// @Field: C6: channel 6 output
// @Field: C7: channel 7 output
// @Field: C8: channel 8 output
// @Field: C9: channel 9 output
// @Field: C10: channel 10 output
// @Field: C11: channel 11 output
// @Field: C12: channel 12 output
// @Field: C13: channel 13 output
// @Field: C14: channel 14 output

// @LoggerMessage: RCO2
// @Description: Servo channel output values 15 to 18
// @Field: TimeUS: Time since system startup
// @Field: C15: channel 15 output
// @Field: C16: channel 16 output
// @Field: C17: channel 17 output
// @Field: C18: channel 18 output

// @LoggerMessage: RCO3
// @Description: Servo channel output values 19 to 32
// @Field: TimeUS: Time since system startup
// @Field: C19: channel 19 output
// @Field: C20: channel 20 output
// @Field: C21: channel 21 output
// @Field: C22: channel 22 output
// @Field: C23: channel 23 output
// @Field: C24: channel 24 output
// @Field: C25: channel 25 output
// @Field: C26: channel 26 output
// @Field: C27: channel 27 output
// @Field: C28: channel 28 output
// @Field: C29: channel 29 output
// @Field: C30: channel 30 output
// @Field: C31: channel 31 output
// @Field: C32: channel 32 output

// @LoggerMessage: RFND
// @Description: Rangefinder sensor information
// @Field: TimeUS: Time since system startup
// @Field: Instance: rangefinder instance number this data is from
// @Field: Dist: Reported distance from sensor
// @Field: Stat: Sensor state
// @Field: Orient: Sensor orientation

// @LoggerMessage: RSSI
// @Description: Received Signal Strength Indicator for RC receiver
// @Field: TimeUS: Time since system startup
// @Field: RXRSSI: RSSI
// @Field: RXLQ: RX Link Quality

// @LoggerMessage: SIM
// @Description: SITL simulator state
// @Field: TimeUS: Time since system startup
// @Field: Roll: Simulated roll
// @Field: Pitch: Simulated pitch
// @Field: Yaw: Simulated yaw
// @Field: Alt: Simulated altitude
// @Field: Lat: Simulated latitude
// @Field: Lng: Simulated longitude
// @Field: Q1: Attitude quaternion component 1
// @Field: Q2: Attitude quaternion component 2
// @Field: Q3: Attitude quaternion component 3
// @Field: Q4: Attitude quaternion component 4

// @LoggerMessage: SRTL
// @Description: SmartRTL statistics
// @Field: TimeUS: Time since system startup
// @Field: Active: true if SmartRTL could be used right now
// @Field: NumPts: number of points currently in use
// @Field: MaxPts: maximum number of points that could be used
// @Field: Action: most recent internal action taken by SRTL library
// @Field: N: point associated with most recent action (North component)
// @Field: E: point associated with most recent action (East component)
// @Field: D: point associated with most recent action (Down component)

// @LoggerMessage: TERR
// @Description: Terrain database infomration
// @Field: TimeUS: Time since system startup
// @Field: Status: Terrain database status
// @Field: Lat: Current vehicle latitude
// @Field: Lng: Current vehicle longitude
// @Field: Spacing: terrain Tile spacing
// @Field: TerrH: current Terrain height
// @Field: CHeight: Vehicle height above terrain
// @Field: Pending: Number of tile requests outstanding
// @Field: Loaded: Number of tiles in memory
// @Field: ROfs: terrain reference offset for arming altitude

// @LoggerMessage: TSYN
// @Description: Time synchronisation response information
// @Field: TimeUS: Time since system startup
// @Field: SysID: system ID this data is for
// @Field: RTT: round trip time for this system

// @LoggerMessage: UNIT
// @Description: Message mapping from single character to SI unit
// @Field: TimeUS: Time since system startup
// @Field: Id: character referenced by FMTU
// @Field: Label: Unit - SI where available

// @LoggerMessage: WENC
// @Description: Wheel encoder measurements
// @Field: TimeUS: Time since system startup
// @Field: Dist0: First wheel distance travelled
// @Field: Qual0: Quality measurement of Dist0
// @Field: Dist1: Second wheel distance travelled
// @Field: Qual1: Quality measurement of Dist1

// @LoggerMessage: WINC
// @Description: Winch
// @Field: TimeUS: Time since system startup
// @Field: Heal: Healthy
// @Field: ThEnd: Reached end of thread
// @Field: Mov: Motor is moving
// @Field: Clut: Clutch is engaged (motor can move freely)
// @Field: Mode: 0 is Relaxed, 1 is Position Control, 2 is Rate Control
// @Field: DLen: Desired Length
// @Field: Len: Estimated Length
// @Field: DRate: Desired Rate
// @Field: Tens: Tension on line
// @Field: Vcc: Voltage to Motor
// @Field: Temp: Motor temperature

// @LoggerMessage: PSCN
// @Description: Position Control North
// @Field: TimeUS: Time since system startup
// @Field: TPN: Target position relative to EKF origin
// @Field: PN: Position relative to EKF origin
// @Field: DVN: Desired velocity North
// @Field: TVN: Target velocity North
// @Field: VN: Velocity North
// @Field: DAN: Desired acceleration North
// @Field: TAN: Target acceleration North
// @Field: AN: Acceleration North

// @LoggerMessage: PSCE
// @Description: Position Control East
// @Field: TimeUS: Time since system startup
// @Field: TPE: Target position relative to EKF origin
// @Field: PE: Position relative to EKF origin
// @Field: DVE: Desired velocity East
// @Field: TVE: Target velocity East
// @Field: VE: Velocity East
// @Field: DAE: Desired acceleration East
// @Field: TAE: Target acceleration East
// @Field: AE: Acceleration East

// @LoggerMessage: PSCD
// @Description: Position Control Down
// @Field: TimeUS: Time since system startup
// @Field: TPD: Target position relative to EKF origin
// @Field: PD: Position relative to EKF origin
// @Field: DVD: Desired velocity Down
// @Field: TVD: Target velocity Down
// @Field: VD: Velocity Down
// @Field: DAD: Desired acceleration Down
// @Field: TAD: Target acceleration Down
// @Field: AD: Acceleration Down

// @LoggerMessage: STAK
// @Description: Stack information
// @Field: TimeUS: Time since system startup
// @Field: Id: thread ID
// @Field: Pri: thread priority
// @Field: Total: total stack
// @Field: Free: free stack
// @Field: Name: thread name

// @LoggerMessage: SCR
// @Description: Scripting runtime stats
// @Field: TimeUS: Time since system startup
// @Field: Name: script name
// @Field: Runtime: run time
// @Field: Total_mem: total memory usage
// @Field: Run_mem: run memory usage

// @LoggerMessage: MOTB
// @Description: Motor mixer information
// @Field: TimeUS: Time since system startup
// @Field: LiftMax: Maximum motor compensation gain
// @Field: BatVolt: Ratio between detected battery voltage and maximum battery voltage
// @Field: ThLimit: Throttle limit set due to battery current limitations
// @Field: ThrAvMx: Maximum average throttle that can be used to maintain attitude control, derived from throttle mix params
// @Field: FailFlags: bit 0 motor failed, bit 1 motors balanced, should be 2 in normal flight