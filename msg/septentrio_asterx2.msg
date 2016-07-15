# Stores septentrio's specific informations   
# For more details check septentrio SBF reference guide

Header header

# intMode: GNSS+INS integration mode
#  0: No solution available
#  1: Loosely-integrated solution (INS only)
#  2: Loosely-integrated solution (INS+GNSS)
#  4: GNSS-only solution

uint8 intMode

# gnssMode: Indicates the GNSS solution mode 
#  0: No PVT available (the Error field indicates the
#     cause of the absence of the PVT solution)
#  1: Stand-Alone PVT
#  2: Differential PVT
#  3: Fixed location
#  4: RTK with fixed ambiguities
#  5: RTK with float ambiguities
#  6: SBAS aided PVT
#  7: moving-base RTK with fixed ambiguities
#  8: moving-base RTK with float ambiguities
#  9: Precise Point Positioning (PPP) with fixed
#     ambiguities
# 10: Precise Point Positioning (PPP) with float
#     ambiguities

uint8 gnssMode

# gnssAge: Mean age of the differential corrections (just in case of DGPS or RTK)

float32 gnssAge

# error: GPS device error
#  0: no error
#  4: sum-of-squared residuals too large
#  5: no convergence
#  6: not enough measurements after outlier rejection
#  7: position output prohibited due to export laws
# 20: integrated PV not requested by user
# 21: not enough valid external sensor measurements
# 22: calibration not ready
# 23: static alignment ongoing
# 24: waiting for GNSS PVT
# 27: waiting for FINETIME synchronization level (see the
#     SyncLevel field of the ReceiverTime block )
# 28: in-motion alignment ongoing

uint8 error

# info: Bit field providing more information regarding the status and
#       type of measurements used
# Bit 0: set if acceleration measurements are used
# Bit 1: set if angular rate measurements are used
# Bits 2-10: reserved.
# Bit 11: set if 180-degree heading ambiguity fixed. If not
#         set and the vehicle is moving backwards, the
#         heading angle has a bias of 180 degrees. This bit is
#         only relevant in the IntAttEuler SBF block.
# Bit 12: set if zero-velocity constraints used
# Bit 13: set if GNSS-based position used
# Bit 14: set if GNSS-based velocity used
# Bit 15: set if GNSS-based attitude used

uint16 info

# Number of satellites used in the PV computation

uint8 nrSv

# poseLat: latitude in degrees

float64 poseLat

# poseLon: longitude in degrees

float64 poseLon

# poseAlt: Altitude

float64 poseAlt

# heading, roll, pitch: INS attitude in radians

float32 heading
float32 roll
float32 pitch

# cog: Course over ground: this is defined as the angle of the vehicle
#      with respect to the local level North, ranging from 0 to 2PI,
#      and increasing towards east. Set to the do-not-use value when
#      the speed is lower than 0.1m/s. COG is in radians.

float32 cog
