int32 ARM_DISARM=0
int32 SET_STAGE=1
int32 NAV_TO_GPS=2
int32 SEARCH_AREA=3
int32 START_EXPLORATION=4

int32 type

# ARM_DISARM
bool arm

# SET_STAGE
uint32 stage

# NAV_TO_GPS
float64 latitude
float64 longitude

# SEARCH_AREA
## Reuse center from NAV_TO_GPS
float32 radius  # The radius (in meters).

# START_EXPLORATION
## empty
