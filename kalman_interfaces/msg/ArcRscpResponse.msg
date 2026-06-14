int32 ACK=0
int32 TASK_FINISHED=1
int32 GPS_COORDINATE=2
int32 DISTANCE=3
int32 MESSAGE=4
int32 ROVER_STATUS=5

int32 type

# ACK
## empty

# TASK_FINISHED
## empty

# GPS_COORDINATE
float64 latitude
float64 longitude
float32 altitude

# DISTANCE
float64 distance

# MESSAGE
string message

# ROVER_STATUS
## Rover state
int32 ROVER_STATE_DISARMED=0
int32 ROVER_STATE_AUTONOMOUS=1
int32 ROVER_STATE_MANUAL=2
int32 rover_state

## coordinate - reuse from GPS_COORDINATE
float64 heading

## battey state
float32 voltage  # in volts
float32 current  # in amperes
float32 state_of_charge  # Value between 0.0 and 1.0
