# maybe 7 because of 6 joints and 1 gripper #TODO
float64[6] positions 
uint8[<=6] joints_set # determine if the joints are set or not
uint8[<=6] joints_reversed #  5 and 6 joint should move in reverse way #TODO change into bool
uint8[<=6] joints_checked # TODO old code used to check if the joints are far enough