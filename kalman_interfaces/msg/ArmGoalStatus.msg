uint8 SUCCESS = 0 # The goal was achieved successfully. 
uint8 CANCELLING = 1 # Sent cancel request to the action server, which is still processing the goal.
uint8 CANCEL_SUCCESS = 2 # The goal was successfully cancelled.
uint8 CANCEL_FAILED = 3 # The goal could not be cancelled
uint8 ABORT_RECEIVED = 4 # Received an abort request
uint8 GOAL_ACCEPTED = 5 # The goal was accepted by the action server
uint8 GOAL_REJECTED = 6 # The goal was rejected by the action server
uint8 FAILED = 7 # Failed to achieve the goal
uint8 PREEMPTING = 8 # Sent preempt request because another goal was received
uint8 INVALID_ID = 9 # The goal ID is invalid
uint8 TOO_FAR = 10 # The start of goal is too far from the current position
uint8 EXCEPTION = 11 # An exception occurred
uint8 GOAL_SENDING = 12 # The goal is being sent to the action server

uint8 IDLE = 255

# status id
uint8 status