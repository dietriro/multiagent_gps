int32 id  # ID must be echoed back in SampleResult
int8 robot_num  # which robot we are refering to
State2D init_state

ControllerParams controller

# Trial information
int32 T  # Trajectory length
float64 frequency  # Controller frequency
