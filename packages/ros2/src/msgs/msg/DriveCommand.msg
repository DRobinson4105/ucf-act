std_msgs/Header header
uint16 throttle          # DAC level 800-4095 (0 = no throttle, 800+ = wheel movement)
uint16 steering_position # 0-720
uint8 braking_level      # 0-3 (0=released, 3=full brake)
