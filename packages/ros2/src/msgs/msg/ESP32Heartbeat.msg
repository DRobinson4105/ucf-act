std_msgs/Header header
uint8 source          # 3=control, 4=safety (matches orin_link message type)
uint8 sequence
uint8 state           # 0=INIT, 1=NOT_READY, 2=READY, 3=ENABLE, 4=ACTIVE
uint8 fault_flags
uint8 status_flags
uint8 stop_flags
uint8 soc_pct         # battery SOC 0-100 (safety only, 0 on control)
