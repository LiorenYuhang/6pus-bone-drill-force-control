# Protocol-reported force and ADC, not calibrated SI force. Axes ID1..ID6.
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_MOCK=1
uint8 SOURCE_HARDWARE=2
uint8 source
std_msgs/Header header
int64[6] rx_monotonic_ns
int32[6] force_g_raw
uint16[6] force_adc
bool[6] valid
int32[6] fault_codes
