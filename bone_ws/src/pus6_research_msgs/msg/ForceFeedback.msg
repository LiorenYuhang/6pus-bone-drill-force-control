# Feedback availability must be explicit; zero value alone is not validity.
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_MOCK=1
uint8 SOURCE_HARDWARE=2
uint8 source
std_msgs/Header header
int64[6] rx_monotonic_ns
float64[6] force_n
bool[6] valid
int32[6] fault_codes
