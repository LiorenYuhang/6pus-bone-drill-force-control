# Standard Header with ROS node clock publish time
std_msgs/Header header

# Host-side CLOCK_MONOTONIC timing in nanoseconds
int64 snapshot_start_monotonic_ns
int64 snapshot_end_monotonic_ns
# Host-side status reception/drain completion for ID1..ID6; position and force
# come from the SAME status response. Not device sample times.
int64[6] per_axis_rx_done_monotonic_ns

# 6-axis actual feedback measurements
float64[6] positions_meters
int32[6] raw_steps
int32[6] currents_ma
int32[6] forces_g_raw
int32[6] temperatures_c
int32[6] fault_codes

# Shared 1-to-1 UART cycle sequence counter
uint64 uart_cycle_sequence

# Force metadata is explicit: old/uninitialized publishers remain UNKNOWN/invalid.
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_MOCK=1
uint8 SOURCE_HARDWARE=2
uint8 source
bool[6] force_valid
# Separate 0x2D sensor ADC counts, not force or calibrated N.
uint16[6] force_adc_raw

# UART owner's acknowledged Mode 3 handover reference, not a new sensor zero.
# False/zero metadata from legacy publishers must not initialize stabilization.
bool force_mode_confirmed
bool force_hold_valid
int32[6] force_hold_g
int64 force_hold_reference_monotonic_ns
