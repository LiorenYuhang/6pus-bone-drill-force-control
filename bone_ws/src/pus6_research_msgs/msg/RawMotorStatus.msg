# Full LAF status response frame, already received by an independently owned transport.
# rx_monotonic_ns is host receipt-completion time, not device sample time.
uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_MOCK=1
uint8 SOURCE_HARDWARE=2
uint8 axis_id
uint8 source
int64 rx_monotonic_ns
uint8[] frame
