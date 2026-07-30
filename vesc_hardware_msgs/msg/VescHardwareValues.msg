# VESC hardware interface telemetry values
# Mirrors the VescPacketValues structure from vesc_driver

# Temperature measurements (degrees Celsius)
float64 temp_fet             # FET temperature
float64 temp_motor           # Motor temperature
float64 temp_mos1            # MOSFET 1 temperature
float64 temp_mos2            # MOSFET 2 temperature
float64 temp_mos3            # MOSFET 3 temperature

# Current measurements (amperes)
float64 avg_motor_current    # Average motor current
float64 avg_input_current    # Average input current
float64 avg_id               # Average D-axis current
float64 avg_iq               # Average Q-axis current

# Voltage measurements (volts)
float64 v_in                 # Input voltage
float64 avg_vd               # Average D-axis voltage
float64 avg_vq               # Average Q-axis voltage

# Duty cycle and speed
float64 duty_cycle_now       # Current duty cycle (0.0 to 1.0)
float64 rpm                  # Motor electrical speed (RPM)

# Energy and charge tracking
float64 amp_hours            # Charge drawn (ampere-hours)
float64 amp_hours_charged    # Charge regenerated (ampere-hours)
float64 watt_hours           # Energy drawn (watt-hours)
float64 watt_hours_charged   # Energy regenerated (watt-hours)

# Position and distance tracking
int32 tachometer             # Net tachometer count
int32 tachometer_abs         # Absolute tachometer count
float64 pid_pos_now          # Current PID position (degrees)

# Status
int32 fault_code             # Fault code (0 = no fault)
int32 controller_id          # VESC controller ID
