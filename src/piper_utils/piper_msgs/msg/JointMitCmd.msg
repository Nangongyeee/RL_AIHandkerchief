# MIT Mode Control Command Message
# This message is used to control joints in MIT (Model-based Impedance Tuning) mode
# which allows for force/torque control with adjustable impedance parameters

# Motor number (1-6 for joints)
int32 motor_num

# Target position (radians)
float64 pos_ref

# Target velocity (rad/s) 
float64 vel_ref

# Proportional gain (0.0-500.0, reference: 10)
float64 kp

# Derivative gain (0.0-5.0, reference: 0.8)
float64 kd

# Target torque (Nm) - this is the key for force control
float64 t_ref
