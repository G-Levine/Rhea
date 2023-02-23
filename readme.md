# State machine:
## if operating_mode == IDLE:
    action = idle_control(state)
## if operating_mode == NOT_ON_GROUND:
    if contact_state_both:
        operating_mode = ON_GROUND
        reset_x_estimate()
        reset_yaw_estimate()
    else:
        action = z_height_control(state, command)
## if operating_mode == ON_GROUND:
	if not contact_state_both:
		operating_mode = NOT_ON_GROUND
	else:
        action = z_height_control(state, command) + balance_control(state, command) + yaw_control(state, command)

# State estimation:
contact_state_r = leg_force_r > threshold
contact_state_l = leg_force_l > threshold
contact_state_both = contact_state_r and contact_state_l

z height


# Control:
z-height: PD
roll: none
pitch: LQR
yaw: PD for the difference between wheel positions and velocities# Rhea
