# save positions, speeds, etc

# offset
VBTS_TCP_OFFSET = [0, 0, 0.1225, 0, 0, 0]  # for raised indenter

# positions
INDENTER_POS_A = [0.600, -0.187, 0.067]
ABRASION_POS_A = [0.750, -0.2031, 0.129]
ABRASION_POS_B = [0.750, -0.16, 0.129]
GRATINGS_POS_A = [0.50385, -0.2377, 0.012]
SENS_POS_A = [0.480, -0.260, 0.032]

# speeds (m/s)
COMPRESSION_Z_SPEED = 5e-5
SHEAR_Z_SPEED = 5e-5
SHEAR_LAT_SPEED = 1e-5
RANGE_Z_SPEED = 1e-5
GRATINGS_Z_SPEED = 1e-6
SENS_Z_SPEED = 2e-5

# shear lateral distance
SHEAR_LAT_DISTANCE = 0.01   # in m

# force thresholds (N)
COMPRESSION_FORCE_THRESHOLD = 20
SHEAR_FORCE_Z_THRESHOLD = 10
SHEAR_FORCE_Y_THRESHOLD = 5
RANGE_FORCE_THRESHOLD = 30
GRATINGS_FORCE_THRESHOLD = 10
CONST_FORCE_THRESHOLD = 5

# cycles
COMPRESSION_CYCLES = 400
SHEAR_CYCLES = 5

# time period for saving still data
SAVE_PERIOD = 0.1  # in seconds

# asdf
#asdf