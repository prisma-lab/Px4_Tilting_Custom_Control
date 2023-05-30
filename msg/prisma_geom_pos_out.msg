uint64 timestamp

# Force expressed in the inertia frame
float32[3] f_w

# Row-major representation of computed rotation matrix
float32[9] rc

# Computer angular velocity
float32[3] wc

# Computer angular acceleration
float32[3] wc_dot

# TOPICS prisma_geom_pos_out