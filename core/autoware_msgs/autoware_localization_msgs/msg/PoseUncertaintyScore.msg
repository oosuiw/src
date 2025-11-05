# Individual uncertainty scores for each pose component
# Used for validation and research purposes to analyze uncertainty behavior

std_msgs/Header header

# Uncertainty score for x position
float64 score_x

# Uncertainty score for y position
float64 score_y

# Uncertainty score for z position
float64 score_z

# Uncertainty score for yaw angle
float64 score_yaw

# Combined total score (sum of all components)
float64 score_total
