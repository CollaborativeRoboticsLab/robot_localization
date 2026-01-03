std_msgs/Header header

# matrix containing innovation
std_msgs/Float64MultiArray innovation

# matrix containing innovation_covariance
std_msgs/Float64MultiArray innovation_covariance

# True if the last processed measurement failed the Mahalanobis gate
bool fault

# Origin of the last processed measurement (e.g., sensor topic name)
string origin

# Mahalanobis distance (in N-sigmas) for the last processed measurement
float64 mahalanobis_distance

# Configured Mahalanobis threshold (N-sigmas) used for the last measurement
float64 mahalanobis_threshold
