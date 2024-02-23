# Message containing the info of the 3D lidar sensor

std_msgs/Header header

int16 rows
int16 cols

float64 phi_start
float64 phi_end

float64 theta_start
float64 theta_end

float64[] theta_vec

bool is_theta_linear
