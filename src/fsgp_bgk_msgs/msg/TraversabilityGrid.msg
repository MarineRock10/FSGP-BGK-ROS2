# Stamped traversability grid message for inter-node communication
std_msgs/Header header

# Grid metadata
float32 resolution
float32 x_length
float32 y_length

# Robot pose when this grid was computed
float64[6] pose

# Grid data (all arrays have the same length = num_cells)
uint32 num_cells
float32[] grid_x
float32[] grid_y
float32[] mean_heights
float32[] traversability
