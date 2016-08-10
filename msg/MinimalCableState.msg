# This is a representation of a cable in 3d space as a Gaussian Process
# regression model.

float32[] t
float32[] chi_t

# Each chi_t corresponds to a t at the same index, which should be increasing
# as the index increases.
