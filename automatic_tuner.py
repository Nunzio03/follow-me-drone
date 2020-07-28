import safeopt

import GPy
import numpy as np

import safeopt


# Measurement noise
noise_var = 0.05 ** 2

# Bounds on the inputs variable
bounds = [(-5., 5.), (-5., 5.), (-5., 5.)]   # riguarda gli input

# Define Kernel
kernel = GPy.kern.RBF(input_dim=len(bounds), variance=2., lengthscale=1.0,
                      ARD=True)

# Initial safe point
x0 = np.zeros((1, len(bounds)))


# Generate function with safe initial point at x=0
def sample_safe_fun():
    while True:
        fun = safeopt.sample_gp_function(kernel, bounds, noise_var, 10)
        if fun([0,0,0], noise=False) > 0.5:
            break
    return fun


# Define the objective function
fun = sample_safe_fun()

# The statistical model of our objective function
gp = GPy.models.GPRegression(x0, fun(x0), kernel, noise_var=noise_var)

# The optimization routine
opt = safeopt.SafeOptSwarm(gp, 0., bounds=bounds, threshold=0.2)
# parameter_set = safeopt.linearly_spaced_combinations(bounds, 100)
# opt = safeopt.SafeOpt(gp, parameter_set, 0., lipschitz=None, threshold=0.2)
# Obtain next query point
i=0
while i<20:
    x_next = opt.optimize()
    # Get a measurement from the real system
    y_meas = fun(x_next)
    # Add this to the GP model
    opt.add_new_data_point(x_next, y_meas)
    i+=1
    print(x_next, y_meas)

print(opt.get_maximum())
