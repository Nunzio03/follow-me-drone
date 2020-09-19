import GPy
import numpy as np
import math
import safeopt


class Tuner:
    def __init__(self, noise_var, p0, i0, d0):

        # Measurement noise
        self.noise_var = noise_var #0.025 ** 2

        # Bounds on the inputs variable
        bounds = [(-5., 5.), (-5., 5.), (-5., 5.)]   # riguarda gli input

        # Define Kernel
        kernel = GPy.kern.RBF(input_dim=len(bounds), variance=2., lengthscale=1.0,
                              ARD=True)

        # Initial safe point

        self.x0 = np.array([[p0, i0, d0]])
        self.x_next = self.x0

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
        gp = GPy.models.GPRegression(self.x0, fun(self.x0), kernel, noise_var=noise_var)

        # The optimization routine
        self.opt = safeopt.SafeOptSwarm(gp, 0., bounds=bounds, threshold=0.2)


# parameter_set = safeopt.linearly_spaced_combinations(bounds, 100)
# opt = safeopt.SafeOpt(gp, parameter_set, 0., lipschitz=None, threshold=0.2)
# Obtain next query point

    def get_param(self):
        self.x_next = self.opt.optimize()
        return self.x_next

    def optimize(self, measurement):
        y_meas = measurement
        self.opt.add_new_data_point(self.x_next, y_meas)

    def get_best_param(self):
        print(self.opt.get_maximum())