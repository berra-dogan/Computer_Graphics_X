#include "liblbfgs/lbfgs.h"
#include "headers/Fluid.hpp"
#include "headers/helpers.hpp"

int main() {
	int N = 100;
    double fluid_ratio = 0.25;
    double m_particle_ratio = 1.5;
    double dt = 0.002;
    double k = 0.004;
    double n_steps = 100;

    FluidEnvironment env(N, fluid_ratio, m_particle_ratio, dt);
    env.run_simulation(k, dt, n_steps);
}


