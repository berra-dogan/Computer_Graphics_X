#include "OptimalTransport.hpp"
#include "helpers.hpp"

class FluidEnvironment{
    public:
        FluidEnvironment(int N, double fluid_ratio, double m_particle_ratio, double dt ) : opt_trans(OptimalTransport(N)), dt(dt){
            velocities.resize( N, Vector( 0, 0, 0 ) );
            opt_trans.vor.weights.resize( N + 1 );
            std::fill(opt_trans.vor.weights.begin(), opt_trans.vor.weights.end(), 1.);
            is_fluid.resize(N, false);
            masses.resize(N, 200.);
    
            for (int i = 0; i < N * fluid_ratio; ++i) {
                is_fluid[i] = true;
                masses[i] = 200.*m_particle_ratio;
            }
        };
    
        void time_step(double k, double dt) {
            Vector g(0, -9.81, 0);
            opt_trans.optimize();
        
            for (int i = 0; i < opt_trans.N; i++) {
                Vector centerCell = opt_trans.vor.diagram[i].centroid();
                Vector F_g = masses[i] * g;
                Vector F_spring = (centerCell - opt_trans.vor.points[i]) / (k * k);
                Vector F_total = F_spring + F_g;
                velocities[i] = velocities[i] +(dt / masses[i]) * F_total;
                opt_trans.vor.points[i] = opt_trans.vor.points[i] + dt * velocities[i];
            }
        }
           
        void run_simulation(double k, double dt, int n_steps) {
            std::cout << "Starting simulation with " << opt_trans.N << " particles..." << std::endl;
            for (int i = 0; i <= 100; i++) {
                if (i % 10 == 0) {
                    std::cout << "Processing frame " << i << " of 1000" << std::endl;
                    save_frame(opt_trans.vor.diagram, is_fluid, "output/test", i);
                }
                time_step(k, dt);
            }
        }
    
        OptimalTransport opt_trans;
        std::vector<bool> is_fluid;
        std::vector<Vector> velocities;
        std::vector<double> masses;
        double dt;
    };