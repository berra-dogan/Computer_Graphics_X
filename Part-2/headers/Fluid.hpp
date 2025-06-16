#include "OptimalTransport.hpp"
#include "helpers.hpp"

class Fluid{
    public:
        Fluid(int N) : opt_trans(OptimalTransport(N)){
            velocities.resize( N, Vector( 0, 0, 0 ) );
            particles.resize(N);
            // opt_trans.vor.weights.resize( N + 1 );
            // std::fill(opt_trans.vor.weights.begin(), opt_trans.vor.weights.end(), 1.);

            for (int i = 0; i < N; ++i) {
                particles[i] = Vector(uniform(engine), uniform(engine), 0);
            }
        };
    
        void time_step(double eps, double dt, double m) {
            Vector g(0, -9.81, 0);
            opt_trans.vor.points = particles;
            opt_trans.optimize();
        
            for (int i = 0; i < opt_trans.N; i++) {
                Vector centerCell = opt_trans.vor.diagram[i].centroid();
        
                Vector F_g = m * g;
                Vector F_spring = (centerCell - particles[i]) / (eps*eps);
                Vector F_total = F_spring + F_g;
        
                velocities[i] = velocities[i] + (dt / m) * F_total;
                particles[i] =  particles[i] + dt * velocities[i];
            }
        }
        
           
        void run_simulation(double k, double dt, int n_steps, double m) {
            std::cout << "Starting simulation with " << opt_trans.N << " particles..." << std::endl;
            for (int i = 0; i <= n_steps; i++) {
                if (i % 10 == 0) {
                    std::cout << "Processing frame " << i << " of " << n_steps << std::endl;
                }
                save_frame(opt_trans.vor.diagram, "output/test", i);
                time_step(k, dt, m);
            }
        }
    
        OptimalTransport opt_trans;
        std::vector<Vector> particles;
        std::vector<Vector> velocities;
    };