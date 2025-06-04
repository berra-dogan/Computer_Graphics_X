#pragma once
#include <random>
#include "Voronoi.hpp"
#include "../liblbfgs/lbfgs.h"

#define VOL_FLUID 0.6

std::default_random_engine engine(10);
static std::uniform_real_distribution<double> uniform(0, 1);

class OptimalTransport {
public:
    VoronoiDiagram vor;
    const int N;

    OptimalTransport(const int N) : N(N) {
        vor.points.resize(N);
        vor.weights.resize(N+1, 0);
        for (int i = 0;  i<N; ++i){
            vor.points[i] = Vector(uniform(engine), uniform(engine), 0.0);
            vor.weights[i] = 0.01;
        }
    }

    double compute_cell_area_variance() {
        int N = vor.diagram.size();
        std::vector<double> areas(N);
        double sum = 0.0;
    
        for (int i = 0; i < N; ++i) {
            areas[i] = vor.diagram[i].area();
            sum += areas[i];
        }
    
        double mean = sum / N;
        double variance = 0.0;
    
        for (int i = 0; i < N; ++i) {
            double diff = areas[i] - mean;
            variance += diff * diff;
        }
    
        return variance / N;
    }

    static lbfgsfloatval_t evaluate(
        void* context,
        const lbfgsfloatval_t* w,  // Weights
        lbfgsfloatval_t* g,        // Gradient
        const int n,
        const lbfgsfloatval_t step
    ) {
        OptimalTransport* ot = static_cast<OptimalTransport*>(context);
        
        std::memcpy(&ot->vor.weights[0], w, n * sizeof(w[0]));
        ot->vor.compute();
    
        lbfgsfloatval_t fx = 0.0;
        double sum_fluid_areas = 0.0;
    
        int n_particles = n - 1;
    
        for (int i = 0; i < n_particles; ++i) {
            double area = ot->vor.diagram[i].area();
            sum_fluid_areas += area;

            g[i] = -(VOL_FLUID / n_particles - area);
    
            fx += ot->vor.diagram[i].integral_square_distance(ot->vor.points[i])
                  - w[i] * (area - VOL_FLUID / n_particles);
        }
    
        // Handle the air weight (last component)
        double estimated_air_volume = 1.0 - sum_fluid_areas;
        double desired_air_volume = 1.0 - VOL_FLUID;
    
        g[n - 1] = -(desired_air_volume - estimated_air_volume);
        fx += w[n - 1] * (desired_air_volume - estimated_air_volume);
    
        return -fx;  // L-BFGS minimizes, so return negative
    }
    

    void optimize(){
        int N_weights = vor.weights.size();
        lbfgsfloatval_t fx;
        lbfgs_parameter_t param;
        lbfgs_parameter_init( &param );
        std::vector<double> weights( N_weights, 0 );
        
        memcpy( &weights[0], &vor.weights[0], N_weights * sizeof( weights[0] ) );
        int ret = lbfgs( N_weights, &weights[0], &fx, evaluate, nullptr, ( void*)this, &param );
        // if (ret < 0){
        //     std::cerr << "L-BFGS optimization failed: ret = " << ret << std::endl;
        //     return;
        // }
        memcpy( &vor.weights[0], &weights[0], N_weights * sizeof( weights[0] ) );
        
        vor.compute(); //maybe
    }
    
};
