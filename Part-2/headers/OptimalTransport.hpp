#pragma once
#include <random>
#include "Voronoi.hpp"
#include "../liblbfgs/lbfgs.h"

#define FLUID_VOL_PCT 0.4

std::default_random_engine engine(10);
static std::uniform_real_distribution<double> uniform(0, 1);

class OptimalTransport {
public:
    VoronoiDiagram vor;
    const int N;

    OptimalTransport(const int N) : N(N) {
        vor.points.resize(N);
        vor.weights.resize(N);
        for (int i = 0;  i<N; ++i){
            vor.points[i] = Vector(uniform(engine), uniform(engine), 0.0);
            vor.weights[i] = 0.0;
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
        const lbfgsfloatval_t *w, //weights
        lbfgsfloatval_t *g, //gradient
        const int n,
        const lbfgsfloatval_t step
        )
    {
        OptimalTransport* opt_trans = static_cast<OptimalTransport*>(context);
        memcpy(&opt_trans->vor.weights[0], w, n*sizeof(w[0]));
        opt_trans->vor.compute();

        lbfgsfloatval_t fx = 0.0;
        for (int i = 0; i < n; ++i) {
            double A = opt_trans->vor.diagram[i].area();
            g[i] = -(FLUID_VOL_PCT/n - A); //Negative gradient
            fx += opt_trans->vor.diagram[i].integral_square_distance(opt_trans->vor.points[i]) - w[i]*(A-FLUID_VOL_PCT/n);
        }

        return -fx;
    }

    void optimize(){
        lbfgsfloatval_t fx;
        std::vector<lbfgsfloatval_t> weights(N, 0);
        lbfgs_parameter_t param;
        lbfgs_parameter_init(&param);

        int ret = lbfgs(N, &weights[0], &fx, evaluate, nullptr, this, &param);

        if (ret < 0){
            std::cerr << "L-BFGS optimization failed: ret = " << ret << std::endl;
            return;
        }
        for (int i = 0; i<N; ++i){
            vor.weights[i] = weights[i];
        }
        vor.compute();
    }
};
