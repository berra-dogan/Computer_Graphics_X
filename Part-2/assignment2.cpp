#include <vector>
#include <random>
#include "headers/Polygon.hpp"
#include "headers/Voronoi.hpp"
#include <iostream>
#include "liblbfgs/lbfgs.h"

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
            vor.points[i] = Vector(uniform(engine), uniform(engine), 0);
        }
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
            float A = opt_trans->vor.diagram[i].area();
            g[i] = -(1.0/n - A); //Negative gradient
            fx += opt_trans->vor.diagram[i].integral_square_distance(opt_trans->vor.points[i]) - w[i]*(A-1.0/n);
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

int main() {

    int N = 100;
    OptimalTransport opt_trans(N);
    opt_trans.vor.compute();
    save_svg(opt_trans.vor.diagram, "rtd7_before_result_vor.svg", &opt_trans.vor.points);
    opt_trans.optimize();
    save_svg(opt_trans.vor.diagram, "rtd7_after_result_vor.svg", &opt_trans.vor.points);
    
    // show areas are the same (optional)
    return 0;
}


