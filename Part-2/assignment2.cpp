#include "liblbfgs/lbfgs.h"
#include "headers/OptimalTransport.hpp"

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


