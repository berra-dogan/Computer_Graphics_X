#include "liblbfgs/lbfgs.h"
#include "headers/OptimalTransport.hpp"
#include "headers/helpers.hpp"

void test_integral(){
    Polygon pp;
    pp.vertices.push_back(Vector(0., 0., 0.));
    pp.vertices.push_back(Vector(0., 1., 0.));
    pp.vertices.push_back(Vector(1., 1., 0.));
    pp.vertices.push_back(Vector(1., 0., 0.));

    double result1 = pp.integral_square_distance(Vector(1.5, 1.5, 0.));
    double result2 = pp.integral_square_distance(Vector(0.5, 0.5, 0));

    double expected1 = 2.16667;
    double expected2 = 0.16667;
    double epsilon = 1e-5;

    assert(std::abs(result1 - expected1) < epsilon);
    assert(std::abs(result2 - expected2) < epsilon);
}

int main() {

    test_integral();

    int N = 1000;
    OptimalTransport opt_trans(N);
    opt_trans.vor.compute();
    double var_before = opt_trans.compute_cell_area_variance();
    save_svg(opt_trans.vor.diagram, "rtd7_before_result_vor.svg", &opt_trans.vor.points);
    opt_trans.optimize();
    save_svg(opt_trans.vor.diagram, "rtd7_after_result_vor.svg", &opt_trans.vor.points);
    double var_after = opt_trans.compute_cell_area_variance();

    std::cout << "Area variance before: " << var_before << std::endl;
    std::cout << "Area variance after:  " << var_after << std::endl;
    
    // show areas are the same (optional)
    return 0;
}


