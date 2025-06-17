#include "liblbfgs/lbfgs.h"
#include "headers/Fluid.hpp"
#include "headers/helpers.hpp"
#include <random>

int lab6() {
    std::default_random_engine engine(10);
    static std::uniform_real_distribution<double> uniform(0, 1);

    int N = 1000;
    VoronoiDiagram Vor;
    for (int i = 0; i <N; i++){
        Vor.points.push_back(Vector(uniform(engine), uniform(engine), 0.0));
    }
    Vor.weights.resize(N, 1.0);
    auto start_time = std::chrono::high_resolution_clock::now();
    Vor.compute();
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    std::cout << "Elapsed time: " << duration.count() << " seconds" << std::endl;

    save_svg(Vor.diagram, "output/lab6_result.svg", &Vor.points);
    return 0;
}

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

int lab7() {

    test_integral();

    int N = 1000;
    OptimalTransport opt_trans(N);
    opt_trans.vor.compute();
    double var_before = opt_trans.compute_cell_area_variance();
    save_svg(opt_trans.vor.diagram, "output/lab7_before_optim.svg", &opt_trans.vor.points);

    auto start = std::chrono::high_resolution_clock::now();
    opt_trans.optimize();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    std::cout << "Optimization for lab7 took " << duration.count() << " .\n";

    save_svg(opt_trans.vor.diagram, "output/lab7_after_optim.svg", &opt_trans.vor.points);
    double var_after = opt_trans.compute_cell_area_variance();

    std::cout << "Area variance before: " << var_before << std::endl;
    std::cout << "Area variance after:  " << var_after << std::endl;
    
    return 0;
}


int lab8() {
	int N = 100;
    double dt = 0.002;
    double k = 0.004;
    double n_steps = 175;
    double m = 200.;

    Fluid fluid(N);

    auto start = std::chrono::high_resolution_clock::now();

    fluid.run_simulation(k, dt, n_steps, m);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;

    std::cout << "Simulation took " << duration.count() << " seconds for " << n_steps <<" frames. \n";

    return 0;
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Wrong number of arguments";
        std::cerr << "The intended usage: " << argv[0] << " <example_number>\n";
        return 1;
    }

    int n = std::atoi(argv[1]);

    switch (n) {
        case 6: lab6(); break;
        case 7: lab7(); break;
        case 8: lab8(); break;
        default:
            std::cerr << "Invalid Lab Result number. Please input a number from {6, 7, 8}.\n";
            return 1;
    }

    return 0;
}
