#include <vector>
#include <random>
#include "headers/Polygon.hpp"
#include "headers/Voronoi.hpp"
#include <iostream>

int main() {
    std::default_random_engine engine(10);
    static std::uniform_real_distribution<double> uniform(0, 1);

    int N = 100;
    VoronoiDiagram Vor;
    for (int i = 0; i <N; i++){
        Vor.points.push_back(Vector(uniform(engine), uniform(engine), 0.0));
    }
    auto start_time = std::chrono::high_resolution_clock::now();
    Vor.compute();
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    std::cout << "Elapsed time: " << duration.count() << " seconds" << std::endl;

    save_svg(Vor.diagram, "result_vor.svg", &Vor.points);
    return 0;
}


