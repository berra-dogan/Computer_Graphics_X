#include "liblbfgs/lbfgs.h"
#include "headers/Fluid.hpp"
#include "headers/helpers.hpp"

// // Tutte's Embedding Algorithm
// std::vector<Vector> tutte(std::vector<Vector>& vertices){
//     std::vector<Vector> boundary_vertices = {}; 
//     double s = 0;
//     int n = boundary_vertices.size();
//     for (int i = 0; i < n; ++i){
//         int next_i = (i < n - 1) ? i+1 : 0;
//         s += (boundary_vertices[next_i]-boundary_vertices[i]).norm();
//     }
//     double cs = 0;
//     std::vector<Vector> res(vertices);
//     for (int i = 0; i<n; ++i){
//         int next_i = (i < n - 1) ? i+1 : 0;
//         double theta = 2*M_PI*cs/s;
//         cs += (boundary_vertices[next_i]-boundary_vertices[i]).norm();
//     }
//     for (int iter =0; i<vertices.size(); ++i){
//         if (true){ //interior

//         } else{ //

//         }
//     }

// }

int main() {
	int N = 100;
    double fluid_ratio = 0.25;
    double m_particle_ratio = 1.5;
    double dt = 0.002;
    double k = 0.004;
    double n_steps = 100;
    double m = 200.;

    Fluid fluid(N);
    fluid.run_simulation(k, dt, n_steps, m);
}


