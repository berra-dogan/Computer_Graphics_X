#pragma once

#include <Vector.hpp>
#include <random>

#define EPSILON 1E-6
static std::uniform_real_distribution<double> uniform(0, 1);

void boxMuller2D(std::default_random_engine &engine, double stdev , double &x, double &y) { 
    double r1 = uniform ( engine ) ;
    double r2 = uniform ( engine ) ;
    x = sqrt(-2 * log(r1))*cos(2 * M_PI*r2)*stdev; 
    y = sqrt(-2 * log(r1))*sin(2 * M_PI*r2)*stdev;
}

void boxMuller3D(std::default_random_engine &engine, double &x, double &y, double &z) { 
    double r1 = uniform ( engine ) ;
    double r2 = uniform ( engine ) ;
    x = cos(2 * M_PI*r2)*sqrt(1-r1); 
    y = sin(2 * M_PI*r2)*sqrt(1-r1);
    z = sqrt(r1);
}

Vector random_cos(std::default_random_engine &engine, const Vector &N){
    Vector T1;
    double x, y, z = 0;
    boxMuller3D(engine, x, y, z);
    int min_index = (std::abs(N[0]) <= std::abs(N[1]) && std::abs(N[0]) <= std::abs(N[2])) ? 0 :
                    (std::abs(N[1]) <= std::abs(N[2])) ? 1 : 2;

                    
    T1[(min_index+2)%3] = N[(min_index+1)%3];
    T1[(min_index+1)%3] = -N[(min_index+2)%3];
    T1[min_index] = 0;

    T1.normalize();

    Vector T2 = cross(T1, N);

    return x*T1+y*T2+z*N;
}