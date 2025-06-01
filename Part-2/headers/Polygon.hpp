#pragma once
#include <vector>
#include <iostream>
#include "../../Part-1/headers/Vector.hpp"

class Polygon {  

    public:
    
        double area() {
            double s = 0;
            for (int i = 0; i < vertices.size(); i++){ 
                int ip = (i==vertices.size()-1) ? 0 : (i+1);
                s += vertices[i][0]*vertices[ip][1]-vertices[ip][0]*vertices[i][1];
            }
            return std::abs(s)/2.;
        }
    
        double integral_square_distance(const Vector& Pi){
            if (vertices.size() < 3) {
                std::cout << "Error: Polygon has fewer than 3 vertices.\n";
                return 0.0; // or throw exception if preferred
            }
            double s = 0;
            for (int t=0; t<vertices.size()-2; t++){
                Vector c[3] = {vertices[0], vertices[t], vertices[t+1]};
                double integralT = 0;
                for (int k=0; k<3; k++) {
                    for (int l = k; l < 3; l++){
                        integralT += dot(c[k]-Pi, c[l]-Pi);
                    }
                }
                double areaT = std::abs(((c[1][1]-c[0][1])*(c[2][0]-c[0][0]))-((c[1][0]-c[0][0])*(c[2][0]-c[1][1])));
                s += integralT * areaT / 6;
            }
            return s;
        }
       
        std::vector<Vector> vertices;
    };  

