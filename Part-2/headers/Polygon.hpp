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
                return 0.0;
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
                Vector edge1 = c[1] - c[0];
                Vector edge2 = c[2] - c[0];
                double areaT = 0.5 * std::abs( edge1[0] * edge2[1] - edge1[1] * edge2[0] );
                s += integralT * areaT / 6;
            }
            return s;
        }
       
        std::vector<Vector> vertices;
    };  

