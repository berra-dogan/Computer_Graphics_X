#pragma once
#include <vector>
#include <iostream>
#include "../../Part-1/headers/Vector.hpp"

class Polygon {  

    public:

        Vector centroid(){
            double A = area();
            double c_x = 0;
            double c_y = 0;
            for (int i = 0; i < vertices.size(); i++){ 
                int next_i = i<vertices.size()-1 ? i+1 : 0;
                Vector v1 = vertices[i];
                Vector v2 = vertices[next_i];

                c_x += (v1[0]+v2[0])*(v1[0]*v2[1]-v2[0]*v1[1]);
                c_y += (v1[1]+v2[1])*(v1[0]*v2[1]-v2[0]*v1[1]);
            }

            return Vector(c_x/(6*A), c_y/(6*A), 0);
        }
        
        double area() {
            double s = 0;
            for (int i = 0; i < vertices.size(); i++){ 
                int ip = (i==vertices.size()-1) ? 0 : (i+1);
                s += vertices[i][0]*vertices[ip][1]-vertices[ip][0]*vertices[i][1];
            }
            return std::abs(s)/2.;
        }
    
        double integral_square_distance(const Vector& Pi ){
            if( vertices.size() < 3 ) 
                return 0;
    
            double s = 0;
    
            for( int t = 1; t < vertices.size() - 1; t++ ){
                Vector c[3] = { vertices[0], vertices[t], vertices[t + 1] };
    
                double integral = 0;
    
                for( int k = 0; k < 3; k++ ){
                    for( int l = k; l < 3; l++ ){
                        integral += dot( c[k] - Pi, c[l] - Pi );
                    }
                }
                Vector edge1 = c[1] - c[0];
                Vector edge2 = c[2] - c[0];
                //double areaT = (c[1][1] - c[0][1]) *( c[2][0]-c[0][0] ) ..(POZA)
                double areaT = 0.5 * std::abs( edge1[0] * edge2[1] - edge1[1] * edge2[0] );
                s += integral * areaT / 6.;
            }
            return s;
        }
       
        std::vector<Vector> vertices;
    };  

