#pragma once
#include "Polygon.hpp"

class VoronoiDiagram {
    public:
        VoronoiDiagram() {}
    
        Polygon clip_by_bisector(const Polygon& V, const Vector P0, const Vector& Pi, double w0, double wi){
            const Vector P0Pi = Pi - P0;
            Vector M = (P0 + Pi) * 0.5;
            Vector Mprime = M + (w0-wi)/(2*P0Pi.norm2())*P0Pi; //Power Diagram (Laguerre Diagram)
    
            Polygon result;
            for (int i =0; i<V.vertices.size(); i++){
    
                const Vector& A = V.vertices[(i==0)? V.vertices.size()-1 : i-1];
                const Vector& B = V.vertices[i];
    
                double t = dot(Mprime-A, P0Pi)/dot(B-A, P0Pi);
                Vector P =  A + t * (B - A);
    
                if ((B-P0).norm2()-w0<=(B-Pi).norm2()-wi){
                    if ((A-P0).norm2()-w0>(A-Pi).norm2()-wi){
                        result.vertices.push_back(P);
                    }
                    result.vertices.push_back(B);
                } else if ((A-P0).norm2()-w0<=(A-Pi).norm2()-wi){
                    result.vertices.push_back(P);
                }
            }
            return result;
        }
    
        void compute() {
            Polygon square;
            square.vertices.push_back(Vector(1, 1));
            square.vertices.push_back(Vector(1, 0));
            square.vertices.push_back(Vector(0, 0));
            square.vertices.push_back(Vector(0, 1));
        
            diagram.resize(points.size());
        
            #pragma omp parallel for schedule(dynamic)
            for (int i = 0; i < points.size(); i++) {
                Polygon cell = square;
                for (int j = 0; j < points.size(); j++) {
                    if (i == j) continue;
                    cell = clip_by_bisector(cell, points[i], points[j], weights[i], weights[j]);
                }
                diagram[i] = cell;
            }
        }
        
        std::vector<Vector> points;
        std::vector<Polygon> diagram;  
        std::vector<double> weights;
    
    };

