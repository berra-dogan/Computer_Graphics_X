#pragma once
#include "Polygon.hpp"

class VoronoiDiagram {
    public:
        VoronoiDiagram() {
            int N_disk = 100;
            disk.vertices.resize( N_disk );
            for( int i = 0; i < N_disk; i++ ) {
                double theta;
                theta = i * 2 * M_PI / (double) N_disk;
                disk.vertices[i] = Vector( sin( -theta ), cos( -theta ), 0 );
                //unit_disk[i] = Vector( cos( -theta ), sin( -theta ), 0 );
            }
        }
    
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

        Polygon clip_by_edge(const Polygon& V, const Vector& u, const Vector& v ) const{
        
            const Vector N( v[1] - u[1], u[0] - v[0], 0 );
            Polygon result;
            result.vertices.reserve( V.vertices.size() + 1 );
    
            for ( int i = 0 ; i < V.vertices.size() ; i++) {
                const Vector &A = (i==0)? V.vertices[V.vertices.size() - 1]:V.vertices[i-1];
                const Vector &B = V.vertices[i];
                double t = dot(u-A, N) / dot(B-A, N);
                Vector P = A + t*(B-A);
               
                if (dot(u-B, N) >= 0) { 
                    // B is inside or on the plane
                    if (dot(u - A, N) < 0) {
                        // A is outside → edge crosses plane from outside to inside
                        result.vertices.push_back( P ) ;
                    }
                    result.vertices.push_back( B ) ;
                } 
                else if (dot(u - A, N) >= 0 ) {
                    // A is inside and B is outside → edge crosses plane from inside to outside
                    result.vertices.push_back( P ) ;
                }
            }
    
            return result;
        }

        Polygon clip_by_disk(const Polygon& V, const Vector& center, double R) const {
            Polygon result(V);
            for (int i=0; i<disk.vertices.size(); i++) {
                const Vector& u = disk.vertices[i]*R + center; 
                const Vector& v = disk.vertices[(i+1)%disk.vertices.size()]* R + center;
                result = clip_by_edge(result, u, v);
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
                diagram[i] = clip_by_disk(cell, points[i], sqrt(weights[i] - weights[weights.size()-1]));
            }
        }
        
        Polygon disk;
        std::vector<Vector> points;
        std::vector<Polygon> diagram;  
        std::vector<double> weights;
    
    };

