#include <vector>
#include <random>
#include "../Part-1/headers/Vector.hpp"
#include "../Part-1/headers/Ray.hpp"
#include <iostream>

// if the Polygon class name conflicts with a class in wingdi.h on Windows, use a namespace or change the name
class Polygon {  

public:

    static Vector get_normal2D(Vector& v){
        return Vector(v[0], -v[1], 0);
    }
   
    std::vector<Vector> vertices;
};  

class VoronoiDiagram {
public:
    VoronoiDiagram() {}

    Polygon clip_by_bisector(const Polygon& V, const Vector P0, const Vector& Pi){
        const Vector P0Pi = Pi - P0;
        Polygon result;
        for (int i =0; i<V.vertices.size(); i++){

            const Vector& A = V.vertices[(i==0)? V.vertices.size()-1 : i-1];
            const Vector& B = V.vertices[i];

            Vector M = (P0 + Pi) * 0.5;
            double t = dot(M-A, P0Pi)/dot(B-A, P0Pi);
            Vector P =  A + t * (B - A);

            if ((B-P0).norm2()<=(B-Pi).norm2()){
                if ((A-P0).norm2()>(A-Pi).norm2()){
                    result.vertices.push_back(P);
                }
                result.vertices.push_back(B);
            } else if ((A-P0).norm2()<=(A-Pi).norm2()){
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
        square.vertices.push_back(Vector(0, 1)); // Fixed copy-paste error
    
        diagram.resize(points.size());
    
        #pragma omp parallel for schedule(dynamic)
        for (int i = 0; i < points.size(); i++) {
            Polygon cell = square;
            for (int j = 0; j < points.size(); j++) {
                if (i == j) continue;
                cell = clip_by_bisector(cell, points[i], points[j]);
            }
            diagram[i] = cell;
        }
    }
    
    std::vector<Vector> points;
    std::vector<Polygon> diagram;  

};
     
// saves a static svg file. The polygon vertices are supposed to be in the range [0..1], and a canvas of size 1000x1000 is created
void save_svg(const std::vector<Polygon>& polygons, std::string filename, const std::vector<Vector>* points = NULL, std::string fillcol = "none") {
	FILE* f = fopen(filename.c_str(), "w+");
	fprintf(f, "<svg xmlns = \"http://www.w3.org/2000/svg\" width = \"1000\" height = \"1000\">\n");
	for (int i = 0; i < polygons.size(); i++) {
		fprintf(f, "<g>\n");
		fprintf(f, "<polygon points = \"");
		for (int j = 0; j < polygons[i].vertices.size(); j++) {
			fprintf(f, "%3.3f, %3.3f ", (polygons[i].vertices[j][0] * 1000), (1000 - polygons[i].vertices[j][1] * 1000));
		}
		fprintf(f, "\"\nfill = \"%s\" stroke = \"black\"/>\n", fillcol.c_str());
		fprintf(f, "</g>\n");
	}

	if (points) {
		fprintf(f, "<g>\n");		
		for (int i = 0; i < points->size(); i++) {
			fprintf(f, "<circle cx = \"%3.3f\" cy = \"%3.3f\" r = \"3\" />\n", (*points)[i][0]*1000., 1000.-(*points)[i][1]*1000);
		}
		fprintf(f, "</g>\n");

	}

	fprintf(f, "</svg>\n");
	fclose(f);
}
     
     
// Adds one frame of an animated svg file. frameid is the frame number (between 0 and nbframes-1).
// polygons is a list of polygons, describing the current frame.
// The polygon vertices are supposed to be in the range [0..1], and a canvas of size 1000x1000 is created
void save_svg_animated(const std::vector<Polygon> &polygons, std::string filename, int frameid, int nbframes) {
    FILE* f;
    if (frameid == 0) {
        f = fopen(filename.c_str(), "w+");
        fprintf(f, "<svg xmlns = \"http://www.w3.org/2000/svg\" width = \"1000\" height = \"1000\">\n");
        fprintf(f, "<g>\n");
    } else {
        f = fopen(filename.c_str(), "a+");
    }
    fprintf(f, "<g>\n");
    for (int i = 0; i < polygons.size(); i++) {
        fprintf(f, "<polygon points = \""); 
        for (int j = 0; j < polygons[i].vertices.size(); j++) {
            fprintf(f, "%3.3f, %3.3f ", (polygons[i].vertices[j][0] * 1000), (1000-polygons[i].vertices[j][1] * 1000));
        }
        fprintf(f, "\"\nfill = \"none\" stroke = \"black\"/>\n");
    }
    fprintf(f, "<animate\n");
    fprintf(f, "    id = \"frame%u\"\n", frameid);
    fprintf(f, "    attributeName = \"display\"\n");
    fprintf(f, "    values = \"");
    for (int j = 0; j < nbframes; j++) {
        if (frameid == j) {
            fprintf(f, "inline");
        } else {
            fprintf(f, "none");
        }
        fprintf(f, ";");
    }
    fprintf(f, "none\"\n    keyTimes = \"");
    for (int j = 0; j < nbframes; j++) {
        fprintf(f, "%2.3f", j / (double)(nbframes));
        fprintf(f, ";");
    }
    fprintf(f, "1\"\n   dur = \"5s\"\n");
    fprintf(f, "    begin = \"0s\"\n");
    fprintf(f, "    repeatCount = \"indefinite\"/>\n");
    fprintf(f, "</g>\n");
    if (frameid == nbframes - 1) {
        fprintf(f, "</g>\n");
        fprintf(f, "</svg>\n");
    }
    fclose(f);
}

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


