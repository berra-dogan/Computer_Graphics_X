#pragma once

#include "Vector.hpp"
#include "Ray.hpp"
#include "helpers.hpp"
#include <cassert>
#include <iostream>
#include <list>

class Geometry {
    public:
        virtual ~Geometry() = default;  // Virtual destructor for proper cleanup
    
        // Pure virtual function that must be implemented by derived classes
        virtual Intersection findIntersection(const Ray& ray) const = 0;
    
        // Common properties that will be inherited by derived classes
        Vector albedo;    
        bool is_mirror = false;   
        bool is_transparent = false;   
        double ref_idx = 1.0;     
};
    

class Sphere: public Geometry {
    public:
        Sphere(const Vector& center, double radius, const Vector& albedo, bool is_mirror = false, bool is_transparent = false, double ref_idx = 1.5)
        : center(center), R(radius) {
            assert(!(is_mirror && is_transparent) && "Error message");
            
            this->albedo = albedo;         
            this->is_mirror = is_mirror;
            this->is_transparent = is_transparent;
            this->ref_idx = ref_idx;

        }
        
        const Vector& getCenter() const { return center; }
        double getRadius() const { return R; }

        Intersection findIntersection (const Ray& ray) const override {
            const Vector oc = ray.O - center;
            double b = dot(ray.u, oc);
            double c = oc.norm2() - R*R;
            double discriminant = b*b - c;
            
            if (discriminant>=0) {
                double t = -b - sqrt(discriminant);
                if (t<0){
                    t = -b + sqrt(discriminant);
                    if (t<0) return Intersection(false);
                }
                Vector intersectionPoint = ray.getOrigin() + ray.getDirection() * t; //P
                Vector pc = intersectionPoint - center;
                Vector normal = pc / pc.norm();
                return Intersection(true, discriminant==0, t, intersectionPoint, normal);
            }
            return Intersection(false);  // No intersection
        }
    public:
        const Vector center;
        const double R;
};

class LightSource : public Sphere {
    public:
        explicit LightSource(const Vector& c, double R = 1, const Vector& albedo = Vector(1, 1, 1), double I = 1e10)
            : Sphere(c, R, albedo), I(I) {}

        Vector GetRandomPoint(std::default_random_engine &engine, const Vector& x){
            Vector D = x - center;
            D.normalize();
            Vector V = random_cos(engine, D);
            return R*V+center;
        }
    
        double I;
};

class BoundingBox {
    public:
        BoundingBox(Vector m = Vector(0, 0, 0), Vector M = Vector(0, 0, 0)) : m(m), M(M) {}
    
        bool intersect(const Ray& r, double& inter_distance) const {
            // Compute t values for the X axis
            if (r.u[0]==0 || r.u[1]==0 || r.u[2]==0) return true;

            double tx1 = (m[0] - r.O[0]) / r.u[0];
            double tx2 = (M[0] - r.O[0]) / r.u[0];
            double txMin = std::min(tx1, tx2);
            double txMax = std::max(tx1, tx2);
    
            // Compute t values for the Y axis
            double ty1 = (m[1] - r.O[1]) / r.u[1];
            double ty2 = (M[1] - r.O[1]) / r.u[1];
            double tyMin = std::min(ty1, ty2);
            double tyMax = std::max(ty1, ty2);
    
            // Compute t values for the Z axis
            double tz1 = (m[2] - r.O[2]) / r.u[2];
            double tz2 = (M[2] - r.O[2]) / r.u[2];
            double tzMin = std::min(tz1, tz2);
            double tzMax = std::max(tz1, tz2);
    
            // Compute the overall min and max t values
            double t1 = std::max(txMin, std::max(tyMin, tzMin));
            double t2 = std::min(txMax, std::min(tyMax, tzMax));
    
            // Check if there is an intersection
            inter_distance = t1;
            return (t2 >= t1 && t1>-EPSILON);
        }
    
        Vector m, M;
};

struct NodeBVH {
    BoundingBox bbox;
    int starting_triangle;
    int ending_triangle;
    NodeBVH* child_left;
    NodeBVH* child_right;

    NodeBVH(const BoundingBox& bbox_ = BoundingBox(), int start_ = 0, size_t end_ = 0, NodeBVH* left_ = nullptr, NodeBVH* right_ = nullptr)
        : bbox(bbox_), starting_triangle(start_), ending_triangle(end_), child_left(left_), child_right(right_) {}
};
    

class TriangleIndices {
    public:
        TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
        };
        int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
        int uvi, uvj, uvk;  // indices within the uv coordinates array
        int ni, nj, nk;  // indices within the normals array
        int group;       // face group

};
     
    
class TriangleMesh: public Geometry {
public:
    TriangleMesh(const Vector& albedo, 
        bool is_mirror = false, bool is_transparent = false, double ref_idx = 1.5) {
            assert(!(is_mirror && is_transparent) && "Error message");
            this->albedo = albedo;         
            this->is_mirror = is_mirror;
            this->is_transparent = is_transparent;
            this->ref_idx = ref_idx;
    }
    
    void readOBJ(const char* obj) {

        char matfile[255];
        char grp[255];

        FILE* f;
        f = fopen(obj, "r");
        int curGroup = -1;
        while (!feof(f)) {
            char line[255];
            if (!fgets(line, 255, f)) break;

            std::string linetrim(line);
            linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
            strcpy(line, linetrim.c_str());

            if (line[0] == 'u' && line[1] == 's') {
                sscanf(line, "usemtl %[^\n]\n", grp);
                curGroup++;
            }

            if (line[0] == 'v' && line[1] == ' ') {
                Vector vec;

                Vector col;
                if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
                    col[0] = std::min(1., std::max(0., col[0]));
                    col[1] = std::min(1., std::max(0., col[1]));
                    col[2] = std::min(1., std::max(0., col[2]));

                    vertices.push_back(vec);
                    vertexcolors.push_back(col);

                } else {
                    sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                    vertices.push_back(vec);
                }
            }
            if (line[0] == 'v' && line[1] == 'n') {
                Vector vec;
                sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                normals.push_back(vec);
            }
            if (line[0] == 'v' && line[1] == 't') {
                Vector vec;
                sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
                uvs.push_back(vec);
            }
            if (line[0] == 'f') {
                TriangleIndices t;
                int i0, i1, i2, i3;
                int j0, j1, j2, j3;
                int k0, k1, k2, k3;
                int nn;
                t.group = curGroup;

                char* consumedline = line + 1;
                int offset;

                nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
                if (nn == 9) {
                    if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
                    if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
                    if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
                    if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
                    if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
                    if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
                    if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
                    if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
                    if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
                    indices.push_back(t);
                } else {
                    nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
                    if (nn == 6) {
                        if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
                        if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
                        if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
                        if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
                        if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
                        if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
                        indices.push_back(t);
                    } else {
                        nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
                        if (nn == 3) {
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
                            indices.push_back(t);
                        } else {
                            nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
                            if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
                            if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
                            if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
                            indices.push_back(t);
                        }
                    }
                }

                consumedline = consumedline + offset;

                while (true) {
                    if (consumedline[0] == '\n') break;
                    if (consumedline[0] == '\0') break;
                    nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
                    TriangleIndices t2;
                    t2.group = curGroup;
                    if (nn == 3) {
                        if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
                        if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
                        if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
                        if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
                        if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
                        if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
                        if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
                        if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
                        if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;
                        indices.push_back(t2);
                        consumedline = consumedline + offset;
                        i2 = i3;
                        j2 = j3;
                        k2 = k3;
                    } else {
                        nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
                        if (nn == 2) {
                            if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
                            if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
                            if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
                            if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
                            if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
                            if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
                            consumedline = consumedline + offset;
                            i2 = i3;
                            j2 = j3;
                            indices.push_back(t2);
                        } else {
                            nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
                            if (nn == 2) {
                                if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
                                if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
                                if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
                                if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
                                if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
                                if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;								
                                consumedline = consumedline + offset;
                                i2 = i3;
                                k2 = k3;
                                indices.push_back(t2);
                            } else {
                                nn = sscanf(consumedline, "%u%n", &i3, &offset);
                                if (nn == 1) {
                                    if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
                                    if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
                                    if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
                                    consumedline = consumedline + offset;
                                    i2 = i3;
                                    indices.push_back(t2);
                                } else {
                                    consumedline = consumedline + 1;
                                }
                            }
                        }
                    }
                }

            }

        }
        fclose(f);

    }

    void applyTransform(const Vector& scale, const Vector& translation) {
        for (Vector& vertex : vertices) {
            vertex = vertex * scale + translation;
        }
    }

    static int get_longest(const Vector& diag) {
        if (diag[0]>=diag[1] && diag[0]>=diag[2]) return 0;
        else if (diag[1]>=diag[2]) return 1;
        return 2;
    }

    Vector FindBarycenter(const TriangleIndices& triangle) const {
        Vector A = vertices[triangle.vtxi];
        Vector B = vertices[triangle.vtxj];
        Vector C = vertices[triangle.vtxk];
        return (A+B+C)/3;
    }

    BoundingBox compute_bbox(int starting_triangle, int ending_triangle) const {
        BoundingBox bounding_box;
        bounding_box.m = Vector(1E9,1E9,1E9);
        bounding_box.M = Vector(-1E9,-1E9,-1E9);
        for (int i = starting_triangle; i<ending_triangle; i++) {
            for (Vector vertex : {vertices[indices[i].vtxi], vertices[indices[i].vtxj], vertices[indices[i].vtxk]}){
                for (int j=0; j<3; j++){
                    bounding_box.m[j] = std::min(bounding_box.m[j], vertex[j]);
                    bounding_box.M[j] = std::max(bounding_box.M[j], vertex[j]);
                }
            }
        }

        return bounding_box;
    }

    void ConstructBVH(NodeBVH& node, int starting_triangle, int ending_triangle) const {

        node.bbox = compute_bbox(starting_triangle, ending_triangle); 
        node.starting_triangle = starting_triangle;
        node.ending_triangle = ending_triangle;

        Vector diag = node.bbox.M-node.bbox.m;
        Vector middle_diag = node.bbox.m+diag*0.5;
        int longest_axis = get_longest(diag);
        int pivot_index = starting_triangle;

        for (int i=starting_triangle; i<ending_triangle; ++i){
            Vector barycenter = FindBarycenter(indices[i]);
            if (barycenter[longest_axis] < middle_diag[longest_axis]){
                std::swap(indices[i], indices[pivot_index]);
                ++pivot_index;
            }
        }

        if (pivot_index<=starting_triangle || pivot_index>=ending_triangle-1 || ending_triangle-starting_triangle<5) return;
    
        ConstructBVH(*node.child_left, starting_triangle, pivot_index);
        ConstructBVH(*node.child_right, pivot_index, ending_triangle);

    }    
    
    Intersection findIntersection (const Ray& ray) const override {

        NodeBVH root;
        //ConstructBVH(root, 0, indices.size());
        
        double distance;
        // if (!root.bbox.intersect(ray, distance)) return Intersection();

        // std::list<NodeBVH*> nodes_to_visit;
        // nodes_to_visit.push_front(&root);
        double best_inter_distance = std::numeric_limits<double>::max();
        NodeBVH curNode;
        
        // while (!nodes_to_visit.empty()){
        //     curNode = nodes_to_visit.back();
        //     nodes_to_visit.pop_back();
        //     double inter_distance = 0;
        //     if (curNode->child_left){
        //         if (curNode->child_left->bbox.intersect(ray, inter_distance)){
        //             if (inter_distance<best_inter_distance){
        //                 nodes_to_visit.push_back(curNode->child_left);
        //             }
        //         }
        //     }
        //     if (curNode->child_right){
        //         if (curNode->child_right->bbox.intersect(ray, inter_distance)){
        //             if (inter_distance<best_inter_distance){
        //                 nodes_to_visit.push_back(curNode->child_right);
        //             }
        //         }
        //     }   
        //     if (!curNode->child_left && !curNode->child_right){
        //         break;
        //     }
        // }

        Vector closest_P, closest_N, texture = Vector(0, 0, 0);
        for (int i = 0; i<indices.size(); i++){
            TriangleIndices* triangle = &indices[i];

            Vector A = vertices[triangle->vtxi];
            Vector B = vertices[triangle->vtxj];
            Vector C = vertices[triangle->vtxk];

            Vector N_A = normals[triangle->vtxi];
            Vector N_B = normals[triangle->vtxj];
            Vector N_C = normals[triangle->vtxk];

            Vector uv_A = uvs[triangle->vtxi];
            Vector uv_B = uvs[triangle->vtxj];
            Vector uv_C = uvs[triangle->vtxk];

            Vector e1 = B - A;
            Vector e2 = C - A;
            Vector N = cross(e1, e2);
            
            double uN_dot = dot(ray.u, N);
            if (abs(uN_dot)<EPSILON) continue;
    
            Vector AO = A - ray.O;
            double t = dot(AO, N)/uN_dot;
            if (t<EPSILON || t>best_inter_distance) continue;
    
            double beta = dot(e2, cross(AO, ray.u)) / uN_dot;
            double gamma =  -dot(e1, cross(AO, ray.u)) / uN_dot;
            double alpha = 1 - beta - gamma;
    
            if (!((alpha >= 0.0f && alpha <= 1.0f) && (beta >= 0.0f && beta <= 1.0f) && (gamma >= 0.0f && gamma <= 1.0f))){
                continue;
            }
    
            best_inter_distance = t;
            closest_P = A + beta*e1 + gamma*e2;
            N.normalize();
            //closest_N = N;
            closest_N = alpha*N_A+beta*N_B+gamma*N_C;
            texture = alpha*uv_A+beta*uv_B+gamma*uv_C;
        }
    
        if (best_inter_distance == std::numeric_limits<double>::max()) return Intersection(false);
    
        return Intersection(true, false, best_inter_distance, closest_P, closest_N, texture);
        
    }    
    
    mutable std::vector<TriangleIndices> indices;
    std::vector<Vector> vertices;
    std::vector<Vector> normals;
    std::vector<Vector> uvs;
    std::vector<Vector> vertexcolors;
    BoundingBox bounding_box;
    
};
