#pragma once

#include "Vector.hpp"

class Ray {
    public:
        explicit Ray(double x_o = 0, double y_o = 0, double z_o = 0, double x_u = 1, double y_u = 0, double z_u = 0)
            : O(x_o, y_o, z_o), u(x_u, y_u, z_u) {u.normalize();}
        
        explicit Ray(const Vector& O, const Vector& u)
            : O(O), u(u) {this->u.normalize();}
    
        const Vector& getOrigin() const { return O; }
        void changeOrigin(const Vector& new_O) {}

        const Vector& getDirection() const { return u; }
    
        Vector O;
        Vector u;
};

class Intersection {
    public:
        explicit Intersection(bool is_intersected = false, bool is_tangent = false, double t = 0, const Vector intersection =  Vector(), const Vector normal =  Vector() )
        : is_intersected(is_intersected), is_tangent(is_tangent), t(t), intersection(intersection), normal(normal) {}

        bool isIntersected() const { return is_intersected; }
        bool isTangent() const { return is_tangent; }
        const Vector& getIntersection() const { return intersection; }
        const Vector& getNormal() const { return normal; }

    bool is_intersected;
    bool is_tangent;
    double t;
    Vector intersection;
    Vector normal;
};