#pragma once

#include "Vector.hpp"
#include "Ray.hpp"
#include "helpers.hpp"
#include <cassert>

class Sphere {
    public:
        explicit Sphere(double x, double y, double z, double R, double r, double g, double b, 
                        bool is_mirror = false, bool is_transparent = false, double ref_idx = 1., bool is_light = false)
        : center(x, y, z), R(R), albedo(r, g, b), is_mirror(is_mirror), is_transparent(is_transparent), ref_idx(ref_idx), is_light(is_light) {
            albedo.normalize();
            assert(!(is_mirror && is_transparent) && "Error message");
        }

        explicit Sphere(const Vector& c, double R = 1, const Vector& albedo = Vector(0,0,0), 
                        bool is_mirror = false, bool is_transparent = false, double ref_idx = 1., bool is_light = false)
            : center(c), R(R), albedo(albedo), is_mirror(is_mirror), is_transparent(is_transparent), ref_idx(ref_idx), is_light(is_light) {
                assert(!(is_mirror && is_transparent) && "Error message");
            }
        
        const Vector& getCenter() const { return center; }
        double getRadius() const { return R; }

        Intersection findIntersection (const Ray& ray) const {
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
        Vector albedo;
        bool is_mirror;
        bool is_transparent;
        bool is_light;
        double ref_idx;
};

class LightSource {
    public:
        explicit LightSource(Vector c, double I) : c(c), I(I) {}
    Vector c;
    double I;
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
    

class LightSource {
public:
    explicit LightSource(const Vector& c, double I)
        : c(c), I(I) {}

    virtual Vector GetRandomPoint(std::default_random_engine& engine, const Vector& x) const {
        // Optional: default is just returning the position (e.g., for point lights)
        return c;
    }

    double I;  // Intensity
    Vector c;
};


class PointLight : public LightSource {
    public:
        explicit PointLight(const Vector& c, double I)
            : LightSource(c, I) {}
};


class SphereLight : public Sphere, public LightSource {
    public:
        explicit SphereLight(const Vector& c, double R = 1, const Vector& albedo = Vector(1, 1, 1), double I = 1e10)
            : Sphere(c, R, albedo), LightSource(c, I) {}
    
        Vector GetRandomPoint(std::default_random_engine& engine, const Vector& x) const override {
            Vector D = x - center;
            D.normalize();
            Vector V = random_cos(engine, D);
            return R * V + center;
        }
};
    