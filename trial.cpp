#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
 
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "Vector.hpp"
#include "Ray.hpp"
#include "Geometry.hpp"

#define NB_PATHS 10

#include <random>
#include <omp.h>
#include <iostream>

class Scene {
    public:
        explicit Scene(std::vector<std::shared_ptr<Geometry>> objects, std::vector<std::shared_ptr<LightSource>> lights, double ref_idx = 1.)
            : objects(std::move(objects)), lights(std::move(lights)), ref_idx(ref_idx) {}
    
        // Method to find the closest intersection with a ray
        std::pair<Intersection, int> findIntersection(const Ray& ray, int in_obj_idx = std::numeric_limits<double>::max()) const {
            Intersection closestIntersection;
            double closest_t = std::numeric_limits<double>::max();
            int closestIdx = -1;
    
            for (size_t idx = 0; idx < objects.size(); ++idx) {
                if (idx==in_obj_idx) continue;
                const Geometry& sphere = *objects[idx];
                Intersection intersection = sphere.findIntersection(ray);  // Find intersection with the sphere
                if (intersection.isIntersected()) {
                    if (!closestIntersection.is_intersected || intersection.t < closest_t) { 
                        closestIntersection = intersection;
                        closestIdx = idx;
                        closest_t = intersection.t;
                    }
                }
            }

            return {closestIntersection, closestIdx};
        }

        std::pair<Intersection, int> findIntersectionLight(const Ray& ray) const {
            Intersection closestIntersection;
            double closest_t = std::numeric_limits<double>::max();
            int closestLightIdx = -1;
    
            for (size_t idx = 0; idx < lights.size(); ++idx) {
                const LightSource& light = *lights[idx];
                Intersection intersection = light.findIntersection(ray);  
                if (intersection.isIntersected()) {
                    if (!closestIntersection.is_intersected || intersection.t < closest_t) { 
                        closestIntersection = intersection;
                        closestLightIdx = idx;
                        closest_t = intersection.t;
                    }
                }
            }
    
            return {closestIntersection, closestLightIdx};  // Return the closest intersection (if any)
        }

        Vector getColor(Ray& r, std::default_random_engine &engine, bool last_bounce_diffuse, int ray_depth = 10, int in_obj_idx = -1) {
            if (ray_depth==0) return Vector(0,0,0);
            std::pair<Intersection, int> result = findIntersection(r, in_obj_idx);
            std::pair<Intersection, int> result_light = findIntersectionLight(r);

            if (result_light.first.isIntersected() && (!result.first.isIntersected() || (result_light.first.t<result.first.t))){
                if (last_bounce_diffuse){
                    return Vector(0.,0.,0.);
                } else {
                    LightSource light = *lights[result_light.second];
                    return light.albedo*light.I/(4*M_PI*M_PI*light.R*light.R);
                }
            }
            if (result.first.isIntersected()){
                int obj_idx = result.second;
                std::shared_ptr<Geometry> obj = objects[obj_idx];
                Vector P = result.first.intersection;
                Vector N = result.first.normal; 
                if (obj->is_mirror){
                    r.O.update(P+EPSILON*N);
                    r.u = r.u - 2*dot(r.u, N)*N;
                    return getColor(r, engine, false, ray_depth-1);
                } 
                else if (obj->is_transparent){
                    double k0_sqrt = (ref_idx-obj->ref_idx)/(ref_idx+obj->ref_idx);
                    double k0 = k0_sqrt*k0_sqrt;
                    double dp = dot(r.u, N);
                    double R = k0 + (1-k0)*std::pow(1-abs(dp), 5);
                    double T = 1 - R;
                    double ratio = ref_idx/obj->ref_idx;
                    if (dp>=0) {
                        ratio = 1/ratio;
                        N = -N;
                        dp = -dp;
                    }

                    double t_N_val = 1-ratio*ratio*(1-dp*dp);

                    std::uniform_real_distribution<double> distribution(0.0,1.0);
                    double num = distribution(engine);
                    if (t_N_val<0 || num<R){
                        Ray r_rfl(P+EPSILON*N, r.u - 2*dot(r.u, N)*N);
                        return getColor(r_rfl, engine,  true, ray_depth-1);
                    } else {
                        Vector t_N = -sqrt(t_N_val)*N;
                        Vector t_T = ratio*(r.u-dp*N);
                        Ray r_rfr(P-EPSILON*N, t_N + t_T);
                        return getColor(r_rfr, engine, true, ray_depth-1, obj_idx);
                    }

                } else {
                    Vector Lo(0.,0.,0.);
                    for (const std::shared_ptr<LightSource> light_ptr: lights){
                        LightSource& light = *light_ptr;
                        Vector xprime = light.GetRandomPoint(engine, P);
                        Vector Nprime = (xprime-light.center)/(xprime-light.center).norm();
                        Vector omega_i = (xprime-P)/(xprime-P).norm();
                        double d2 = (xprime-P).norm2()-EPSILON;
                        
                        Ray shadowRay(P + EPSILON * N, omega_i);
                        std::pair<Intersection, int> shadowResult = findIntersection(shadowRay, obj_idx);

                        bool inShadow = shadowResult.first.isIntersected() && shadowResult.first.t * shadowResult.first.t < d2;
                        
                        if (inShadow) {
                            Lo = Lo + Vector(0., 0., 0.);
                        } else {
                            double pdf = std::max(dot(Nprime, (P-light.center)/(P-light.center).norm())/(M_PI*light.R*light.R), 0.);
                            Lo = Lo + light.I/(4*M_PI*M_PI*light.R*light.R) * (*objects[obj_idx]).albedo/M_PI * std :: max(
                                dot(N, omega_i), 0.)*std::max(dot(Nprime, -omega_i), 0.)/((xprime-P).norm2() * pdf);
                        }
                    }

                    //Indirect Lighting
                    Ray randomRay(P+EPSILON*N, random_cos(engine, N));
                    Lo = Lo + obj->albedo * getColor(randomRay, engine, ray_depth - 1, obj_idx);

                    return Lo;
                }
            }
            return Vector(0.,0.,0.);
            
        }
    
    std::vector<std::shared_ptr<Geometry>> objects; 
    std::vector<std::shared_ptr<LightSource>> lights; 
    double ref_idx;
};

int main() {
    int W = 512;
    int H = 512;
    Vector camera_origin(0, 0, 55);
    double fov = 60*M_PI/180;
    Vector albedo(0.5, 0.5, 0.5);
    //Sphere S(Vector(0,0,0), 10, albedo);

    TriangleMesh catMesh;
    catMesh.readOBJ("cadnav.com_model/Models_F0202A090/cat.obj");
    catMesh.applyTransform(Vector(0.6, 0.6, 0.6), Vector(0, -10, 0));

    std::vector<std::shared_ptr<Geometry>> objects = {
        std::make_shared<Sphere>(Vector(0, 0, 1000), 940, Vector(0.9, 0.2, 0.9)),  // left-wall
        std::make_shared<Sphere>(Vector(0, -1000, 0), 990, Vector(0.3, 0.4, 0.7)), // floor
        std::make_shared<Sphere>(Vector(0, 0, -1000), 940, Vector(0.6, 0.5, 0.1)), // right-wall
        std::make_shared<Sphere>(Vector(0, 1000, 0), 940, Vector(0.2, 0.5, 0.9)),  // ceiling
        std::make_shared<Sphere>(Vector(-1000, 0, 0), 940, Vector(0.4, 0.8, 0.7)), // back-wall
        std::make_shared<Sphere>(Vector(1000, 0, 0), 940, Vector(0.9, 0.4, 0.3)),  // wall-behind-camera
        // std::make_shared<Sphere>(Vector(-18, 0, 0), 8, Vector(0.8, 0.8, 0.8), true), // mirror
        // std::make_shared<Sphere>(Vector(18, 0, 0), 8, Vector(0.8, 0.8, 0.8), false, true, 1.5), // transparent
        // std::make_shared<Sphere>(Vector(0, 0, 0), 8, Vector(0, 0, 0), false, true, 1.5),       // hollow-transp 1
        // std::make_shared<Sphere>(Vector(0, 0, 0), 8 - EPSILON, Vector(0, 0, 0), false, true, 1), // hollow-transp 2
        std::make_shared<TriangleMesh>(catMesh),
        //std::make_shared<Sphere>(Vector(0,0,0), 10, Vector(0.8,0.8,0.8)), //trial
    };

    std::vector<std::shared_ptr<LightSource>> standard_scene_lights = {
        std::make_shared<LightSource>(Vector(-10, 25, -10), 5, Vector(1., 1., 1.), 3e10),
        //LightSource(Vector(20, 25, -10), 1, Vector(1, 1, 1), 1e10),
    };

    Scene scene(objects, standard_scene_lights);
    std::vector<unsigned char> image(W * H * 3, 0);
    
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {
            std::default_random_engine engine(i * W + j);
            double z = -W / (2. * tan(fov/2));
            Vector r_dir(j-W/2+0.5, H/2-i+0.5, z);
            r_dir.normalize();
            
            // Antialiasing
            double stdev = 0.001;
            Vector pixelColor(0., 0., 0.);
            for (int k = 0; k < NB_PATHS; k++) {
                Vector sample = random_cos(engine, r_dir);
                sample.normalize();
                Vector random_dir = r_dir+sample*stdev;
                random_dir.normalize();
                Ray ray(camera_origin, random_dir);
                pixelColor = pixelColor + scene.getColor(ray, engine, false);
            }
            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(pixelColor[0]/NB_PATHS, 1/2.2));
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(pixelColor[1]/NB_PATHS, 1/2.2));
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(pixelColor[2]/NB_PATHS, 1/2.2));
        }
    }
    stbi_write_png("image.png", W, H, 3, &image[0], 0);
 
    return 0;
}
