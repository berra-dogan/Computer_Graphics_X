#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
 
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "Vector.hpp"
#include "Ray.hpp"
#include "Objects.hpp"

#define EPSILON 1e-6
#define NB_PATHS 10

#include <random>
#include <omp.h>


class Scene {
    public:
        explicit Scene(std::vector<Sphere> spheres, double ref_idx = 1.)
            : spheres(std::move(spheres)), ref_idx(ref_idx) {}
    
        void add(const Sphere& new_sphere) { spheres.push_back(new_sphere); }
        const std::vector<Sphere>& getSceneSpheres() const { return spheres; }
    
        // Method to find the closest intersection with a ray
        std::pair<Intersection, int> findIntersection(const Ray& ray, int in_obj_idx = -1) const {
            Intersection closestIntersection;
            double closest_t = std::numeric_limits<double>::max();
            int closestSphereIdx = -1;
    
            for (size_t idx = 0; idx < spheres.size(); ++idx) {
                if (idx==in_obj_idx) continue;
                const Sphere& sphere = spheres[idx];
                Intersection intersection = sphere.findIntersection(ray);  // Find intersection with the sphere
                if (intersection.isIntersected()) {
                    if (!closestIntersection.is_intersected || intersection.t < closest_t) { 
                        closestIntersection = intersection;
                        closestSphereIdx = idx;
                        closest_t = intersection.t;
                    }
                }
            }
    
            return std::make_pair(closestIntersection, closestSphereIdx);  // Return the closest intersection (if any)
        }

        Vector getColor(Ray& r, const LightSource L, std::default_random_engine &engine, int ray_depth = 10, int in_obj_idx = -1) {
            if (ray_depth==0) return Vector(0,0,0);
            std::pair<Intersection, int> result = findIntersection(r, in_obj_idx);

            if (result.first.isIntersected()){
                Sphere obj = spheres[result.second];
                Vector P = result.first.intersection;
                Vector N = result.first.normal; 
                if (obj.is_mirror){
                    r.O.update(P+EPSILON*N);
                    r.u = r.u - 2*dot(r.u, N)*N;
                    return getColor(r, L, engine, ray_depth-1);
                } 
                else if (obj.is_transparent){
                    double k0_sqrt = (ref_idx-obj.ref_idx)/(ref_idx+obj.ref_idx);
                    double k0 = k0_sqrt*k0_sqrt;
                    double dp = dot(r.u, N);
                    double R = k0 + (1-k0)*std::pow(1-abs(dp), 5);
                    double T = 1 - R;
                    double ratio = ref_idx/obj.ref_idx;
                    if (dp>=0) {
                        ratio = 1/ratio;
                        N = -N;
                        dp = -dp;
                    }

                    //Reflection
                    Ray r_copy2 = r;
                    r_copy2.O.update(P+EPSILON*N);
                    r_copy2.u = r_copy2.u - 2*dot(r_copy2.u, N)*N;
                    Vector res_rfl = getColor(r_copy2, L, engine, ray_depth-1);

                    double t_N_val = 1-ratio*ratio*(1-dp*dp);
                    if (t_N_val<0) return res_rfl;

                    //Refraction
                    Ray r_copy = r;
                    Vector t_N = -sqrt(t_N_val)*N;
                    Vector t_T = ratio*(r_copy.u-dp*N);
                    r_copy.O.update(P-EPSILON*N);
                    r_copy.u = t_N + t_T;
                    Vector res_rfr = getColor(r_copy, L, engine, ray_depth-1, result.second);

                    return T*res_rfr+R*res_rfl;
                } else {
                    Vector lightDir = L.pos - P;
                    N.normalize();
                    double d2 = lightDir.norm2();
                    lightDir.normalize();

                    Ray shadowRay(P + EPSILON * N, lightDir);
                    std::pair<Intersection, int> shadowResult = findIntersection(shadowRay, result.second);

                    bool inShadow = shadowResult.first.isIntersected() && shadowResult.first.t * shadowResult.first.t < d2;
                    if (inShadow) return Vector(0, 0, 0); // No direct lighting if in shadow

                    Vector Lo((L.I/(4*M_PI*d2)) * (obj.albedo/M_PI) * std::max(0., dot(N, lightDir)));

                    // Indirect Lighting
                    // Ray randomRay = r;
                    // randomRay.O.update(P+EPSILON*N);
                    // randomRay.u = random_cos(engine, N);
                    // Lo = Lo + obj.albedo * getColor(randomRay, L, engine, ray_depth - 1, result.second);

                    return Lo;
                }
            }
            return Vector(0,0,0);
            
        }
    
    std::vector<Sphere> spheres; 
    std::vector<LightSource> lights; 
    double ref_idx;
};

int main() {
    int W = 512;
    int H = 512;
    Vector camera_origin(0, 0, 55);
    double fov = 60*M_PI/180;
    Vector albedo(0.5, 0.5, 0.5);
    Sphere S(Vector(0,0,0), 10, albedo);
    LightSource L(Vector(-10, 20, 40), 1e10);

    std::vector<Sphere> standard_scene_spheres = {
        Sphere(Vector(0,0,1000), 940, Vector(0.9,0.2,0.9)), // left-wall
        Sphere(Vector(0,-1000,0), 990, Vector(0.3,0.4,0.7)), // floor
        Sphere(Vector(0,0,-1000), 940, Vector(0.6,0.5,0.1)), // right-wall
        Sphere(Vector(0,1000,0), 940, Vector(0.2,0.5,0.9)), // ceiling
        Sphere(Vector(-1000,0,0), 940, Vector(0.4,0.8,0.7)), // back-wall
        Sphere(Vector(1000,0,0), 940, Vector(0.9,0.4,0.3)), // wall-behind-camera
        Sphere(Vector(-18,0,0), 8, Vector(0.8,0.8,0.8), true), //mirror
        Sphere(Vector(0,0,0), 8, Vector(0.8,0.8,0.8), false, true, 1.5, false), //transperent
        Sphere(Vector(18,0,0), 8, Vector(0,0,0), false, true, 1.5, false), //hallow-transperent1
        Sphere(Vector(18,0,0), 8-EPSILON, Vector(0,0,0), false, true, 1, false), //hallow-transperent2
    };
    
    Scene scene(standard_scene_spheres);
    std::vector<unsigned char> image(W * H * 3, 0);
    
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < H; i++) {
        std::default_random_engine engine(i);
        for (int j = 0; j < W; j++) {

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
                pixelColor = pixelColor + scene.getColor(ray, L, engine);
            }
            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(pixelColor[0]/NB_PATHS, 1/2.2));
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(pixelColor[1]/NB_PATHS, 1/2.2));
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(pixelColor[2]/NB_PATHS, 1/2.2));
        }
    }
    stbi_write_png("image.png", W, H, 3, &image[0], 0);
 
    return 0;
}


int correct_main() {
    int W = 512;
    int H = 512;
    Vector camera_origin(0, 0, 55);
    double fov = 60*M_PI/180;
    Vector albedo(0.5, 0.5, 0.5);
    Sphere S(Vector(0,0,0), 10, albedo);
    LightSource L(Vector(-10, 20, 40), 1e10);
    // Vector L_pos(-10, 20, 40);
    // LightSource L(L_pos, 1., Vector(1, 1, 1), 1e10);

    std::vector<Sphere> standard_scene_spheres = {
        Sphere(Vector(0,0,1000), 940, Vector(0.9,0.2,0.9)), // left-wall
        Sphere(Vector(0,-1000,0), 990, Vector(0.3,0.4,0.7)), // floor
        Sphere(Vector(0,0,-1000), 940, Vector(0.6,0.5,0.1)), // right-wall
        Sphere(Vector(0,1000,0), 940, Vector(0.2,0.5,0.9)), // ceiling
        Sphere(Vector(-1000,0,0), 940, Vector(0.4,0.8,0.7)), // back-wall
        Sphere(Vector(1000,0,0), 940, Vector(0.9,0.4,0.3)), // wall-behind-camera
        Sphere(Vector(-18,0,0), 8, Vector(0.8,0.8,0.8), true), //mirror
        Sphere(Vector(0,0,0), 8, Vector(0.8,0.8,0.8), false, true, 1.5), //transperent
        Sphere(Vector(18,0,0), 8, Vector(0,0,0), false, true, 1.5), //hallow-transperent1
        Sphere(Vector(18,0,0), 8-EPSILON, Vector(0,0,0), false, true, 1), //hallow-transperent2
    };
    
    Scene scene(standard_scene_spheres);
    std::vector<unsigned char> image(W * H * 3, 0);
    
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < H; i++) {
        std::default_random_engine engine(i);
        for (int j = 0; j < W; j++) {
            double z = -W / (2. * tan(fov/2));
            Vector r_dir(j-W/2+0.5, H/2-i+0.5, z);
            r_dir.normalize();
            Ray r(camera_origin, r_dir);

            Vector color = scene.getColor(r, L, engine);
            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(color[0], 1/2.2));
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(color[1], 1/2.2));
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(color[2], 1/2.2));
        }
    }
    stbi_write_png("image.png", W, H, 3, &image[0], 0);
 
    return 0;
}
