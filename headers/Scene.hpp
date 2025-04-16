#pragma once
#include "Vector.hpp"
#include "Geometry.hpp"
#include <iostream>

class Scene {
    public:
        explicit Scene(std::vector<std::shared_ptr<Geometry>> objects, std::vector<std::shared_ptr<LightSource>> lights, double ref_idx = 1.)
            : objects(std::move(objects)), lights(std::move(lights)), ref_idx(ref_idx) {
                includeIndirectLight = true;
            }
    
        // Method to find the closest intersection with a ray
        std::pair<Intersection, int> findIntersection(const Ray& ray, int in_obj_idx = std::numeric_limits<double>::max()) const {
            Intersection closestIntersection;
            double closest_t = std::numeric_limits<double>::max();
            int closestIdx = -1;
    
            for (size_t idx = 0; idx < objects.size(); ++idx) {
                if (idx==in_obj_idx) continue;
                const Geometry& obj = *objects[idx];
                Intersection intersection = obj.findIntersection(ray);  // Find intersection with the sphere
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
                const std::shared_ptr<LightSource>& lightPtr = lights[idx];
                if (lightPtr->type != LightType::SPHERE) {
                    continue;
                }
                const SphereLight* sphereLight = dynamic_cast<const SphereLight*>(lightPtr.get());
                if (!sphereLight) {
                    std::cerr << "Warning: Light type is SPHERE but dynamic_cast failed.\n";
                    continue;
                }
                Intersection intersection = sphereLight->findIntersection(ray);  
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
                } 
                std::shared_ptr<LightSource> basePtr = lights[result_light.second];
                if (basePtr->type == LightType::SPHERE) {
                    SphereLight* light = dynamic_cast<SphereLight*>(basePtr.get());
                    return light->albedo*light->I/(4*M_PI*M_PI*light->R*light->R);
                }
            }
            if (result.first.isIntersected()){
                int obj_idx = result.second;
                std::shared_ptr<Geometry> obj = objects[obj_idx];
                Vector P = result.first.intersection;
                Vector N = result.first.normal; 
                Vector texture = result.first.texture; 
                if (obj->is_mirror){
                    r.O.update(P+EPSILON*N);
                    r.u = r.u - 2*dot(r.u, N)*N;
                    return getColor(r, engine, false, ray_depth-1);
                } 
                else if (obj->is_transparent){
                    // With Fresnel
                    double k0_sqrt = (ref_idx-obj->ref_idx)/(ref_idx+obj->ref_idx);
                    double k0 = k0_sqrt*k0_sqrt;
                    double dp = dot(r.u, N);
                    double R = k0 + (1-k0)*std::pow(1-abs(dp), 5);
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

                } else { //Diffuse Object
                    Vector Lo(0.,0.,0.);
                    for (const std::shared_ptr<LightSource>& light_ptr: lights){
                        if ((*light_ptr).type==LightType::POINT){
                            const PointLight& pointLight = static_cast<const PointLight&>(*light_ptr);
                            Vector lightDir = pointLight.c - P;
                            N.normalize();
                            double d2 = lightDir.norm2();
                            lightDir.normalize();

                            Ray shadowRay(P + EPSILON * N, lightDir);
                            std::pair<Intersection, int> shadowResult = findIntersection(shadowRay, result.second);

                            bool inShadow = shadowResult.first.isIntersected() && shadowResult.first.t * shadowResult.first.t < d2;

                            if (inShadow) {
                                Lo = Lo + Vector(0., 0., 0.);
                            } else {
                                Lo = Lo + (pointLight.I/(4*M_PI*d2)) * (obj->albedo/M_PI) * std::max(0., dot(N, lightDir));
                            }

                        } else if ((*light_ptr).type==LightType::SPHERE){
                            const SphereLight& sphereLight = static_cast<const SphereLight&>(*light_ptr);

                            Vector xprime = sphereLight.GetRandomPoint(engine, P);
                            Vector Nprime = (xprime-sphereLight.c)/(xprime-sphereLight.c).norm();
                            Vector omega_i = (xprime-P)/(xprime-P).norm();
                            double d2 = (xprime-P).norm2()-EPSILON;
                            
                            Ray shadowRay(P + EPSILON * N, omega_i);
                            std::pair<Intersection, int> shadowResult = findIntersection(shadowRay, obj_idx);

                            bool inShadow = shadowResult.first.isIntersected() && shadowResult.first.t * shadowResult.first.t < d2;
                            
                            if (inShadow) {
                                Lo = Lo + Vector(0., 0., 0.);
                            } else {
                                double pdf = std::max(dot(Nprime, (P-sphereLight.c)/(P-sphereLight.c).norm())/(M_PI*sphereLight.R*sphereLight.R), 0.);
                                Lo = Lo + sphereLight.I/(4*M_PI*M_PI*sphereLight.R*sphereLight.R) * texture/M_PI * std :: max(
                                    dot(N, omega_i), 0.)*std::max(dot(Nprime, -omega_i), 0.)/((xprime-P).norm2() * pdf);
                            }
                        }
                    }

                    if (includeIndirectLight){
                        Ray randomRay(P+EPSILON*N, random_cos(engine, N));
                        Lo = Lo + obj->albedo * getColor(randomRay, engine, true, ray_depth - 1, obj_idx);
                    }

                    return Lo;
                }
            }
            return Vector(0.,0.,0.);
            
        }
    
    std::vector<std::shared_ptr<Geometry>> objects; 
    std::vector<std::shared_ptr<LightSource>> lights; 
    double ref_idx;
    bool includeIndirectLight;
};