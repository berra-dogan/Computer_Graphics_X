
#pragma once
#include "Vector.hpp"
#include "Geometry.hpp"
#include "Scene.hpp"
#include <iostream>

int example1() {
    int W = 512;
    int H = 512;
    Vector camera_origin(0, 0, 55);
    double fov = 60*M_PI/180;
    Vector albedo(0.5, 0.5, 0.5);

    std::vector<std::shared_ptr<Geometry>> objects = {
        std::make_shared<Sphere>(Vector(0, 0, 1000), 940, Vector(0.9, 0.2, 0.9)),  // left-wall
        std::make_shared<Sphere>(Vector(0, -1000, 0), 990, Vector(0.3, 0.4, 0.7)), // floor
        std::make_shared<Sphere>(Vector(0, 0, -1000), 940, Vector(0.6, 0.5, 0.1)), // right-wall
        std::make_shared<Sphere>(Vector(0, 1000, 0), 940, Vector(0.2, 0.5, 0.9)),  // ceiling
        std::make_shared<Sphere>(Vector(-1000, 0, 0), 940, Vector(0.4, 0.8, 0.7)), // back-wall
        std::make_shared<Sphere>(Vector(1000, 0, 0), 940, Vector(0.9, 0.4, 0.3)),  // wall-behind-camera
        std::make_shared<Sphere>(Vector(0, 0, 0), 10, Vector(0.8, 0.8, 0.8)), // ball in the middle
        // std::make_shared<Sphere>(Vector(-18, -2, 0), 8, Vector(0.8, 0.8, 0.8), true), // mirror
        // std::make_shared<Sphere>(Vector(18, -2, 0), 8, Vector(0.8, 0.8, 0.8), false, true, 1.5), // transparent
        // std::make_shared<Sphere>(Vector(0, -2, 0), 8, Vector(0, 0, 0), false, true, 1.5),       // hollow-transp 1
        // std::make_shared<Sphere>(Vector(0, -2, 0), 8 - EPSILON, Vector(0, 0, 0), false, true, 1), // hollow-transp 2
        //std::make_shared<Sphere>(Vector(0,0,0), 10, Vector(0.8,0.8,0.8)), //trial
    };

    std::vector<std::shared_ptr<LightSource>> standard_scene_lights = {
        std::make_shared<LightSource>(Vector(-10, 20, 40), 2, Vector(1., 1., 1.), 3e10),
        // std::make_shared<LightSource>(Vector(-10, 25, -10), 5, Vector(1., 1., 1.), 3e10),
        //LightSource(Vector(20, 25, -10), 1, Vector(1, 1, 1), 1e10),
    };

    Scene scene(objects, standard_scene_lights);
    std::vector<unsigned char> image(W * H * 3, 0);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < H; i++) {
        std::default_random_engine engine;
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
                pixelColor = pixelColor + scene.getColor(ray, engine, false);
            }
            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(pixelColor[0]/NB_PATHS, 1/2.2));
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(pixelColor[1]/NB_PATHS, 1/2.2));
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(pixelColor[2]/NB_PATHS, 1/2.2));
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    std::cout << "Elapsed time: " << duration.count() << " seconds" << std::endl;
    
    stbi_write_png("cat.png", W, H, 3, &image[0], 0);
 
    return 0;
}
