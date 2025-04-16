#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
 
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>

#include "Vector.hpp"
#include "Geometry.hpp"
#include "Scene.hpp"

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
    };

    std::vector<std::shared_ptr<LightSource>> standard_scene_lights = {
        std::make_shared<PointLight>(Vector(-10, 20, 40), 3e10),
        //std::make_shared<SphereLight>(Vector(-10, 20, 40), 2, Vector(1., 1., 1.), 3e10),
    };

    Scene scene(objects, standard_scene_lights);
    scene.includeIndirectLight = false;
    std::vector<unsigned char> image(W * H * 3, 0);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < H; i++) {
        std::default_random_engine engine(i);
        for (int j = 0; j < W; j++) {
            double z = -W / (2. * tan(fov/2));
            Vector r_dir(j-W/2+0.5, H/2-i+0.5, z);
            r_dir.normalize();
            
            Ray ray(camera_origin, r_dir);
            Vector pixelColor = scene.getColor(ray, engine, false);
            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(pixelColor[0], 1/2.2));
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(pixelColor[1], 1/2.2));
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(pixelColor[2], 1/2.2));
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    std::cout << "Elapsed time: " << duration.count() << " seconds" << std::endl;
    
    stbi_write_png("output_images/example1.png", W, H, 3, &image[0], 0);
 
    return 0;
}

int example2() {
    int nb_paths = 32;
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
        std::make_shared<Sphere>(Vector(-18, -2, 0), 8, Vector(0.8, 0.8, 0.8), true), // mirror
        std::make_shared<Sphere>(Vector(18, -2, 0), 8, Vector(0.8, 0.8, 0.8), false, true, 1.5), // transparent
        std::make_shared<Sphere>(Vector(0, -2, 0), 8, Vector(0, 0, 0), false, true, 1.5),       // hollow-transp 1
        std::make_shared<Sphere>(Vector(0, -2, 0), 8 - EPSILON, Vector(0, 0, 0), false, true, 1), // hollow-transp 2
    };

    std::vector<std::shared_ptr<LightSource>> standard_scene_lights = {
        std::make_shared<PointLight>(Vector(-10, 20, 40), 3e10),
    };

    Scene scene(objects, standard_scene_lights);
    std::vector<unsigned char> image(W * H * 3, 0);
    
    auto start_time = std::chrono::high_resolution_clock::now();
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
            for (int k = 0; k < nb_paths; k++) {
                Vector sample = random_cos(engine, r_dir);
                sample.normalize();
                Vector random_dir = r_dir+sample*stdev;
                random_dir.normalize();
                Ray ray(camera_origin, random_dir);
                pixelColor = pixelColor + scene.getColor(ray, engine, false);
            }
            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(pixelColor[0]/nb_paths, 1/2.2));
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(pixelColor[1]/nb_paths, 1/2.2));
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(pixelColor[2]/nb_paths, 1/2.2));
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    std::cout << "Elapsed time: " << duration.count() << " seconds" << std::endl;
    
    stbi_write_png("output_images/example2.png", W, H, 3, &image[0], 0);
 
    return 0;
}

int example3() {
    int nb_paths = 32;

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
    };

    std::vector<std::shared_ptr<LightSource>> standard_scene_lights = {
        std::make_shared<SphereLight>(Vector(-20, 30, -40), 5, Vector(1., 1., 1.), 1e10),
    };

    Scene scene(objects, standard_scene_lights);
    std::vector<unsigned char> image(W * H * 3, 0);
    
    auto start_time = std::chrono::high_resolution_clock::now();
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
            for (int k = 0; k < nb_paths; k++) {
                Vector sample = random_cos(engine, r_dir);
                sample.normalize();
                Vector random_dir = r_dir+sample*stdev;
                random_dir.normalize();
                Ray ray(camera_origin, random_dir);
                pixelColor = pixelColor + scene.getColor(ray, engine, false);
            }
            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(pixelColor[0]/nb_paths, 1/2.2));
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(pixelColor[1]/nb_paths, 1/2.2));
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(pixelColor[2]/nb_paths, 1/2.2));
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    std::cout << "Elapsed time: " << duration.count() << " seconds" << std::endl;
    
    stbi_write_png("output_images/example3.png", W, H, 3, &image[0], 0);
 
    return 0;
}

int example4() {
    int nb_paths = 32;

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
    };

    std::vector<std::shared_ptr<LightSource>> standard_scene_lights = {
        std::make_shared<SphereLight>(Vector(-20, 30, -40), 5, Vector(1., 1., 1.), 1e10),
        std::make_shared<SphereLight>(Vector(20, 30, -40), 5, Vector(1., 1., 1.), 1e10),
    };

    Scene scene(objects, standard_scene_lights);
    std::vector<unsigned char> image(W * H * 3, 0);
    
    auto start_time = std::chrono::high_resolution_clock::now();
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
            for (int k = 0; k < nb_paths; k++) {
                Vector sample = random_cos(engine, r_dir);
                sample.normalize();
                Vector random_dir = r_dir+sample*stdev;
                random_dir.normalize();
                Ray ray(camera_origin, random_dir);
                pixelColor = pixelColor + scene.getColor(ray, engine, false);
            }
            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(pixelColor[0]/nb_paths, 1/2.2));
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(pixelColor[1]/nb_paths, 1/2.2));
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(pixelColor[2]/nb_paths, 1/2.2));
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    std::cout << "Elapsed time: " << duration.count() << " seconds" << std::endl;
    
    stbi_write_png("output_images/example4.png", W, H, 3, &image[0], 0);
 
    return 0;
}

int example_mesh(){
    int nb_paths = 2;

    int W = 512;
    int H = 512;
    Vector camera_origin(0, 0, 55);
    double fov = 60*M_PI/180;
    Vector albedo(0.5, 0.5, 0.5);

    TriangleMesh catMesh(Vector(1.,1.,1.));
    catMesh.readOBJ("cadnav.com_model/Models_F0202A090/cat.obj");
    catMesh.readOBJ("cadnav.com_model/Models_F0202A090/cat.obj");
    catMesh.applyTransform(Vector(0.6, 0.6, 0.6), Vector(0, -10, 0));
    // catMesh.bounding_box = catMesh.compute_bbox();

    catMesh.bounding_box_root = new NodeBVH();
    catMesh.ConstructBVH(catMesh.bounding_box_root, 0, catMesh.indices.size());

    std::vector<std::shared_ptr<Geometry>> objects = {
        std::make_shared<Sphere>(Vector(0, 0, 1000), 940, Vector(0.9, 0.2, 0.9)),  // left-wall
        std::make_shared<Sphere>(Vector(0, -1000, 0), 990, Vector(0.3, 0.4, 0.7)), // floor
        std::make_shared<Sphere>(Vector(0, 0, -1000), 940, Vector(0.6, 0.5, 0.1)), // right-wall
        std::make_shared<Sphere>(Vector(0, 1000, 0), 940, Vector(0.2, 0.5, 0.9)),  // ceiling
        std::make_shared<Sphere>(Vector(-1000, 0, 0), 940, Vector(0.4, 0.8, 0.7)), // back-wall
        std::make_shared<Sphere>(Vector(1000, 0, 0), 940, Vector(0.9, 0.4, 0.3)),  // wall-behind-camera
        std::make_shared<TriangleMesh>(catMesh),
    };

    std::vector<std::shared_ptr<LightSource>> standard_scene_lights = {
        std::make_shared<PointLight>(Vector(-10, 20, 40), 1e10),
        //std::make_shared<SphereLight>(Vector(-10, 20, 40), 2, Vector(1., 1., 1.), 3e10),
    };

    Scene scene(objects, standard_scene_lights);
    std::vector<unsigned char> image(W * H * 3, 0);
    
    auto start_time = std::chrono::high_resolution_clock::now();
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
            for (int k = 0; k < nb_paths; k++) {
                Vector sample = random_cos(engine, r_dir);
                sample.normalize();
                Vector random_dir = r_dir+sample*stdev;
                random_dir.normalize();
                Ray ray(camera_origin, random_dir);
                pixelColor = pixelColor + scene.getColor(ray, engine, false);
            }
            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(pixelColor[0]/nb_paths, 1/2.2));
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(pixelColor[1]/nb_paths, 1/2.2));
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(pixelColor[2]/nb_paths, 1/2.2));
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    std::cout << "Elapsed time: " << duration.count() << " seconds" << std::endl;
    
    stbi_write_png("output_images/example_mesh.png", W, H, 3, &image[0], 0);
 
    return 0;
}


int main() {
    // example1();
    // example2();
    // example3();
    // example4();
    example_mesh();
}
