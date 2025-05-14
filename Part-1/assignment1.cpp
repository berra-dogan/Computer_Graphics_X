#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
 
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>
#include <cstdlib>

#include "Vector.hpp"
#include "Geometry.hpp"
#include "Scene.hpp"

std::vector<std::shared_ptr<Geometry>> background = {
    std::make_shared<Sphere>(Vector(0, 0, 1000), 940, Vector(0.9, 0.2, 0.9)),  // left-wall
    std::make_shared<Sphere>(Vector(0, -1000, 0), 990, Vector(0.3, 0.4, 0.7)), // floor
    std::make_shared<Sphere>(Vector(0, 0, -1000), 940, Vector(0.6, 0.5, 0.1)), // right-wall
    std::make_shared<Sphere>(Vector(0, 1000, 0), 940, Vector(0.2, 0.5, 0.9)),  // ceiling
    std::make_shared<Sphere>(Vector(-1000, 0, 0), 940, Vector(0.4, 0.8, 0.7)), // back-wall
    std::make_shared<Sphere>(Vector(1000, 0, 0), 940, Vector(0.9, 0.4, 0.3)),  // wall-behind-camera
};

int example_struct(int W, int H, Scene scene, char const * file_name, bool include_antialiasing, int nb_paths = 1, int ray_depth = 5){

    Vector camera_origin(0, 0, 55);
    double fov = 60*M_PI/180;
    
    std::vector<unsigned char> image(W * H * 3, 0);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < H; i++) {
        std::default_random_engine engine(i);
        for (int j = 0; j < W; j++) {
            double z = -W / (2. * tan(fov/2));
            Vector r_dir(j-W/2+0.5, H/2-i+0.5, z);
            r_dir.normalize();
            
            Vector pixelColor(0., 0., 0.);
            if (include_antialiasing){
                double stdev = 0.001;
                for (int k = 0; k < nb_paths; k++) {
                    Vector sample = random_cos(engine, r_dir);
                    sample.normalize();
                    Vector random_dir = r_dir+sample*stdev;
                    random_dir.normalize();
                    Ray ray(camera_origin, random_dir);
                    pixelColor = pixelColor + scene.getColor(ray, engine, false, ray_depth);
                }
            } else {
                Ray ray(camera_origin, r_dir);
                pixelColor = scene.getColor(ray, engine, false, ray_depth);
            }
            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(pixelColor[0]/nb_paths, 1/2.2));
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(pixelColor[1]/nb_paths, 1/2.2));
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(pixelColor[2]/nb_paths, 1/2.2));
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    std::cout << "Elapsed time: " << duration.count() << " seconds" << std::endl;
    
    stbi_write_png(file_name, W, H, 3, &image[0], 0);
 
    return 0;

}

int example1() {
    int W = 512;
    int H = 512;

    std::vector<std::shared_ptr<Geometry>> objects = background;
    objects.push_back(std::make_shared<Sphere>(Vector(0, 0, 0), 10, Vector(0.8, 0.8, 0.8))); // ball in the middle

    std::vector<std::shared_ptr<LightSource>> standard_scene_lights = {
        std::make_shared<PointLight>(Vector(-10, 20, 40), 3e10),
    };

    Scene scene(objects, standard_scene_lights);
    scene.includeIndirectLight = false;
    
    return example_struct(W, H, scene, "output_images/example1.png", false);
}

int example2() {
    int nb_paths = 32;
    int W = 512;
    int H = 512;

    std::vector<std::shared_ptr<Geometry>> objects = background;
    std::vector<std::shared_ptr<Geometry>> more_objects = {
        std::make_shared<Sphere>(Vector(-18, -2, 0), 8, Vector(0.8, 0.8, 0.8), true), // mirror
        std::make_shared<Sphere>(Vector(18, -2, 0), 8, Vector(0.8, 0.8, 0.8), false, true, 1.5), // transparent
        std::make_shared<Sphere>(Vector(0, -2, 0), 8, Vector(0, 0, 0), false, true, 1.5),       // hollow-transp 1
        std::make_shared<Sphere>(Vector(0, -2, 0), 8 - EPSILON, Vector(0, 0, 0), false, true, 1), // hollow-transp 2
    };
    objects.insert(objects.end(), more_objects.begin(), more_objects.end());

    std::vector<std::shared_ptr<LightSource>> standard_scene_lights = {
        std::make_shared<PointLight>(Vector(-10, 20, 40), 3e10),
    };

    Scene scene(objects, standard_scene_lights);
    return example_struct(W, H, scene, "output_images/example2.png", true, nb_paths, 10);

}

int example3() {
    int nb_paths = 32;
    int W = 512;
    int H = 512;

    std::vector<std::shared_ptr<Geometry>> objects = background;
    objects.push_back(std::make_shared<Sphere>(Vector(0, 0, 0), 10, Vector(0.8, 0.8, 0.8))); // ball in the middle
    
    std::vector<std::shared_ptr<LightSource>> standard_scene_lights = {
        std::make_shared<SphereLight>(Vector(-20, 30, -40), 5, Vector(1., 1., 1.), 1e10),
    };

    Scene scene(objects, standard_scene_lights);
    
    return example_struct(W, H, scene, "output_images/example3.png", true, nb_paths);
}

int example4() {
    int nb_paths = 32;
    int W = 512;
    int H = 512;

    std::vector<std::shared_ptr<Geometry>> objects = background;
    objects.push_back(std::make_shared<Sphere>(Vector(0, 0, 0), 10, Vector(0.8, 0.8, 0.8))); // ball in the middle

    std::vector<std::shared_ptr<LightSource>> standard_scene_lights = {
        std::make_shared<SphereLight>(Vector(-20, 30, -40), 5, Vector(1., 1., 1.), 1e10),
        std::make_shared<SphereLight>(Vector(20, 30, -40), 5, Vector(1., 1., 1.), 1e10),
    };

    Scene scene(objects, standard_scene_lights);

    return example_struct(W, H, scene, "output_images/example4.png", true, nb_paths);
}

int example5(){
    int nb_paths = 32;
    int W = 512;
    int H = 512;

    TriangleMesh catMesh(Vector(1.,1.,1.));
    catMesh.readOBJ("cadnav.com_model/Models_F0202A090/cat.obj");
    catMesh.load_texture("cadnav.com_model/Models_F0202A090/cat_diff.png");
    catMesh.applyTransform(Vector(0.6, 0.6, 0.6), Vector(0, -10, 0));
    catMesh.includeNormals = false;
    catMesh.includeTexture = false;
    // catMesh.bounding_box = catMesh.compute_bbox();

    catMesh.bounding_box_root = new NodeBVH();
    catMesh.ConstructBVH(catMesh.bounding_box_root, 0, catMesh.indices.size());

    std::vector<std::shared_ptr<Geometry>> objects = background;
    objects.push_back(std::make_shared<TriangleMesh>(catMesh));

    std::vector<std::shared_ptr<LightSource>> standard_scene_lights = {
        std::make_shared<PointLight>(Vector(-10, 20, 40), 1e10),
    };

    Scene scene(objects, standard_scene_lights);
    
    return example_struct(W, H, scene, "output_images/example5.png", true, nb_paths);
}

int example6(){
    int nb_paths = 32;
    int W = 512;
    int H = 512;

    TriangleMesh catMesh(Vector(1.,1.,1.));
    catMesh.readOBJ("cadnav.com_model/Models_F0202A090/cat.obj");
    catMesh.load_texture("cadnav.com_model/Models_F0202A090/cat_diff.png");
    catMesh.applyTransform(Vector(0.6, 0.6, 0.6), Vector(0, -10, 0));
    catMesh.includeTexture = false;

    catMesh.bounding_box_root = new NodeBVH();
    catMesh.ConstructBVH(catMesh.bounding_box_root, 0, catMesh.indices.size());

    std::vector<std::shared_ptr<Geometry>> objects = background;
    objects.push_back(std::make_shared<TriangleMesh>(catMesh));

    std::vector<std::shared_ptr<LightSource>> standard_scene_lights = {
        std::make_shared<PointLight>(Vector(-10, 20, 40), 1e10),
    };

    Scene scene(objects, standard_scene_lights);

    return example_struct(W, H, scene, "output_images/example6.png", true, nb_paths);
}

int example7(){
    int nb_paths = 32;
    int W = 512;
    int H = 512;

    TriangleMesh catMesh(Vector(1.,1.,1.));
    catMesh.readOBJ("cadnav.com_model/Models_F0202A090/cat.obj");
    catMesh.load_texture("cadnav.com_model/Models_F0202A090/cat_diff.png");
    catMesh.applyTransform(Vector(0.6, 0.6, 0.6), Vector(0, -10, 0));
    // catMesh.bounding_box = catMesh.compute_bbox();

    catMesh.bounding_box_root = new NodeBVH();
    catMesh.ConstructBVH(catMesh.bounding_box_root, 0, catMesh.indices.size());

    std::vector<std::shared_ptr<Geometry>> objects = background;
    objects.push_back(std::make_shared<TriangleMesh>(catMesh));

    std::vector<std::shared_ptr<LightSource>> standard_scene_lights = {
        std::make_shared<PointLight>(Vector(-10, 20, 40), 1e10),
    };

    Scene scene(objects, standard_scene_lights);
    return example_struct(W, H, scene, "output_images/example7.png", true, nb_paths);
}


// int main() {
//     example1();
//     example2();
//     example3();
//     example4();
//     example5();
//     example6();
//     example7();
// }

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Wrong number of arguments";
        std::cerr << "The intended usage: " << argv[0] << " <example_number>\n";
        return 1;
    }

    int n = std::atoi(argv[1]);

    switch (n) {
        case 1: example1(); break;
        case 2: example2(); break;
        case 3: example3(); break;
        case 4: example4(); break;
        case 5: example5(); break;
        case 6: example6(); break;
        case 7: example7(); break;
        default:
            std::cerr << "Invalid example number. Please input a number from 1 to 7.\n";
            return 1;
    }

    return 0;
}
