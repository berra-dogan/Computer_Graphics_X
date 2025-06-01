#include "../Part-1/headers/Vector.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "../Part-1/stb/stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../Part-1/stb/stb_image_write.h"

#include <vector>
#include <random>

#define ITERATIONS 1000

#define EPSILON 1E-6
static std::uniform_real_distribution<double> uniform(0, 1);

// Points are uniformly distributed on the entire sphere, not on the positive z hemisphere like boxMuller.
Vector random_direction(std::default_random_engine &engine) {
    double r1 = uniform (engine);
    double r2 = uniform (engine);
    double x = cos(2 * M_PI * r1) * sqrt(r2 * (1 - r2));
    double y = sin(2 * M_PI * r1) * sqrt(r2 * (1 - r2));
    double z = 1 - 2 * r2;
    return Vector(x, y, z);
}

void color_matching(unsigned char* I, unsigned char* M, int I_W, int I_H, int I_C, int M_C, int nbiter) {
    size_t n = I_W*I_H;
    std::vector<std::pair<int, int>> projI(n);
    std::vector<std::pair<int, int>> projM(n);
    Vector image_pixel, color_pixel, v;

    #pragma omp parallel for schedule(dynamic)
    for (int iter = 0; iter < nbiter; ++iter){
        std::default_random_engine engine(iter);
        Vector v = random_direction(engine);
        //Project
        for (int i = 0; i < n; ++i){
            int idx = i * I_C;

            Vector Ii(I[idx], I[idx + 1], I[idx + 2]);
            Vector Mi(M[idx], M[idx + 1], M[idx + 2]);

            projI[i] = {dot(Ii, v), (int)i};
            projM[i] = {dot(Mi, v), (int)i};
        }
        std::sort(projI.begin(), projI.end());
        std::sort(projM.begin(), projM.end());

        for (int i = 0; i < n; ++i){
            int idx = projI[i].second * I_C;

            double delta = projM[i].first - projI[i].first;
            int r = static_cast<int>(I[idx]     + delta * v[0]);
            int g = static_cast<int>(I[idx + 1] + delta * v[1]);
            int b = static_cast<int>(I[idx + 2] + delta * v[2]);

            I[idx]     = std::min(255, std::max(0, r));
            I[idx + 1] = std::min(255, std::max(0, g));
            I[idx + 2] = std::min(255, std::max(0, b));
        }

    }
}

int main(int argc, char **argv) {

    int num_iterations = 1000;
	int I_W, I_H, I_C;
	int M1_W, M1_H, M1_C;
    int M2_W, M2_H, M2_C;

	unsigned char *I1 = stbi_load("I.png", &I_W, &I_H, &I_C, 0);
    unsigned char *I2 = stbi_load("I.png", &I_W, &I_H, &I_C, 0);
	unsigned char *M1 = stbi_load("M1.png", &M1_W, &M1_H, &M1_C, 0);
    unsigned char *M2 = stbi_load("M2.png", &M2_W, &M2_H, &M2_C, 0);

    if (!I1 || !I2 || !M1 || !M2) {
        fprintf(stderr, "Failed to load images\n");
        return -1;
    }

	color_matching(I1, M1, I_W, I_H, I_C, M1_C, num_iterations);
	stbi_write_png("output1.png", I_W, I_H, I_C, I1, I_W * I_C);

    color_matching(I2, M2, I_W, I_H, I_C, M2_C, num_iterations);
	stbi_write_png("output2.png", I_W, I_H, I_C, I2, I_W * I_C);

    stbi_image_free(I1);
    stbi_image_free(I2);
    stbi_image_free(M1);
    stbi_image_free(M2);

	return 0;
}