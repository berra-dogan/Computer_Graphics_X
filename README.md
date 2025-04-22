# Ray Tracer — Computer Graphics Assignment 1

Ray tracer implemented in C++. The project supports:
- Diffuse and mirror surfaces
- (extra) Transparent and hollow spheres
- Point and (extra) spherical light sources
- (extra) Use of multiple light sources within the same scene
- Direct and indirect lighting
- Antialiasing
- Ray-mesh intersection with BVH
- (extra) Normals and textures for textures for triangle meshes

---

## Build Instructions

* Run the command "make SRC=assignment1.cpp"
* Run the command "./assignment1 image\_num" with the example from ./assignment1 that you would like to reconstruct

## Project Structure

CG_code_env/
├── headers/              # Scene, Geometry, Light, Ray classes
├── output_images/               # Saved rendered images
├── assignment1.cpp       # Main file with exampleX() functions
├── Makefile              # Build configuration
└── README.md             # This file


## Author

Berra Dogan  
Ecole Polytechnique  
Email: berra.dogan@polytechnique.edu  
