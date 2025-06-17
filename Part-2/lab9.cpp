// // Tutte's Embedding Algorithm
// // I could not manage to implement the algorithm to check if a vertex is at the boundary

// std::vector<Vector> tutte(std::vector<Vector>& vertices){
//     std::vector<Vector> boundary_vertices = {}; // could not manage
//     double s = 0;
//     int n = boundary_vertices.size();
//     for (int i = 0; i < n; ++i){
//         int next_i = (i < n - 1) ? i+1 : 0;
//         s += (boundary_vertices[next_i]-boundary_vertices[i]).norm();
//     }
//     double cs = 0;
//     std::vector<Vector> res(vertices);
//     for (int i = 0; i<n; ++i){
//         int next_i = (i < n - 1) ? i+1 : 0;
//         double theta = 2*M_PI*cs/s;
//         cs += (boundary_vertices[next_i]-boundary_vertices[i]).norm();
//     }
//     for (int iter =0; i<vertices.size(); ++i){
//         if (true){ //interior

//         } else{ 

//         }
//     }

// }