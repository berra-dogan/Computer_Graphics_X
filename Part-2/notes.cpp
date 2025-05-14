// class Polygon {
// public:
//     Polygon() : vertices({}), normals({}) {}
//     Polygon(std::vector<Vector>& vertices) : vertices(vertices) {
//         set_normals();
//     }
//     void set_normals() {
//         normals.clear();
//         if (vertices.size()>2){
//             for (size_t i = 0; i < vertices.size(); ++i) {
//                 Vector A = vertices[(i>0)?(i-1):(vertices.size()-1)];
//                 Vector B = vertices[i];
//                 Vector C = vertices[(i + 1) % vertices.size()];
//                 Vector N = cross(A-B,C-B);
//                 normals.push_back(N);
//             }
//         }
//     }

//     void add_next_vertex(const Vector vertex, bool ignore_normals = true){
//         vertices.push_back(vertex);
//         if (!ignore_normals){
//             set_normals();
//         };
//     }

//     Polygon* clip(const Polygon& clipPolygon){
//         Polygon* outPolygon = new Polygon();
//         for (int j = 0; j < clipPolygon.vertices.size(); ++j){
//             Vector u = clipPolygon.vertices[j];
//             Vector N = clipPolygon.normals[j]; 

//             for (int i = 0; i < vertices.size(); ++i){
//                 Vector B = vertices[i];
//                 Vector A = vertices[(i>0)?(i-1):(vertices.size()-1)];
//                 Vector P =  A + dot(u - A, N)/dot(B - A, N) * (B - A);
//                 if (dot(u-B, N)>=0){
//                     if ((dot(u-A, N)<0)){
//                         outPolygon->add_next_vertex(P);
//                     }
//                     outPolygon->add_next_vertex(B);
//                 } else if (dot(u-A, N)>=0){
//                     outPolygon->add_next_vertex(P);
//                 }
//             }
//         }
//         outPolygon->set_normals();
//         return outPolygon;
//     }
//     std::vector<Vector> vertices; // ordering is important
//     std::vector<Vector> normals;
// };