// bool intersect(const Ray& r) {
            
//     double delta = dot(r.u, r.O - center)*dot(r.u, r.O - center) - ((r.O - center).norm2() - R*R); // the discriminant

//     // if delta < 0, there is no intersection
//     if (delta < 0) {
//         return false;
//     }

//     double x = dot(r.u, center - r.O);
//     double t1 = x - sqrt(delta);
//     double t2 = x + sqrt(delta);

//     if (t2 < 0) {
//         return false;
//     }

//     return true;
// }

#include <type_traits>
#include <cmath>

template <typename T>
constexpr T epsilon = std::is_floating_point_v<T> ? T(1e-6) : T(0);

enum class Comparison {
    Equal,
    Less,
    Greater,
    LessEqual,
    GreaterEqual
};

template <typename T1, typename T2>
bool compare(T1 x, T2 y, Comparison comp) {
    using CommonType = std::common_type_t<T1, T2>;
    CommonType a = static_cast<CommonType>(x);
    CommonType b = static_cast<CommonType>(y);

    if (std::is_integral_v<T1> && std::is_integral_v<T2>) {
        switch (comp) {
            case Comparison::Equal:         return a == b;
            case Comparison::Less:          return a < b;
            case Comparison::Greater:       return a > b;
            case Comparison::LessEqual:     return a <= b;
            case Comparison::GreaterEqual:  return a >= b;
        }
    } else {
        switch (comp) {
            case Comparison::Equal:
                return std::abs(a - b) < epsilon<CommonType>;
            case Comparison::Less:
                return a < b + epsilon<CommonType>;
            case Comparison::Greater:
                return a > b - epsilon<CommonType>;
            case Comparison::LessEqual:
                return a < b + epsilon<CommonType> || std::abs(a - b) < epsilon<CommonType>;
            case Comparison::GreaterEqual:
                return a > b - epsilon<CommonType> || std::abs(a - b) < epsilon<CommonType>;
        }
    }
    throw; // Should never reach here
}


Intersection findIntersection (const Ray& ray) const override {

    double closest_t = std::numeric_limits<double>::max();
    Vector closest_P, closest_N = Vector(0, 0, 0);

    for (const TriangleIndices& triangle : indices){
        Vector A = vertices[triangle.vtxi];
        Vector B = vertices[triangle.vtxj];
        Vector C = vertices[triangle.vtxk];
        Vector e1 = B - A;
        Vector e2 = C - A;
        Vector N = cross(e1, e2);
        
        double uN_dot = dot(ray.u, N);
        if (abs(uN_dot)<EPSILON) continue;

        Vector AO = A - ray.O;
        double t = dot(AO, N)/uN_dot;
        if (t<EPSILON || t>closest_t) continue;

        double beta = dot(e2, cross(AO, ray.u)) / uN_dot;
        double gamma =  dot(e1, cross(AO, ray.u)) / uN_dot;
        double alpha = 1 - beta - gamma;

        if (!((alpha >= 0.0f && alpha <= 1.0f) && (beta >= 0.0f && beta <= 1.0f) && (gamma >= 0.0f && gamma <= 1.0f))){
            continue;
        }

        closest_t = t;
        closest_P = A + beta*e1 + gamma*e2;
        closest_N = N;
    }

    if (closest_t == std::numeric_limits<double>::max()) return Intersection(false);

    return Intersection(true, false, closest_t, closest_P, closest_N);

    
}

// class TriangleIndices {
//     public:
//         TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : V(vtxi, vtxj, vtxk), UV(uvi, uvj, uvk), N(ni, nj, nk), group(group) {
//         };
//         TriangleIndices(const Vector V, const Vector UV, const Vector N, int group = -1) :  V(V), UV(UV), N(N), group(group) {};
//         Vector V; // vertex coordinates
//         Vector UV;  // uv coordinates
//         Vector N;  // normal
//         int group;  // face group
// };

// struct AABB {
//     Vector min, max;

//     bool intersect(const Ray& ray, double& t_min, double& t_max) const {
//         t_min = 0;
//         t_max = std::numeric_limits<double>::max();
//         for (int i = 0; i < 3; i++) {
//             double invD = 1.0 / ray.u[i];
//             double t0 = (min[i] - ray.O[i]) * invD;
//             double t1 = (max[i] - ray.O[i]) * invD;
//             if (invD < 0.0) std::swap(t0, t1);
//             t_min = std::max(t_min, t0);
//             t_max = std::min(t_max, t1);
//             if (t_max <= t_min) return false;
//         }
//         return true;
//     }
// };

// struct BVHNode {
//     AABB box;
//     BVHNode* left = nullptr;
//     BVHNode* right = nullptr;
//     std::vector<int> triangleIndices;

//     bool isLeaf() const {
//         return left == nullptr && right == nullptr;
//     }
// };
