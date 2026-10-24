#include "build/hull.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <fstream>
#include <map>
#include <random>
#include <stdexcept>

using namespace r2c;

void Check(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

void CheckHull(const std::vector<DVec3>& points, double volume) {
    Hull hull = ConvexHull(points);
    Check(hull.valid, "expected a valid hull");
    double actual = ComputeMassProps(hull).volume;
    if (std::fabs(actual - volume) > volume * 1e-5) {
        std::cerr << "volume: " << actual << ", expected: " << volume << '\n';
        Check(false, "incorrect collision volume");
    }
    std::map<std::pair<int, int>, int> edges;
    double extent = 0;
    for (auto& p : points) extent = std::max(extent, DLen(p - points[0]));
    for (auto& t : hull.tris) {
        auto a = hull.verts[t[0]], b = hull.verts[t[1]], c = hull.verts[t[2]];
        DVec3 n = DCross(b - a, c - a);
        Check(DLen(n) > extent * extent * 1e-12, "degenerate face");
        for (auto& p : points)
            Check(DDot(n, p - a) / DLen(n) <= extent * 1e-6, "point outside hull");
        for (int i = 0; i < 3; ++i) ++edges[{t[i], t[(i + 1) % 3]}];
    }
    for (auto& edge : edges) {
        Check(edge.second == 1, "duplicate directed edge");
        Check(edges.count({edge.first.second, edge.first.first}) == 1, "open hull edge");
    }
    Check(static_cast<int>(hull.verts.size() + hull.tris.size()) -
              static_cast<int>(edges.size() / 2) == 2, "non-spherical hull topology");
}

int main(int argc, char** argv) {
    try {
        std::mt19937 random(53);
        if (argc > 1) {
            // Points from the Blender SMD cylinder after float bone transforms
            // and PHY serialization. The old hull contained overlapping faces.
            std::ifstream fixture(argv[1]);
            std::vector<DVec3> points;
            DVec3 p;
            while (fixture >> p.x >> p.y >> p.z) points.push_back(p);
            Check(points.size() == 24, "missing cylinder regression fixture");
            for (int order = 0; order < 30; ++order) {
                std::shuffle(points.begin(), points.end(), random);
                CheckHull(points, 12288);
            }
        }
        // Blender's twelve-sided cylinder: area 192, height 64. Float transforms
        // of repeated SMD vertices introduce tiny differences at shared corners.
        for (double scale : {0.001, 1.0, 1000.0}) {
            std::vector<DVec3> cylinder;
            for (int i = 0; i < 12; ++i) {
                double angle = i * std::acos(-1.0) / 6;
                for (double z : {0.0, 64.0}) {
                    DVec3 p(8 * std::cos(angle), 8 * std::sin(angle), z);
                    for (int repeat = 0; repeat < 6; ++repeat) {
                        double jitter = (repeat - 2.5) * 1e-7;
                        cylinder.push_back((p + DVec3(jitter, -jitter, jitter)) * scale);
                    }
                }
            }
            for (int order = 0; order < 30; ++order) {
                std::shuffle(cylinder.begin(), cylinder.end(), random);
                CheckHull(cylinder, 12288 * scale * scale * scale);
            }
        }
        std::vector<DVec3> cube;
        for (double x : {-1.0, 1.0})
            for (double y : {-1.0, 1.0})
                for (double z : {-1.0, 1.0}) cube.emplace_back(x, y, z);
        for (int i = 0; i < 300; ++i) {
            auto coordinate = [&]() { return static_cast<double>(random() % 2001) / 1000 - 1; };
            cube.emplace_back(coordinate(), coordinate(), coordinate());
        }
        CheckHull(cube, 8);
        Check(!ConvexHull({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}).valid, "three-point hull");
        Check(!ConvexHull({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}}).valid, "planar hull");
        Check(!ConvexHull({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}).valid, "identical points");
        std::cout << "Collision hull regressions passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
