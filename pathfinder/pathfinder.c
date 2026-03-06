/* 
    Mariana Hernandez
    g++ pathfinder.c -o output
    ./output
*/


#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <fstream>

// Field dimensions in mm (96x36 inches)
const double FIELD_W = 2438.4;
const double FIELD_H = 914.4;

// Safety margin buffer in mm
const double BUFFER = 30.0;

// Max iterations for random sampling
const int MAX_SAMPLES = 5000;

// Targets (index 0 = start)
// const int x_targets[] = {1450, 200, 200, 650};
// const int y_targets[] = {150,  600, 150, 150};
const int x_targets[] = {120, 1430, 668, 200, 2120}; //estimated from south side
const int y_targets[] = { 800, 144, 163, 150,  320}; //estimated from south side
const int NUM_TARGETS = 5;

// --- Obstacle definitions (convex polygons) ---
struct Point { double x, y; };
struct Polygon { std::vector<Point> verts; };

// Obstacles 
Polygon obstacles[] = {
    // Obstacle A: 254x254, bottom-left at (300, 0)
    { {{300,0},{554,0},{554,254},{300,254}} },

    // Obstacle B: 209.6x209.6, bottom-left at (560, 610)
    { {{560,610},{770,610},{770,820},{560,820}} },

    // Obstacle C: 228.6x381, bottom-left at (1130, 380)
    { {{1130,380},{1511,380},{1511,608},{1130,608}} },

    // Obstacle D: 167x243, bottom-left at (850, 610)
    { {{850,610},{1017,610},{1017,853},{850,853}} },

    // Obstacle E: 244.3x243.3, bottom-left at (1830, 500)
    { {{1830,500},{2074,500},{2074,743},{1830,743}} },
};
const int NUM_OBSTACLES = 5;

// --- Geometry helpers ---

// Cross product of vectors (b-a) and (c-a)
double cross(Point a, Point b, Point c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// Check if point p is inside convex polygon (with buffer expansion)
bool pointInPolygon(const Point& p, const Polygon& poly, double buffer) {
    int n = poly.verts.size();
    for (int i = 0; i < n; i++) {
        Point a = poly.verts[i];
        Point b = poly.verts[(i + 1) % n];
        double dx = b.x - a.x, dy = b.y - a.y;
        double len = std::sqrt(dx*dx + dy*dy);
        // Inward normal distance from edge
        double dist = ((p.x - a.x) * dy - (p.y - a.y) * dx) / len;
        if (dist < -buffer) return false; //ccw check
        // if (dist > buffer) return false; // clock wise check
    }
    return true;
}

// Check if point is in obtacle
// specific for axis-aligned obstacles
bool pointInObstacle(const Point& p) {
    for (int o = 0; o < NUM_OBSTACLES; o++) {
        // Get bounding box from vertices
        double minX = obstacles[o].verts[0].x, maxX = minX;
        double minY = obstacles[o].verts[0].y, maxY = minY;
        for (auto& v : obstacles[o].verts) {
            minX = std::min(minX, v.x); maxX = std::max(maxX, v.x);
            minY = std::min(minY, v.y); maxY = std::max(maxY, v.y);
        }
        // Expand by buffer
        if (p.x > minX - BUFFER && p.x < maxX + BUFFER &&
            p.y > minY - BUFFER && p.y < maxY + BUFFER)
            return true;
    }
    return false;
}

// Check if segment (p1->p2) intersects any obstacle (with buffer)
bool segmentClear(const Point& p1, const Point& p2) {
    int steps = (int)(std::sqrt(std::pow(p2.x-p1.x,2) + std::pow(p2.y-p1.y,2)) / 5.0) + 1;
    for (int i = 0; i <= steps; i++) {
        double t = (double)i / steps;
        Point p = { p1.x + t*(p2.x-p1.x), p1.y + t*(p2.y-p1.y) };
        for (int o = 0; o < NUM_OBSTACLES; o++) {
            // if (pointInPolygon(p, obstacles[o], BUFFER)) return false;
            if (pointInObstacle(p))
            {
                return false;
            }
        }
        // Also check field bounds
        if (p.x < 0 || p.x > FIELD_W || p.y < 0 || p.y > FIELD_H) return false;
    }
    return true;
}

// --- RRT-style random sampling path finder ---
struct Node {
    Point pos;
    int parent;
};

Point randomPoint() {
    return { (double)(rand() % (int)FIELD_W), (double)(rand() % (int)FIELD_H) };
}

int nearestNode(const std::vector<Node>& tree, const Point& p) {
    int best = 0;
    double bestDist = std::numeric_limits<double>::max();
    for (int i = 0; i < (int)tree.size(); i++) {
        double d = std::pow(tree[i].pos.x - p.x, 2) + std::pow(tree[i].pos.y - p.y, 2);
        if (d < bestDist) { bestDist = d; best = i; }
    }
    return best;
}

Point steer(const Point& from, const Point& to, double stepSize = 50.0) {
    double dx = to.x - from.x, dy = to.y - from.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    if (dist < stepSize) return to;
    return { from.x + dx/dist * stepSize, from.y + dy/dist * stepSize };
}

// RRT from 'start' to 'goal', returns waypoints or empty if failed
std::vector<Point> rrtFind(const Point& start, const Point& goal) {
    std::vector<Node> tree = {{ start, -1 }};
    double goalThresh = 60.0;

    for (int iter = 0; iter < MAX_SAMPLES; iter++) {
        // Bias 10% of samples toward goal
        Point sample = (rand() % 10 == 0) ? goal : randomPoint();
        int nearIdx = nearestNode(tree, sample);
        Point newPt = steer(tree[nearIdx].pos, sample);

        if (segmentClear(tree[nearIdx].pos, newPt)) {
            tree.push_back({ newPt, nearIdx });
            int newIdx = tree.size() - 1;

            // Check if we reached the goal
            double dx = newPt.x - goal.x, dy = newPt.y - goal.y;
            if (std::sqrt(dx*dx + dy*dy) < goalThresh && segmentClear(newPt, goal)) {
                tree.push_back({ goal, newIdx });
                // Trace back path
                std::vector<Point> path;
                int cur = tree.size() - 1;
                while (cur != -1) {
                    path.push_back(tree[cur].pos);
                    cur = tree[cur].parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
        }
    }
    // Failed
    return {}; 
}

void writePlotHTML(const std::vector<Point>& path, const std::string& filename = "path_plot.html") {
    std::ofstream f(filename);

    f << R"(<!DOCTYPE html><html><head><title>Path Plot</title></head><body>
    <canvas id="c" width="800" height="300" style="border:1px solid #333;background:#1a1a2e;"></canvas>
    <script>
    const canvas = document.getElementById('c');
    const ctx = canvas.getContext('2d');

    const FW = 2438.4, FH = 914.4;
    const CW = canvas.width, CH = canvas.height;
    const sx = CW / FW, sy = CH / FH;
    const tx = x => x * sx;
    const ty = y => CH - y * sy; // flip Y

    // Obstacles
    const obstacles = [
                      // A: 254x254, BL=(300,0)
                      [[300,0],[554,0],[554,254],[300,254]],
                      // B: 209.6x209.6, BL=(560,610)
                      [[560,610],[770,610],[770,820],[560,820]],
                      // C: 228.6x381, BL=(1130,380)
                      [[1130,380],[1511,380],[1511,608],[1130,608]],
                      // D: 228.6x381, BL=(850,610)
                      [[850,610],[1079,610],[1079,914],[850,914]],
                      // E: 244.3x243.3, BL=(1830,500)
                      [[1830,500],[2074,500],[2074,743],[1830,743]],
                    ];

    ctx.fillStyle = 'rgba(255,80,80,0.4)';
    ctx.strokeStyle = 'rgba(255,80,80,0.9)';
    for (const obs of obstacles) {
        ctx.beginPath();
        ctx.moveTo(tx(obs[0][0]), ty(obs[0][1]));
        for (let i = 1; i < obs.length; i++) ctx.lineTo(tx(obs[i][0]), ty(obs[i][1]));
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
    }
    )";

    // pass path from C++
    f << "const path = [\n";
    for (auto& p : path)
        f << "  [" << p.x << "," << p.y << "],\n";
    f << "];\n";

    // pass targets
    f << "const targets = [\n";
    for (int i = 0; i < NUM_TARGETS; i++)
        f << "  [" << x_targets[i] << "," << y_targets[i] << "],\n";
    f << "];\n";

    f << R"(
    // Draw path
    ctx.strokeStyle = '#00ffcc';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(tx(path[0][0]), ty(path[0][1]));
    for (let i = 1; i < path.length; i++) ctx.lineTo(tx(path[i][0]), ty(path[i][1]));
    ctx.stroke();

    // Draw waypoints
    ctx.fillStyle = '#00ffcc';
    for (const p of path) {
        ctx.beginPath();
        ctx.arc(tx(p[0]), ty(p[1]), 2, 0, Math.PI*2);
        ctx.fill();
    }

    // Draw targets
    const colors = ['#ffff00','#ff9900','#ff9900','#ff9900'];
    const labels = ['START','T1','T2','T3','T4'];
    targets.forEach(([x,y], i) => {
        ctx.fillStyle = colors[i];
        ctx.beginPath();
        ctx.arc(tx(x), ty(y), 7, 0, Math.PI*2);
        ctx.fill();
        ctx.fillStyle = 'white';
        ctx.font = '11px monospace';
        ctx.fillText(labels[i], tx(x)+9, ty(y)+4);
    });

    </script></body></html>)";

    f.close();
    std::cout << "Plot saved to " << filename << "\n";
}

int main() {
    srand((unsigned)time(0));

    std::vector<Point> fullPath;

    for (int i = 0; i < NUM_TARGETS - 1; i++) {
        Point start = { (double)x_targets[i],   (double)y_targets[i] };
        Point goal  = { (double)x_targets[i+1], (double)y_targets[i+1] };

        std::cout << "Planning segment " << i << " -> " << i+1 << "...\n";
        std::vector<Point> seg = rrtFind(start, goal);

        if (seg.empty()) {
            std::cout << "  FAILED to find path for segment " << i << " -> " << i+1 << "\n";
            return 1;
        }

        std::cout << "  Found path with " << seg.size() << " waypoints\n";

        // Avoid duplicating junction points
        if (!fullPath.empty()) seg.erase(seg.begin());
        for (auto& p : seg) fullPath.push_back(p);
    }

    // std::cout << "\nFull path (" << fullPath.size() << " waypoints):\n";
    // for (auto& p : fullPath) {
    //     std::cout << "  (" << p.x << ", " << p.y << ")\n";
    // }

    // After path is found, print as Arduino-ready array
    std::cout << "\nconst int PATH_X[] = {";
    for (int i = 0; i < fullPath.size(); i++)
        std::cout << (int)fullPath[i].x << (i < fullPath.size()-1 ? "," : "");
    std::cout << "};\n";

    std::cout << "const int PATH_Y[] = {";
    for (int i = 0; i < fullPath.size(); i++)
        std::cout << (int)fullPath[i].y << (i < fullPath.size()-1 ? "," : "");
    std::cout << "};\n";

    std::cout << "const int PATH_LEN = " << fullPath.size() << ";\n";

    // Plot
    writePlotHTML(fullPath);

    return 0;
}
