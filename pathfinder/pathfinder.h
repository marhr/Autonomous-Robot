#pragma once

struct Point { double x, y; };

#define MAX_WAYPOINTS 200
#define MAX_TREE_NODES 500
#define NUM_OBSTACLES 5
#define NUM_TARGETS 4

struct PathResult {
    Point waypoints[MAX_WAYPOINTS];
    int count;
    bool success;
};

PathResult planPath(
    const int* x_targets,
    const int* y_targets,
    int numTargets
);