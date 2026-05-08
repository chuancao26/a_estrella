#ifndef ASTAR_H
#define ASTAR_H
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include "grid.h"

struct AStarResult {
    std::vector<int> path;
    std::vector<int> explored;
    float cost = 0.0f;
    bool found = false;
};

class AStar {
public:
    AStarResult solve(const GridMap& map, int start, int end) {
        int N = map.cols * map.rows;
        std::vector<float> g(N, 1e9);
        std::vector<int> parent(N, -1);
        std::vector<bool> closed(N, false);
        AStarResult res;

        auto h = [&](int a, int b) {
            int dx = abs(map.toCol(a) - map.toCol(b)), dy = abs(map.toRow(a) - map.toRow(b));
            return (dx + dy) + (1.4142f - 2.0f) * std::min(dx, dy);
        };

        std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> open;
        g[start] = 0;
        open.push({ h(start, end), start });

        int dc[8] = {1, 0, -1, 0, 1, 1, -1, -1}, dr[8] = {0, 1, 0, -1, 1, -1, 1, -1};
        float stepCosts[8] = {1, 1, 1, 1, 1.4142f, 1.4142f, 1.4142f, 1.4142f};

        while (!open.empty()) {
            int curr = open.top().second; open.pop();
            if (closed[curr]) continue;
            closed[curr] = true;
            res.explored.push_back(curr);
            if (curr == end) { res.found = true; break; }

            for (int d = 0; d < 8; ++d) {
                int nc = map.toCol(curr) + dc[d], nr = map.toRow(curr) + dr[d];
                if (!map.isWalkable(nc, nr)) continue;
                if (d >= 4 && (!map.isWalkable(map.toCol(curr) + dc[d], map.toRow(curr)) || 
                               !map.isWalkable(map.toCol(curr), map.toRow(curr) + dr[d]))) continue;

                int neighbor = map.toIdx(nc, nr);
                float newG = g[curr] + stepCosts[d];
                if (newG < g[neighbor]) {
                    g[neighbor] = newG;// Nota: corregido a 'curr'
                    parent[neighbor] = curr;
                    open.push({ newG + h(neighbor, end), neighbor });
                }
            }
        }
        if (res.found) {
            for (int v = end; v != -1; v = parent[v]) res.path.push_back(v);
            res.cost = g[end];
        }
        return res;
    }
};

#endif // ASTAR_H
