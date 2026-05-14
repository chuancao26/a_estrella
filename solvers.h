#ifndef SOLVERS_H
#define SOLVERS_H
#include <queue>
#include <stack>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include "grid.h"

enum SearchMode { ASTAR, DIJKSTRA, BFS, DFS };

struct SearchResult {
    std::vector<int> path;
    std::vector<int> explored;
    float cost = 0.0f;
    bool found = false;
    SearchMode modeUsed;
};

class PathFinder {
public:
    SearchResult solve(const GridMap& map, int start, int end, SearchMode mode) {
        int N = map.cols * map.rows;
        std::vector<float> g(N, 1e9);
        std::vector<int> parent(N, -1);
        std::vector<bool> closed(N, false);
        SearchResult res;
        res.modeUsed = mode;

        auto h = [&](int a, int b) {
            if (mode != ASTAR) return 0.0f; 
            int dx = abs(map.toCol(a) - map.toCol(b)), dy = abs(map.toRow(a) - map.toRow(b));
            return (dx + dy) + (1.4142f - 2.0f) * std::min(dx, dy);
        };

        std::priority_queue<std::pair<float, int>, std::vector<std::pair<float, int>>, std::greater<>> openPQ;
        std::queue<int> openQ;
        std::stack<int> openS;

        g[start] = 0;
        if (mode == ASTAR || mode == DIJKSTRA) openPQ.push({ h(start, end), start });
        else if (mode == BFS) openQ.push(start);
        else openS.push(start);

        int dc[8] = {1, 0, -1, 0, 1, 1, -1, -1}, dr[8] = {0, 1, 0, -1, 1, -1, 1, -1};
        float stepCosts[8] = {1, 1, 1, 1, 1.4142f, 1.4142f, 1.4142f, 1.4142f};

        while (true) {
            int curr;
            if (mode == ASTAR || mode == DIJKSTRA) {
                if (openPQ.empty()) break;
                curr = openPQ.top().second; openPQ.pop();
            } else if (mode == BFS) {
                if (openQ.empty()) break;
                curr = openQ.front(); openQ.pop();
            } else {
                if (openS.empty()) break;
                curr = openS.top(); openS.pop();
            }

            if (closed[curr]) continue;
            closed[curr] = true;
            res.explored.push_back(curr);
            if (curr == end) { res.found = true; break; }

            for (int d = 0; d < 8; ++d) {
                int nc = map.toCol(curr) + dc[d], nr = map.toRow(curr) + dr[d];
                if (!map.isWalkable(nc, nr)) continue;
                int neighbor = map.toIdx(nc, nr);
                float newG = g[curr] + stepCosts[d];
                if (newG < g[neighbor]) {
                    g[neighbor] = newG;
                    parent[neighbor] = curr;
                    if (mode == ASTAR || mode == DIJKSTRA) openPQ.push({ newG + h(neighbor, end), neighbor });
                    else if (mode == BFS) openQ.push(neighbor);
                    else openS.push(neighbor);
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
#endif