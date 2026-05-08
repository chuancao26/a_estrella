#ifndef GRID_H
#define GRID_H
#include <vector>
#include <cstdlib>
#include <ctime>

class GridMap {
public:
    int cols, rows;
    std::vector<bool> active;

    GridMap(int c, int r) : cols(c), rows(r), active(c * r, true) {}

    void randomize(float chance) {
        srand((unsigned)time(nullptr));
        for (int i = 0; i < cols * rows; ++i) {
            active[i] = ((float)rand() / RAND_MAX) > chance;
        }
    }

    inline int toIdx(int c, int r) const { return r * cols + c; }
    inline int toCol(int i) const { return i % cols; }
    inline int toRow(int i) const { return i / cols; }

    bool isWalkable(int c, int r) const {
        return c >= 0 && c < cols && r >= 0 && r < rows && active[toIdx(c, r)];
    }
};

#endif
