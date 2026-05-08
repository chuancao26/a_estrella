#include <GLFW/glfw3.h>
#include <iostream>
#include <iomanip>
#include "grid.h"
#include "astar.h"

class App {
private:
    GridMap map;
    AStar solver;
    AStarResult lastResult;
    int startNode = -1, endNode = -1;
    const float padding = 20.0f;

public:
    App() : map(100, 100) { map.randomize(0.30f); }

    void handleMouse(int c, int r) {
        if (!map.isWalkable(c, r)) return;
        int n = map.toIdx(c, r);

        if (startNode == -1 || (startNode != -1 && endNode != -1)) {
            startNode = n; endNode = -1; lastResult = {};
        } else {
            endNode = n;
            lastResult = solver.solve(map, startNode, endNode);
            printReport();
        }
    }

    void printReport() {
        std::cout << "\n--- Resultados A* ---\n"
                  << "Costo: " << std::fixed << std::setprecision(2) << lastResult.cost << "\n"
                  << "Explorados: " << lastResult.explored.size() << "\n"
                  << "Nodos Camino: " << lastResult.path.size() << "\n";
    }

    void drawCircle(float x, float y, float r, int segs = 6) {
        glBegin(GL_TRIANGLE_FAN);
        glVertex2f(x, y);
        for (int i = 0; i <= segs; ++i) {
            float a = 2.0f * 3.14159f * i / segs;
            glVertex2f(x + cos(a) * r, y + sin(a) * r);
        }
        glEnd();
    }

    void render(int w, int h) {
        glClearColor(0.05f, 0.05f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        float sX = (w - 2 * padding) / (map.cols - 1);
        float sY = (h - 2 * padding) / (map.rows - 1);

        // Nodos Base
        glColor3f(0.15f, 0.15f, 0.25f);
        for (int i = 0; i < map.cols * map.rows; ++i) {
            if (map.active[i]) drawCircle(padding + map.toCol(i) * sX, padding + map.toRow(i) * sY, sX * 0.4f);
        }

        // Explorados
        glColor3f(0.4f, 0.2f, 0.6f);
        for (int i : lastResult.explored) drawCircle(padding + map.toCol(i) * sX, padding + map.toRow(i) * sY, sX * 0.45f);

        // Camino
        glColor3f(0.0f, 1.0f, 0.5f);
        for (int i : lastResult.path) drawCircle(padding + map.toCol(i) * sX, padding + map.toRow(i) * sY, sX * 0.6f, 8);

        // Inicio/Fin
        if (startNode != -1) { glColor3f(0, 0.6f, 1); drawCircle(padding + map.toCol(startNode) * sX, padding + map.toRow(startNode) * sY, sX * 0.8f, 12); }
        if (endNode != -1) { glColor3f(1, 0.2f, 0.2f); drawCircle(padding + map.toCol(endNode) * sX, padding + map.toRow(endNode) * sY, sX * 0.8f, 12); }
    }

    void resetMap() { map.randomize(0.30f); startNode = -1; endNode = -1; lastResult = {}; }
};

// Instancia global para los callbacks
App app;

void mouse_cb(GLFWwindow* win, int b, int a, int m) {
    if (b == GLFW_MOUSE_BUTTON_LEFT && a == GLFW_PRESS) {
        double x, y; glfwGetCursorPos(win, &x, &y);
        int w, h; glfwGetFramebufferSize(win, &w, &h);
        int c = (int)round((x - 20.0f) / ((w - 40.0f) / 99));
        int r = (int)round((y - 20.0f) / ((h - 40.0f) / 99));
        app.handleMouse(c, r);
    }
}

int main() {
    if (!glfwInit()) return -1;
    GLFWwindow* win = glfwCreateWindow(800, 800, "Algoritmo A*", NULL, NULL);
    glfwMakeContextCurrent(win);
    glfwSetMouseButtonCallback(win, mouse_cb);
    glfwSetKeyCallback(win, [](GLFWwindow* w, int k, int s, int a, int m) {
        if (k == GLFW_KEY_R && a == GLFW_PRESS) app.resetMap();
    });

    while (!glfwWindowShouldClose(win)) {
        int w, h; glfwGetFramebufferSize(win, &w, &h);
        glViewport(0, 0, w, h);
        glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, w, h, 0, -1, 1);
        app.render(w, h);
        glfwSwapBuffers(win);
        glfwPollEvents();
    }
    glfwTerminate();
    return 0;
}