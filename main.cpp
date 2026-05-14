#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <string>
#include "grid.h"
#include "solvers.h"

const int GRID_COLS = 50; 
const int GRID_ROWS = 50;

class App {
public:
    GridMap map;
    PathFinder finder;
    SearchResult result;
    SearchMode currentMode = ASTAR;
    int startNode = -1, endNode = -1;
    const float PAD = 30.0f;

    App() : map(GRID_COLS, GRID_ROWS) { map.randomize(0.35f); }

    void printInfo() {
        std::string modeStr[] = {"A*", "Dijkstra", "BFS (Anchura)", "DFS (Profundidad)"};
        std::cout << "\n> Busqueda: " << modeStr[currentMode] 
                  << " | Explorados: " << result.explored.size() 
                  << " | Costo: " << result.cost << std::endl;
    }

    void runSearch() {
        if (startNode != -1 && endNode != -1) {
            result = finder.solve(map, startNode, endNode, currentMode);
            printInfo();
        }
    }

    void render(int w, int h) {
        // Fondo ligeramente gris para que el negro de los muros resalte
        glClearColor(0.1f, 0.1f, 0.12f, 1.0f); 
        glClear(GL_COLOR_BUFFER_BIT);
        
        float availableW = (float)w - 2 * PAD;
        float availableH = (float)h - 2 * PAD;
        float sX = availableW / (map.cols - 1);
        float sY = availableH / (map.rows - 1);
        
        float pointSize = (availableW / map.cols) * 0.8f;
        if (pointSize < 1.0f) pointSize = 1.0f;

        // 1. Dibujar Grid (Muros negros y Nodos morado claro)
        glPointSize(pointSize * 0.6f); 
        glBegin(GL_POINTS);
        for(int i = 0; i < map.cols * map.rows; ++i) {
            if (!map.active[i]) {
                glColor3f(0.0f, 0.0f, 0.0f); // MUROS EN NEGRO
            } else {
                glColor3f(0.6f, 0.5f, 0.8f); // NODOS ACTIVOS EN MORADO CLARO
            }
            glVertex2f(PAD + map.toCol(i) * sX, PAD + map.toRow(i) * sY);
        }
        glEnd();

        // 2. Explorados (Morado más fuerte)
        glPointSize(pointSize * 0.8f);
        glBegin(GL_POINTS); 
        glColor3f(1.0f, 1.0f, 0.0f); // EXPLORADOS EN MORADO FUERTE
        for(int i : result.explored) {
            glVertex2f(PAD + map.toCol(i) * sX, PAD + map.toRow(i) * sY);
        }
        glEnd();

        // 3. Camino (Verde neón para que contraste)
        if(!result.path.empty()){
            glLineWidth(pointSize * 0.5f);
            glBegin(GL_LINE_STRIP); 
            glColor3f(0.0f, 1.0f, 0.5f); 
            for(int i : result.path) {
                glVertex2f(PAD + map.toCol(i) * sX, PAD + map.toRow(i) * sY);
            }
            glEnd();
        }

        // 4. Inicio (Azul) y Fin (Rojo)
        glPointSize(pointSize * 1.5f);
        glBegin(GL_POINTS);
        if(startNode != -1){ 
            glColor3f(0.0f, 0.7f, 1.0f); 
            glVertex2f(PAD + map.toCol(startNode) * sX, PAD + map.toRow(startNode) * sY); 
        }
        if(endNode != -1){ 
            glColor3f(1.0f, 0.2f, 0.2f); 
            glVertex2f(PAD + map.toCol(endNode) * sX, PAD + map.toRow(endNode) * sY); 
        }
        glEnd();
    }
};

App app;

void mouse_cb(GLFWwindow* win, int b, int a, int m) {
    if (b == GLFW_MOUSE_BUTTON_LEFT && a == GLFW_PRESS) {
        double x, y; glfwGetCursorPos(win, &x, &y);
        int w, h; glfwGetFramebufferSize(win, &w, &h);
        
        float sX = (w - 2 * app.PAD) / (app.map.cols - 1);
        float sY = (h - 2 * app.PAD) / (app.map.rows - 1);
        
        int c = (int)round((x - app.PAD) / sX);
        int r = (int)round((y - app.PAD) / sY);
        
        if (app.map.isWalkable(c, r)) {
            if (app.startNode == -1 || (app.startNode != -1 && app.endNode != -1)) {
                app.startNode = app.map.toIdx(c, r); app.endNode = -1; app.result = {};
            } else {
                app.endNode = app.map.toIdx(c, r); app.runSearch();
            }
        }
    }
}

void key_cb(GLFWwindow* win, int k, int s, int a, int m) {
    if (a != GLFW_PRESS) return;
    if (k == GLFW_KEY_R) { app.map.randomize(0.35f); app.startNode = app.endNode = -1; app.result = {}; }
    if (k == GLFW_KEY_1) { app.currentMode = ASTAR; app.runSearch(); }
    if (k == GLFW_KEY_2) { app.currentMode = DIJKSTRA; app.runSearch(); }
    if (k == GLFW_KEY_3) { app.currentMode = BFS; app.runSearch(); }
    if (k == GLFW_KEY_4) { app.currentMode = DFS; app.runSearch(); }
}

int main() {
    if (!glfwInit()) return -1;
    GLFWwindow* win = glfwCreateWindow(800, 800, "Algoritmos de Busqueda", NULL, NULL);
    glfwMakeContextCurrent(win);
    glfwSetMouseButtonCallback(win, mouse_cb);
    glfwSetKeyCallback(win, key_cb);

    std::cout << "1: A* | 2: Dijkstra | 3: BFS | 4: DFS | R: Nuevo Mapa" << std::endl;

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