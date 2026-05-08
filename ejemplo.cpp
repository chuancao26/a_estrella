/*#include <GLFW/glfw3.h>
#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <iostream> // Para imprimir en consola
#include <iomanip>  // Para dar formato a los decimales

// ─── Configuración ─────────────────────────────────────────
const int COLS = 100;
const int ROWS = 100;
const float PADDING = 20.0f;
const float DELETE_CHANCE = 0.30f;

bool active[COLS * ROWS];
int startNode = -1;
int endNode = -1;

std::vector<int> pathNodes;
std::vector<int> exploredNodes;

// ─── Utilidades ────────────────────────────────────────────
inline int idx(int c, int r) { return r * COLS + c; }
inline int col(int i) { return i % COLS; }
inline int row(int i) { return i / COLS; }
inline bool inBounds(int c, int r) { return c >= 0 && c < COLS && r >= 0 && r < ROWS; }

void randomizeMap() {
    srand((unsigned)time(nullptr));
    for (int i = 0; i < COLS * ROWS; i++) {
        active[i] = ((float)rand() / RAND_MAX) > DELETE_CHANCE;
    }
    startNode = -1;
    endNode = -1;
    pathNodes.clear();
    exploredNodes.clear();
}

// ─── Heurística (octil) ────────────────────────────────────
float heuristic(int a, int b) {
    int dx = abs(col(a) - col(b));
    int dy = abs(row(a) - row(b));
    return (dx + dy) + (1.4142f - 2.0f) * std::min(dx, dy);
}

// ─── A* ────────────────────────────────────────────────────
void runAstar() {
    pathNodes.clear();
    exploredNodes.clear();
    if (startNode < 0 || endNode < 0) return;

    active[startNode] = true;
    active[endNode] = true;

    int N = COLS * ROWS;
    std::vector<float> g(N, 1e9);
    std::vector<int> parent(N, -1);
    std::vector<bool> closed(N, false);

    using Node = std::pair<float, int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    g[startNode] = 0;
    open.push({heuristic(startNode, endNode), startNode});

    int dc[8] = {1, 0, -1, 0, 1, 1, -1, -1};
    int dr[8] = {0, 1, 0, -1, 1, -1, 1, -1};
    float costs[8] = {1.0f, 1.0f, 1.0f, 1.0f, 1.4142f, 1.4142f, 1.4142f, 1.4142f};

    while (!open.empty()) {
        auto [f, current] = open.top();
        open.pop();

        if (closed[current]) continue;
        closed[current] = true;
        exploredNodes.push_back(current);

        if (current == endNode) break;

        int c = col(current);
        int r = row(current);

        for (int d = 0; d < 8; d++) {
            int nc = c + dc[d];
            int nr = r + dr[d];

            if (!inBounds(nc, nr)) continue;
            int neighbor = idx(nc, nr);
            if (!active[neighbor] || closed[neighbor]) continue;

            if (d >= 4) { // Evitar atravesar esquinas
                if (!active[idx(c + dc[d], r)] || !active[idx(c, r + dr[d])]) continue;
            }

            float newG = g[current] + costs[d];
            if (newG < g[neighbor]) {
                g[neighbor] = newG;
                parent[neighbor] = current;
                open.push({newG + heuristic(neighbor, endNode), neighbor});
            }
        }
    }

    std::cout << "Punto Inicio: (" << col(startNode) << ", " << row(startNode) << ")" << std::endl;
    std::cout << "Punto Fin:    (" << col(endNode) << ", " << row(endNode) << ")" << std::endl;

    if (parent[endNode] != -1 || startNode == endNode) {
        int cur = endNode;
        while (cur != -1) {
            pathNodes.push_back(cur);
            cur = parent[cur];
        }
        std::reverse(pathNodes.begin(), pathNodes.end());

        std::cout << "Estado:        RUTA ENCONTRADA" << std::endl;
        std::cout << "Costo Total:   " << std::fixed << std::setprecision(2) << g[endNode] << " unidades" << std::endl;
        std::cout << "Nodos Camino:  " << pathNodes.size() << std::endl;
        std::cout << "Explorados:    " << exploredNodes.size() << std::endl;
        std::cout << "Optimizacion:  " << (1.0f - (float)pathNodes.size() / exploredNodes.size()) * 100.0f << "% de nodos descartados" << std::endl;
    } else {
        std::cout << "Estado:        SIN RUTA POSIBLE" << std::endl;
        std::cout << "Explorados:    " << exploredNodes.size() << " nodos revisados." << std::endl;
    }
    std::cout << "========================================\n" << std::endl;
}

// ─── Dibujo ────────────────────────────────────────────────
void drawCircle(float x, float y, float r, int segments = 10) {
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(x, y);
    for (int i = 0; i <= segments; i++) {
        float a = 2.0f * 3.14159f * i / segments;
        glVertex2f(x + cos(a) * r, y + sin(a) * r);
    }
    glEnd();
}

void render(int w, int h) {
    glClearColor(0.05f, 0.05f, 0.1f, 1.0f); // Fondo oscuro
    glClear(GL_COLOR_BUFFER_BIT);

    float areaW = w - 2 * PADDING;
    float areaH = h - 2 * PADDING;
    float stepX = areaW / (COLS - 1);
    float stepY = areaH / (ROWS - 1);
    float nodeRadius = stepX * 0.45f;

    // 1. Nodos Base (Activos)
    glColor3f(0.15f, 0.15f, 0.25f);
    for (int i = 0; i < COLS * ROWS; i++) {
        if (!active[i]) continue;
        drawCircle(PADDING + col(i) * stepX, PADDING + row(i) * stepY, nodeRadius, 6);
    }

    // 2. Nodos Explorados (Violeta suave)
    glColor3f(0.4f, 0.2f, 0.6f);
    for (int i : exploredNodes) {
        drawCircle(PADDING + col(i) * stepX, PADDING + row(i) * stepY, nodeRadius * 1.1f, 6);
    }

    // 3. Camino (Verde brillante)
    glColor3f(0.0f, 1.0f, 0.5f);
    for (int i : pathNodes) {
        drawCircle(PADDING + col(i) * stepX, PADDING + row(i) * stepY, nodeRadius * 1.3f, 8);
    }

    // 4. Inicio y Fin
    if (startNode >= 0) {
        glColor3f(0.0f, 0.6f, 1.0f); // Azul
        drawCircle(PADDING + col(startNode) * stepX, PADDING + row(startNode) * stepY, nodeRadius * 1.5f, 12);
    }
    if (endNode >= 0) {
        glColor3f(1.0f, 0.2f, 0.2f); // Rojo
        drawCircle(PADDING + col(endNode) * stepX, PADDING + row(endNode) * stepY, nodeRadius * 1.5f, 12);
    }
}

// ─── Callbacks ─────────────────────────────────────────────
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_R && action == GLFW_PRESS) randomizeMap();
}

void mouse_callback(GLFWwindow* win, int button, int action, int) {
    if (button != GLFW_MOUSE_BUTTON_LEFT || action != GLFW_PRESS) return;

    double mx, my;
    glfwGetCursorPos(win, &mx, &my);
    int w, h;
    glfwGetFramebufferSize(win, &w, &h);

    int c = (int)round((mx - PADDING) / ((w - 2 * PADDING) / (COLS - 1)));
    int r = (int)round((my - PADDING) / ((h - 2 * PADDING) / (ROWS - 1)));

    if (!inBounds(c, r) || !active[idx(c, r)]) return;
    int n = idx(c, r);

    if (startNode < 0) {
        startNode = n;
        std::cout << "Inicio fijado en (" << c << ", " << r << ")" << std::endl;
    } else if (endNode < 0) {
        endNode = n;
        std::cout << "Fin fijado en (" << c << ", " << r << ")" << std::endl;
        runAstar();
    } else {
        startNode = n;
        endNode = -1;
        pathNodes.clear();
        exploredNodes.clear();
        std::cout << "Reinicio. Nuevo inicio en (" << c << ", " << r << ")" << std::endl;
    }
}

int main() {
    if (!glfwInit()) return -1;

    GLFWwindow* win = glfwCreateWindow(800, 800, "Algoritmo A*", NULL, NULL);
    if (!win) { glfwTerminate(); return -1; }

    glfwMakeContextCurrent(win);
    glfwSetMouseButtonCallback(win, mouse_callback);
    glfwSetKeyCallback(win, key_callback);

    std::cout << "--- ALGORITMO A* ---" << std::endl;
    
    randomizeMap();

    while (!glfwWindowShouldClose(win)) {
        glfwPollEvents();
        int w, h;
        glfwGetFramebufferSize(win, &w, &h);
        glViewport(0, 0, w, h);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, w, h, 0, -1, 1);

        render(w, h);
        glfwSwapBuffers(win);
    }

    glfwTerminate();
    return 0;
}*/
/*
#include <GLFW/glfw3.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#include "grid.h"
#include "astar.h"

class TextRenderer {
    stbtt_bakedchar bakedData[96]; 
    GLuint fontTexture;
    bool loaded = false;
public:
    void init(const char* fontPath) {
        std::ifstream file(fontPath, std::ios::binary | std::ios::ate);
        if (!file.is_open()) return;
        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);
        std::vector<unsigned char> buffer(size);
        file.read((char*)buffer.data(), size);
        unsigned char tempBitmap[512 * 512];
        stbtt_BakeFontBitmap(buffer.data(), 0, 20.0, tempBitmap, 512, 512, 32, 96, bakedData);
        glGenTextures(1, &fontTexture);
        glBindTexture(GL_TEXTURE_2D, fontTexture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // CRÍTICO: Para fuentes
        glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA, 512, 512, 0, GL_ALPHA, GL_UNSIGNED_BYTE, tempBitmap);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        loaded = true;
    }
    void draw(float x, float y, const std::string& text) {
        if (!loaded) return;
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, fontTexture);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE); // Usar color de glColor
        glBegin(GL_QUADS);
        float curX = x, curY = y;
        for (char c : text) {
            if (c >= 32 && c < 128) {
                stbtt_aligned_quad q;
                stbtt_GetBakedQuad(bakedData, 512, 512, c - 32, &curX, &curY, &q, 1);
                glTexCoord2f(q.s0, q.t0); glVertex2f(q.x0, q.y0);
                glTexCoord2f(q.s1, q.t0); glVertex2f(q.x1, q.y0);
                glTexCoord2f(q.s1, q.t1); glVertex2f(q.x1, q.y1);
                glTexCoord2f(q.s0, q.t1); glVertex2f(q.x0, q.y1);
            }
        }
        glEnd();
        glDisable(GL_BLEND);
        glDisable(GL_TEXTURE_2D);
    }
};

class App {
    GridMap map; AStar solver; AStarResult result; TextRenderer font;
    int startNode = -1, endNode = -1;
    const float SIDEBAR_WIDTH = 300.0f; const float PAD = 20.0f;
public:
    App() : map(100, 100) { map.randomize(0.30f); font.init("C:/Windows/Fonts/arial.ttf"); }
    void click(double mx, double my, int w, int h) {
        float gridW = (float)w - SIDEBAR_WIDTH - 2 * PAD;
        if (mx > gridW + PAD) return; 
        float gridH = (float)h - 2 * PAD;
        int c = (int)round((mx - PAD) / (gridW / 99.0f)), r = (int)round((my - PAD) / (gridH / 99.0f));
        if (!map.isWalkable(c, r)) return;
        int n = map.toIdx(c, r);
        if (startNode == -1 || (startNode != -1 && endNode != -1)) { startNode = n; endNode = -1; result = {}; }
        else { endNode = n; result = solver.solve(map, startNode, endNode); }
    }
    void render(int w, int h) {
        glClearColor(0.06f, 0.06f, 0.1f, 1.0f); glClear(GL_COLOR_BUFFER_BIT);
        float gW = (float)w - SIDEBAR_WIDTH - 2 * PAD, gH = (float)h - 2 * PAD;
        float sX = gW / 99.0f, sY = gH / 99.0f;
        glPointSize(1.5f); glBegin(GL_POINTS); glColor3f(0.2f, 0.2f, 0.3f);
        for(int i=0; i<10000; ++i) if(map.active[i]) glVertex2f(PAD + map.toCol(i)*sX, PAD + map.toRow(i)*sY);
        glEnd();
        glPointSize(2.0f); glBegin(GL_POINTS); glColor3f(0.4f, 0.2f, 0.6f);
        for(int i : result.explored) glVertex2f(PAD + map.toCol(i)*sX, PAD + map.toRow(i)*sY);
        glEnd();
        if(!result.path.empty()){
            glLineWidth(2.5f); glBegin(GL_LINE_STRIP); glColor3f(0.0f, 1.0f, 0.5f);
            for(int i : result.path) glVertex2f(PAD + map.toCol(i)*sX, PAD + map.toRow(i)*sY);
            glEnd();
        }
        glPointSize(10.0f); glBegin(GL_POINTS);
        if(startNode!=-1){ glColor3f(0, 0.6f, 1); glVertex2f(PAD+map.toCol(startNode)*sX, PAD+map.toRow(startNode)*sY); }
        if(endNode!=-1){ glColor3f(1, 0.2f, 0.2f); glVertex2f(PAD+map.toCol(endNode)*sX, PAD+map.toRow(endNode)*sY); }
        glEnd();

        // PANEL LATERAL
        float sideX = (float)w - SIDEBAR_WIDTH + 30;
        glColor3f(0.12f, 0.12f, 0.18f); glBegin(GL_QUADS);
        glVertex2f((float)w - SIDEBAR_WIDTH, 0); glVertex2f((float)w, 0);
        glVertex2f((float)w, (float)h); glVertex2f((float)w - SIDEBAR_WIDTH, (float)h);
        glEnd();

        glColor3f(1.0f, 1.0f, 1.0f);
        font.draw(sideX, 60, "DATOS DEL ALGORITMO");
        font.draw(sideX, 85, "-----------------------");
        if (result.found) {
            std::stringstream ss1, ss2, ss3;
            ss1 << "Costo: " << std::fixed << std::setprecision(2) << result.cost << " u";
            ss2 << "Explorados: " << result.explored.size();
            ss3 << "Camino: " << result.path.size() << " nodos";
            font.draw(sideX, 130, ss1.str());
            font.draw(sideX, 170, ss2.str());
            font.draw(sideX, 210, ss3.str());
        } else {
            font.draw(sideX, 130, (startNode != -1 && endNode != -1) ? "Estado: Sin salida" : "Seleccione puntos...");
        }
        font.draw(sideX, (float)h - 50, "Tecla R: Reiniciar Mapa");
    }
    void reset() { map.randomize(0.30f); startNode = -1; endNode = -1; result = {}; }
};

App app;
void mouse_cb(GLFWwindow* win, int b, int a, int m) {
    if (b == GLFW_MOUSE_BUTTON_LEFT && a == GLFW_PRESS) {
        double x, y; glfwGetCursorPos(win, &x, &y);
        int w, h; glfwGetFramebufferSize(win, &w, &h);
        app.click(x, y, w, h);
    }
}

int main() {
    if (!glfwInit()) return -1;
    GLFWwindow* win = glfwCreateWindow(1150, 850, "Algoritmo A*", NULL, NULL);
    glfwMakeContextCurrent(win);
    glfwSetMouseButtonCallback(win, mouse_cb);
    glfwSetKeyCallback(win, [](GLFWwindow* w, int k, int, int a, int){ if(k==GLFW_KEY_R && a==GLFW_PRESS) app.reset(); });
    while (!glfwWindowShouldClose(win)) {
        int w, h; glfwGetFramebufferSize(win, &w, &h);
        glViewport(0, 0, w, h);
        glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(0, w, h, 0, -1, 1);
        app.render(w, h);
        glfwSwapBuffers(win); glfwPollEvents();
    }
    glfwTerminate(); return 0;
}*/