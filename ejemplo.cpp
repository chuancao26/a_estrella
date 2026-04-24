#include <GLFW/glfw3.h>
#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
#include <cstdlib>
#include <ctime>

// ─── Configuración ─────────────────────────────────────────
const int COLS = 100;
const int ROWS = 100;
const float PADDING = 20.0f;
const float DELETE_CHANCE = 0.30f; // 30% de los nodos serán borrados

bool active[COLS * ROWS]; // True si el nodo existe, False si fue borrado
int startNode = -1;
int endNode = -1;

std::vector<int> pathNodes;
std::vector<int> exploredNodes;

// ─── Utilidades ────────────────────────────────────────────
inline int idx(int c, int r) { return r * COLS + c; }
inline int col(int i) { return i % COLS; }
inline int row(int i) { return i / COLS; }
inline bool inBounds(int c, int r) {
    return c >= 0 && c < COLS && r >= 0 && r < ROWS;
}

// ─── Generar Mapa Aleatorio ────────────────────────────────
void randomizeMap() {
    srand((unsigned)time(nullptr));
    for (int i = 0; i < COLS * ROWS; i++) {
        active[i] = ((float)rand() / RAND_MAX) > DELETE_CHANCE;
    }
    
    // Limpiar estados previos
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

    // Asegurarse de que inicio y fin estén activos
    active[startNode] = true;
    active[endNode] = true;

    int N = COLS * ROWS;
    std::vector<float> g(N, 1e9);
    std::vector<float> f(N, 1e9);
    std::vector<int> parent(N, -1);
    std::vector<bool> closed(N, false);

    using Node = std::pair<float, int>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

    g[startNode] = 0;
    f[startNode] = heuristic(startNode, endNode);
    open.push({f[startNode], startNode});

    int dc[8] = {1, 0, -1, 0, 1, 1, -1, -1};
    int dr[8] = {0, 1, 0, -1, 1, -1, 1, -1};
    float cost[8] = {1, 1, 1, 1, 1.4142f, 1.4142f, 1.4142f, 1.4142f};

    while (!open.empty()) {
        auto [_, current] = open.top();
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
            // Si el nodo NO está activo, es como un muro
            if (!active[neighbor] || closed[neighbor]) continue;

            // Evitar atravesar esquinas de nodos borrados
            if (d >= 4) {
                if (!active[idx(c + dc[d], r)] || !active[idx(c, r + dr[d])]) continue;
            }

            float newG = g[current] + cost[d];
            if (newG < g[neighbor]) {
                g[neighbor] = newG;
                f[neighbor] = newG + heuristic(neighbor, endNode);
                parent[neighbor] = current;
                open.push({f[neighbor], neighbor});
            }
        }
    }

    int cur = endNode;
    if (parent[endNode] != -1 || startNode == endNode) {
        while (cur != -1) {
            pathNodes.push_back(cur);
            cur = parent[cur];
        }
        std::reverse(pathNodes.begin(), pathNodes.end());
    }
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
    glClear(GL_COLOR_BUFFER_BIT);

    float areaW = w - 2 * PADDING;
    float areaH = h - 2 * PADDING;
    float stepX = areaW / (COLS - 1);
    float stepY = areaH / (ROWS - 1);
    float nodeRadius = stepX * 0.45f;

    // 1. Nodos Base (Solo los activos) - Color Gris oscuro
    glColor3f(0.15f, 0.15f, 0.25f);
    for (int i = 0; i < COLS * ROWS; i++) {
        if (!active[i]) continue;
        float x = PADDING + col(i) * stepX;
        float y = PADDING + row(i) * stepY;
        drawCircle(x, y, nodeRadius, 6);
    }

    // 2. Nodos Explorados (Violeta)
    glColor3f(0.4f, 0.2f, 0.6f);
    for (int i : exploredNodes) {
        float x = PADDING + col(i) * stepX;
        float y = PADDING + row(i) * stepY;
        drawCircle(x, y, nodeRadius * 1.1f, 6);
    }

    // 3. Camino (Verde)
    glColor3f(0.0f, 1.0f, 0.5f);
    for (int i : pathNodes) {
        float x = PADDING + col(i) * stepX;
        float y = PADDING + row(i) * stepY;
        drawCircle(x, y, nodeRadius * 1.3f, 8);
    }

    // 4. Inicio y Fin - AHORA DEL MISMO TAMAÑO QUE UN NODO BASE
    if (startNode >= 0) {
        float x = PADDING + col(startNode) * stepX;
        float y = PADDING + row(startNode) * stepY;
        glColor3f(0.0f, 0.6f, 1.0f); // Azul
        drawCircle(x, y, nodeRadius, 12); // <-- Sin multiplicador
    }
    if (endNode >= 0) {
        float x = PADDING + col(endNode) * stepX;
        float y = PADDING + row(endNode) * stepY;
        glColor3f(1.0f, 0.2f, 0.2f); // Rojo
        drawCircle(x, y, nodeRadius, 12); // <-- Sin multiplicador
    }
}

// ─── Callbacks ─────────────────────────────────────────────
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_R && action == GLFW_PRESS) {
        randomizeMap();
    }
}

void mouse_callback(GLFWwindow* win, int button, int action, int) {
    if (button != GLFW_MOUSE_BUTTON_LEFT || action != GLFW_PRESS) return;

    double mx, my;
    glfwGetCursorPos(win, &mx, &my);
    int w, h;
    glfwGetFramebufferSize(win, &w, &h);

    int c = (int)round((mx - PADDING) / ((w - 2 * PADDING) / (COLS - 1)));
    int r = (int)round((my - PADDING) / ((h - 2 * PADDING) / (ROWS - 1)));

    if (!inBounds(c, r)) return;
    int n = idx(c, r);

    // No permitir seleccionar un nodo borrado como inicio o fin
    if (!active[n]) return;

    if (startNode < 0) {
        startNode = n;
    } else if (endNode < 0) {
        endNode = n;
        runAstar();
    } else {
        startNode = n;
        endNode = -1;
        pathNodes.clear();
        exploredNodes.clear();
    }
}

int main() {
    if (!glfwInit()) return -1;

    GLFWwindow* win = glfwCreateWindow(800, 800, "A*", NULL, NULL);
    if (!win) { glfwTerminate(); return -1; }

    glfwMakeContextCurrent(win);
    glfwSetMouseButtonCallback(win, mouse_callback);
    glfwSetKeyCallback(win, key_callback);

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
}