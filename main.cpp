#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <fstream> 
#include "grid.h"
#include "solvers.h"

const int GRID_COLS = 50; 
const int GRID_ROWS = 50;
const int MAP_SIZE = 800; 
const int PANEL_WIDTH = 300; 
const int WIN_WIDTH = MAP_SIZE + PANEL_WIDTH;
const int WIN_HEIGHT = MAP_SIZE;

class App {
public:
    GridMap map;
    PathFinder finder;
    SearchResult result;
    SearchMode currentMode = ASTAR;
    int startNode = -1, endNode = -1;
    double lastSearchTime = 0.0; 
    const float PAD = 30.0f;

    App() : map(GRID_COLS, GRID_ROWS) { map.randomize(0.35f); }

    void guardarNodosTxt() {
        if (result.explored.empty()) return;

        std::ofstream archivo("nodos_explorados.txt");
        if (archivo.is_open()) {
            for (int idx : result.explored) {
                archivo << map.toRow(idx) << ", " << map.toCol(idx) << "\n";
            }
            archivo.close();
            std::cout << "> Nodos explorados guardados en 'nodos_explorados.txt'" << std::endl;
        }
    }

    void runSearch() {
        if (startNode != -1 && endNode != -1) {
            auto start = std::chrono::high_resolution_clock::now();
            result = finder.solve(map, startNode, endNode, currentMode);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end - start;
            lastSearchTime = duration.count();
            guardarNodosTxt();
        }
    }

    void render(cv::Mat& canvas) {
        canvas = cv::Scalar(20, 20, 20); 

        cv::rectangle(canvas, cv::Rect(MAP_SIZE, 0, PANEL_WIDTH, WIN_HEIGHT), cv::Scalar(40, 40, 40), -1);
        cv::line(canvas, cv::Point(MAP_SIZE, 0), cv::Point(MAP_SIZE, WIN_HEIGHT), cv::Scalar(100, 100, 100), 2);

        float availableW = (float)MAP_SIZE - 2 * PAD;
        float availableH = (float)WIN_HEIGHT - 2 * PAD;
        float sX = availableW / (map.cols - 1);
        float sY = availableH / (map.rows - 1);
        float pointRadius = (availableW / map.cols) * 0.4f;
        if (pointRadius < 1.0f) pointRadius = 1.0f;

        for(int i = 0; i < map.cols * map.rows; ++i) {
            cv::Point center(PAD + map.toCol(i) * sX, PAD + map.toRow(i) * sY);
            if (!map.active[i]) {
                cv::circle(canvas, center, pointRadius, cv::Scalar(0, 0, 0), -1);
            } else {
                cv::circle(canvas, center, pointRadius, cv::Scalar(100, 80, 80), -1);
            }
        }

        for(int i : result.explored) {
            cv::Point center(PAD + map.toCol(i) * sX, PAD + map.toRow(i) * sY);
            cv::circle(canvas, center, pointRadius, cv::Scalar(0, 200, 220), -1);
        }

        if(!result.path.empty()){
            for(size_t i = 0; i < result.path.size() - 1; ++i) {
                cv::Point p1(PAD + map.toCol(result.path[i]) * sX, PAD + map.toRow(result.path[i]) * sY);
                cv::Point p2(PAD + map.toCol(result.path[i+1]) * sX, PAD + map.toRow(result.path[i+1]) * sY);
                cv::line(canvas, p1, p2, cv::Scalar(0, 255, 100), 2, cv::LINE_AA);
            }
        }

        if(startNode != -1){ 
            cv::circle(canvas, cv::Point(PAD + map.toCol(startNode) * sX, PAD + map.toRow(startNode) * sY), 
                       pointRadius * 1.5f, cv::Scalar(255, 150, 0), -1); 
        }
        if(endNode != -1){ 
            cv::circle(canvas, cv::Point(PAD + map.toCol(endNode) * sX, PAD + map.toRow(endNode) * sY), 
                       pointRadius * 1.5f, cv::Scalar(50, 50, 255), -1); 
        }

        // --- RENDERIZADO DEL PANEL DE INFO ---
        int tx = MAP_SIZE + 20;
        int ty = 50;
        std::string modeStr[] = {"A*", "Dijkstra", "BFS", "DFS"};
        
        auto drawText = [&](std::string text, cv::Scalar color, float scale = 0.6) {
            cv::putText(canvas, text, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
            ty += 35;
        };

        // 1. CONFIGURACION
        drawText("CONFIGURACION", cv::Scalar(200, 200, 200), 0.7);
        ty += 5;
        drawText("Modo: " + modeStr[currentMode], cv::Scalar(0, 255, 255));
        drawText("Mapa: " + std::to_string(GRID_COLS) + "x" + std::to_string(GRID_ROWS), cv::Scalar(180, 180, 180));
        
        ty += 30; // Espacio entre secciones

        // 2. ESTADISTICAS
        drawText("ESTADISTICAS", cv::Scalar(200, 200, 200), 0.7);
        ty += 5;
        drawText("Explorados: " + std::to_string(result.explored.size()), cv::Scalar(255, 255, 255));
        drawText("Costo Total: " + std::to_string((int)result.cost), cv::Scalar(0, 255, 100));
        drawText("Tiempo: " + std::to_string(lastSearchTime).substr(0, 5) + " ms", cv::Scalar(100, 200, 255));

        ty += 150; // ESPACIO MÁS GRANDE ANTES DE CONTROLES

        // 3. CONTROLES
        drawText("CONTROLES", cv::Scalar(200, 200, 200), 0.7);
        ty += 5;
        drawText("1: A*", cv::Scalar(150, 150, 150), 0.5);
        drawText("2: Dijkstra", cv::Scalar(150, 150, 150), 0.5);
        drawText("3: BFS", cv::Scalar(150, 150, 150), 0.5);
        drawText("4: DFS", cv::Scalar(150, 150, 150), 0.5);
        drawText("R: Reiniciar Mapa", cv::Scalar(150, 150, 150), 0.5);
        drawText("Click: Set Inicio/Fin", cv::Scalar(150, 150, 150), 0.5);
    }
};

App app;

void mouse_callback(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (x < MAP_SIZE) {
            float sX = (MAP_SIZE - 2 * app.PAD) / (app.map.cols - 1);
            float sY = (WIN_HEIGHT - 2 * app.PAD) / (app.map.rows - 1);
            int c = (int)std::round((x - app.PAD) / sX);
            int r = (int)std::round((y - app.PAD) / sY);
            
            if (app.map.isWalkable(c, r)) {
                if (app.startNode == -1 || (app.startNode != -1 && app.endNode != -1)) {
                    app.startNode = app.map.toIdx(c, r); 
                    app.endNode = -1; 
                    app.result = {};
                } else {
                    app.endNode = app.map.toIdx(c, r); 
                    app.runSearch();
                }
            }
        }
    }
}

int main() {
    cv::namedWindow("Algoritmos de Busqueda", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Algoritmos de Busqueda", mouse_callback);

    cv::Mat canvas(WIN_HEIGHT, WIN_WIDTH, CV_8UC3);

    while (true) {
        app.render(canvas);
        cv::imshow("Algoritmos de Busqueda", canvas);

        int key = cv::waitKey(15);
        if (key == 27) break; 
        if (cv::getWindowProperty("Algoritmos de Busqueda", cv::WND_PROP_VISIBLE) < 1) break;

        if (key == 'r' || key == 'R') { 
            app.map.randomize(0.35f); 
            app.startNode = app.endNode = -1; 
            app.result = {}; 
            app.lastSearchTime = 0;
        }
        if (key == '1') { app.currentMode = ASTAR; app.runSearch(); }
        if (key == '2') { app.currentMode = DIJKSTRA; app.runSearch(); }
        if (key == '3') { app.currentMode = BFS; app.runSearch(); }
        if (key == '4') { app.currentMode = DFS; app.runSearch(); }
    }
    return 0;
}