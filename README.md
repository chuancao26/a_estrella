# Visualizador Interactivo de Algoritmos de Búsqueda de Caminos (Pathfinding)

![C++](https://img.shields.io/badge/Language-C%2B%2B-blue?style=for-the-badge&logo=cplusplus)
![OpenCV](https://img.shields.io/badge/Graphics-OpenCV-green?style=for-the-badge&logo=opencv)
![License](https://img.shields.io/badge/Academic-Project-orange?style=for-the-badge)

Este proyecto es una herramienta interactiva desarrollada en **C++** y **OpenCV** para la simulación, análisis y evaluación del problema del camino más corto en un entorno matricial bidimensional de $50 \times 50$. El sistema genera mapas con obstáculos aleatorios y permite comparar de manera gráfica el comportamiento y eficiencia de diversas estrategias de búsqueda.

---

## 🚀 Características del Sistema

* **Interfaz Interactiva:** Configuración dinámica de los nodos de Inicio (Naranja) y Fin (Azul) mediante clics sobre el lienzo OpenCV.
* **Generación Aleatoria:** Creación de mapas con un 35% de densidad de obstáculos transitables o bloqueados mediante una matriz de ocupación linealizada.
* **4 Solucionadores en Tiempo Real:** Alterna instantáneamente entre algoritmos informados (heurísticos) y no informados con solo presionar una tecla.
* **Persistencia de Datos:** Exportación automática de las coordenadas analizadas a un archivo físico (`nodos_explorados.txt`) para procesos posteriores de auditoría espacial.

---

## 🧠 Algoritmos Implementados

1. **A\* (A-Estrella):** Búsqueda informada que utiliza una **heurística diagonal/octil** optimizada para 8 grados de libertad, garantizando el camino métrico más corto con la menor exploración espacial.
2. **Dijkstra:** Búsqueda no informada que garantiza la ruta óptima expandiéndose radialmente en forma de ondas concéntricas de costo incremental.
3. **BFS (Breadth-First Search):** Explora por niveles uniformes usando una estructura de Cola (FIFO). Halla el camino con menor número de aristas.
4. **DFS (Depth-First Search):** Explora de forma agresiva ramificaciones profundas mediante una estructura de Pila (LIFO).

---

## 🎮 Controles de la Aplicación

Al ejecutar el simulador, puedes interactuar usando el ratón y las siguientes teclas:

| Tecla / Acción | Función |
| :--- | :--- |
| **`Click Izquierdo`** | Establecer nodo de Inicio / Fin en la cuadrícula |
| **`1`** | Cambiar al algoritmo **A\*** y ejecutar |
| **`2`** | Cambiar al algoritmo **Dijkstra** y ejecutar |
| **`3`** | Cambiar al algoritmo **BFS** y ejecutar |
| **`4`** | Cambiar al algoritmo **DFS** y ejecutar |
| **`R` / `r`** | Generar un nuevo mapa aleatorio y reiniciar nodos |
| **`ESC`** | Cerrar la aplicación de manera segura |

---

## 🛠️ Requisitos e Instalación

### Prerrequisitos
* Un compilador compatible con **C++11** o superior (GCC, Clang o MSVC).
* Biblioteca **OpenCV 4.x** correctamente vinculada en tu entorno de desarrollo.

### Instrucciones de Compilación (Consola)
Si estás utilizando una terminal con GCC/G++, puedes compilar el proyecto ejecutando el siguiente comando:

```bash
g++ -std=c++11 main.cpp -o pathfinding_visualizer `pkg-config --cflags --libs opencv4`

## 👥 Autores
Este proyecto fue desarrollado colaborativamente por:

👤 Ferro Vásquez, Arleen Maritza
👤 Huanca Olazabal, Cristhian David
👤 Mamani Casilla, Wilson Isaac
