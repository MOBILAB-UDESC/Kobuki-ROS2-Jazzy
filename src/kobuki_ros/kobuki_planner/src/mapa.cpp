#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <stack>

struct Nodo {
    int x, y;
    float costo_g, costo_h;
    Nodo* padre;

    float costo_f() const { return costo_g + costo_h; }

    bool operator>(const Nodo& otro) const {
        return this->costo_f() > otro.costo_f();
    }
};

// Direcciones: arriba, abajo, izquierda, derecha
const int dx[] = {-1, -1, 0, 1, 1,  1,  0, -1};
const int dy[] = { 0,  1, 1, 1, 0, -1, -1, -1};

bool dentro_del_mapa(int x, int y, int ancho, int alto) {
    return x >= 0 && x < alto && y >= 0 && y < ancho;
}

float heuristica(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2); // Manhattan
}

bool leerPGM_P5(const std::string& nombreArchivo, std::vector<std::vector<unsigned char>>& mapa, int& ancho, int& alto) {
    std::ifstream archivo(nombreArchivo, std::ios::binary);
    if (!archivo.is_open()) return false;

    std::string linea;
    std::getline(archivo, linea);
    if (linea != "P5") return false;

    do {
        std::getline(archivo, linea);
    } while (linea[0] == '#');

    std::stringstream ss(linea);
    ss >> ancho >> alto;

    int max_valor;
    archivo >> max_valor;
    archivo.get(); // Salto de línea

    std::vector<unsigned char> datos(ancho * alto);
    archivo.read(reinterpret_cast<char*>(&datos[0]), ancho * alto);

    mapa.resize(alto, std::vector<unsigned char>(ancho));
    for (int i = 0; i < alto; ++i)
        for (int j = 0; j < ancho; ++j)
            mapa[i][j] = datos[i * ancho + j];

    return true;
}

void imprimir_camino(std::vector<std::vector<unsigned char>>& mapa, Nodo* destino) {
    Nodo* actual = destino;
    while (actual && actual->padre) {
        mapa[actual->x][actual->y] = 127; // Gris para el camino
        actual = actual->padre;
    }
}

bool es_libre(const std::vector<std::vector<unsigned char>>& mapa, int x, int y) {
    return mapa[x][y] > 0;
}

Nodo* A_estrella(const std::vector<std::vector<unsigned char>>& mapa, int sx, int sy, int gx, int gy) {
    int alto = mapa.size(), ancho = mapa[0].size();
    std::vector<std::vector<bool>> visitado(alto, std::vector<bool>(ancho, false));

    auto comp = [](Nodo* a, Nodo* b) { return *a > *b; };
    std::priority_queue<Nodo*, std::vector<Nodo*>, decltype(comp)> abierta(comp);

    Nodo* inicio = new Nodo{sx, sy, 0, heuristica(sx, sy, gx, gy), nullptr};
    abierta.push(inicio);

    while (!abierta.empty()) {
        Nodo* actual = abierta.top(); abierta.pop();

        if (actual->x == gx && actual->y == gy)
            return actual;

        if (visitado[actual->x][actual->y])
            continue;

        visitado[actual->x][actual->y] = true;

        for (int i = 0; i < 8; ++i) {
            int nx = actual->x + dx[i];
            int ny = actual->y + dy[i];
        
            if (!dentro_del_mapa(nx, ny, ancho, alto) || !es_libre(mapa, nx, ny))
                continue;
        
            float costo_mov = (i % 2 == 0) ? 1.0 : std::sqrt(2.0);  // recto o diagonal
        
            if (!visitado[nx][ny]) {
                Nodo* vecino = new Nodo{nx, ny, actual->costo_g + costo_mov,
                                        heuristica(nx, ny, gx, gy), actual};
                abierta.push(vecino);
            }
        }
        
    }

    return nullptr; // No se encontró camino
}

void guardarPGM(const std::string& nombre, const std::vector<std::vector<unsigned char>>& mapa) {
    std::ofstream out(nombre, std::ios::binary);
    int alto = mapa.size(), ancho = mapa[0].size();
    out << "P5\n" << ancho << " " << alto << "\n255\n";

    for (int i = 0; i < alto; ++i)
        for (int j = 0; j < ancho; ++j)
            out.put(static_cast<char>(mapa[i][j]));
}
std::vector<std::vector<unsigned char>> inflar_mapa(const std::vector<std::vector<unsigned char>>& mapa, int radio_robot) {
    int alto = mapa.size();
    int ancho = mapa[0].size();
    std::vector<std::vector<unsigned char>> mapa_inflado = mapa;

    for (int i = 0; i < alto; ++i) {
        for (int j = 0; j < ancho; ++j) {
            if (mapa[i][j] == 0) {
                // Marca todos los píxeles en el radio como obstáculo
                for (int dx = -radio_robot; dx <= radio_robot; ++dx) {
                    for (int dy = -radio_robot; dy <= radio_robot; ++dy) {
                        int ni = i + dx;
                        int nj = j + dy;
                        if (dentro_del_mapa(ni, nj, ancho, alto)) {
                            mapa_inflado[ni][nj] = 0;
                        }
                    }
                }
            }
        }
    }
    return mapa_inflado;
}

int main() {
    std::string archivo = "/home/nilton/Desktop/Ros2/Kobuki/src/kobuki_ros/kobuki_planner/map/kobuki_map.pgm";
    std::vector<std::vector<unsigned char>> mapa;
    int ancho, alto;

    if (!leerPGM_P5(archivo, mapa, ancho, alto)) {
        std::cerr << "No se pudo leer el mapa.\n";
        return 1;
    }

    int radio_robot = 5; // ya que 3 píxeles de ancho → radio ≈ 1
    mapa = inflar_mapa(mapa, radio_robot);


    int sx = 20, sy = 20;         // Punto de inicio
    int gx = alto - 20, gy = ancho - 20; // Meta (modifica según el mapa)

    Nodo* destino = A_estrella(mapa, sx, sy, gx, gy);

    if (destino) {
        std::cout << "Camino encontrado.\n";
        imprimir_camino(mapa, destino);
        guardarPGM("camino.pgm", mapa);
    } else {
        std::cout << "No se encontró un camino.\n";
    }

    return 0;
}
