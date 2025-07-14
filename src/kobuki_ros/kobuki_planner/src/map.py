import numpy as np
import matplotlib.pyplot as plt

def leer_pgm_binario(ruta):
    with open(ruta, 'rb') as f:
        assert f.readline() == b'P5\n'
        while True:
            linea = f.readline()
            if linea.startswith(b'#'):
                continue
            else:
                ancho, alto = [int(i) for i in linea.split()]
                break
        max_valor = int(f.readline())

        datos = np.frombuffer(f.read(), dtype=np.uint8)
        imagen = datos.reshape((alto, ancho))
        # imagen = np.rot90(imagen, k=-1)
        return imagen


img = leer_pgm_binario('/home/nilton/Desktop/Ros2/Kobuki-ROS2-Jazzy/explored_nodes_A_star.pgm')
print(len(img))
plt.imshow(img, cmap='gray')
plt.title('Imagen PGM')
plt.axis('off')
plt.show()
