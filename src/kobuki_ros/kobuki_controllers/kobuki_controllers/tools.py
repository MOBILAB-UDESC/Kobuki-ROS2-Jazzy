# Tools for graphics and saving data

import numpy as np
import os
import matplotlib.pyplot as plt

def save_data(xref, yref, thref, vref, wref, 
              x, y , theta, v , w,
              route):
    
    if not os.path.exists(route):
        os.makedirs(route)

    np.savetxt(route+"/Xref", xref, delimiter=',')
    np.savetxt(route+"/Yref", yref, delimiter=',')
    np.savetxt(route+"/Tref", thref, delimiter=',')
    np.savetxt(route+"/Vref", vref, delimiter=',')
    np.savetxt(route+"/Wref", wref, delimiter=',')
    np.savetxt(route+"/X", x, delimiter=',')
    np.savetxt(route+"/Y", y, delimiter=',')
    np.savetxt(route+"/Theta", theta, delimiter=',')
    np.savetxt(route+"/Vel", v, delimiter=',')
    np.savetxt(route+"/Wang", w, delimiter=',')

def draw_data(xref, yref, thref, vref, wref, 
              x, y , theta, v , w,
              name = ''):
    
    fig, axs = plt.subplots(2, 2, figsize=(12, 10))
            
    # Gráfica 1: Trayectoria
    axs[0, 0].plot(xref, yref, label="Referencia", linestyle="--")
    axs[0, 0].plot(x, y, label="Trajectory")
    axs[0, 0].set_xlabel("x")
    axs[0, 0].set_ylabel("y")
    axs[0, 0].set_title("Path tracking with LQR")
    axs[0, 0].legend()
    axs[0, 0].grid()
    axs[0, 0].axis("equal")
    
    # Gráfica 2: Velocidad lineal
    axs[0, 1].plot(vref, label="Desired Linear Velocity")
    axs[0, 1].plot(v, label="Actual Linear Velocity")
    axs[0, 1].set_xlabel("Index")
    axs[0, 1].set_ylabel("Linear Velocity (m/s)")
    axs[0, 1].set_title("Linear Velocity over trajectory")
    axs[0, 1].legend()
    axs[0, 1].grid()
    
    # Gráfica 3: Velocidad angular
    axs[1, 0].plot(wref, label="Desired Angular Velocity")
    axs[1, 0].plot(w, label="Actual Angular Velocity")
    axs[1, 0].set_xlabel("Index")
    axs[1, 0].set_ylabel("Angular Velocity (rads/s)")
    axs[1, 0].set_title("Angular Velocity over trajectory")
    axs[1, 0].legend()
    axs[1, 0].grid()
    
    # Gráfica 4: Orientación
    axs[1, 1].plot(thref, label="Desired Orientation")
    axs[1, 1].plot(theta, label="Actual Orientation")
    axs[1, 1].set_xlabel("Index")
    axs[1, 1].set_ylabel("Orientation (rads)")
    axs[1, 1].set_title("Orientation over trajectory")
    axs[1, 1].legend()
    axs[1, 1].grid()
    
    plt.tight_layout()
    # plt.savefig(name)
    plt.show()