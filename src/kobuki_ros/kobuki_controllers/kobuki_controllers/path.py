
import numpy as np
import matplotlib.pyplot as plt

class Trajectory():
    def __init__(self, dt, trajectory):
        self.dt         = dt
        self.trajectory = trajectory

    def get_path(self):

        if self.trajectory == "circle":
            theta = np.arange(0, 2 * np.pi, self.dt)
            x_ref = np.sin(theta)
            y_ref = np.cos(theta)
        elif self.trajectory == "random":
            eta, alpha = 1.75, 3
            theta = np.arange(0, 2 * np.pi * alpha * 4, self.dt)

            # Coeficientes aleatorios para generar una curva suave
            a1, a2 = np.random.rand(2) * 2  # Amplitudes aleatorias
            f1, f2 = np.random.randint(1, 5, 2)  # Frecuencias aleatorias

            x_ref = eta * np.sin(f1 * theta / alpha) + a1 * np.sin(f2 * theta / alpha)
            y_ref = eta * np.sin(f1 * theta / (2 * alpha)) + a2 * np.sin(f2 * theta / (2 * alpha))
            
        elif self.trajectory == "infinite":
            eta, alpha = 1.25, 5
            theta = np.arange(0, 2 * np.pi * alpha * 2, self.dt)
            # x_ref = np.sin(2 * theta / alpha)
            # y_ref = eta * np.sin(theta / alpha)
            x_ref = 1 * np.sin(theta / alpha)
            y_ref = 0.1 +  eta * np.sin(theta / (2 * alpha))
        else:
            print("Options are 1. circle 2. random 3. infinite")

        dx_ref = np.gradient(x_ref, self.dt)
        dy_ref = np.gradient(y_ref, self.dt)
        th_ref = np.unwrap(np.arctan2(np.gradient(y_ref, self.dt), np.gradient(x_ref, self.dt))) # To avoid discontinuity
        # th_ref = np.arctan2(np.gradient(y_ref, self.dt), np.gradient(x_ref, self.dt))

        ddx_ref = np.gradient(dx_ref, self.dt)
        ddy_ref = np.gradient(dy_ref, self.dt)

        vref = np.sqrt(dx_ref**2 + dy_ref**2) # 0.2*np.ones(len(x_ref))# np.sqrt(dx_ref**2 + dy_ref**2)
        wref = (ddy_ref * dx_ref - ddx_ref * dy_ref) / (dx_ref**2 + dy_ref**2)

        return x_ref, y_ref, th_ref, vref, wref

    def plot_trajectory(self, x_ref, y_ref, th_ref, vref, wref):

        plt.figure()
        plt.plot(x_ref, y_ref, label="Desired Trajectory")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title(f"Desired Trajectory: {self.trajectory}")
        plt.legend()
        plt.axis("equal")
        plt.grid()
        plt.show()

        plt.figure()
        plt.plot(th_ref, label="Desired Orientation (th_ref)")
        plt.xlabel("Index")
        plt.ylabel("Desired Orientation (radians)")
        plt.title("Orientation over trajectory")
        plt.legend()
        plt.grid()
        plt.show()

        plt.figure()
        plt.plot(vref, label="Desired Linear Velocity")
        plt.xlabel("Index")
        plt.ylabel("Desired Linear Velocity (m/s)")
        plt.title("Linear Velocity over trajectory")
        plt.legend()
        plt.grid()
        plt.show()

        plt.figure()
        plt.plot(wref, label="Desired Angular Velocity")
        plt.xlabel("Index")
        plt.ylabel("Desired Angular Velocity (radians/s)")
        plt.title("Angular Velocity over trajectory")
        plt.legend()
        plt.grid()
        plt.show()

if __name__ == "__main__":
    dt                              = 0.01
    trajectory                      = "infinite"
    path                            = Trajectory(dt, trajectory)
    xref, yref, thref, vref, wref   = path.get_path()

    path.plot_trajectory(xref, yref, thref, vref, wref)