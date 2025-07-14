
import numpy as np
import matplotlib.pyplot as plt

class Trajectory():
    def __init__(self, _dt = 0.1, _eta = 1.25, _rho = 5, _cycles = 1, _center = [0.0, 0.1], _type = "line"):
        """
        Args:
            _dt      (float)  : Time step
            _eta     (float)  : Trajectory amplitude scaling factor
             _cycles (int)    : Nº cycles/repetitions of the trajectory
            _rho   (float)  : Parameter to control trajectory density (Nº points = 2π * _rho * _cycles / _dt)
            _type    (str)    : Trajectory type - "circular" or "infinite"
        """
        
        self._dt     = _dt
        self._eta    = _eta
        self._rho  = _rho
        self._cycles = _cycles
        self._type   = _type
        self._center = _center

        self.theta_end = 2 * np.pi * self._rho * self._cycles
        self.n_points  = int(self.theta_end / self._dt)

    def get_path(self):
        """
        Generate reference trajectory based on specified type.

        Returns:
            tuple: Contains arrays for:
                - x_ref : X-axis positions
                - y_ref : Y-axis positions 
                - th_ref: Reference orientations
                - vref  : Reference linear velocities
                - wref  : Reference angular velocities
        """

        if self._type == "circle":

            theta = np.arange(0, self.theta_end, self._dt)
            x_ref = self._center[0] + self._eta * np.sin(theta / self._rho)
            y_ref = self._center[1] + self._eta * np.cos(theta / self._rho)
            
        elif self._type == "infinite":

            theta = np.arange(0, self.theta_end, self._dt)
            x_ref = self._center[0] + 1 * np.sin(2 * theta / self._rho)
            y_ref = self._center[1] + self._eta * np.sin(theta / self._rho)

            x_rot = x_ref * np.cos(self._center[2]) - y_ref * np.sin(self._center[2])
            y_rot = x_ref * np.sin(self._center[2]) + y_ref * np.cos(self._center[2])

            x_ref = x_rot
            y_ref = y_rot

        elif self._type == "line":

            theta = np.pi / 4
            x_ref = np.zeros(self.n_points)
            y_ref = np.zeros(self.n_points)
            for i in range(self.n_points):
                x_ref[i] = self._center[0] + (i/50) * np.cos(theta)
                y_ref[i] = self._center[1] +  (i/50) * np.sin(theta)

        else:
            print("Options are 1. circle 2. infinite")

        dx_ref = np.gradient(x_ref, self._dt)
        dy_ref = np.gradient(y_ref, self._dt)

        # Calculate orientation avoiding discontinuities with unwrap() function
        th_ref = np.unwrap(np.arctan2(np.gradient(y_ref, self._dt), np.gradient(x_ref, self._dt)))

        ddx_ref = np.gradient(dx_ref, self._dt)
        ddy_ref = np.gradient(dy_ref, self._dt)

        vref = np.sqrt(dx_ref**2 + dy_ref**2)
        wref = (ddy_ref * dx_ref - ddx_ref * dy_ref) / (dx_ref**2 + dy_ref**2)

        return x_ref, y_ref, th_ref, vref, wref

    def plot_trajectory(self, x_ref, y_ref, th_ref, vref, wref):
        """
        Visualize the generated trajectory and its characteristics.
        
        Args:
            x_ref, y_ref (array): Position XY
            th_ref (array)      : Orientation
            v_ref (array)       : Linear velocity input
            w_ref (array)       : Angular velocity input
        """

        plt.figure()
        plt.plot(x_ref, y_ref, label="Desired Trajectory")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title(f"Desired Trajectory: {self._type}")
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
    path                            = Trajectory()
    xref, yref, thref, vref, wref   = path.get_path()
    print(path.n_points)

    path.plot_trajectory(xref, yref, thref, vref, wref)