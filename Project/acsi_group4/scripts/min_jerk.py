import numpy as np
import matplotlib
import matplotlib.pyplot as plt


def solve_equation(xi,xf, vi, vf, t):
    A = np.array([[0, 0, 0, 0, 0, 1],
                    [t**5, t**4, t**3, t**2, t**1, 1],
                    [0, 0, 0, 0, 1, 0],
                    [5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0],
                    [0, 0, 0, 2, 0, 0],
                    [20*t**3, 12*t**2, 6*t, 2, 0,0]])
    B = np.array([xi, xf, vi, vf, 0, 0])
    param = np.linalg.solve(A, B)
    param_velocity = [param[0] * 5, param[1] * 4, param[2] * 3, param[3] * 2,  param[4], 0]
    return param, param_velocity

def build_equation(param, param_velocity, t):
    position = np.dot(param, [t**5, t**4, t**3, t**2, t, 1])
    velocity = np.dot(param_velocity, [t**4, t**3, t**2, t, 1, 0])
    return position, velocity

def main():
    param, param_velocity = solve_equation(0, 1, 0, 0, 3)
    position_list = []
    for t in np.arange(0, 3, 0.1):
        position, velocity = build_equation(param, param_velocity, t)
        position_list.append(position)


    plt.plot(position_list)
    plt.show()
    plt.savefig('trajectory.png')





if __name__ == "__main__":
    main()