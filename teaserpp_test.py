import numpy as np
import time
import matplotlib.pyplot as plt
import teaserpp_python


def visualize(X, Y):
    plt.cla()
    plt.scatter(X[0, :], X[1, :], color="red", label="Target")
    plt.scatter(Y[0, :], Y[1, :], color="blue", label="Source")
    plt.legend(loc="upper left", fontsize="x-large")
    plt.draw()
    plt.show()
    plt.pause(0)


def main():

    X = np.loadtxt("saved/data/saved_X.out")
    Y = np.loadtxt("saved/data/saved_Y.out")
    src = np.vstack((X[:, 0], X[:, 1], np.zeros(X.shape[0])))
    dst = np.vstack((Y[:, 0], Y[:, 1], np.zeros(Y.shape[0])))

    # Populate the parameters
    solver_params = teaserpp_python.RobustRegistrationSolver.Params()
    solver_params.cbar2 = 1e14
    solver_params.noise_bound = 0.01
    solver_params.estimate_scaling = False
    solver_params.rotation_estimation_algorithm = (
        teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
    )
    solver_params.rotation_gnc_factor = 1.4
    solver_params.rotation_max_iterations = 100
    solver_params.rotation_cost_threshold = 1e-12
    print("TEASER++ Parameters are:", solver_params)
    teaserpp_solver = teaserpp_python.RobustRegistrationSolver(solver_params)

    tic = time.time()
    solver = teaserpp_python.RobustRegistrationSolver(solver_params)
    solver.solve(src, dst)

    solution = solver.getSolution()
    toc = time.time()

    # Print the solution
    print("Solution is:", solution)
    print("Time: {:.2f}".format(toc - tic))
    result = solution.scale * np.matmul(solution.rotation, src)
    result[0, :] = result[0, :] + np.ones_like(result[0, :]) * solution.translation[0]
    result[1, :] = result[1, :] + np.ones_like(result[1, :]) * solution.translation[1]
    visualize(src, result)


if __name__ == "__main__":
    main()
