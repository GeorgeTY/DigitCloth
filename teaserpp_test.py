import numpy as np
import time
import matplotlib.pyplot as plt
import teaserpp_python


def visualize(X, Y):
    plt.cla()
    plt.scatter(X[:, 0], X[:, 1], color="red", label="Target")
    plt.scatter(Y[:, 0], Y[:, 1], color="blue", label="Source")
    plt.legend(loc="upper left", fontsize="x-large")
    plt.draw()
    plt.pause(0.001)


def main():

    X = np.loadtxt("output/saved_X.out")
    Y = np.loadtxt("output/saved_Y.out")
    src = np.vstack((X[:, 0], X[:, 1], np.zeros(X.shape[0])))
    dst = np.vstack((Y[:, 0], Y[:, 1], np.zeros(Y.shape[0])))

    # Populate the parameters
    solver_params = teaserpp_python.RobustRegistrationSolver.Params()
    solver_params.cbar2 = 1
    solver_params.noise_bound = 0.01
    solver_params.estimate_scaling = True
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
    visualize(src, dst)
    print("Solution is:", solution)
    print("Time: {:.2f}".format(toc - tic))


if __name__ == "__main__":
    main()
