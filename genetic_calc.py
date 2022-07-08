import numpy as np
import math
import time


def rectMatrix(matrix):
    """
    Add zeros to the matrix to make it rectangular
    :param matrix to be rectangular
    :return: matrix
    """
    (m, n) = np.shape(matrix)
    if m > n:
        matrix = np.hstack((matrix, np.zeros((m, m - n))))
    elif m < n:
        matrix = np.vstack((matrix, np.zeros((n - m, n))))
    return matrix


def initRoute(n_route, n_dots):
    """
    Initialize the route
    :param n_route: number of routes
    :param n_dots: number of dots
    :return: route
    """
    routes = np.zeros((n_route, n_dots)).astype(int)
    for i in range(n_route):
        routes[i] = np.random.choice(range(n_dots), size=n_dots, replace=False)
    return routes


def getRouteE(route, P):
    """
    Calculate the Expectation of the route
    :param route: route
    :param P: matrix
    :return: Expectation
    """
    E = 0
    for i in range(len(route)):
        E += P[i][route[i]]
    return E


def getRoutesEs(routes, P):
    """
    Calculate the Expectation of the route
    :param routes: routes
    :param P: matrix
    :return: Expectation
    """
    Es = np.zeros(len(routes))
    for i in range(len(routes)):
        Es[i] = getRouteE(routes[i], P)
    return Es


def selectRoute(routes, E):
    """
    Select the route with the lowest Expectation
    :param routes: routes
    :param E: Expectation
    :return: route selected
    """
    selected_routes = np.zeros(routes.shape).astype(int)
    probability = E / np.sum(E)
    n_routes = routes.shape[0]
    for i in range(n_routes):
        choice = np.random.choice(range(n_routes), p=probability)
        selected_routes[i] = routes[choice]
    return selected_routes


def crossoverRoute(routes, n_dots):
    """
    Crossover the route
    :param routes: routes
    :param n_dots: number of dots
    :return: routes
    """
    for i in range(0, len(routes), 2):
        r1_new, r2_new = np.zeros(n_dots), np.zeros(n_dots)
        seg_point = np.random.randint(0, n_dots)
        cross_len = n_dots - seg_point
        r1, r2 = routes[i], routes[i + 1]
        r1_cross, r2_cross = r2[seg_point:], r1[seg_point:]
        r1_non_cross = r1[np.in1d(r1, r1_cross) == False]
        r2_non_cross = r2[np.in1d(r2, r2_cross) == False]
        r1_new[:cross_len], r2_new[:cross_len] = r1_cross, r2_cross
        r1_new[cross_len:], r2_new[cross_len:] = r1_non_cross, r2_non_cross
        routes[i], routes[i + 1] = r1_new, r2_new
    return routes


def mutateRoute(routes, n_dots):
    """
    Mutate the route
    :param routes: routes
    :param n_dots: number of dots
    :return: routes
    """
    prob = 0.2
    p_rand = np.random.rand(len(routes))
    for i in range(len(routes)):
        if p_rand[i] < prob:
            mut_position = np.random.choice(range(n_dots), size=2, replace=False)
            l, r = mut_position[0], mut_position[1]
            routes[i, l], routes[i, r] = routes[i, r], routes[i, l]
    return routes


def toMatrixM(route):
    """
    Convert the route to matrix
    :param route: route
    :return: matrix
    """
    matrix = np.zeros((len(route), len(route)))
    for i in range(len(route)):
        matrix[i][route[i]] = 1
    return matrix


def calcMatrixM(P, n_route=100, epoch=100000):
    """
    Calculate the Expectation of the route
    :param routes: routes
    :param dots: dots
    :param n_dots: number of dots
    :return: best Matrix
    """
    n_dots = np.shape(P)[0]
    routes = initRoute(n_route, n_dots)
    Es = getRoutesEs(routes, P)
    best_index = Es.argmax()
    best_route, best_E = routes[best_index], Es[best_index]

    not_improve_time = 0
    for i in range(epoch):
        routes = selectRoute(routes, Es)
        routes = crossoverRoute(routes, n_dots)
        routes = mutateRoute(routes, n_dots)
        Es = getRoutesEs(routes, P)
        best_route_index = Es.argmax()
        if Es[best_route_index] > best_E:
            not_improve_time = 0
            best_route, best_E = (
                routes[best_route_index],
                Es[best_route_index],
            )
        else:
            not_improve_time += 1
        if (i + 1) % 200 == 0:
            print("epoch: {},  Max E: {}".format(i + 1, getRouteE(best_route, P)))
        if not_improve_time >= 2000:
            print("End of epoch")
            break

    return toMatrixM(best_route)


def main():
    P = np.loadtxt("./output/saved_P.out", delimiter=",")
    P = rectMatrix(P)
    M = calcMatrixM(P, 50, 10000)
    print(M)


if __name__ == "__main__":
    main()
