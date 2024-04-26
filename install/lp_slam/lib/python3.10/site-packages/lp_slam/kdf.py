import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import gaussian_kde

class Kde:
    def __init__(self):
        pass

    def kde_2d(self):
        # Generate random 2D data
        mean = [0, 0]
        cov = [[1, 0.5], [0.5, 1]]
        data = np.random.multivariate_normal(mean, cov, 1000).T

        # Create kernel density estimate
        kde = gaussian_kde(data)

        # Define grid for density estimation
        x_grid, y_grid = np.meshgrid(np.linspace(-3, 3, 100), np.linspace(-3, 3, 100))
        positions = np.vstack([x_grid.ravel(), y_grid.ravel()])

        # Calculate density estimate at each point
        density = kde(positions)
        density = density.reshape(x_grid.shape)

        # Plot the results
        plt.figure(figsize=(8, 6))
        plt.contourf(x_grid, y_grid, density, cmap='Blues')
        plt.colorbar(label='Density')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('2D Kernel Density Estimation')
        plt.show()

    def kde_1d(self):
        # Generate random 1D data
        data = np.random.normal(0, 1, 1000)

        # Create kernel density estimate
        kde = gaussian_kde(data)

        # Define grid for density estimation
        x_grid = np.linspace(-3, 3, 100)

        # Calculate density estimate at each point
        density = kde(x_grid)

        # Plot the results
        plt.figure(figsize=(8, 6))
        plt.plot(x_grid, density, label='Kernel Density Estimate')
        plt.hist(data, bins=30, density=True, alpha=0.5, label='Histogram')
        plt.xlabel('X')
        plt.ylabel('Density')
        plt.title('1D Kernel Density Estimation')
        plt.legend()
        plt.show()

def main():
    kde = Kde()
    kde.kde_2d()
    # kde.kde_1d()    