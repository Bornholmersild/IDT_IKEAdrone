from rdp import *
import numpy as np
import matplotlib.pyplot as plt

class track_simplifyer:
    def __init__(self):
        pass

    def plot_track(arr):
        # Print original array
        print("Original Array:\n", arr)

        # Plot the original array as points in 3D space
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(arr[:, 0], arr[:, 1], arr[:, 2], 'o-', label='Original Path')
        plt.legend()
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')


        # Simplify the array using the RDP algorithm with epsilon = 0.5
        epsilon = 0.1
        simplified_arr = rdp(arr.tolist(), epsilon=epsilon)
        simplified_arr = np.array(simplified_arr)  # Convert back to numpy array
        print(simplified_arr)
        # Plot the simplified array
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(arr[:, 0], arr[:, 1], arr[:, 2], 'o-', label='Original Path')
        ax.plot(simplified_arr[:, 0], simplified_arr[:, 1], simplified_arr[:, 2], 'x-', label=f'Simplified Path (epsilon={epsilon})')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.legend()
        plt.axis = 'equal'
        plt.show()


def main():
    trk = track_simplifyer

    arr = np.array([1, 1, 1, 1.5, 1.5, 2, 2, 2, 2, 3, 3, 3, 4, 4, 4]).reshape(5, 3)

    trk.plot_track(arr)

if __name__ ==  "__main__":
    main()


