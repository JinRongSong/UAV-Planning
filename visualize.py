import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation, PillowWriter
import pandas as pd
import numpy as np


def path_loader(filename="path.txt"):
    with open(filename) as f:
        lines = f.readlines()
    return [tuple(map(int, line.strip().split())) for line in lines]


def obstacle_loader(filename="obstacles_dynamic.txt"):
    return pd.read_csv(filename, sep=" ", names=["t", "x", "y", "z", "radius"])

path = path_loader()
obstacles = obstacle_loader()
max_time = max(max(t for t, _, _, _ in path), obstacles["t"].max())


scale_factor = 30

def create_animation(elev, azim, output_file):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    uav_plot, = ax.plot([], [], [], 'bo-', label="UAV Path")
    obstacle_plot = ax.scatter([], [], [], c='r', marker='o', alpha=0.7, label="Obstacles")

    def init():
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.set_zlim(0, 10)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("UAV Path Planning")
        ax.legend()
        ax.view_init(elev=elev, azim=azim)
        return uav_plot, obstacle_plot

    def update(frame):
        current_path = [(x, y, z) for t, x, y, z in path if t <= frame]
        if current_path:
            x, y, z = zip(*current_path)
            uav_plot.set_data(x, y)
            uav_plot.set_3d_properties(z)

        obs_frame = obstacles[obstacles["t"] == frame]
        if not obs_frame.empty:
            x = obs_frame["x"].values
            y = obs_frame["y"].values
            z = obs_frame["z"].values
            r = obs_frame["radius"].values
            sizes = (r * scale_factor) ** 2
            obstacle_plot._offsets3d = (x, y, z)
            obstacle_plot.set_sizes(sizes)

        else:
            obstacle_plot._offsets3d = ([], [], [])
            obstacle_plot.set_sizes([])
            ax.set_title(f"Time: {frame}")
        return uav_plot, obstacle_plot

    ani = FuncAnimation(fig, update, frames=max_time + 1, init_func=init, interval=500, blit=False)
    ani.save(output_file, writer=PillowWriter(fps=2))
    plt.close(fig)

# Define viewing angles (elevation, azimuth)
angles = [
    (90, 0),  
    (0, 0),   
    (45, 45),  
    (30, 120)   
]

# Generate animations
for elev, azim in angles:
    filename = f"{elev}.gif"
    print(f"Creating {filename}...")
    create_animation(elev, azim, filename)

print("All views saved!")
