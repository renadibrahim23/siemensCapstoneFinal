import pandas as pd
import matplotlib.pyplot as plt
import os

# Load the data
file_path = "results_test_run_circular_path.csv"
data = pd.read_csv(file_path)

# Plot configurations
plots = [
    {"x": "time_s", "y": "pos_x", "title": "Position X vs Time", "xlabel": "Time (s)", "ylabel": "Position X"},
    {"x": "time_s", "y": "pos_y", "title": "Position Y vs Time", "xlabel": "Time (s)", "ylabel": "Position Y"},
    {"x": "time_s", "y": "theta", "title": "Theta vs Time", "xlabel": "Time (s)", "ylabel": "Theta"},
    {"x": "time_s", "y": "lateral_error", "title": "Lateral Error vs Time", "xlabel": "Time (s)", "ylabel": "Lateral Error"},
    {"x": "time_s", "y": "angular_w", "title": "Angular Velocity vs Time", "xlabel": "Time (s)", "ylabel": "Angular Velocity"},
]

# Get all CSV files in the current directory
csv_files = [f for f in os.listdir('.') if f.endswith('.csv')]

for file_path in csv_files:
    print(f"Processing file: {file_path}")
    data = pd.read_csv(file_path)

    # Generate time-series plots
    for plot in plots:
        plt.figure()
        plt.plot(data[plot["x"]], data[plot["y"]])
        plt.title(plot["title"])
        plt.xlabel(plot["xlabel"])
        plt.ylabel(plot["ylabel"])
        plt.grid()
        output_file = f"{os.path.splitext(file_path)[0]}_{plot['y']}_vs_{plot['x']}.png"
        plt.savefig(output_file)
        plt.show()

    # Generate trajectory plot
    plt.figure()
    plt.plot(data["pos_x"], data["pos_y"], label="Trajectory")
    plt.title("Trajectory (Position X vs Position Y)")
    plt.xlabel("Position X")
    plt.ylabel("Position Y")
    plt.grid()
    plt.legend()
    output_file = f"{os.path.splitext(file_path)[0]}_trajectory_plot.png"
    plt.savefig(output_file)
    plt.show()