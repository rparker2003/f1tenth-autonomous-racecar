import numpy as np

def interpolate_smooth(file_path, num_points=10):
    # Read the waypoints from the file
    with open(file_path, 'r') as file:
        waypoints = file.readlines()

    # Extract the values from the first and last rows
    first_row = np.array([float(value) for value in waypoints[0].strip().split(', ')])
    last_row = np.array([float(value) for value in waypoints[-1].strip().split(', ')])

    # Interpolate between the first and last rows
    interpolated_rows = []
    for i in range(1, num_points + 1):
        alpha = i / (num_points + 1)
        interpolated_values = (1 - alpha) * first_row + alpha * last_row
        interpolated_rows.append(', '.join(map(str, interpolated_values)) + '\n')

    # Update the intermediate rows
    waypoints = waypoints[:1] + interpolated_rows + waypoints[1:]

    # Write the updated waypoints back to the file
    with open(file_path, 'w') as file:
        file.writelines(waypoints)

if __name__ == "__main__":
    file_path = "/sim_ws/src/pure_pursuit/waypoints/waypoints.csv"
    interpolate_smooth(file_path, num_points=10)
    print("Done smoothing waypoints!")