import matplotlib.pyplot as plt
import sys

# Function to parse the data from the text file
def parse_data(file_path):
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('#'): # Skip comment lines
                continue
            values = line.strip().split(',')
            data.append([float(value) for value in values])
    return data

# Function to plot the data
def plot_data(data):
    # Group data by k and j values
    groups = {}
    for entry in data:
        k, j, i, vx_max, vx_std, wz_max, wz_std, vx_in, wz_in, cmd_vel_vx, cmd_vel_wz = entry
        key = (k, j, vx_max, vx_std, wz_max, wz_std)
        if key not in groups:
            groups[key] = []
        groups[key].append(entry)

    # Plot each group
    colormap = plt.get_cmap('viridis')
    num_groups = len(groups)
    for index,(key, group) in enumerate(groups.items()):
        k, j, vx_max, vx_std, wz_max, wz_std = key
        i_values = [entry[2] for entry in group]
        cmd_vel_vx_values = [entry[9] for entry in group]
        color = colormap(index / num_groups) # Normalize index to colormap range
        plt.plot(i_values, cmd_vel_vx_values, marker='o', color=color,
                 label=f'vx_max={vx_max}, vx_std={vx_std}, wz_max={wz_max}, wz_std={wz_std}')


    plt.xlabel('Iterations of optimizer->evalControl (model_dt=0.05, 4s)')
    plt.ylabel('cmd_vel_vx')
    plt.title('Plot of cmd_vel_vx response varying vx_max and vx_std')
    plt.tight_layout(rect=[0, 0, 0.75, 1])
    # Put a legend to the right of the current axis
    plt.legend(bbox_to_anchor=(1.04, 1), loc="upper left")
    plt.show()

# Main execution
if __name__ == "__main__":
    data_file = sys.argv[1]
    data = parse_data(data_file)
    plot_data(data)

