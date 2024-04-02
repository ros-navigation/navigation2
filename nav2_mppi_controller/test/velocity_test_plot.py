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

    # Create a figure with two subplots
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1)


    # Plot each group - first cmd_vel_vx
    colormap = plt.get_cmap('viridis')
    num_groups = len(groups)
    for index,(key, group) in enumerate(groups.items()):
        k, j, vx_max, vx_std, wz_max, wz_std = key
        i_values = [entry[2] for entry in group]
        cmd_vel_vx_values = [entry[9] for entry in group]
        color = colormap(index / num_groups) # Normalize index to colormap range
        ax1.plot(i_values, cmd_vel_vx_values, marker='o', color=color,
                 label=f'vx_max={vx_max}, vx_std={vx_std}, wz_max={wz_max}, wz_std={wz_std}')


    ax1.set_xlabel('Iterations of optimizer->evalControl (model_dt=0.05, 4s)')
    ax1.set_ylabel('cmd_vel_vx')
    ax1.set_title('Plot of cmd_vel_vx response varying vx_max and vx_std, and v_in.linear.x')
    # Put a legend to the right of the current axis
    #ax1.legend(bbox_to_anchor=(1.04, 1), loc="upper left")

    # Plot each group - second cmd_vel_wz
    colormap = plt.get_cmap('viridis')
    num_groups = len(groups)
    for index,(key, group) in enumerate(groups.items()):
        k, j, vx_max, vx_std, wz_max, wz_std = key
        i_values = [entry[2] for entry in group]
        cmd_vel_vx_values = [entry[10] for entry in group]
        color = colormap(index / num_groups) # Normalize index to colormap range
        ax2.plot(i_values, cmd_vel_vx_values, marker='o', color=color,
                 label=f'vx_max={vx_max}, vx_std={vx_std}, wz_max={wz_max}, wz_std={wz_std}')


    ax2.set_xlabel('Iterations of optimizer->evalControl (model_dt=0.05, 4s)')
    ax2.set_ylabel('cmd_vel_wz')
    ax2.set_title('Plot of cmd_vel_wz response varying vx_max and vx_std, and v_in.linear.x')
    # Put a legend to the right of the current axis
    #ax2.legend(bbox_to_anchor=(1.04, 1), loc="upper left")
    # Place the legend outside the plot
    ax1.legend(handles=ax1.get_legend_handles_labels()[0],
               labels=ax1.get_legend_handles_labels()[1],
               loc='upper left', bbox_to_anchor=(1.02, 1))

    # Plot each group - second cmd_vel_wz
    colormap = plt.get_cmap('viridis')
    num_groups = len(groups)
    for index,(key, group) in enumerate(groups.items()):
        k, j, vx_max, vx_std, wz_max, wz_std = key
        i_values = [entry[2] for entry in group]
        vx_in_values = [entry[7] for entry in group]
        color = colormap(index / num_groups) # Normalize index to colormap range
        ax3.plot(i_values, vx_in_values, marker='o', color=color,
                 label=f'vx_max={vx_max}, vx_std={vx_std}, wz_max={wz_max}, wz_std={wz_std}')
    ax3.set_xlabel('Iterations of optimizer->evalControl (model_dt=0.05, 4s)')
    ax3.set_ylabel('vx_in')
    ax3.set_title('Plot of vx_in vs iteration')
    fig.tight_layout(rect=(0.0, 0.0, 0.70, 0.7))
    #fig.canvas.draw()
    #fig.tight_layout(rect=[0, 0, 0.5, 0.5])

    plt.show()
    plt.tight_layout()

# Main execution
if __name__ == "__main__":
    data_file = sys.argv[1]
    data = parse_data(data_file)
    plot_data(data)

