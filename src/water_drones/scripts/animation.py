import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pathlib import Path

def interpolate_positions(df, time_step=0.1):
    """Interpolates drone positions to have a point every `time_step` seconds."""
    new_data = []
    for drone_id in df['drone_id'].unique():
        drone_data = df[df['drone_id'] == drone_id].sort_values(by=['timestamp'])
        for i in range(len(drone_data) - 1):
            t1, t2 = drone_data.iloc[i]['timestamp'], drone_data.iloc[i + 1]['timestamp']
            x1, x2 = drone_data.iloc[i]['x'], drone_data.iloc[i + 1]['x']
            y1, y2 = drone_data.iloc[i]['y'], drone_data.iloc[i + 1]['y']
            
            times = np.arange(t1, t2, time_step)
            for t in times:
                alpha = (t - t1) / (t2 - t1)
                new_x = x1 + alpha * (x2 - x1)
                new_y = y1 + alpha * (y2 - y1)
                new_data.append([t, drone_id, new_x, new_y])
    
    new_df = pd.DataFrame(new_data, columns=['timestamp', 'drone_id', 'x', 'y'])
    return new_df

def animate_drones():
    # Load data from fixed file location
    script_path = Path(__file__).resolve()
    file = script_path.parent.parent / "data" / "out.csv"
    df = pd.read_csv(file)
    
    # Ensure timestamp is sorted
    df = df.sort_values(by=['timestamp'])
    
    # Interpolate positions to have points every 0.1 second
    df = interpolate_positions(df, time_step=0.1)
    
    # Get unique drone IDs
    drone_ids = df['drone_id'].unique()
    
    # Normalize timestamps to start from zero
    df['timestamp'] = df['timestamp'] - df['timestamp'].min()
    
    # Create figure and axis
    fig, ax = plt.subplots()
    ax.set_xlim(df['x'].min() - 10, df['x'].max() + 10)
    ax.set_ylim(df['y'].min() - 10, df['y'].max() + 10)
    ax.set_title("Drone Position Animation")
    ax.set_xlabel("Position X")
    ax.set_ylabel("Position Y")
    
    # Define dot size in axis units (0.6 meters diameter)
    dot_diameter = 0.6
    dot_size = (dot_diameter / (ax.get_xlim()[1] - ax.get_xlim()[0])) * fig.get_size_inches()[0] * fig.dpi
    
    # Create a dictionary to store scatter plots for each drone with size adjustment
    drone_plots = {drone_id: ax.scatter([], [], s=dot_size**2, label=f"Drone {drone_id}") for drone_id in drone_ids}
    ax.legend()
    
    timestamps = df['timestamp'].unique()
    
    def update(frame):
        timestamp = timestamps[frame]
        current_data = df[df['timestamp'] == timestamp]
        for drone_id in drone_ids:
            drone_data = current_data[current_data['drone_id'] == drone_id]
            if not drone_data.empty:
                drone_plots[drone_id].set_offsets(np.c_[drone_data['x'], drone_data['y']])
        return drone_plots.values()
    
    ani = animation.FuncAnimation(fig, update, frames=len(timestamps), interval=25, blit=True)
    plt.show()

def main():
    animate_drones()

if __name__ == "__main__":
    main()

# Example usage:
# animate_drones("drones_data.csv")
