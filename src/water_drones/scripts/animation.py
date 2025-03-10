import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pathlib import Path


def animate_drones():

    script_path = Path(__file__).resolve()
    file = script_path.parent.parent / "data" / "adjusted.csv"
    # Load data
    df = pd.read_csv(file)
    
    # Ensure timestamp is sorted
    df = df.sort_values(by=['timestamp'])
    
    # Get unique drone IDs
    drone_ids = df['drone_id'].unique()
    
    # Create figure and axis
    fig, ax = plt.subplots()
    ax.set_xlim(df['x'].min() - 10, df['x'].max() + 10)
    ax.set_ylim(df['y'].min() - 10, df['y'].max() + 10)
    ax.set_title("Drone Position Animation")
    ax.set_xlabel("Position X")
    ax.set_ylabel("Position Y")
    
    # Create a dictionary to store scatter plots for each drone
    drone_plots = {drone_id: ax.plot([], [], 'o', label=f"Drone {drone_id}")[0] for drone_id in drone_ids}
    ax.legend()
    
    def update(frame):
        timestamp = df['timestamp'].unique()[frame]
        current_data = df[df['timestamp'] == timestamp]
        for drone_id in drone_ids:
            drone_data = current_data[current_data['drone_id'] == drone_id]
            if not drone_data.empty:
                drone_plots[drone_id].set_data(drone_data['x'], drone_data['y'])
        return drone_plots.values()
    
    ani = animation.FuncAnimation(fig, update, frames=len(df['timestamp'].unique()), interval=200, blit=True)
    plt.show()

def main():
    animate_drones()

if __name__ == "__main__":
    main()

# Example usage:
# animate_drones("drones_data.csv")
