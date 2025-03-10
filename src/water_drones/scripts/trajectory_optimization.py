import pandas as pd
import numpy as np
import casadi as ca
import sys
import os
from pathlib import Path



### Steps to go ###
# - Ask user which csv file to read in
#       directly search in /home/sfr/gz_ws/src/water_drones/data
#       maybe return the available files (not important)
# - Ask the user whether collision should be checked:
#       No:  - hand the csv file over to the controller (actually timed_pursuit_controller)
#       Yes: - check for collisions, return the result (collision or no collision)
#               - No Collision => Hand the original csv file over to the controller
#               - Collision => Do an optimization (MPC) and hand a optimized csv file over to the controller  


# Parameters
drone_radius = 0.3  # Define the minimum safety radius for drones
max_velocity = 2.0  # Max velocity constraint (m/s)
max_acceleration = 1.0  # Max acceleration constraint (m/s^2)

# def read_csv(file_path):
def read_csv():
    """Read the drone trajectory CSV file."""
    print("Which file do you want to read in? \nYou have the following options: \n")
    script_path = Path(__file__).resolve()
    data_folder = script_path.parent.parent / "data"
    files = [f.name for f in data_folder.iterdir() if f.is_file()]
    print(files)
    print("Please enter the entire name of the file you want to use from the list above")
    user_input = input()
    while not (Path.exists(data_folder / user_input)):
        print("Invalid input. Try again.")
        user_input = input()

    file_path = data_folder / user_input
    df = pd.read_csv(file_path)
    print("read in the csv file")
    return df

def check_collisions(df):
    """Check if there are any collisions in the given trajectory data."""
    collisions = []
    for t in df['timestamp'].unique():
        snapshot = df[df['timestamp'] == t]
        positions = snapshot[['x', 'y']].values
        for i in range(len(positions)):
            for j in range(i+1, len(positions)):
                if np.linalg.norm(positions[i] - positions[j]) < 2 * drone_radius:
                    collisions.append((t, snapshot.iloc[i]['drone_id'], snapshot.iloc[j]['drone_id']))
    return collisions

def safe_norm2(a, b, epsilon=1e-6):
    return ca.sqrt(a**2 + b**2 + epsilon)


def test_fun(df):
    # for t in range(1, len(df['timestamp'].unique())):
    #     print(df[(df['timestamp'] == df['timestamp'].unique()[t]) & (df['drone_id'] == 'drone_1')]['y'].values[0])
    print("Finished")
    return



def mpc_optimization(df):
    """Perform MPC-based collision avoidance."""
    timesteps = len(df['timestamp'].unique())
    drone_ids = df['drone_id'].unique()
    num_drones = len(drone_ids)
    delta_t = 3
    
    # Define optimization variables
    opti = ca.Opti()
    x = opti.variable(num_drones, timesteps)
    y = opti.variable(num_drones, timesteps)
    v_x = opti.variable(num_drones, timesteps)
    v_y = opti.variable(num_drones, timesteps)
    
    # Define cost function
    cost = 0


    for t in range(1, timesteps):
        for i in range(num_drones):
            # Motion model constraints
            opti.subject_to(x[i, t] == x[i, t-1] + v_x[i, t-1] * delta_t)
            opti.subject_to(y[i, t] == y[i, t-1] + v_y[i, t-1] * delta_t)

            # velocity constraints
            opti.subject_to(safe_norm2(v_x[i, t], v_y[i, t]) <= max_velocity)

            
            # Smoothness constraint (minimize acceleration)
            # if t > 1:
            #     delta_x = v_x[i, t] - v_x[i, t-1]
            #     delta_y = v_y[i, t] - v_y[i, t-1]
            #     # constraints.append(ca.norm_2([v_x[i, t] - v_x[i, t-1], v_y[i, t] - v_y[i, t-1]]) <= max_acceleration)
            #     constraints.append((np.sqrt(delta_x**2 + delta_y**2)) <= max_acceleration)
            
            # Cost function: deviation from original trajectory
            target_x = df[(df['timestamp'] == df['timestamp'].unique()[t]) & (df['drone_id'] == drone_ids[i])]['x'].values[0]
            target_y = df[(df['timestamp'] == df['timestamp'].unique()[t]) & (df['drone_id'] == drone_ids[i])]['y'].values[0]
            cost += ca.sumsqr(x[i, t] - target_x) + ca.sumsqr(y[i, t] - target_y)
            
        # Collision avoidance constraints
        for i in range(num_drones):
            for j in range(i+1, num_drones):
                delta_x = x[i, t] - x[j, t]
                delta_y = y[i, t] - y[j, t]
                epsilon = 1e-3  # A small margin
                opti.subject_to(safe_norm2(delta_x, delta_y) >= 2*drone_radius + epsilon)  
    
    print(f"opti is: \n{opti}")

    # minimize the cost function
    opti.minimize(cost)

    # solver settings
    opti.solver('ipopt')

    # solve
    sol = opti.solve()

    # extract the results
    x_solution = sol.value(x)
    y_solution = sol.value(y)
    v_x_solution = sol.value(v_x)
    v_y_solution = sol.value(v_y)

    # Solve optimization problem
    # opt_variables = ca.vertcat(x.reshape((-1, 1)), y.reshape((-1, 1)), v_x.reshape((-1, 1)), v_y.reshape((-1, 1)))
    
    # nlp = {'x': opt_variables, 'f': cost, 'g': ca.vertcat(*constraints)}
    # solver = ca.nlpsol('solver', 'ipopt', nlp)
    # solution = solver(lbx=-ca.inf, ubx=ca.inf, lbg=0, ubg=ca.inf)
    
    # x_solution = np.array(solution['x']).reshape((num_drones, timesteps, 4))[:, :, 0]
    # y_solution = np.array(solution['x']).reshape((num_drones, timesteps, 4))[:, :, 1]
    # v_x_solution = np.array(solution['x']).reshape((num_drones, timesteps, 4))[:, :, 2]
    # v_y_solution = np.array(solution['x']).reshape((num_drones, timesteps, 4))[:, :, 3]
    df['v_x'] = timesteps*num_drones*[None]
    df['v_y'] = timesteps*num_drones*[None]
    
    # Update dataframe with new values
    for i, drone_id in enumerate(drone_ids):
        df.loc[df['drone_id'] == drone_id, 'x'] = x_solution[i]
        df.loc[df['drone_id'] == drone_id, 'y'] = y_solution[i]
        df.loc[df['drone_id'] == drone_id, 'v_x'] = v_x_solution[i]
        df.loc[df['drone_id'] == drone_id, 'v_y'] = v_y_solution[i]
    
    return df

# def main(input_file, output_file):
def main():

    df = read_csv()
    print("Do you want to analyse whether there is a collision in your file or not?")
    input_var = input('y/n? \n')

    if input_var == 'y':
        print("Collisions will be checked")
        collisions = check_collisions(df)
        if collisions:
            print("Collisions detected! Running MPC...")
            df = mpc_optimization(df)
            df_name = input('Define a name for the adjusted csv file (with .csv). \n')
            file_path = Path(__file__).resolve().parent.parent / "data" / df_name
            df.to_csv(file_path, index=False)
        else:
            print("No collisions detected.")
    else:
        print("csv will be handed directly to path controller")


    # df = read_csv(input_file)
    # collisions = check_collisions(df)
    
    # if collisions:
    #     print("Collisions detected! Running MPC...")
    #     df = mpc_optimization(df)
    #     # test_fun(df)
    # else:
    #     print("No collisions detected.")
    
    # df.to_csv(output_file, index=False)
    # print("Updated trajectory saved to", output_file)


if __name__ == "__main__":
    # if len(sys.argv) != 3:
    #     print("Usage: python3 file.py <arg1> <arg2>")
    #     sys.exit(1)
    
    # arg1 = sys.argv[1]
    # arg2 = sys.argv[2]
    # main(arg1, arg2)
    main()

# Example usage
# main("input.csv", "output.csv")
