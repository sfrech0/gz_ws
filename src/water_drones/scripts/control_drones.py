#!/usr/bin/env python3
"""
control_drones.py

This script reads a CSV file containing drone commands and sends position and velocity commands
to the corresponding Ardupilot instances via MAVLink.

CSV format (each row):
    timestamp, drone_id, x, y

Commands are linearly interpolated so the drones move smoothly. The script sets each drone into
guided mode and sends commands with the proper system IDs.
"""

import csv
import time
from collections import defaultdict
from pymavlink import mavutil

# ----- Configuration -----

CSV_FILENAME = "/home/sfr/gz_ws/src/water_drones/data/adjusted.csv"
UPDATE_RATE = 10.0    # Hz
DT = 1.0 / UPDATE_RATE

# Mapping from drone_id (as given in CSV) to MAVLink connection strings.
DRONE_CONNECTIONS = {
    "drone_1": "udpout:localhost:14540",
    "drone_2": "udpout:localhost:14541"
}

# Mapping from drone_id to its MAVLink system id (vehicle_id in Ardupilot)
DRONE_SYSTEM_IDS = {
    "drone_1": 1,
    "drone_2": 2
}

# Set type mask to ignore acceleration and yaw fields (only position and velocity used).
TYPE_MASK = (1 << 10) | (1 << 11) | (1 << 12) | (1 << 14) | (1 << 15)
COORDINATE_FRAME = mavutil.mavlink.MAV_FRAME_LOCAL_NED

# We fix z (altitude) at 0 for this example.
FIXED_Z = 0.0

# ----- CSV Loading and Interpolation Functions -----

def load_commands(filename):
    """
    Reads the CSV file and returns a dict:
       { drone_id: [(timestamp, x, y), ...] }
    """
    commands = defaultdict(list)
    with open(filename, newline='') as csvfile:
        reader = csv.reader(csvfile)
        header = next(reader)
        try:
            float(header[0])
            # first row is not a header; process it
            row = header
            timestamp, drone_id, x, y = float(row[0]), str(row[1]), float(row[2]), float(row[3])
            commands[drone_id].append((timestamp, x, y))
        except ValueError:
            # header exists; skip it
            pass
        for row in reader:
            if len(row) < 4:
                continue
            try:
                timestamp = float(row[0])
                drone_id = str(row[1])
                x = float(row[2])
                y = float(row[3])
                commands[drone_id].append((timestamp, x, y))
            except ValueError:
                continue
    for drone_id in commands:
        commands[drone_id].sort(key=lambda tup: tup[0])
    return commands

def interpolate_command(cmd_list, current_time):
    """
    Given a sorted list of (timestamp, x, y) and current_time,
    return interpolated (position, velocity) as ((x, y), (vx, vy)).
    """
    if current_time <= cmd_list[0][0]:
        t0, x0, y0 = cmd_list[0]
        if len(cmd_list) > 1:
            t1, x1, y1 = cmd_list[1]
            dt = t1 - t0 if (t1 - t0) != 0 else 1.0
            vx = (x1 - x0) / dt
            vy = (y1 - y0) / dt
        else:
            vx = vy = 0.0
        return (x0, y0), (vx, vy)
    if current_time >= cmd_list[-1][0]:
        return (cmd_list[-1][1], cmd_list[-1][2]), (0.0, 0.0)
    for i in range(len(cmd_list) - 1):
        t0, x0, y0 = cmd_list[i]
        t1, x1, y1 = cmd_list[i + 1]
        if t0 <= current_time <= t1:
            ratio = (current_time - t0) / (t1 - t0)
            x = x0 + ratio * (x1 - x0)
            y = y0 + ratio * (y1 - y0)
            dt = t1 - t0 if (t1 - t0) != 0 else 1.0
            vx = (x1 - x0) / dt
            vy = (y1 - y0) / dt
            return (x, y), (vx, vy)
    return (cmd_list[-1][1], cmd_list[-1][2]), (0.0, 0.0)

# ----- MAVLink Helper Functions -----

def set_guided_mode(connection, system_id):
    """
    Sends a command to switch the drone into guided mode so that it will accept external commands.
    """
    print(f"Setting guided mode for system {system_id}...")
    connection.mav.command_long_send(
        system_id,      # target_system
        1,              # target_component (usually 1)
        mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE,
        0,              # confirmation
        1,              # param1: 1 to enable guided mode
        0, 0, 0, 0, 0, 0
    )
    # Allow a brief time for the mode switch to take effect.
    time.sleep(1)

def send_mavlink_command(connection, pos, vel, time_boot_ms, system_id):
    """
    Sends a SET_POSITION_TARGET_LOCAL_NED message with the given position and velocity.
    """
    connection.mav.set_position_target_local_ned_send(
        time_boot_ms,
        system_id,      # target_system (set to vehicle's system ID)
        1,              # target_component
        COORDINATE_FRAME,
        TYPE_MASK,
        pos[0], pos[1], FIXED_Z,   # positions (x, y, z)
        vel[0], vel[1], 0.0,        # velocities (vx, vy, vz)
        0, 0, 0,                  # accelerations (ignored)
        0, 0                      # yaw and yaw_rate (ignored)
    )

def send_mavlink_new(connection, pos, vel, time_boot_ms, system_id):
    """
    Sends a SET_POSITION_TARGET_LOCAL_NED message with the given position and velocity.
    """
    message = connection.set_position_target_local_ned(
        time_boot_ms,
        system_id,
        1,                        # target component
        COORDINATE_FRAME,
        TYPE_MASK, 
        pos[0], pos[1], FIXED_Z,  # positions (x, y, z)
        vel[0], vel[1], 0.0,      # velocities (vx, vy, vz)
        0, 0, 0,                  # accelerations (ignored)
        0, 0                      # yaw and yaw_rate (ignored)
    )
    connection.mav.send(message)


# ----- Main Control Loop -----

def main():
    # Load CSV commands.
    commands = load_commands(CSV_FILENAME)
    if not commands:
        print("No commands loaded. Exiting.")
        return

    # Connect to each drone and set its system ID and guided mode.
    drone_links = {}
    for drone_id, connection_string in DRONE_CONNECTIONS.items():
        print(f"Connecting to drone {drone_id} on {connection_string}...")
        conn = mavutil.mavlink_connection(connection_string)
        # conn.wait_heartbeat(timeout=30)
        conn.wait_heartbeat()
        system_id = DRONE_SYSTEM_IDS.get(drone_id, 1)
        conn.target_system = system_id
        print(f"Drone {drone_id} connected with system ID {system_id}.")

        # necessary ?????????????
        set_guided_mode(conn, system_id)
        drone_links[drone_id] = conn

    # Determine simulation time range from CSV commands.
    start_time = min(cmds[0][0] for cmds in commands.values())
    end_time = max(cmds[-1][0] for cmds in commands.values())
    print(f"Simulation will run from t={start_time:.2f} to t={end_time:.2f} seconds.")

    # Reset simulation clock so it starts at 0 now.
    sim_start = time.time()

    try:
        while True:
            # Use real time elapsed since sim_start.
            current_sim_time = time.time() - sim_start
            # Check if we have reached the end of the command timeline.
            if current_sim_time > (end_time - start_time):
                print("Reached end of simulation commands.")
                break

            # For each drone, interpolate the desired state and send a MAVLink command.
            for drone_id, cmd_list in commands.items():
                # Use (current_sim_time + start_time) to map real elapsed time into CSV timeline.
                pos, vel = interpolate_command(cmd_list, current_sim_time + start_time)
                time_boot_ms = int((time.time() - sim_start) * 1000)
                if drone_id in drone_links:
                    send_mavlink_new(drone_links[drone_id], pos, vel, time_boot_ms,
                                         drone_links[drone_id].target_system)
                    print(f"Sent to {drone_id}: pos={pos}, vel={vel}, sim_t={current_sim_time:.2f}")
                else:
                    print(f"No MAVLink connection for drone_id {drone_id}")
            time.sleep(DT)
    except KeyboardInterrupt:
        print("Interrupted by user. Exiting.")

if __name__ == "__main__":
    main()
