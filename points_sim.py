# Example of input.txt:

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import plotly.graph_objects as go
from points_utils import Points, STEPS_PER_CM

string = [l.strip('\n') for l in open('input.txt', 'r').readlines()]

# Parse coordinates, times and calculate speeds
coords = {}
current_coord = None
x_points, y_points, z_points = [], [], []
x_times, y_times, z_times = [], [], []
x_speeds, y_speeds, z_speeds = [], [], []
x_speeds2, y_speeds2, z_speeds2 = [], [], []
colors = []  # Store colors based on vacuum state

CM_PER_STEPS = 1 / STEPS_PER_CM

# Initialize first values 
x, y, z = 0, 0, 0

def simulate_points(points: Points):
    for i in range(points.length):
        x = points.x[i] * CM_PER_STEPS
        y = points.y[i] * CM_PER_STEPS
        z = points.z[i] * CM_PER_STEPS
        # direction = int(line[1])  # First digit is direction
        # vacuum_state = int(line[2])  # Second digit is vacuum state
        # colors.append('red' if vacuum_state == 1 else 'blue')

        x_points.append(x + y)
        y_points.append(y - x)
        z_points.append(z - x)

        # x_speeds.append(points.vx[i])
        # y_speeds.append(points.vy[i])
        # z_speeds.append(points.vz[i])
    #             speed = step_to_cm / dt if dt > 0 else 0
    #             x_speeds.append((1 if direction == 1 else -1) * speed)

    #     elif current_coord == 'Y':
    #         y += (1 if direction == 1 else -1) * step_to_cm
    #         x -= (1 if direction == 1 else -1) * step_to_cm
    #         y_points.append(y)
    #         x_points.append(x)
    #         z_points.append(z)
    #         y_times.append(time)
    #         if len(y_times) > 1:
    #             dt = y_times[-1] - y_times[-2]
    #             speed = step_to_cm / dt if dt > 0 else 0
    #             y_speeds.append((1 if direction == 1 else -1) * speed)

    #     elif current_coord == 'Z':
    #         z += (1 if direction == 1 else -1) * step_to_cm
    #         z_points.append(z)
    #         x_points.append(x)
    #         y_points.append(y)
    #         z_times.append(time)
    #         if len(z_times) > 1:
    #             dt = z_times[-1] - z_times[-2]
    #             speed = step_to_cm / dt if dt > 0 else 0
    #             z_speeds.append((1 if direction == 1 else -1) * speed)

    # elif line[0].isdigit() or line[1].isdigit() and "." in line:
    #     line = float(line)
    #     # Convert steps/s to cm/s
    #     speed_cm = line * step_to_cm
    #     if current_coord == 'Z':
    #         z_speeds2.append(speed_cm)
    #     elif current_coord == 'Y':
    #         y_speeds2.append(speed_cm)
    #     elif current_coord == 'X':
    #         x_speeds2.append(speed_cm)

# Create interactive 3D plot using plotly

    # print(*points.vz.items(), sep="\n")
    # print(points.vz)

    points.vx = {k / 1000: v * CM_PER_STEPS for k, v in points.vx.items()}
    points.vy = {k / 1000: v * CM_PER_STEPS for k, v in points.vy.items()}
    points.vz = {k / 1000: v * CM_PER_STEPS for k, v in points.vz.items()}

    X_HIGH = max(list(points.vx.keys()) + list(points.vy.keys()) + list(points.vz.keys())) + 0.1
    Y_HIGH = max(list(points.vx.values()) + list(points.vy.values()) + list(points.vz.values())) + 0.1
    Y_LOW = min(list(points.vx.values()) + list(points.vy.values()) + list(points.vz.values())) - 0.1

    fig_3d = go.Figure(data=[go.Scatter3d(
        x=x_points,
        y=y_points,
        z=z_points,
        mode='lines+markers',
        marker=dict(
            size=2,
            color=colors,  # Use colors list for coloring points
        ),
        line=dict(
            width=2,
            color=colors  # Use same colors for lines
        )
    )])

    fig_3d.update_layout(
        title='3D Trajectory',
        scene=dict(
            xaxis_title='X Position (cm)',
            yaxis_title='Y Position (cm)',
            zaxis_title='Z Position (cm)',
            aspectmode='cube'  # Force equal aspect ratio
        ),
        width=1000,  # Increase width to better show 3D plot
        height=1000, # Increase height to match width
        showlegend=False
    )

    fig_3d.show()

    # Create speed plots
    plt.figure(figsize=(15, 8))

    # Velocity plots
    plt.subplot(231)
    plt.plot(points.vx.keys(), points.vx.values(), 'r--', label='X Speed')
    plt.axhline(y=3/110.65, color='k', linestyle='-', alpha=0.5)
    plt.axhline(y=-3/110.65, color='k', linestyle='-', alpha=0.5)
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (cm/s)')
    plt.title('X Axis Speed')
    plt.ylim(Y_LOW, Y_HIGH)
    plt.xlim(-0.2, X_HIGH)
    plt.grid(True)

    plt.subplot(232)
    plt.plot(points.vy.keys(), points.vy.values(), 'g--', label='Y Speed')
    plt.axhline(y=3/110.65, color='k', linestyle='-', alpha=0.5)
    plt.axhline(y=-3/110.65, color='k', linestyle='-', alpha=0.5)
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (cm/s)')
    plt.title('Y Axis Speed')
    plt.ylim(Y_LOW, Y_HIGH)
    plt.xlim(-0.2, X_HIGH)
    plt.grid(True)

    plt.subplot(233)
    plt.plot(points.vz.keys(), points.vz.values(), 'b--', label='Z Speed')
    plt.axhline(y=3/110.65, color='k', linestyle='-', alpha=0.5)
    plt.axhline(y=-3/110.65, color='k', linestyle='-', alpha=0.5)
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (cm/s)')
    plt.title('Z Axis Speed')
    plt.ylim(Y_LOW, Y_HIGH)
    plt.xlim(-0.2, X_HIGH)
    plt.grid(True)

    # Position plots
    plt.subplot(234)
    plt.plot(range(len(x_points)), x_points, 'r-', label='X Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (cm)')
    plt.title('X Axis Position')
    plt.xlim(-0.2, X_HIGH)
    plt.grid(True)

    plt.subplot(235)
    plt.plot(range(len(y_points)), y_points, 'g-', label='Y Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (cm)')
    plt.title('Y Axis Position')
    plt.xlim(-0.2, X_HIGH)
    plt.grid(True)

    plt.subplot(236)
    plt.plot(range(len(z_points)), z_points, 'b-', label='Z Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (cm)')
    plt.title('Z Axis Position')
    plt.xlim(-0.2, X_HIGH)
    plt.grid(True)

    plt.tight_layout()
    plt.show()
