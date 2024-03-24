import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import pandas as pd

def update_lines(num, data, line):
    line.set_data(data[0:2, :num])
    line.set_3d_properties(data[2, :num])
    return line

# Attaching 3D axis to the figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Reading the data from a CSV file using pandas
repo = pd.read_csv('~/Tech516/lab7/lab7_code/data.csv', sep=',', header=0)
data = np.array((repo['x'].values, repo['y'].values, repo['z'].values))
print("Number of data points: ", data.shape[1])

# Creating an empty line object
line, = ax.plot([], [], [])

# Setting the axes properties with units
ax.set_xlabel('X (units)')
ax.set_ylabel('Y (units)')
ax.set_zlabel('Z (units)')
ax.set_title('3D Position Visualizer')

# Adjusting axis limits dynamically
ax.set_xlim3d(np.min(data[0]), np.max(data[0]))
ax.set_ylim3d(np.min(data[1]), np.max(data[1]))
ax.set_zlim3d(np.min(data[2]), np.max(data[2]))

# Adding grid and labels
ax.grid(True)
ax.xaxis.pane.fill = False
ax.yaxis.pane.fill = False
ax.zaxis.pane.fill = False

# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_lines, data.shape[1], fargs=(data, line), interval=200, blit=False)

plt.show()

