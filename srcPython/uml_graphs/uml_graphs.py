from graphviz import Digraph

# Create a new directed graph for the DFD
dfd = Digraph(comment='System DFD')

# Define graph properties for a larger layout
dfd.attr(rankdir='LR', size='30,30')

# Increase the default font size for nodes and edges
dfd.attr('node', fontsize='20')
dfd.attr('edge', fontsize='18')

# Define styles for different types of nodes with increased size and margin for better visibility
dfd.attr('node', shape='ellipse', style='filled', color='#619bcc', width='1', height='1', margin='0.2,0.2')

# External Entities
dfd.node('Simtwo', 'Robotics\nSimulator')
dfd.node('YOLOv8', 'YOLOv8.1\nDetection')

# Data Stores with specific styling adjustments
dfd.node('Log', 'Data Logs', shape='cylinder', color='gray', width='1.5', height='1.25', margin='0.2,0.2')

# Processes with specific styling adjustments
dfd.attr('node', shape='rectangle', style='filled', color='orange', width='1.5', height='1.25', margin='0.2,0.2')
dfd.node('Manager', 'Manager')
dfd.node('Localization', 'Localization')
dfd.node('Visualizer', 'Visualizer')
dfd.node('SimtwoInterface', 'Simtwo\nInterface')
dfd.node('PerfectMatch', 'PM\nRoPM')
dfd.node('EKF', 'EKF')
dfd.node('LoggerData', 'Logger')

# Data Flows with adjusted attributes for clarity
dfd.edge('Simtwo', 'SimtwoInterface', '(Encs, GT) 40Hz\n(LIDAR) 7 Hz')
dfd.edge('Simtwo', 'YOLOv8', '(RGB)\n15 Hz')

dfd.edge('SimtwoInterface', 'Manager', '(Encs, GT, LIDAR) 40 Hz\n(RGB) 15 Hz')

dfd.edge('Manager', 'Localization', '(Encs, GT, LIDAR) 40 Hz\n(RGB) 15 Hz')
dfd.edge('Manager', 'LoggerData', '(Encs, GT, timestamp) 40Hz\n(LIDAR) 7 Hz\n(Inference metadata) 15 Hz')
dfd.edge('Manager', 'Visualizer', '(EKF μ, EKF Out. μ, GT) 40 Hz\n(LIDAR) 7 Hz')

dfd.edge('Localization', 'PerfectMatch', '(LIDAR, LIDAR Out.) 15 Hz')
dfd.edge('Localization', 'EKF', '(Odo) 40 Hz\n(PM Pose, PM Out. Pose) 15 Hz')
dfd.edge('Localization', 'Manager', '(EKF μ, EKF Out. μ) 40 Hz')

dfd.edge('PerfectMatch', 'Localization', '(PM Pose, Pm Out. Pose) 15 Hz\n(PM Error, PM Out. Error) 15 Hz')

dfd.edge('EKF', 'Localization', '(EKF μ, EKF Out. μ) 40 Hz')

dfd.edge('YOLOv8', 'SimtwoInterface', 'Inference metadata\n15 Hz')

dfd.edge('LoggerData', 'Log', '(Encs, GT, timestamp) 40Hz\n(LIDAR) 7 Hz\n(Inference metadata) 15 Hz')

# Save the DFD to a file with increased visibility settings
file_path = 'system_dfd_v2'
dfd.render(file_path, format='png', cleanup=True)
