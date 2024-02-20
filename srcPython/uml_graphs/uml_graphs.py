from graphviz import Digraph

# Create a new directed graph for the DFD
dfd = Digraph(comment='System DFD')

# Define graph properties
dfd.attr(rankdir='LR', size='30,30')

# Define styles for different types of nodes
dfd.attr('node', shape='ellipse', style='filled', color='#619bcc')

# External Entities
dfd.node('Simtwo', 'Robotics\nSimulator')
dfd.node('YOLOv8', 'YOLOv8.1\nDetection')

# Data Stores
dfd.node('Log', 'Data Logs', shape='cylinder', color='gray')

# Processes
dfd.attr('node', shape='rectangle', style='filled', color='orange')
dfd.node('Manager', 'Manager')
dfd.node('Localization', 'Localization')
dfd.node('Visualizer', 'Visualizer')
dfd.node('SimtwoInterface', 'Simtwo\nInterface')
# dfd.node('OfflineAnalysis', 'OfflineAnalysis')
dfd.node('PerfectMatch', 'Perfect Match')
dfd.node('EKF', 'EKF')
dfd.node('LoggerData', 'Logger')

# Data Flows
dfd.edge('Simtwo', 'SimtwoInterface', 'Sensor Data\n(40 Hz)')
dfd.edge('Simtwo', 'YOLOv8', 'RGB Data\n(15 Hz)')
dfd.edge('SimtwoInterface', 'Manager', 'Parsed Sensor\nData')
dfd.edge('Manager', 'Localization', 'Parsed Sensor\nData')
dfd.edge('Localization', 'Manager', 'Localization\nData')
dfd.edge('Manager', 'Visualizer', 'Visualization\nData')
# dfd.edge('Manager', 'OfflineAnalysis', 'Log File\nPath')
dfd.edge('Localization', 'PerfectMatch', 'Laser and\nBounding Boxes data')
dfd.edge('Localization', 'EKF', 'Pose\ndata')
dfd.edge('PerfectMatch', 'Localization', 'Filtered Matched\nPose Data')
dfd.edge('EKF', 'Localization', 'Filtered Pose\nData')
dfd.edge('Localization', 'LoggerData', 'Localization\nLog')
dfd.edge('Visualizer', 'LoggerData', 'Visualization\nLog')
# dfd.edge('OfflineAnalysis', 'Manager', 'Parsed Sensor\nData')
dfd.edge('YOLOv8', 'SimtwoInterface', 'Object Detection\nData')
dfd.edge('LoggerData', 'Log', 'Data logs')

# Save the DFD to a file
file_path = 'system_dfd_v2'
dfd.render(file_path, format='png', cleanup=True)
