from graphviz import Digraph

# Create a new directed graph for the DFD
dfd = Digraph(comment="System DFD")

# Define graph properties for a larger layout
dfd.attr(rankdir="TB", size="30,30")
dfd.graph_attr["dpi"] = "300"  # Setting DPI to 300 for high resolution

# Increase the default font size for nodes and edges
dfd.attr("node", fontsize="24")
dfd.attr("edge", fontsize="22")

# Define styles for different types of nodes with increased size and margin for better visibility
dfd.attr(
    "node",
    shape="rectangle",
    style="filled",
    color="#619bcc",
    width="0.5",
    height="0.5",
    margin="0.2,0.2",
)

# External Entities
dfd.node("Simtwo", "Robotics\nSimulator")

dfd.attr(
    "node",
    shape="rectangle",
    style="filled",
    color="#95C5A5",
    width="0.5",
    height="0.5",
    margin="0.2,0.2",
)

dfd.node("YOLOv8", "YOLOv8.2\nDetection")

# Data Stores with specific styling adjustments
dfd.node(
    "Log",
    "Data Logs",
    shape="cylinder",
    color="gray",
    width="0.5",
    height="0.5",
    margin="0.2,0.2",
)

# Processes with specific styling adjustments
dfd.attr(
    "node",
    shape="rectangle",
    style="filled",
    color="orange",
    width="0.5",
    height="0.5",
    margin="0.2,0.2",
)
dfd.node("Manager", "Manager")
dfd.node("Localization", "Localization")
dfd.node("Visualizer", "Visualizer")
dfd.node("SimtwoInterface", "Simtwo\nInterface")
dfd.node("PerfectMatch", "PM\nRoPM")
dfd.node("EKF", "EKF")
dfd.node("LoggerData", "Logger")

# Data Flows with adjusted attributes for clarity
dfd.edge("Simtwo", "SimtwoInterface", "(Encs, GT) 40Hz\n(LIDAR) 7 Hz")
dfd.edge("Simtwo", "YOLOv8", "(RGB)\n15 Hz")

dfd.edge("SimtwoInterface", "Manager", "(Encs, GT) 40 Hz\n(LIDAR) 7 Hz\n(RGB) 15 Hz")

dfd.edge(
    "Manager",
    "Localization",
    "(Encs) 40 Hz\n(LIDAR) 7 Hz\n(RGB) 15 Hz",
)
dfd.edge(
    "Manager",
    "LoggerData",
    "(Encs, GT, timestamp) 40Hz\n(LIDAR) 7 Hz\n(Inference) 15 Hz",
)
dfd.edge(
    "Manager",
    "Visualizer",
    "(Localization) 40 Hz\n(LIDAR, LIDAR Out.) 7 Hz\n(Inference) 15 Hz",
)
# dfd.edge(
#     "Manager",
#     "Visualizer",
#     "(EKF μ, EKF Out. μ, GT) 40 Hz\n(LIDAR, LIDAR Out.) 7 Hz\n(Inference ) 15 Hz\n(Localization ) 40 Hz",
# )

dfd.edge("Localization", "PerfectMatch", "(LIDAR,\nLIDAR Out.)\n7 Hz")
dfd.edge("Localization", "EKF", "(Odo) 40 Hz\n(PM Pose,\nPM Out. Pose)\n7 Hz")
# dfd.edge(
#     "Localization",
#     "Manager",
#     "(EKF μ, EKF Out. μ) 40 Hz\n(Localization ) 40 Hz",
# )
dfd.edge(
    "Localization",
    "Manager",
    "(Localization) 40 Hz",
)

dfd.edge(
    "PerfectMatch",
    "Localization",
    "(PM Pose,\nPm Out. Pose,\nPM Error,\nPM Out. Error)\n7 Hz",
)

dfd.edge("EKF", "Localization", "(EKF μ,\nEKF Out. μ) 40 Hz")

dfd.edge("YOLOv8", "SimtwoInterface", "Inference\n15 Hz")

dfd.edge(
    "LoggerData",
    "Log",
    "(Encs, GT, timestamp) 40Hz\n(LIDAR) 7 Hz\n(Inference) 15 Hz",
)


dfd.attr("node", style="filled", color="none")  # Set no fill color for legend node
legend_label = """<
    <TABLE BORDER="1" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4" COLOR="black">
        <TR><TD COLSPAN="2" ALIGN="CENTER"><FONT POINT-SIZE="24"><B>Legend</B></FONT></TD></TR>
        <TR>
            <TD WIDTH="40" HEIGHT="20" FIXEDSIZE="TRUE" BGCOLOR="#619bcc"></TD>
            <TD ALIGN="LEFT">Software</TD>
        </TR>
        <TR>
            <TD WIDTH="40" HEIGHT="20" FIXEDSIZE="TRUE" BGCOLOR="#95C5A5"></TD>
            <TD ALIGN="LEFT">Python script</TD>
        </TR>
        <TR>
            <TD WIDTH="40" HEIGHT="20" FIXEDSIZE="TRUE" BGCOLOR="orange"></TD>
            <TD ALIGN="LEFT">Components in the C++ program</TD>
        </TR>
        <TR>
            <TD WIDTH="40" HEIGHT="20" FIXEDSIZE="TRUE" BGCOLOR="gray"></TD>
            <TD ALIGN="LEFT">Logging processes</TD>
        </TR>
    </TABLE>>"""
dfd.node("legend", legend_label)

# Save the DFD to a file with increased visibility settings
file_path = "system_dfd_v2"
dfd.render(file_path, format="pdf", cleanup=True)
