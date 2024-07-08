from graphviz import Digraph

# Create a new directed graph for the DFD
dfd = Digraph(comment="System DFD")

# Define graph properties for a larger layout
dfd.attr(rankdir="TB", size="30,30")
dfd.graph_attr["dpi"] = "300"  # Setting DPI to 300 for high resolution

# Increase the default font size for nodes and edges
dfd.attr("node", fontsize="24")
dfd.attr("edge", fontsize="22")


dfd.attr(
    "node",
    shape="rectangle",
    style="filled",
    color="gray",
    width="0.5",
    height="0.5",
    margin="0.2,0.2",
)

dfd.node("DataLog", "Raw Data\nLogs")

# Data Stores with specific styling adjustments
dfd.node(
    "Log",
    "Processed\nData Logs",
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
dfd.node("Decoder", "OfflineAnalysis")
dfd.node("Manager", "Manager")
dfd.node("Localization", "Localization")
dfd.node("Visualizer", "Visualizer")
dfd.node("PerfectMatch", "PM\nRoPM")
dfd.node("EKF", "EKF")
dfd.node("LoggerData", "Logger")

dfd.edge("DataLog", "Manager", "Raw log\ndata")
dfd.edge("Manager", "Decoder", "Raw log\ndata")
dfd.edge("Decoder", "Manager", "Encoder, GT,\nInference, LIDAR,\nTimestamp")


dfd.edge("Manager", "Localization", "(Encs, GT, LIDAR) 40 Hz\n(RGB) 15 Hz")
dfd.edge(
    "Manager",
    "LoggerData",
    "(GT, timestamp) 40Hz\n(Inference) 15 Hz\n(Localization) 40 Hz",
)
dfd.edge(
    "Manager",
    "Visualizer",
    "(Localization) 40 Hz\n(LIDAR, LIDAR Out.) 7 Hz\n(Inference) 15 Hz",
)

dfd.edge("Localization", "PerfectMatch", "(LIDAR, LIDAR Out.) 15 Hz")
dfd.edge("Localization", "EKF", "(Odo) 40 Hz\n(PM Pose, PM Out. Pose) 15 Hz")
dfd.edge(
    "Localization",
    "Manager",
    "(Localization) 40 Hz",
)

dfd.edge(
    "PerfectMatch",
    "Localization",
    "(PM Pose, Pm Out. Pose) 15 Hz\n(PM Error, PM Out. Error) 15 Hz",
)

dfd.edge("EKF", "Localization", "(EKF μ, EKF Out. μ) 40 Hz")


dfd.edge(
    "LoggerData",
    "Log",
    "(GT, timestamp) 40Hz\n(Inference metadata) 15 Hz\n(Localization data) 40 Hz",
)


dfd.attr("node", style="filled", color="none")  # Set no fill color for legend node
legend_label = """<
    <TABLE BORDER="1" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4" COLOR="black">
        <TR><TD COLSPAN="2" ALIGN="CENTER"><FONT POINT-SIZE="24"><B>Legend</B></FONT></TD></TR>
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
file_path = "system_dfd_v2_offline"
dfd.render(file_path, format="pdf", cleanup=True)
