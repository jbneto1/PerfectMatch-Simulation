from graphviz import Digraph


def create_uml():
    dot = Digraph(comment="Robot Main Script Communication Diagram")

    # Define graph properties for a larger layout
    dot.attr(rankdir="LR", size="30,30")
    dot.graph_attr["dpi"] = "300"  # Setting DPI to 300 for high resolution

    # Increase the default font size for nodes and edges
    dot.attr("node", fontsize="24")
    dot.attr("edge", fontsize="22")

    width = "0.5"
    height = "0.5"

    esp32color = "#008147"
    microcontrollercolor = "#95C5A5"
    edgedevicecolor = "#619bcc"
    gtcolor = "#D30000"
    robotcolor = "orange"

    # Define node styles with increased size and margin for better visibility
    dot.attr(
        "node",
        shape="cylinder",
        style="filled",
        color="gray",
        width=width,
        height=height,
        margin="0.2, 0.2",
    )

    dot.node("DataLog", "Data logs")

    dot.attr(
        "node",
        shape="rectangle",
        style="filled",
        color=microcontrollercolor,
        width=width,
        height=height,
        margin="0.2, 0.2",
    )

    dot.node("Microcontroller", "Microcontroller Firmware")

    dot.attr(
        "node",
        shape="rectangle",
        style="filled",
        color=edgedevicecolor,
        width=width,
        height=height,
        margin="0.2, 0.2",
    )
    dot.node("EdgeDevice", "Edge Device")
    dot.node("RemoteControl", "Remote Control Program")

    dot.attr(
        "node",
        shape="rectangle",
        style="filled",
        color=esp32color,
        width=width,
        height=height,
        margin="0.2, 0.2",
    )

    dot.node("ESP32Cams", "ESP32 Cam Firmwares")

    dot.attr(
        "node",
        shape="rectangle",
        style="filled",
        color=gtcolor,
        width=width,
        height=height,
        margin="0.2, 0.2",
    )
    dot.node("PythonGT", "Ground Truth Script")

    dot.attr(
        "node",
        shape="rectangle",
        style="filled",
        color=robotcolor,
        width=width,
        height=height,
        margin="0.2, 0.2",
    )

    # Define nodes
    dot.node("RobotScript", "Robot Main Program")
    dot.node("PythonAruco", "ArUco Script")
    dot.node("CPPLidar", "Lidar Program")
    dot.node("CamScript", "RpiCam Stream Script")

    # Define edges with styled attributes for clarity
    dot.edge(
        "Microcontroller",
        "RobotScript",
        "Est. Wheels' Speeds & Switch Data @40Hz\nvia Serial",
        color="red",
    )

    dot.edge(
        "CamScript",
        "EdgeDevice",
        "Raspicam V2 Stream@15Hz\nvia ZMQ",
        color="orange",
    )

    dot.edge(
        "RobotScript",
        "Microcontroller",
        "Ref. Wheels' Speeds & Electromagnet Commands @40Hz\nvia Serial",
        color="red",
    )
    dot.edge(
        "RobotScript",
        "DataLog",
        "Ground truth, encoders, yolo metadata, and timestamp sensor data;\nLocalization data @40Hz",
        color="blue",
    )
    dot.edge(
        "EdgeDevice",
        "RobotScript",
        "Inference @15Hz\nvia UDP (4 Ports)",
        color="green",
    )
    dot.edge(
        "PythonAruco", "RobotScript", "ArUco Metadata @10Hz\nvia UDP", color="green"
    )
    dot.edge("ESP32Cams", "EdgeDevice", "Imagery Data @v15Hz\nvia UDP", color="green")

    dot.edge("CPPLidar", "RobotScript", "Lidar Data @7Hz\nvia UDP", color="green")
    dot.edge(
        "PythonGT", "RobotScript", "Ground Truth Data @30Hz\nvia UDP", color="green"
    )
    dot.edge(
        "RemoteControl",
        "RobotScript",
        "Remote Control Metadata @40Hz\nvia UDP",
        color="green",
    )

    dot.attr("node", style="filled", color="none")  # Set no fill color for legend node
    legend_label = """<
    <TABLE BORDER="1" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4" COLOR="black">
        <TR><TD COLSPAN="2" ALIGN="CENTER"><FONT POINT-SIZE="20"><B>Legend</B></FONT></TD></TR>
        <TR>
            <TD WIDTH="40" HEIGHT="20" FIXEDSIZE="TRUE" BGCOLOR="orange"></TD>
            <TD ALIGN="LEFT">Runs on the Raspberry Pi 4B</TD>
        </TR>
        <TR>
            <TD WIDTH="40" HEIGHT="20" FIXEDSIZE="TRUE" BGCOLOR="#D30000"></TD>
            <TD ALIGN="LEFT">Runs on the Raspberry Pi 5</TD>
        </TR>
        <TR>
            <TD WIDTH="40" HEIGHT="20" FIXEDSIZE="TRUE" BGCOLOR="#619bcc"></TD>
            <TD ALIGN="LEFT">Runs on the Edge Device</TD>
        </TR>
        <TR>
            <TD WIDTH="40" HEIGHT="20" FIXEDSIZE="TRUE" BGCOLOR="#008147"></TD>
            <TD ALIGN="LEFT">Runs on the ESP32-Cams</TD>
        </TR>
        <TR>
            <TD WIDTH="40" HEIGHT="20" FIXEDSIZE="TRUE" BGCOLOR="#95C5A5"></TD>
            <TD ALIGN="LEFT">Runs on the Arduino Mega</TD>
        </TR>
    </TABLE>>"""
    dot.node("legend", legend_label)

    # Save the UML diagram to a file
    output_path = "robot_system_interactions_high_quality"
    file_path = dot.render(output_path, format="pdf", cleanup=True)
    print("Diagram saved to:", file_path)


if __name__ == "__main__":
    create_uml()
