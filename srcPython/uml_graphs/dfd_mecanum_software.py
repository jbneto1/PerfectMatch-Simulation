from graphviz import Digraph


def create_uml():
    dot = Digraph(comment="Robot Main Script Communication Diagram")

    # Define graph properties for a vertical layout
    dot.attr(rankdir="TB", size="30,30")
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
    dot.node("EdgeDevice", "Object Detection Script")
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
    dot.node("CPPLidar", "LIDAR Program")
    dot.node("CamScript", "RpiCam Stream Script")

    # Define edges with styled attributes for clarity
    dot.edge(
        "Microcontroller",
        "RobotScript",
        "Est. Wheels' Speeds &\nSwitch Data @40Hz\nvia Serial",
        color="red",
    )

    dot.edge(
        "CamScript",
        "EdgeDevice",
        "Raspicam V2\nStream@15Hz\nvia ZMQ",
        color="orange",
    )

    dot.edge(
        "RobotScript",
        "Microcontroller",
        "Ref. Wheels' Speeds &\nCommands @40Hz\nvia Serial",
        color="red",
    )
    dot.edge(
        "RobotScript",
        "DataLog",
        "Ground truth, encoders,\nYOLO metadata,\ntimestamp sensor data;\nLocalization data @40Hz",
        color="blue",
    )
    dot.edge(
        "EdgeDevice",
        "RobotScript",
        "Inference @15Hz\n(4 Ports)\nvia UDP",
        color="green",
    )
    dot.edge(
        "PythonAruco", "RobotScript", "ArUco\nMetadata @10Hz\nvia UDP", color="green"
    )
    dot.edge("ESP32Cams", "EdgeDevice", "Imagery Data\n@v15Hz\nvia UDP", color="green")

    dot.edge("CPPLidar", "RobotScript", "LIDAR\nData @7Hz\nvia UDP", color="green")
    dot.edge(
        "PythonGT", "RobotScript", "Ground Truth\nData @30Hz\nvia UDP", color="green"
    )
    dot.edge(
        "RemoteControl",
        "RobotScript",
        "Remote Control\nMetadata @40Hz\nvia UDP",
        color="green",
    )

    dot.node("anchor", style="invis")  # Invisible node

    with dot.subgraph() as s:
        s.attr(rank="max")
        # Define the legend node
        s.node(
            "legend",
            """<
        <TABLE BORDER="1" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4" COLOR="black">
            <TR><TD COLSPAN="2" ALIGN="CENTER"><FONT POINT-SIZE="20"><B>Legend (nodes)</B></FONT></TD></TR>
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
        </TABLE>>""",
            style="filled",
            color="none",
        )
        # Invisible edge to push the legend to the top-right
        s.edge("anchor", "legend", style="invis")

    with dot.subgraph() as s:
        s.attr(rank="sink")
        s.attr(rankdir="TB")  # Direction from top to bottom within the subgraph
        s.node(
            "protocol_legend",
            """<
        <TABLE BORDER="1" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4" COLOR="black">
            <TR><TD COLSPAN="2" ALIGN="CENTER"><FONT POINT-SIZE="20"><B>Legend (edges)</B></FONT></TD></TR>
            <TR>
                <TD WIDTH="40" HEIGHT="20" FIXEDSIZE="TRUE" BGCOLOR="red"></TD>
                <TD ALIGN="LEFT">USART</TD>
            </TR>
            <TR>
                <TD WIDTH="40" HEIGHT="20" FIXEDSIZE="TRUE" BGCOLOR="orange"></TD>
                <TD ALIGN="LEFT">ZMQ Pub/Sub</TD>
            </TR>
            <TR>
                <TD WIDTH="40" HEIGHT="20" FIXEDSIZE="TRUE" BGCOLOR="blue"></TD>
                <TD ALIGN="LEFT">Data Logging</TD>
            </TR>
            <TR>
                <TD WIDTH="40" HEIGHT="20" FIXEDSIZE="TRUE" BGCOLOR="green"></TD>
                <TD ALIGN="LEFT">UDP</TD>
            </TR>
        </TABLE>>""",
            style="invisible",
        )

    # Save the UML diagram to a file
    output_path = "robot_system_interactions_high_quality"
    file_path = dot.render(output_path, format="pdf", cleanup=True)
    print("Diagram saved to:", file_path)


if __name__ == "__main__":
    create_uml()
