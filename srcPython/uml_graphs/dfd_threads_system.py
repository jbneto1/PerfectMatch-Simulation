import graphviz

# Create a new directed graph with formatting
dot = graphviz.Digraph(
    comment="Application Threads and Synchronization",
    graph_attr={
        "rankdir": "TB",
    },  # Setting direction left-to-right
    node_attr={"shape": "box", "style": "rounded,filled", "color": "lightblue2"},
    edge_attr={"fontsize": "12", "fontcolor": "black"},
)

# Main components and threads
dot.node("Main", "Main Thread", shape="ellipse", color="lightblue3")
dot.node("Manager", "Manager", shape="component", color="orange")
dot.node("VisT", "Visualization Thread", shape="ellipse", color="lightgreen")
dot.node("SimTwoInt", "SimTwoInterface", shape="component", color="lightcoral")
dot.node("SignalH", "Signal Handling", shape="ellipse", color="gold1")

# Synchronization mechanisms
dot.node("Mutex", "Mutex (Visualizer)", shape="cylinder", color="grey")
dot.node("CondVar", "Condition Variable (Visualizer)", shape="cylinder", color="grey88")

# Threads potentially created by SimTwoInterface
dot.node("NetworkIO", "Network I/O Threads", shape="folder", color="lightsalmon")

# Adding edges to represent interactions and dependencies
dot.edge("Main", "Manager", label="Initializes")
dot.edge("Manager", "VisT", label="Spawns & Updates\nData")
dot.edge("Manager", "SignalH", label="Manages\nSignals")
dot.edge("Manager", "SimTwoInt", label="Instantiates")
dot.edge("SimTwoInt", "NetworkIO", label="Spawns")
dot.edge("VisT", "Mutex", label="Mutex\nLock/Unlock")
dot.edge("VisT", "CondVar", label="Waits/Signals")

# Explicit data flow between components
dot.edge("SimTwoInt", "Manager", label="Sends sensor data")

# Save and render the graph to a file
dot.render("thread_interaction_diagram_refined", cleanup=True, format="pdf")

# Outputting the generated DOT source for review
print(dot.source)
