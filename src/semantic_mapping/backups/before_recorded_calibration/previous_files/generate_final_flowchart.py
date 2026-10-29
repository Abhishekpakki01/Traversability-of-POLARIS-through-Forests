import graphviz

# Initialize a directed graph with a clean vertical layout
dot = graphviz.Digraph(comment="OHM Pipeline Architecture Final", format="png")
dot.attr(rankdir="TD", size="12,14", fontname="Arial")

# Subgraph Phase 1 & 2: Ingestion & Filtering
with dot.subgraph(name="cluster_0") as c:
  c.attr(
      label="1 & 2. Ingestion & Point Cloud Filtering",
      style="rounded",
      color="gray",
      fontname="Arial",
  )
  c.node(
      "A",
      "RELLIS-3D Rosbag Playback\n(/os1_cloud_node/points, /tf)",
      shape="box",
  )
  c.node(
      "B",
      "ROS 2 Subscriber & QoS Setup\n(Best Effort Sensor Data)",
      shape="box",
  )
  c.node(
      "C",
      "PointCloud2ConstIterator\nExtract x, y, z coordinates",
      shape="box",
  )
  c.node("D", "Distance Check:\n0.1m < dist <= 10.0m?", shape="diamond")
  c.node("E", "Discard NaN / Out-of-Range", shape="ellipse")
  c.node("F", "Store in valid_targets vector\n(std::vector<glm::dvec3>)", shape="box")

  c.edge("A", "B")
  c.edge("B", "C")
  c.edge("C", "D")
  c.edge("D", "E", label="No")
  c.edge("D", "F", label="Yes")

# Subgraph Phase 3: OHM Ray-Tracing Engine
with dot.subgraph(name="cluster_1") as c:
  c.attr(
      label="3. OHM Ray-Tracing Engine",
      style="rounded",
      color="gray",
      fontname="Arial",
  )
  c.node(
      "G",
      "Construct Ray Pairs\n[Origin (0,0,0) -> Target (x,y,z)]",
      shape="box",
  )
  c.node(
      "H",
      "ohm::RayMapperOccupancy::integrateRays\nCPU Ray-Marching & Log-Odds Update",
      shape="box",
  )
  c.node("I", "OccupancyMap\nInternal 3D Voxel Grid", shape="box")

  c.edge("G", "H")
  c.edge("H", "I")

# Subgraph Phase 4: Voxel Query & Publishing
with dot.subgraph(name="cluster_2") as c:
  c.attr(
      label="4. Voxel Query & Publishing",
      style="rounded",
      color="gray",
      fontname="Arial",
  )
  c.node(
      "J",
      "Iterate Active Map Voxels\nmap_->begin() to map_->end()",
      shape="box",
  )
  c.node("K", "Occupancy Value > 0.0?", shape="diamond")
  c.node(
      "L",
      "Compute Voxel Center Coordinates\nohm::OccupancyMap::voxelCentre",
      shape="box",
  )
  c.node(
      "M",
      "Assemble CUBE_LIST Marker\nTopic: /semantic_voxel_map_cpp",
      shape="box",
      style="filled",
      fillcolor="#e6f2ff",
  )
  c.node(
      "N",
      "RViz2 Display\n(Renders as Uniform Green Block)",
      shape="box",
      style="filled",
      fillcolor="#ffe6e6",
  )

  c.edge("J", "K")
  c.edge("K", "L", label="Yes")
  c.edge("L", "M")
  c.edge("M", "N")

# Connect across main phases
dot.edge("F", "G")
dot.edge("I", "J")

# Render output image locally
dot.render("ohm_pipeline_flowchart_final", cleanup=True)
print(
    "Final flowchart successfully generated and saved locally as"
    " 'ohm_pipeline_flowchart_final.png'!"
)