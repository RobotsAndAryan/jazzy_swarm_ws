#!/usr/bin/env python3
import random
import os

def generate_synthetic_city():
    print("[1] Bypassing OSM API... Initializing Procedural City Generation...")
    sdf_content = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="stratford_synthetic">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>51.5413</latitude_deg>
      <longitude_deg>-0.0156</longitude_deg>
      <elevation>10.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 50 0 0 0</pose><direction>-0.5 0.5 -0.9</direction></light>
    <model name="ground"><static>true</static><link name="link"><collision name="collision"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision><visual name="visual"><geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry><material><ambient>0.15 0.15 0.15 1</ambient></material></visual></link></model>
"""
    
    bldg_idx = 0
    # Generate an 80m x 80m randomized city grid
    for x in range(-20, 60, 15):
        for y in range(-30, 40, 15):
            # Carve out the L-shaped flight path for the drones so they aren't trapped at spawn
            is_path = False
            if (0 <= x <= 24) and (-5 <= y <= 5): is_path = True
            if (15 <= x <= 25) and (0 <= y <= 20): is_path = True
            
            if not is_path:
                width = random.uniform(8.0, 14.0)
                depth = random.uniform(8.0, 14.0)
                height = random.uniform(15.0, 50.0) # Random Skyscraper Heights
                x_offset = x + random.uniform(-2.0, 2.0)
                y_offset = y + random.uniform(-2.0, 2.0)
                
                # Random shades of urban grey/blue
                r = random.uniform(0.2, 0.4)
                g = random.uniform(0.25, 0.45)
                b = random.uniform(0.3, 0.5)
                
                sdf_content += f"""
    <model name="skyscrape_{bldg_idx}">
      <static>true</static><pose>{x_offset} {y_offset} {height/2} 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>{width} {depth} {height}</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>{width} {depth} {height}</size></box></geometry>
        <material><ambient>{r} {g} {b} 1</ambient></material></visual>
      </link>
    </model>"""
                bldg_idx += 1
    
    # The crucial dead-end wall to force the left turn
    sdf_content += """
    <model name="dead_end">
      <static>true</static><pose>26 0 10 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>4 15 20</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>4 15 20</size></box></geometry><material><ambient>0.2 0.2 0.2 1</ambient></material></visual>
      </link>
    </model>"""

    sdf_content += "\n  </world>\n</sdf>"
    
    os.makedirs(os.path.expanduser("~/jazzy_swarm_ws/src/swarm_core/worlds"), exist_ok=True)
    with open(os.path.expanduser("~/jazzy_swarm_ws/src/swarm_core/worlds/stratford_synthetic.sdf"), "w") as f:
        f.write(sdf_content)
    print(f"[2] SUCCESS! Rendered {bldg_idx} synthetic skyscrapers. Saved to stratford_synthetic.sdf")

if __name__ == "__main__":
    generate_synthetic_city()
