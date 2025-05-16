import subprocess
import time

BOX_NAME = "test_box"
WORLD_NAME = "empty"
SDF_PATH = "/tmp/box_model.sdf"

# Define the box model SDF string
sdf_box = f"""
<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="{BOX_NAME}">
    <static>true</static>
    <link name="link">
      <pose>3 0 0.26 0 0 0.5236</pose>
      <collision name="collision">
        <geometry>
          <box><size>2 0.23 0.52</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>2 0.23 0.52</size></box>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0 0.01 0.05 1</diffuse>
          <specular>0 0.01 0.05 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

# Write to a temp file
with open(SDF_PATH, "w") as f:
    f.write(sdf_box)

def spawn_box():
    subprocess.run([
        "ign", "service", "-s", f"/world/{WORLD_NAME}/create",
        "--reqtype", "ignition.msgs.EntityFactory",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "3000",
        "--req", f'sdf_filename: "{SDF_PATH}"'
    ])

def delete_box():
    subprocess.run([
        "ign", "service", "-s", f"/world/{WORLD_NAME}/remove",
        "--reqtype", "ignition.msgs.Entity",
        "--reptype", "ignition.msgs.Boolean",
        "--timeout", "3000",
        "--req", f'type: MODEL name: "{BOX_NAME}"'
    ])

# Loop to spawn and remove
try:
    while True:
        time.sleep(20)
        print("Spawning box...")
        spawn_box()
        time.sleep(5)
        print("Removing box...")
        delete_box()
        
except KeyboardInterrupt:
    print("Stopped.")
