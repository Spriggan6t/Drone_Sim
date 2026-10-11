from isaacsim import SimulationApp # Starts omnivers
simulation_app = SimulationApp({"headless": False}) # true only if on server without visuals

# ROS
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge") # change


# Import core Isaac Sim modules
# Robot import
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.storage.native import get_assets_root_path

import omni.graph.core as og # change
import numpy as np

frequency:float = 60.0 # Hz

# Create the physics world (60 Hz physics step for PX4 stability)
world = World(physics_dt=1.0/frequency, rendering_dt=1.0/frequency) 
world.scene.add_default_ground_plane()


# Locate assests for Isac sim
assets_root_path = get_assets_root_path()
print("Locating Omniverse assets...")
if assets_root_path is None:
    print("Error: Could not find Nucleus server")
    simulation_app.close()
    quit()

#Quad copter model
drone_usd_path = assets_root_path + "/Isaac/Robots/IsaacSim/Quadcopter/quadcopter.usd"
print(f"Loading drone from: {drone_usd_path}")
print("Please wait...")
add_reference_to_stage(usd_path=drone_usd_path, prim_path="/World/Drone")
print("Drone successfully downloaded and in world")

drone = world.scene.add(
    Robot(
        prim_path="/World/Drone",
        name="my_quadcopter",
        position=np.array([0.0, 0.0, 2.0]), # Drop from 2 meters
    )
)

print("Building ROS 2 OmniGraph...", flush=True)
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/World/ROS_Bridge", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"), # change
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"), # change
        ],
        keys.CONNECT: [
            ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"), # Changed systemTime to simulationTime
        ],
    },
)

world.reset() # Starts physics engine for drone tracking
print("Loaded joints (rotors) that PX4 will control:", flush=True)
print(drone.dof_names, flush=True)
print("Simulation initialized and ROS 2 clock publishing. Press Ctrl+C to exit.", flush=True)


# Main simulation loop
while simulation_app.is_running():
    # Step the physics engine and render the frame at world dt
    world.step(render=True)

# Cleanup the CUDA context
simulation_app.close()