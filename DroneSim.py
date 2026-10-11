from isaacsim import SimulationApp # Starts omnivers
simulation_app = SimulationApp({"headless": False}) # true only if on server without visuals

# Import core Isaac Sim modules
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
import numpy as np

# Create the physics world (60 Hz physics step for PX4 stability)
world = World(physics_dt=1.0/60.0, rendering_dt=1.0/60.0) 
world.scene.add_default_ground_plane()

# Add a placeholder rigid body for the drone chassis
drone_placeholder = world.scene.add(
    DynamicCuboid( #subject to physics 
        prim_path="/World/Drone",
        name="drone_placeholder",
        position=np.array([0.0, 0.0, 2.0]), # Drop from 2 meters
        scale=np.array([0.3, 0.3, 0.1]),
        color=np.array([0.8, 0.1, 0.1]),
        mass=20 # weight >=25 kg
    )
)


world.reset() # Starts physics engine for drone tracking

print("Simulation environment initialized. Press Ctrl+C in terminal to exit.")

# Main simulation loop
while simulation_app.is_running():
    # Step the physics engine and render the frame at world dt
    world.step(render=True)

# Cleanup the CUDA context
simulation_app.close()
