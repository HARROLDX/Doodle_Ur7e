from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'width': 640, 'height': 480, 'renderer': 'RaytracedLighting', 'active_gpu': 0, 'multi_gpu': False})
from isaacsim.core.utils.extensions import enable_extension
enable_extension('isaacsim.ros2.bridge')
for _ in range(10): app.update()
import rclpy
rclpy.init()
n = rclpy.create_node('doodle_isaac_smoke')
print('DOODLE_ISAAC_ROS_READY', flush=True)
n.destroy_node(); rclpy.shutdown(); app.close()
