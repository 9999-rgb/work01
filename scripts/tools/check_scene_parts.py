import rclpy, sys
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState

rclpy.init()
node = Node("check_parts")
cli = node.create_client(GetEntityState, "/get_entity_state")
cli.wait_for_service(timeout_sec=5)

SCENE = "xczs_scene_floor"
DRAWERS = ["s1", "s2", "b1", "s3", "m1"]
LEGACY = ["s1r", "s1p", "s2r", "s2p", "b1r", "b1p", "s3r", "s3p", "m1r", "m1p"]

def get(name):
    req = GetEntityState.Request(); req.name = name
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5)
    resp = fut.result()
    if resp is None or not resp.success:
        return None
    return resp.state

print("=== drawer bodies ===")
for d in DRAWERS:
    st = get(f"{SCENE}::{d}")
    if st:
        p = st.pose.position
        print(f"{d}: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}")
    else:
        print(f"{d}: MISSING/ERR")

print("=== legacy detached parts (should all be MISSING after merge) ===")
found = 0
for l in LEGACY:
    st = get(f"{SCENE}::{l}")
    if st and st.pose.position.z > 0.001:
        p = st.pose.position
        print(f"{l}: x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}")
        found += 1
    elif st:
        print(f"{l}: present z={st.pose.position.z:.4f}")
    else:
        print(f"{l}: MISSING")
print(f"legacy parts still present (z>0.001): {found}")

# also dump full link_states to catch any other stray bodies
from gazebo_msgs.msg import LinkStates
sub = node.create_subscription(LinkStates, "/link_states", lambda m: None, 10)
for _ in range(10): rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node(); rclpy.shutdown()
