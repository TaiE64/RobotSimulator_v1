import pybullet as p
import random
import time
import os
import pybullet_data
import numpy as np

#----------------------------------------------------------------------

# 使用共享内存模式连接到 PyBullet 仿真环境。如果连接失败，回退到 GUI 模式。
cid = p.connect(p.SHARED_MEMORY)
if cid < 0:
    p.connect(p.GUI)

#----------------------------------------------------------------------

# 设置搜索路径，用于加载资源文件
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation()

# 禁用可视化渲染以提高加载速度
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

#----------------------------------------------------------------------

# 加载 PR2 Gripper 模型
initial_position = [5, 5, 5]
initial_orientation = p.getQuaternionFromEuler([0, 0, 0])
pr2_gripper = p.loadURDF("pr2_gripper.urdf", initial_position, initial_orientation)

# 加载桌子模型
table = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "table/table.urdf"),
                   [0.0, -0.4, 0], [0, 0, 0, 1])

# 加载物体模型
objID = p.loadURDF("example/share/cube_small.urdf", [0.0, -0.4, 0.62], globalScaling=2)

#----------------------------------------------------------------------

# 初始化关节位置
jointPositions = [3, 0.0, 3, 0.0]
for jointIndex in range(p.getNumJoints(pr2_gripper)):
    p.resetJointState(pr2_gripper, jointIndex, jointPositions[jointIndex])

# 建立约束
pr2_cid = p.createConstraint(pr2_gripper, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])

# 启用可视化渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

#----------------------------------------------------------------------

# 设置重力
p.setGravity(0, 0, -10)

#----------------------------------------------------------------------

def calculate_safe_position_random(aabb):
    """根据 AABB 和偏移量，在物体周围随机生成一个安全位置。"""
    noise_1 = random.gauss(mu=0.2, sigma=0.1) * 0.1  # x noise
    noise_2 = random.gauss(mu=0.2, sigma=0.1) * 0.1  # y noise
    noise_3 = random.gauss(mu=0.25, sigma=0.1)       # z noise

    min_bound, max_bound = aabb

    # 爪子在物体上方
    x_safe = (max_bound[0] + min_bound[0]) / 2 + noise_1
    y_safe = (max_bound[1] + min_bound[1]) / 2 + noise_2
    z_safe = max_bound[2] + noise_3

    return [x_safe, y_safe, z_safe]

def move_to_initial_position(pr2_cid, initial_position):
    """将爪子移动到初始位置。"""
    initial_orientation = p.getQuaternionFromEuler([0, 0, 0])  # 无旋转
    p.changeConstraint(pr2_cid, initial_position, jointChildFrameOrientation=initial_orientation)

    for step in range(100):
        target_z = initial_position[2] + step * 0.005
        p.changeConstraint(pr2_cid, [initial_position[0], initial_position[1], target_z])
        p.stepSimulation()
        time.sleep(0.01)
    print("Moved to initial position.")

def move_to_random_safe_position(robot_id, end_effector_link, object_id):
    """机械手移动到物体周围的随机安全位置。"""
    aabb = p.getAABB(object_id)
    safe_position = calculate_safe_position_random(aabb)
    return safe_position

def gradual_move_to_target(pr2_cid, current_position, target_position):
    """逐步将爪子移动到目标位置。"""
    steps = 100
    for i in range(steps):
        interpolated_position = [
            current_position[j] + (target_position[j] - current_position[j]) * (i / steps)
            for j in range(3)
        ]
        p.changeConstraint(pr2_cid, interpolated_position)
        p.stepSimulation()
        time.sleep(0.01)
    print("Reached target position.")

def attempt_grasp():
    """控制爪子尝试抓取物体。"""
    p.setJointMotorControl2(pr2_gripper, 0, p.POSITION_CONTROL, targetPosition=0, maxVelocity=2000, force=2000)
    p.setJointMotorControl2(pr2_gripper, 2, p.POSITION_CONTROL, targetPosition=0, maxVelocity=2000, force=2000)
    time.sleep(2)

def update_hand_orientation(safe_position):
    """更新机械手的朝向以适应抓取。"""
    hand_ori = p.getQuaternionFromEuler([0, np.pi / 2, 0])
    p.changeConstraint(pr2_cid, safe_position, jointChildFrameOrientation=hand_ori)
    p.stepSimulation()
    
def random_grasp_object(robot_id, object_id, end_effector_link):
    """从初始位置逐步移动到随机生成的位置并尝试抓取物体。"""
    initial_position = [0, 0, 0.8]  # 初始位置
    move_to_initial_position(pr2_cid, initial_position)

    safe_position = move_to_random_safe_position(robot_id, end_effector_link, object_id)
    print(f"Moving to random safe position: {safe_position}")

    gradual_move_to_target(pr2_cid, initial_position, safe_position)
    update_hand_orientation(safe_position)

    time.sleep(2)

    attempt_grasp()
    # obj_pos, _ = p.getBasePositionAndOrientation(object_id)
    # print("Object position after grasp:", obj_pos)

    for step in range(100):
        target_z = safe_position[2] + step * 0.005
        p.changeConstraint(pr2_cid, [safe_position[0], safe_position[1], target_z])
        p.stepSimulation()
        time.sleep(0.01)

    print("Grasp attempt completed!")

def collect_grasp_data():
    """主函数：执行抓取操作并收集数据。"""
    random_grasp_object(pr2_gripper, objID, 0)
    p.stepSimulation()

collect_grasp_data()

input("Enter to quit")
p.disconnect()
