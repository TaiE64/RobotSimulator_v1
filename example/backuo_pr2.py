import pybullet as p
import time
#p.connect(p.UDP,"192.168.86.100")
import pybullet_data
#----------------------------------------------------------------------

#使用共享内存模式连接到 PyBullet 仿真环境。如果连接失败，回退到 GUI 模式。
cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)
  
#----------------------------------------------------------------------

#setAdditionalSearchPath：设定搜索路径，用于加载资源文件（如 URDF 模型）。
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation()
#disable rendering during loading makes it much faster
#禁用可视化渲染提高加载速度，完成后再启用。
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

#----------------------------------------------------------------------

#根据代码中的 p.setAdditionalSearchPath(pybullet_data.getDataPath())，
# pr2_gripper.urdf 文件在 PyBullet 提供的默认数据路径中
#加载 URDF 模型文件，PR2 Gripper 放置在仿真世界坐标 (0.5, 0.3, 0.7)。
objects = [
    p.loadURDF("pr2_gripper.urdf", 0.500000, 0.300006, 0.700000, -0.000000, -0.000000, -0.000031,
               1.000000)
]
#Robot ID
pr2_gripper = objects[0]

#----------------------------------------------------------------------

#初始化关节位置，分别设置机械手的几个关节到指定角度。
jointPositions = [0.550569, 0.000000, 0.549657, 0.000000]
for jointIndex in range(p.getNumJoints(pr2_gripper)):
  p.resetJointState(pr2_gripper, jointIndex, jointPositions[jointIndex])

pr2_cid = p.createConstraint(pr2_gripper, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0.2, 0, 0],
                             [0.500000, 0.300006, 0.700000])
print("pr2_cid")
print(pr2_cid)
#p.changeConstraint(pr2_cid2, gearRatio=-1, erp=0.5, relativePositionTarget=0, maxForce=100)

#----------------------------------------------------------------------

#启用可视化渲染
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

#设置重力 -10m/s²（沿 z 轴），开启实时仿真。
#有约束，没用
# p.setGravity(0.000000, 0.000000, 0.000000)
# p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

done = False
while (1):
    # p.setGravity(0, 0, -10)
    #用户通过输入触发机械手张开或闭合。
    input("enter to close the gripper")
    #控制关节，分别设置 0 和 2 号关节（左右夹爪）的位置、速度和力。
    p.setJointMotorControl2(pr2_gripper, 0, p.POSITION_CONTROL, 
                                targetPosition=0.7, maxVelocity=1,force=1)
    p.setJointMotorControl2(pr2_gripper, 2, p.POSITION_CONTROL, 
                                targetPosition=0.7, maxVelocity=1,force=1)
    input("enter to open the gripper")
    p.setJointMotorControl2(pr2_gripper, 0, p.POSITION_CONTROL, 
                                targetPosition=0.0, maxVelocity=1,force=1)
    p.setJointMotorControl2(pr2_gripper, 2, p.POSITION_CONTROL, 
                                targetPosition=0.0, maxVelocity=1,force=1)
#断开连接
p.disconnect()
#----------------------------------------------------------------------

# 获取机械手掌和物体的位置信息
def update_hand_orientation(hand_id, object_id, hand_link_index,safe_position):
    """
    更新机械手的朝向，但保持位置固定。
    """
    # 获取物体的位置
    obj_pos, _ = p.getBasePositionAndOrientation(object_id)
    
    # 获取机械手当前的位置和 orientation
    hand_state = p.getLinkState(hand_id, hand_link_index)
    hand_pos = hand_state[0]  # 保持位置不变

    # 计算从机械手指向物体的方向向量
    direction = np.array(obj_pos) - np.array(hand_pos)
    direction = normalize_vector(direction)

    # 计算新的 orientation，使机械手对准物体
    # 假设机械手默认的朝向是沿 Z 轴，可以用方向向量和 up_vector 确定四元数
    roll = np.arctan2(direction[1], direction[2] + 1e-8)  # 防止除以零
    # Pitch (绕 Y 轴): 方向向量在 X-Z 平面中的投影
    pitch = -np.arctan2(direction[0], direction[2] + 1e-8)
    yaw=np.arctan2(direction[1], direction[0]+ 1e-8)
    target_orientation = p.getQuaternionFromEuler([0,0,0])
    hand_ori = p.getQuaternionFromEuler([0,np.pi/2, 0])
    # 应用新的 orientation，保持位置不变
    #p.resetBasePositionAndOrientation(hand_id, safe_position, target_orientation)
    
    
#lifting
for step in range(100):
  target_z = safe_position[2] + step * 0.005  # 每一步逐渐升高
  p.changeConstraint(
    pr2_cid,
    [safe_position[0],safe_position[1],target_z]  # 设置新的目标位置
  )
  p.stepSimulation()  # 执行仿真
  time.sleep(0.01)  # 控制时间步，确保提起缓慢
  # 确保抓取完成
  
  
#以下是添加obj的代码
#address="data/objects/021_bleach_cleanser/poisson/textured.obj"
# basePos = [0, 0, 0]
# inertial = [0, 0, 0]
# shift = [0, 0, 0]
# meshScale = [1.5, 1.5, 1.5]
# initPos=[0.0, -0.4, 0]
# ObjIniOrientation = p.getQuaternionFromEuler([0, 0, 0]) 
# objShapeID = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=address, 
#                                 rgbaColor=[1, 0, 0, 1], specularColor=[0.4, .4, 0], 
#                                 visualFramePosition=shift, meshScale=meshScale)
# objCollisionID = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=address, 
#                         collisionFramePosition=shift, meshScale=meshScale)

# initPos[2] += 0.62

# objID = p.createMultiBody(baseMass=1,
#                     baseInertialFramePosition=inertial,
#                     baseCollisionShapeIndex=objCollisionID,
#                     baseVisualShapeIndex=objShapeID,
#                     basePosition=initPos,
#                     baseOrientation=ObjIniOrientation,
#                     useMaximalCoordinates=True)




def set_camera():
    # 设置相机的距离、角度和目标点
    camera_distance = 2  # 相机距离目标点的距离
    camera_yaw = 45      # 水平方向的角度
    camera_pitch = -30   # 垂直方向的角度
    target_position = [0.0, -0.4, 0.5]  # 相机对准的目标点

    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw,
        cameraPitch=camera_pitch,
        cameraTargetPosition=target_position
    )
