import socket
import json
import robodk as rdk
import numpy as np

def robotIK(robot, pose):
    ikSol = robot.SolveIK(pose)
    return ikSol

def apply_attack_to_velocity(velocity, attack_matrix):

    timestep = 1
    jacb = get_jacobian(robot, robot.Joints())

    target_dot = np.matmul(jacb,velocity)
    target = target_dot.__mul__(timestep)
    robot_xyzrpw = rdk.Pose_2_Fanuc(robot.Pose())
    target = (robot_xyzrpw + target).tolist()
    target_mod = rdk.xyzrpw_2_pose(target)

    # print("target: ", target)
    # print("Target: ", target_mod)
    # print("Target Dot: ", target_dot)
    # print("Jacobian: ", jacb)
    # print("Velocity: ", velocity)
    # print("Robot Pose: ", robot.Pose())
    # print("Robot XYZRPW: ", robot_xyzrpw)
    # print("Robot Joint Angles: ", robot.Joints())


    #SOLVE FOR ALL IK SOLUTIONS
    ikSol = robot.SolveIK_All(target_mod)
    #Check for configurations of the solutions
    for sol in ikSol:
        config = robot.JointsConfig(sol).list()
        if config[0] == 0 and config[1] == 0:
            target_mod = sol
            break

    # print("Target Modified: ", target_mod)
    return target_mod


def apply_attack_to_pose():
    orig_pose = rdk.Pose_2_Fanuc(robot.Pose())
    mod_pose = orig_pose
    mod_pose[0], mod_pose[2] = orig_pose[0]-550, orig_pose[2]-475

    # mod_pose[1], mod_pose[2] = mod_pose[2], mod_pose[1]
    
    return mod_pose
def get_pose_from_api():
    return robot.Pose()

def compute_smsf(pose):
    return pose  

def get_jacobian(robot, joint_angles):
    robot.setJoints(joint_angles)
    joint_poses = robot.JointPoses(joint_angles)
    
    z_vectors = [rdk.Mat(joint_poses[i]).VZ() for i in range(1, 7)]
    end_effector = np.array(rdk.Mat(joint_poses[6]).Pos())
    
    p_vectors = [end_effector - np.array(rdk.Mat(joint_poses[i]).Pos()) for i in range(1, 7)]
    
    J = np.zeros((6, 6))
    for i in range(6):
        J[0:3, i] = np.cross(z_vectors[i], p_vectors[i])
        J[3:6, i] = z_vectors[i]
    
    return J

def handle_client_connection(client_socket):
    request = client_socket.recv(1024).decode()
    data = json.loads(request)
    response = {}

    if data['type'] == 'attack_move':
        velocity = data['velocity']
        modified_velocity = apply_attack_to_velocity(velocity, attack_matrix)
        robot.MoveJ(modified_velocity,blocking=True)
        response = {"status": "success"}

    elif data['type'] == 'modified_pose':
        pose = get_pose_from_api()
        modified_pose = apply_attack_to_pose()
        response = {"modified_pose": modified_pose}

    elif data['type'] == 'get_monitor_value':
        pose = get_pose_from_api()
        smsf_output = compute_smsf(pose)
        response = {"smsf_output": smsf_output}

    elif data['type'] == 'ping':
        response = {"status": "success"}

    elif data['type'] == 'reset':
        robot.MoveJ(robot.JointsHome())
        response = {"status": "Reset successful"}

    elif data['type'] == 'get_jacobian':
        jacobian = get_jacobian(robot, robot.Joints())
        response = {"jacobian": jacobian.tolist()}

    client_socket.send(json.dumps(response).encode())
    client_socket.close()

if __name__ == '__main__':
    RDK = rdk.robolink.Robolink('127.0.0.1', 20500)

    # Setting for running on robot
    RDK.Connect()
    RDK.setRunMode(6) # Uncomment for running physical robot

    robot = RDK.Item('', 2)
    robot.setRounding(-1)
    robot.setSpeed(100)
    if not robot.Valid():
        print("Failed to find the robot.")
        exit(1)

    attack_matrix = [] 

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('0.0.0.0', 5000))
    server.listen(5)
    print('Listening on port 5000...')

    while True:
        # print(robot.Pose())
        client_sock, addr = server.accept()
        handle_client_connection(client_sock)