import socket
import json
import robodk as rdk
import numpy as np

def robotIK(robot, pose):
    ''' Inverse Kinematics: Get joint angles given a pose '''

    ikSol = robot.SolveIK(pose)

    return ikSol

def get_joints_from_control(velocity, attack):
    ''' Get joint angle targets from velocity command which we will move to (this is the plant doing the control action)'''

    timestep = 1
    jacb = get_jacobian(robot, robot.Joints())

    # y reflection attack
    velocity = attack_control(velocity, jacb, attack)

    target_dot = np.matmul(jacb,velocity)
    target = target_dot.__mul__(timestep)
    robot_xyzrpw = rdk.Pose_2_Fanuc(robot.Pose())
    target = (robot_xyzrpw + target).tolist()
    target_mod = rdk.xyzrpw_2_pose(target) # Gives homogenous matrix of target pose


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


def get_modified_pose(attack):
    ''' Get pose of robot with respect to starting position '''
    orig_pose = rdk.Pose_2_Fanuc(robot.Pose())
    mod_pose = orig_pose
    mod_pose[0], mod_pose[2], mod_pose[4], mod_pose[5] = orig_pose[0]-550, orig_pose[2]-475, orig_pose[4]+90, orig_pose[5]+180
    
    # y reflection attack

    mod_pose = attack_obs(mod_pose, attack)

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

def attack_control(control, jacb, attack):
    if attack == None:
        return control
    elif attack == 'reflect-y':
        # 6x6 reflection matrix across y axis
        reflection = np.linalg.inv(jacb) @ np.array([
                            [1, 0, 0, 0, 0, 0],
                            [0, -1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]]) @ jacb
        control = np.dot(reflection,np.array(control))
    return control

def attack_obs(pose, attack):
    # Adjust observable for specific attacks
    if attack == None:
        return pose
    elif attack == 'reflect-y':
        pose[1] = -pose[1]
    
    return pose

def handle_client_connection(client_socket):
    ''' Handle client requests '''

    request = client_socket.recv(1024).decode()
    data = json.loads(request)
    response = {}

    if data['type'] == 'move':
        ''' Move robot to a specific pose '''
        # Attacker should attack control here!
        velocity = data['velocity']
        target_joints = get_joints_from_control(velocity, data['attack'])
        robot.MoveJ(target_joints,blocking=True)
        response = {"status": "success"}

    elif data['type'] == 'modified_pose':
        ''' Get pose of robot with respect to starting position '''
        pose = get_pose_from_api()
        modified_pose = get_modified_pose(attack=data['attack'])
        response = {"modified_pose": modified_pose}

    elif data['type'] == 'get_monitor_value':
        # Attacker should attack observations here!
        pose = get_pose_from_api()
        smsf_output = compute_smsf(pose)
        response = {"smsf_output": smsf_output}

    elif data['type'] == 'ping':
        response = {"status": "success"}

    elif data['type'] == 'reset':
        print("Resetting robot to home position...")
        robot.MoveJ(robot.JointsHome())
        response = {"status": "Reset successful"}

    elif data['type'] == 'get_jacobian':
        jacobian = get_jacobian(robot, robot.Joints())
        response = {"jacobian": jacobian.tolist()}

    # Send response back to the client
    client_socket.send(json.dumps(response).encode())
    client_socket.close()

if __name__ == '__main__':

    # Setup server-client connection to RoboDK (server.py is server, robodk is client)
    RDK = rdk.robolink.Robolink('127.0.0.1', 20500)

    # Connect to RoboDK
    RDK.Connect()
    # RDK.setRunMode(6) # Uncomment for running physical robot

    # Set robot parameters
    robot = RDK.Item('', 2)
    robot.setRounding(-1)
    robot.setSpeed(100)
    if not robot.Valid():
        print("Failed to find the robot.")
        exit(1)

    # Initialize server-client connection looking for IPv4 IP addresses and TCP (controller.py)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('0.0.0.0', 5000)) # Any IP address on port 5000 can connect
    server.listen(5) # Listen with a queue of 5 connections
    print('Listening on port 5000...')

    while True:
        # Wait until a client connects
        client_sock, addr = server.accept() 
        print(f'Accepted connection from {addr}')

        # Handle the client connection once connected
        handle_client_connection(client_sock) 

