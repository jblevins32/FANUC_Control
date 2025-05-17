import socket
import json
import robodk as rdk
import numpy as np
from attacker import attack_control, attack_obs

class Server():
    def __init__(self):
        self.attack_undet = False
        self.attack_det = False
        self.attack_type = None

    def get_joints_from_control(self, velocity):
        ''' Get joint angle targets from velocity command which we will move to (this is the plant doing the control action)'''

        timestep = 1
        jacb = self.get_jacobian(robot, robot.Joints())

        ### attack control ###
        if self.attack_det:
            velocity = attack_control(velocity, jacb, self.attack_type)

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


    def get_modified_pose(self, altered=False):
        ''' Get pose of robot with respect to starting position '''
        orig_pose = rdk.Pose_2_Fanuc(robot.Pose())
        mod_pose = orig_pose

        # Modify the pose by removing the starting position
        mod_pose[0], mod_pose[1], mod_pose[2], mod_pose[3], mod_pose[4], mod_pose[5] = orig_pose[0]-self.start_pose[0], orig_pose[1]-self.start_pose[1], orig_pose[2]-self.start_pose[2], orig_pose[3]-self.start_pose[3], orig_pose[4]-self.start_pose[4], orig_pose[5]-self.start_pose[5]
        
        ### attack obs ###
        if (self.attack_det and altered) or self.attack_undet:
            mod_pose = attack_obs(mod_pose, self.attack_type)
            print('altered')

        return mod_pose
    
    def get_jacobian(self, robot, joint_angles):
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

    def handle_client_connection(self, client_socket):
        ''' Handle client requests '''

        request = client_socket.recv(1024).decode()
        data = json.loads(request)
        response = {}

        if data['type'] == 'move':
            ''' Move robot to a specific pose '''
            velocity = data['velocity']
            target_joints = self.get_joints_from_control(velocity)
            robot.MoveJ(target_joints,blocking=True)
            response = {"status": "success"}

        elif data['type'] == 'get_modified_pose':
            ''' Get pose of robot with respect to starting position '''
            modified_pose = self.get_modified_pose(data['altered'])
            response = {"modified_pose": modified_pose}

        elif data['type'] == 'ping':
            response = {"status": "success"}

        elif data['type'] == 'reset':
            print("Resetting robot to home position...")
            robot.MoveJ(robot.JointsHome())
            self.start_pose = rdk.Pose_2_Fanuc(robot.Pose()) # Get starting pose to remove from movements elsewhere
            response = {"status": "Reset successful"}

        elif data['type'] == 'get_jacobian':
            jacobian = self.get_jacobian(robot, robot.Joints())
            response = {"jacobian": jacobian.tolist()}

        elif data['type'] == 'attack_type':
            self.attack_type = data['attack']

        elif data['type'] == 'attack_det':
            if self.attack_det == False:
                self.attack_det = True
                response = {"status": "Detectable attack applied"}
            else:
                self.attack_det = False
                response = {"status": "Detectable attack removed"}

        elif data['type'] == 'attack_undet':
            if self.attack_undet == False:
                self.attack_undet = True
                response = {"status": "Undetectable attack applied"}
            else:
                self.attack_undet = False
                response = {"status": "Undetectable attack removed"}

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
    server_instance = Server()

    while True:
        # Wait until a client connects
        client_sock, addr = server.accept() 
        print(f'Accepted connection from {addr}')

        # Handle the client connection once connected
        server_instance.handle_client_connection(client_sock) 

