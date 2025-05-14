import time
import socket
import json
import threading
import tkinter as tk
from tkinter import messagebox
import robodk as rdk
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

_side_length = 200  # Default side length for the square trajectory
_subdivision = 30

def compute_trajectory(side_length):
    print(f"Computing trajectory for side length: {side_length}")
    
    # angles = np.linspace(0, 6*np.pi, _subdivision)
    # # x = center[0] + side_length * np.cos(angles)
    # # y = center[2] + side_length * np.sin(angles)
    # x =   side_length * angles
    # y =  side_length * angles

    # traj = list(zip(x, y))

    # return traj
## Fancy Trajectory
###################################################
    # srt3 = 3**0.5
    # side = _side_length
    # height = side / 2
    # length = height * srt3
    # first_int = side/(1+srt3)
    # first_x = -first_int / 2 * srt3
    # first_y = (side - first_int )/ 2

    # side_minor = _side_length/5
    # sm2 = side_minor / 2
    # sm1 = side_minor * srt3 / 2

    # points = [
    #     [0, 0],  # Bottom-left corner
    #     [first_x,first_y],
    #     [-sm1, height-sm2],
    #     [-2*sm1, height],
    #     [-sm1, height + sm2],
    #     [0, height],
    #     [sm1, height+sm2],
    #     [sm1*2, height],
    #     [sm1, height-sm2], #End of first side
    #     [length-sm1, sm2],
    #     [length, 2*sm2],
    #     [length+sm1, sm2],
    #     [length, 0],
    #     [length+sm1, -sm2],
    #     [length, -2*sm2],
    #     [length-sm1, -sm2], #End of second side
    #     [sm1, -height+sm2],
    #     [2*sm1, -height],
    #     [sm1, -height-sm2],
    #     [0, -height],
    #     [-sm1, -height-sm2],
    #     [-2*sm1, -height],
    #     [-sm1, -height+sm2], #End of third side
    #     [-length+sm1, -sm2],
    #     [-length, -2*sm2],
    #     [-length-sm1, -sm2],
    #     [-length, 0],
    #     [-length-sm1, sm2],
    #     [-length, sm2*2],
    #     [-length+sm1, sm2],
    #     [first_x,first_y],
    # ]

    # # Define the spacing between trajectory points
    # spacing = (_side_length * 4 / _subdivision)

    # trajectory = []
    # for i in range(len(points) - 1):
    #     start = np.array(points[i])
    #     end = np.array(points[i + 1])
    #     direction = end - start
    #     distance = np.linalg.norm(direction)
    #     direction = direction / distance  # Normalize the direction vector
    #     num_points = int(distance / spacing)
    #     for j in range(num_points + 1):
    #         point = start + j * spacing * direction
    #         trajectory.append(point.tolist())

    # print(len(trajectory))

    # Home pose is (x:550, y:-1000, z:475, xrot:0, yrot:-90, zrot:-180). Trajectories are changes from to this pose.
    trajectory = [
        [0,0,0,0,0,0],
        [50,-300,0,0,-45,0],
        [50,-300,-400,0,-45,0],
        [50,-300,0,0,-45,0],
        [0,0,0,0,0,0]
        ]
    # Permuated
    trajectory = [
        [0,0,0,0,0,0],
        [50,0,-300,0,-45,0],
        [50,-400,-300,0,-45,0],
        [50,0,-300,0,-45,0],
        [0,0,0,0,0,0]
        ]
    return trajectory

####################################################

def inverse_jacobian_control(iter,trajectory):
    # Placeholder for inverse jacobian control logic
    # This function should determine the needed velocity for a given duration
    # based on the current robot pose and the desired target pose
    current_pose = np.array(get_modified_pose()).flatten() # Get current joint angles (pose)

    targets = trajectory
    target_pose = targets[iter]
    print(f"Going to {target_pose}")

    # error[0] = (target_pose[0] - current_pose[0])+550
    # error[1] = - current_pose[1] + target_pose[1]
    # error[2] = ( - current_pose[2])+475
    # error[3] = - current_pose[3]
    # error[4] = -90 -current_pose[4]
    # error[5] = -180 - current_pose[5]

    # error[0] = (target_pose[0] - current_pose[0])
    # error[1] = - current_pose[1] 
    # error[2] = ( - current_pose[2])+ target_pose[1]
    # error[3] = - current_pose[3]
    # error[4] = -90 -current_pose[4]
    # error[5] = -180 - current_pose[5]

    # Calcualte errors for Jacobian control
    error = np.zeros(6)
    error[0] = ( - current_pose[0]) + target_pose[0]
    error[1] = -1000 - current_pose[1] + target_pose[1]
    error[2] = ( - current_pose[2]) + target_pose[2]
    error[3] = - current_pose[3] + target_pose[3]
    error[4] = -90 - current_pose[4] + target_pose[4]
    error[5] = -180 - current_pose[5] + target_pose[5]
    # print("current_pose : ", current_pose)
    # print("target_pose : ", target_pose)
    # print("Error : ", error)

    # Calculate control
    jacobian = get_jacobian()
    inverse_jacobian = np.linalg.pinv(jacobian)
    velocity = np.matmul(inverse_jacobian, np.transpose(error))

    #Test attacks here
    # attack_matrix_permute_yz = np.array([[1,0,0,0,0,0],
    #                           [0,0,1,0,0,0],
    #                           [0,1,0,0,0,0],
    #                           [0,0,0,1,0,0],
    #                           [0,0,0,0,1,0],
    #                           [0,0,0,0,0,1]])
    
    # velocity = np.dot(attack_matrix_permute_yz, velocity)
    # print("Velocity : ", velocity)
    # print("cmd_velocity : ", velocity)

    return zip(error,velocity)

def send_request_to_attacker(data):
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(('127.0.0.1', 5000))
    client.send(json.dumps(data).encode())
    response = client.recv(1024).decode()
    client.close()
    return json.loads(response)

def send_velocity_to_attacker(velocity):
    data = {"type": "attack_move", "velocity": velocity}
    return send_request_to_attacker(data)

def get_monitor_value():
    data = {"type": "get_monitor_value"}
    return send_request_to_attacker(data)['smsf_output']

def get_modified_pose():
    data = {"type": "modified_pose"}
    return rdk.Mat(send_request_to_attacker(data)['modified_pose'])

def detect_attack(true_values, modified_values):
    # Placeholder for attack detection logic
    pass

def ping_server():
    try:
        data = {"type": "ping"}
        response = send_request_to_attacker(data)
        return response.get("status") == "success"
    except Exception as e:
        return False

def get_jacobian():
    data = {"type": "get_jacobian"}
    response = send_request_to_attacker(data)
    return response['jacobian']

class ControllerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Controller")
        self.error_data = [[], [], []]
        
        self.start_button = tk.Button(root, text="Start", command=self.start_controller)
        self.start_button.grid(row=0, column=0, padx=10, pady=10)
        
        self.stop_button = tk.Button(root, text="Stop", command=self.stop_controller)
        self.stop_button.grid(row=0, column=1, padx=10, pady=10)
        
        self.ping_button = tk.Button(root, text="Ping Server", command=self.ping_server_status)
        self.ping_button.grid(row=1, column=0, columnspan=2, padx=10, pady=10)
        
        self.trajectory_label = tk.Label(root, text="Enter side length for trajectory:")
        self.trajectory_label.grid(row=2, column=0, columnspan=1, padx=10, pady=10)
        
        self.trajectory_entry = tk.Entry(root)
        self.trajectory_entry.grid(row=2, column=1, columnspan=1, padx=10, pady=10)
        
        self.compute_button = tk.Button(root, text="Compute Trajectory", command=self.compute_trajectory_button)
        self.compute_button.grid(row=2, column=2, columnspan=1, padx=10, pady=10)
        
        self.jacobian_button = tk.Button(root, text="Print Jacobian", command=self.print_jacobian)
        self.jacobian_button.grid(row=3, column=0, columnspan=2, padx=10, pady=10)
        
        self.status_label = tk.Label(root, text="Status: Stopped")
        self.status_label.grid(row=4, column=0, columnspan=1, padx=10, pady=10)
        
        self.connection_label = tk.Label(root, text="Connection: Not Connected")
        self.connection_label.grid(row=4, column=1, columnspan=1, padx=10, pady=10)
        

                # Create a matplotlib figure for real-time plotting
        self.fig = Figure(figsize=(13, 8), dpi=100)
        self.ax = self.fig.add_subplot(221)
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().grid(row=0, column=3, rowspan = 4,columnspan=1, padx=10, pady=10)

        self.ax.set_title("Real-Time Velocity Plot")
        self.ax.set_xlabel("Time Step")
        self.ax.set_ylabel("Velocity")

        self.ax1 = self.fig.add_subplot(222)  # 2 rows, 1 column, position 2
        self.ax1.set_title("Real-Time Error Plot")
        self.ax1.set_xlabel("Time Step")
        self.ax1.set_ylabel("Error")
        
        trajectory = np.array(compute_trajectory(_side_length))

        # if trajectory.shape[1] == 2:  # If the trajectory is 2D
        #     trajectory = np.hstack((trajectory, np.zeros((trajectory.shape[0], 1))))

        self.ax2 = self.fig.add_subplot(223)  # 2 rows, 1 column, position 2
        self.ax2.scatter(trajectory[:, 2], trajectory[:, 3], c='blue', label="Trajectory Points")
        self.ax2.set_title("Trajectory Plot")
        self.ax2.set_xlabel("X")
        self.ax2.set_ylabel("Y")
        self.running = False
        self.thread = None

        self.check_connection()
    

    def plot_trajectory_3d(self, trajectory):
        self.ax2.clear()
        trajectory = np.array(trajectory)
        # if trajectory.shape[1] == 2:
        #     trajectory = np.hstack((trajectory, np.zeros((trajectory.shape[0], 1))))
        self.ax2.scatter(trajectory[:, 0], trajectory[:, 1], c='blue', label="Trajectory Points")
        self.ax2.set_title("3D Trajectory")
        self.ax2.set_xlabel("X")
        self.ax2.set_ylabel("Y")
        # self.ax2.set_zlabel("Z")
        self.ax2.legend()
        self.canvas.draw()

    def check_connection(self):
        if ping_server():
            self.connection_label.config(text="Connection: Connected")
        else:
            self.connection_label.config(text="Connection: Not Connected")
        self.root.after(5000, self.check_connection)

    def ping_server_status(self):
        if ping_server():
            messagebox.showinfo("Ping Result", "Connection: Connected")
        else:
            messagebox.showerror("Ping Result", "Connection: Not Connected")

    def compute_trajectory_button(self):
        try:
            side_length = float(self.trajectory_entry.get())
            traj = compute_trajectory(side_length)
            if traj:
                messagebox.showinfo("Success", "Trajectory computation successful!")
                global _side_length
                _side_length = side_length

                self.plot_trajectory_3d(traj)

            else:
                messagebox.showerror("Error", "Trajectory computation failed.")
        except ValueError:
            messagebox.showerror("Error", "Invalid input. Please enter a valid number.")

    def print_jacobian(self):
        try:
            jacobian = get_jacobian()
            print("Jacobian Matrix:")
            for row in jacobian:
                print(row)
            messagebox.showinfo("Jacobian", "Jacobian matrix printed to console.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to get Jacobian: {e}")

    def update_plot(self, _error_data,_iter):
        self.ax.clear()

        self.ax.set_title("Real-Time Velocity Plot")
        self.ax.set_xlabel("Time Step")
        self.ax.set_ylabel("Velocity")

        self.ax.plot(_error_data[0], label="Joint 1")
        self.ax.plot(_error_data[1], label="Joint 2")
        self.ax.plot(_error_data[2], label="Joint 3")
        self.ax.plot(_error_data[3], label="Joint 4")
        self.ax.plot(_error_data[4], label="Joint 5")
        self.ax.plot(_error_data[5], label="Joint 6")
        self.ax.legend()

        self.ax1.clear()

        self.ax1.set_title("Real-Time Error Plot")
        self.ax1.set_xlabel("Time Step")
        self.ax1.set_ylabel("Error")

        self.ax1.plot(_error_data[6], label="Error X")
        self.ax1.plot(_error_data[7],label="Error Y")
        self.ax1.plot(_error_data[8],label="Error Z")
        # self.ax1.plot(_error_data[9],label="Error W")
        # self.ax1.plot(_error_data[10],label="Error P")
        # self.ax1.plot(_error_data[11],label="Error R")
        self.ax1.legend()
        self.canvas.draw()

    def start_controller(self):
        if not self.running:
            self.running = True
            self.status_label.config(text="Status: Running")
            self.thread = threading.Thread(target=self.controller_loop)
            self.thread.start()

    def stop_controller(self):
        if self.running:
            self.running = False
            send_request_to_attacker({"type": "reset"})
            self.status_label.config(text="Status: Stopped")
            if self.thread:
                self.thread.join()

    def controller_loop(self):

        duration = 1  # 1 second interval
        iter = 0
        _traj = compute_trajectory(_side_length)
        length = len(_traj)
        velocities=[]
        errors = []
        while self.running:

            controller_out = inverse_jacobian_control(iter,_traj)
            unzip = list(zip(*controller_out))
            velocity = np.array(unzip[1])
            error = np.array(unzip[0])
            velocities.append(velocity.tolist())  # Correct way to append
            errors.append(error.tolist())
            if iter == 50:
                joint_vel_list = []
                error_list = []
            joint_vel_list = list(zip(*velocities))  # Convert to list for safe indexing
            error_list = list(zip(*errors))
            plot_list = joint_vel_list + error_list 
            self.update_plot(plot_list, iter)
            
            send_request_to_attacker({"type": "attack_move", "velocity": velocity.tolist()})
            iter += 1

            # If the trajectory is completed, reset the robot
            if iter == length:
                send_request_to_attacker({"type": "reset"})
                self.running = False
            
            # time.sleep(0.01)

if __name__ == "__main__":
    root = tk.Tk()
    app = ControllerApp(root)
    root.mainloop()