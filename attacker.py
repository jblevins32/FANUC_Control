import numpy as np

def attack_control(control, jacb, attack):
    if attack == None:
        return control
    elif attack == 'reflect-x':
        # 6x6 reflection matrix across y axis
        reflection = np.linalg.inv(jacb) @ np.array([
                            [-1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]]) @ jacb
        control = np.dot(reflection,np.array(control))
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
    elif attack == 'reflect-z':
        # 6x6 reflection matrix across y axis
        reflection = np.linalg.inv(jacb) @ np.array([
                            [1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, -1, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]]) @ jacb
        control = np.dot(reflection,np.array(control))
    return control

def attack_obs(pose, attack):
    # Adjust observable for specific attacks
    if attack == None:
        return pose
    elif attack == 'reflect-x':
        pose[0] = -pose[0]
    elif attack == 'reflect-y':
        pose[1] = -pose[1]
    elif attack == 'reflect-z':
        pose[2] = -pose[2]    
    
    return pose