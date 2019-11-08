from __future__ import division
from collections import defaultdict
import pybullet as p
import pybullet_data
import numpy as np
import time
import math
import argparse


UR5_JOINT_INDICES = [0, 1, 2]


def find_path(graph, start, end, path =[]): 
    
    path = path + [start]
    
    if tuple(start) == tuple(end):
        return path 

    for node in graph[tuple(start)]:    
        node = tuple(node)
        if node not in path:
            
            nextpath = find_path(graph, node, end, path) 

            if nextpath:  
                return nextpath 


def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        p.resetJointState(body, joint, value)


def draw_sphere_marker(position, radius, color):
   vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
   marker_id = p.createMultiBody(basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
   return marker_id


def remove_marker(marker_id):
   p.removeBody(marker_id)


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--birrt', action='store_true', default=False)
    parser.add_argument('--smoothing', action='store_true', default=False)
    args = parser.parse_args()
    return args


def create_step(p1,p2):
    
    delta = 0.075
    
    if np.linalg.norm(p2-p1) < delta:
        return p2

    else:
        direction_vector = (p2 - p1) / np.linalg.norm(p2-p1)
        return p1 + delta * direction_vector


def extend_rrt(q_near, q_rand):

    q_new = create_step(np.array(q_near), np.array(q_rand))
    q_new = q_new.tolist()
    
    if collision_fn(q_new):   
        pass
    else:
        set_joint_positions(ur5, UR5_JOINT_INDICES, q_near)
        q_near_state = p.getLinkState(ur5, 3)[0]
        
        set_joint_positions(ur5, UR5_JOINT_INDICES, q_new)
        q_next_state = p.getLinkState(ur5, 3)[0]
        p.addUserDebugLine(q_near_state,q_next_state,[0, 1, 0], 1)
        
        return q_near, q_new
    
    return None, None
        
def dist_fn(p1, p2):
    return math.sqrt( ((p2[0] - p1[0]) ** 2) + ((p2[1] - p1[1]) ** 2) + ((p2[2] - p1[2]) ** 2) )

def rrt(max_iter, start_conf, end_conf):
    
    graph = defaultdict(list)
    graph_list = []
    graph_list.append(start_conf)
    
    bias = 0.05 * max_iter
    counter = 0
    
    for i in range(max_iter):
        if counter == bias:
            q_rand = end_conf
            counter = 0
        else:
            rand_joint_3 = np.random.uniform(-np.pi, np.pi, 1)
            rand_joint_2 = np.random.uniform(2*-np.pi, 2*np.pi, 1)
            rand_joint_1 = np.random.uniform(2*-np.pi, 2*np.pi, 1)    
            counter += 1    
            
            rand_conf = [rand_joint_1, rand_joint_2, rand_joint_3]
            q_rand = [rand_conf[0][0], rand_conf[1][0], rand_conf[2][0]]
        
        dist = float('inf')
        for q in graph_list:
            curr_dist = dist_fn(q, q_rand) 
            if curr_dist < dist:
                dist = curr_dist
                q_near = list(q)
        
        q_near, q_new = extend_rrt(q_near, q_rand)    

        if q_new is not None:
            graph_list.append(q_new)
            graph[tuple(q_near)].append(q_new)
            
            dist_to_goal = dist_fn(q_new, end_conf) 
            
            if dist_to_goal <= 0.075:
                
                graph_list.append(end_conf)
                graph[tuple(q_new)].append(end_conf)
                
                path_conf = find_path(graph, start_conf, end_conf)
                
                return path_conf
        
    pass


def birrt(max_iter, start_conf, end_conf):
    
    graph_1 = defaultdict(list)
    graph_list_1 = []
    graph_list_1.append(start_conf)
    
    graph_2 = defaultdict(list)
    graph_list_2 = []
    graph_list_2.append(end_conf)
    
    count = 1
    for i in range(max_iter):
        
        if count % 2 == 0:
            T_1 = graph_1
            T_1_list = graph_list_1
            T_2 = graph_2
            T_2_list = graph_list_2
        else:
            T_1 = graph_2
            T_1_list = graph_list_2
            T_2 = graph_1
            T_2_list = graph_list_1
        
        rand_joint_3 = np.random.uniform(-np.pi, np.pi, 1)
        rand_joint_2 = np.random.uniform(2*-np.pi, 2*np.pi, 1)
        rand_joint_1 = np.random.uniform(2*-np.pi, 2*np.pi, 1)    
   
        
        rand_conf = [rand_joint_1, rand_joint_2, rand_joint_3]
        q_rand = [rand_conf[0][0], rand_conf[1][0], rand_conf[2][0]]
    
        dist = float('inf')
        for q in T_1_list:
            curr_dist = dist_fn(q, q_rand) 
            if curr_dist < dist:
                dist = curr_dist
                q_near_1 = list(q)
        
        q_near_1, q_new_1 = extend_rrt(q_near_1, q_rand)    

        if q_new_1 is not None:
            T_1_list.append(q_new_1)
            T_1[tuple(q_near_1)].append(q_new_1)

            dist = float('inf')
            for q in T_2_list:
                curr_dist = dist_fn(q, q_new_1) 
                if curr_dist < dist:
                    dist = curr_dist
                    q_near_2 = list(q)
        
            q_near_2, q_new_2 = extend_rrt(q_near_2, q_new_1) 
            
            if q_new_2 is not None:
                T_2_list.append(q_new_2)
                T_2[tuple(q_near_2)].append(q_new_2)
                
                dist_to_connect = dist_fn(q_new_1, q_new_2)    
                
                if dist_to_connect < 0.1:
                    
                    graph_1[tuple(q_new_1)].append(q_new_2)
                    
                    graph_2[tuple(q_new_2)].append(q_new_1)
                    #p.addUserDebugLine(q_near_2,q_new_2,[0, 1, 0], 4.0)
                    path_conf_1 = find_path(graph_1, start_conf, q_new_2)
                    path_conf_2 = find_path(graph_2, end_conf, q_new_1)

                    for q in reversed(path_conf_2):
                        path_conf_1.append(q)
                    path_conf = path_conf_1    
                
                    return path_conf
        count += 1    
            
            
    
    pass


def birrt_smoothing():
    ################################################################
    # TODO your code to implement the birrt algorithm with smoothing
    ################################################################
    pass


if __name__ == "__main__":
    args = get_args()

    # set up simulator
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True)
    p.resetDebugVisualizerCamera(cameraDistance=1.400, cameraYaw=58.000, cameraPitch=-42.200, cameraTargetPosition=(0.0, 0.0, 0.0))

    # load objects
    plane = p.loadURDF("plane.urdf")
    ur5 = p.loadURDF('assets/ur5/ur5.urdf', basePosition=[0, 0, 0.02], useFixedBase=True)
    obstacle1 = p.loadURDF('assets/block.urdf',
                           basePosition=[1/4, 0, 1/2],
                           useFixedBase=True)
    obstacle2 = p.loadURDF('assets/block.urdf',
                           basePosition=[2/4, 0, 2/3],
                           useFixedBase=True)
    obstacles = [plane, obstacle1, obstacle2]

    # start and goal
    start_conf = (-0.813358794499552, -0.37120422397572495, -0.754454729356351)
    start_position = (0.3998897969722748, -0.3993956744670868, 0.6173484325408936)
    goal_conf = (0.7527214782907734, -0.6521867735052328, -0.4949270744967443)
    goal_position = (0.35317009687423706, 0.35294029116630554, 0.7246701717376709)
    goal_marker = draw_sphere_marker(position=goal_position, radius=0.02, color=[1, 0, 0, 1])
    set_joint_positions(ur5, UR5_JOINT_INDICES, start_conf)

    
    # additional variables
    max_iter = 5000
    
    
    # place holder to save the solution path
    path_conf = None

    # get the collision checking function
    from collision_utils import get_collision_fn
    collision_fn = get_collision_fn(ur5, UR5_JOINT_INDICES, obstacles=obstacles,
                                       attachments=[], self_collisions=True,
                                       disabled_collisions=set())

    if args.birrt:
        if args.smoothing:
            # using birrt with smoothing
            path_conf = birrt_smoothing()
        else:
            # using birrt without smoothing
            path_conf = birrt(max_iter, start_conf, goal_conf)
    else:
        # using rrt
        path_conf= rrt(max_iter, start_conf, goal_conf)
        
    if path_conf is None:
        # pause here
        raw_input("no collision-free path is found within the time budget, finish?")
        
    else:        
        
        
        
        if args.birrt:
            count = 0
            for q in path_conf:            
                q_start = p.getLinkState(ur5, 3)[0]
                #d = dist_fn(q_start, path_conf[len(path_conf)-1])
                set_joint_positions(ur5, UR5_JOINT_INDICES, q)
                q_end = p.getLinkState(ur5, 3)[0]  
                if count > 1:
                    p.addUserDebugLine(q_start,q_end,[1, 0, 0], 4.0)
                count += 1
        else:
            count = 0
            for q in path_conf:            
                q_start = p.getLinkState(ur5, 3)[0]
                d = dist_fn(q_start, goal_position)
                set_joint_positions(ur5, UR5_JOINT_INDICES, q)
                q_end = p.getLinkState(ur5, 3)[0]  
                if d > 0.07:
                    p.addUserDebugLine(q_start,q_end,[1, 0, 0], 4.0)
                if d < 0.07 and count > 2:
                    p.addUserDebugLine(q_start,q_end,[1, 0, 0], 4.0)
                count += 1
            
            


        # execute the path
        
        while True:
            for q in path_conf:
                set_joint_positions(ur5, UR5_JOINT_INDICES, q)
                time.sleep(0.5)
