from __future__ import division
import pybullet as p
import pybullet_data
import numpy as np
import time
import math
import argparse


UR5_JOINT_INDICES = [0, 1, 2]


from collections import defaultdict


class Graph:
 
    graph = defaultdict(list)
    
    def addEdge(self, node, next_node):
        self.graph[tuple(node)].append(next_node)
        
class VertexGraph:
    
    graph_list = []
    
    def addVertex(self,node):
        if node not in self.graph_list:
            self.graph_list.append(node)


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


def find_path(graph, start, end, path =[]): 
    
    path = path + [start] 
    
    if start == end: 
        return path 
    
    for node in graph.graph[start]: 
        
        if node not in path: 
            newpath = find_path(graph, node, end, path) 
            
            if newpath:  
                return newpath 
            
            return None


def create_step(p1,p2):
    
    delta = 0.1
    
    if np.linalg.norm(p2-p1) < delta:
        
        return p2

    else:
        direction_vector = (p2 - p1) / np.linalg.norm(p2-p1)
        
        return p1 + delta * direction_vector


def extend_rrt(Graph, q_rand, g_v, g):
    dist = float('inf')
    
    
    for q in Graph.graph_list:
        curr_dist = math.sqrt( ((q_rand[0] - q[0]) ** 2) + ((q_rand[1] - q[1]) ** 2) + ((q_rand[2] - q[2]) ** 2) )
        if curr_dist < dist:
            dist = curr_dist
            q_near = list(q)
    
    q_new = create_step(np.array(q_near), np.array(q_rand))
    q_new = q_new.tolist()
    
    q_near_state = p.getLinkState(ur5, 3)[4]
    if collision_fn(q_new):
        #print 'invalid conf'   
        pass
    else:
        q_new_state = p.getLinkState(ur5, 3)[4]
        print q_near_state
        print q_new_state
        p.addUserDebugLine(q_near_state,q_new_state,[0, 1, 0], 1)
        #p.addUserDebugLine(q_near, q_new, [0, 1, 0], 1 )
        #time.sleep(2)
        
        g_v.addVertex(q_new)
        g.addEdge(q_near, q_new)

        return q_new
    
    return None
        


def rrt(max_iter, start_conf, end_conf):
    # ([0, 1, 2]) are [-2pi. 2pi], [-2pi, 2pi] and [-pi. pi]
    
    
    g = Graph()
    g_v = VertexGraph()
    g_v.addVertex(start_conf)
    
    for i in range(max_iter):
        rand_joint_3 = np.random.uniform(-np.pi, np.pi, 1)
        rand_joint_2 = np.random.uniform(2*-np.pi, 2*np.pi, 1)
        rand_joint_1 = np.random.uniform(2*-np.pi, 2*np.pi, 1)    
        
        rand_conf = [rand_joint_1, rand_joint_2, rand_joint_3]
        q_rand = [rand_conf[0][0], rand_conf[1][0], rand_conf[2][0]]
        
        q_new = extend_rrt(g_v, q_rand, g_v, g)    

        if q_new is not None:
            dist_to_goal = np.linalg.norm(np.array(end_conf) - np.array(q_new)) 
            if dist_to_goal < 0.5:
                #path_conf = find_path(g, start_conf, q_new)
                
                #print path_conf
                return 'goal found'

    pass


def birrt():
    #################################################
    # TODO your code to implement the birrt algorithm
    #################################################
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
    max_iter = 500
    
    
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
            path_conf = birrt()
    else:
        # using rrt
        path_conf = rrt(max_iter, start_conf, goal_conf)
        raw_input("no collision-free path is found within the time budget, finish?")
    if path_conf is None:
        # pause here

        raw_input("no collision-free path is found within the time budget, finish?")
        
    #else:
        ###############################################
        # TODO your code to highlight the solution path
        ###############################################
        
        
        #p.getJointState(ur5, <joint index>)
        
        
        
        
        # execute the path
        #while True:
        #    for q in path_conf:
        #        set_joint_positions(ur5, UR5_JOINT_INDICES, q)
        #        time.sleep(0.5)
