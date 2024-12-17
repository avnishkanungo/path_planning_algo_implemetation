from env_creation import generate_random_environment, display_and_save_environment,sampleAPoint, sampleNonObstaclePoint, treeNode
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import random
import math


def env_setup():
    grid_size = (20, 20)  # Grid size (height, width)
    obstacle_density = 0.05  # Density of obstacles


    # Generate random environment
    environment = generate_random_environment(grid_size, obstacle_density)
    # x = sampleNonObstaclePoint(environment)

    return environment

class RRTAlgorithm:
    def __init__(self, start, goal, numIterations, grid, stepSize):
        self.randomTree = treeNode(start[0], start[1])         #The RRT (root position)
        self.goal = treeNode(goal[0], goal[1])                 #goal position
        self.nearestNode = self.randomTree                                #nearest node            
        self.iterations = min(numIterations, 1000000)              #number of iterations to run
        self.grid = grid                                       #the map
        self.rho = stepSize                                    #length of each branch 
        self.path_distance = 0                                 #total path distance  
        self.nearestDist = 100                                 #distance to nearest node (initialize with large)
        self.numWaypoints = 0                                  #number of waypoints
        self.Waypoints = []                                    #the waypoints

    def addChild(self, locationX, locationY):
        if (locationX == self.goal.locationX):
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode
        else:
            temp = treeNode(locationX,locationY)
            self.nearestNode.children.append(temp)
            temp.parent = self.nearestNode

        #steer a distance stepSize from start location to end location (keep in mind the grid limits) (DONE)
    def steerToPoint(self, locationStart:treeNode, locationEnd):
        offset = self.rho*self.unitVector(locationStart, locationEnd)
        print(offset)
        point = np.array([locationStart.locationX + offset[0], locationStart.locationY + offset[1]])
        if point[0] >= self.grid.shape[1]:
            point[0] = self.grid.shape[1]-1
        if point[1] >= self.grid.shape[0]:
            point[1] = self.grid.shape[0]-1
        return point 

    def isObstacle(self, locationStart:treeNode, locationEnd):
        u_hat = self.unitVector(locationStart,locationEnd)
        val = np.array([0,0])
        for i in range(self.rho):
            val[0] = min(self.grid.shape[1]-1,locationStart.locationX+i*u_hat[0])
            val[1] = min(self.grid.shape[0]-1,locationStart.locationY+i*u_hat[1])
            if self.grid[round(val[1]), round(val[0])] == 1:
                return True
        return False 

    def unitVector(self, locationStart: treeNode, locationEnd):
        v = np.array([locationEnd[0]-locationStart.locationX, locationEnd[1]-locationStart.locationY])
        u_hat = v/np.linalg.norm(v)
        return u_hat  

    def findNearest(self, root, point):
        if not root:
            return
        print(f"[{root.locationX},{root.locationY}] and {point}") 
        dist = self.distance(root,point)
        print(f" distance is {dist}")
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children:
            self.findNearest(child, point)
        # pass

    def distance(self, node1:treeNode, point):
        dist = np.sqrt((node1.locationX-point[0])**2 + (node1.locationY-point[1])**2)
        return dist

    def goalFound(self,point):
        if self.distance(self.goal,point) <= self.rho:
            return True
        return False
    
    def resetNearestValues(self):
        self.nearestNode = self.randomTree
        self.nearestDist = 100
        
    
    def retraceRRTPath(self,goal):
        if goal.locationX == self.randomTree.locationX:
            return
        self.numWaypoints += 1
        self.Waypoints.insert(0,np.array([goal.locationX,goal.locationY]))
        self.path_distance += self.rho
        self.retraceRRTPath(goal.parent)
      

if __name__ == "__main__":
    env = env_setup()
    

    start = sampleNonObstaclePoint(env)
    goal = sampleNonObstaclePoint(env)
    numIterations = 250
    stepSize = 2
    goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill = False)

    fig = plt.figure("RRT Algorithm")
    plt.imshow(env, cmap='gray_r', origin='upper')
    # plt.title('Randomized 2D Environment')
    # plt.axis('off')  # Hide axis

    # # Display root, point, and point2
    # plt.scatter(start[0], start[1], c='g', marker='o', s=10)  # green circle for root
    # plt.scatter(goal[0], goal[1], c='r', marker='o', s=10)  # red circle for point
    print(start[0])
    plt.plot(start[0],start[1],'ro')
    plt.plot(goal[0],goal[1],'bo')
    ax = fig.gca()
    ax.add_patch(goalRegion)
    plt.xlabel('X-axis $(m)$')
    plt.ylabel('Y-axis $(m)$')

    # plt.show()

    rrt = RRTAlgorithm(start, goal, numIterations, env, stepSize)
    plt.pause(2)

    for i in range(rrt.iterations):
        rrt.resetNearestValues()
        print(f"Iteration number:{i+1}")
        point = sampleAPoint(env)
        # if i == 0:
        #     rrt.findNearest(rrt.randomTree,point)
        # else:
        rrt.findNearest(rrt.nearestNode,point)
        
        new = rrt.steerToPoint(rrt.nearestNode,point)
        if not rrt.isObstacle(rrt.nearestNode,new):
            rrt.addChild(new[0],new[1])
            plt.pause(0.10)
            plt.plot([rrt.nearestNode.locationX, new[0]],[rrt.nearestNode.locationY, new[1]],'go',linestyle='--')
            if rrt.goalFound(new):
                rrt.addChild(rrt.goal.locationX,rrt.goal.locationY)
                print("Goal Found")
                rrt.retraceRRTPath(rrt.goal)
                break
    
       
    rrt.Waypoints.insert(0,start)
    print("Number of waypoints: ", rrt.numWaypoints)
    print("Path Distance (m): ", rrt.path_distance)    
    print("Waypoints: ", rrt.Waypoints)

    #plot the waypoints in red (DONE)
    for i in range(len(rrt.Waypoints)-1):
        plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]], [rrt.Waypoints[i][1], rrt.Waypoints[i+1][1]],'ro', linestyle="--")
        plt.pause(0.10)

