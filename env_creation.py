import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import random

def generate_random_environment(grid_size=(100, 100), obstacle_density=0.2):

    # Create an empty grid
    env = np.zeros(grid_size)

    # Randomly place obstacles (1s) based on the obstacle density
    num_obstacles = int(obstacle_density * grid_size[0] * grid_size[1])
    obstacle_positions = np.random.choice(grid_size[0] * grid_size[1], num_obstacles, replace=False)
    print(f"{env.ravel()[obstacle_positions][0]:.2f}")
    # Set the obstacle positions to 1
    env.ravel()[obstacle_positions] = 1


    return env

# Function to display and save the environment as a PNG file
def display_and_save_environment(env, filename='random_environment.png'):

    # Display the environment using matplotlib
    plt.imshow(env, cmap='gray_r', origin='upper')
    plt.title('Randomized 2D Environment')
    plt.axis('off')  # Hide axis
    plt.show()

    # Convert the environment to an image and save as PNG using PIL
    img = Image.fromarray(np.uint8(env * 255))  # Convert to 8-bit grayscale (0: white, 1: black)
    img.save(filename)
    print(f"Environment saved as {filename}")

def is_obstacle(point,grid):
    return grid[point[1],point[2]] == 1

# def pick_random_point(grid):
#     while True:
#         point = np.array([np.random.randint(0, grid.shape[1]), np.random.randint(0, grid.shape[2])])
#         if grid[point[0],point[1]] != 1:
#             return point


class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX                #X Location
        self.locationY = locationY                #Y Location  
        self.children = []                        #children list   
        self.parent = None                        #parent node reference 
        
#function to sample a point within grid limits (note the [y,x])
def sampleAPoint(grid):
    x = random.randint(1, grid.shape[1])
    y = random.randint(1, grid.shape[0])
    point = np.array([x, y])
    return point

def sampleNonObstaclePoint(grid):
    x = random.randint(1, grid.shape[1]-1)
    y = random.randint(1, grid.shape[0]-1)
    point = np.array([x, y])
    if grid[x,y] != 1:
        return point   

def traverseTree(root):
    if not root:
        return
    print(f"{root.locationX:.2f}, {root.locationY:.2f}")
    for child in root.children:
        traverseTree(child)


    

# Main code to generate and save a random environment
if __name__ == "__main__":
    # Parameters
    grid_size = (100, 100)  # Grid size (height, width)
    obstacle_density = 0.05  # Density of obstacles


    # Generate random environment
    environment = generate_random_environment(grid_size, obstacle_density)

    print(environment[1,1])
    # x = sampleNonObstaclePoint(environment)
    start = sampleNonObstaclePoint(environment)
    goal = sampleNonObstaclePoint(environment)
    root = treeNode(start[0],start[1])

    point = sampleAPoint(environment)
    point = treeNode(point[0],point[1])
    point2 = sampleAPoint(environment)
    point2 = treeNode(point2[0],point2[1])
    # x_point = treeNode(x[0],x[1])
    root.children.append(point)
    root.children[0].children.append(point2)

    point.parent = root
    point2.parent = point

    plt.imshow(environment, cmap='gray_r', origin='upper')
    plt.title('Randomized 2D Environment')
    plt.axis('off')  # Hide axis

    # Display root, point, and point2
    plt.scatter(root.locationX, root.locationY, c='g', marker='o', s=10)  # green circle for root
    plt.scatter(point.locationX, point.locationY, c='b', marker='o', s=5)  # blue circle for point
    plt.scatter(point2.locationX, point2.locationY, c='r', marker='o', s=5)  # red circle for point2
    # plt.scatter(x_point.locationX, x_point.locationY, c='r', marker='o', s=5)  # red circle for point2


    # plt.show()

    # plt.plot([root.locationX, point.locationX, point2.locationX], [root.locationY, point.locationY, point2.locationY],'go', linestyle="--")
    plt.show()

    traverseTree(root)
    # Display and save the environment
    # display_and_save_environment(environment, 'random_environment.png')