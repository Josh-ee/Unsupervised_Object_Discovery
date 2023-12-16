from define_room import RoboRoom
import numpy as np
import matplotlib.pyplot as plt
import skimage.measure
import time


class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    start_time = time.time()

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0 and time.time() - start_time < 3:

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)



def draw_path(maze, path):
    drawn_path = np.zeros_like(maze)
    # for (x, y) in path:
        # drawn_path[x, y] = 0.3
    plt.imshow(maze + drawn_path, cmap="Greys_r")
    plt.plot([y for x,y in path], [x for x,y in path], c='r')
    plt.show()
    



def place_obstacles(map, objects, pool):
    for obj in objects:
        for x in range(map.shape[0]):
            for y in range(map.shape[1]):
                
                obj_x = obj['coords'][0] // pool
                obj_y = obj['coords'][1] // pool
                if (x-obj_x)**2 + (y-obj_y)**2 <= (obj['size']//pool)**2:
                    map[x,y] = 1

    return map



def main():

    room = RoboRoom(resolution=50, roomtype='H', hall_width=1)
    room.add_goal(.5, .5)
    room.add_object(coords=(1.5, 1.5), size=0.05)
    # room.world(display=True)

    start = room.coords2voxel(0, 0)
    end = room.goals[room.curr_goal_index]

    print('Start: ', start)
    print('End: ', end)

    # Reduce size of map to compute path
    guide_path = None
    low_res = 5
    while guide_path == None:
        pool = room.resolution // low_res
        maze = skimage.measure.block_reduce(room.map, (pool,pool), np.max)
        maze = place_obstacles(maze, room.objects, pool)
        # plt.imshow(maze)
        # plt.show()
        guide_path = astar(maze, (start[0]//pool, start[1]//pool), (end[0]//pool, end[1]//pool))
        if guide_path == None:
            low_res += 1
            if low_res == 10:
                break

    # Store waypoints
    if guide_path is not None:
        path = [(x*pool + pool//2, y*pool + pool//2) for x,y in guide_path]
        print('Path: ', path)
        draw_path(room.world(), path)
    else:
        print('No path was found...')



if __name__ == '__main__':
    main()