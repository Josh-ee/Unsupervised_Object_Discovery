import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage.filters import gaussian_filter


class RoboRoom:


    def __init__(self, room_size=3, resolution=10, roomtype='square', hall_width=1):

        self.room_size = room_size      # in meters
        self.goal_size = 0.1
        self.resolution = resolution   # cells per meter

        self.map = np.zeros((self.room_size * self.resolution, self.room_size * self.resolution))
        self.goals = []
        self.objects = []
        self.curr_goal_index = None

        if roomtype == 'H':
            self.map = self.H_map(hall_width)
        
        # Take into account size of robot
        buffer = np.where(gaussian_filter(self.map, sigma = 0.1 * self.resolution) > 0.01)
        for x, y in zip(*buffer):
            if self.map[x,y] != 1:
                self.map[x,y] = 0.15
        
        # Places goals on a map
        self.goal_map = np.zeros_like(self.map)


    def H_map(self, width):
        l = int(self.room_size * self.resolution)
        w = int(width * self.resolution)

        map = np.ones((l,l))
        map[(l-w)//2 : (l+w)//2, :] = 0
        map[:,:w] = 0
        map[:,-w:] = 0

        return map
    

    def add_goal(self, x, y):

        x = int(x * self.resolution)
        y = int(y * self.resolution)
        self.goals.append((x,y))

        c = int(self.goal_size * self.resolution)
        self.goal_map[x-c//2:x+c//2, y-c//2:y+c//2] = 0.5

        if self.curr_goal_index == None:
            self.curr_goal_index = 0
        

    def add_object(self, coords = (0, 0), size = 0.05, color = None, movable = None):
        
        obj = dict()
        obj['size'] = size * self.resolution        # Radius
        obj['coords'] = (coords[0] * self.resolution, coords[1] * self.resolution)    # (x,y)
        obj['color'] = color
        obj['movable'] = movable

        self.objects.append(obj)


    def coords2voxel(self, x, y):
        x = int(x * self.resolution) + self.map.shape[0] // 2
        y = int(y * self.resolution) + self.map.shape[1] // 2
        return (x, y)

    def voxel2coords(self, x, y):
        x = float(x - self.map.shape[0] // 2) / float(self.resolution)
        y = float(y - self.map.shape[1] // 2) / float(self.resolution)
        return (x, y)


    def world(self, display=False):

        res = self.map + self.goal_map

        for obj in self.objects:
            for x in range(res.shape[0]):
                for y in range(res.shape[1]):
                    obj_x = obj['coords'][0]
                    obj_y = obj['coords'][1]
                    if (x-obj_x)**2 + (y-obj_y)**2 <= (obj['size'])**2:
                        res[x,y] = 0.7
        
        if display:
            plt.imshow(res, cmap="Greys_r")
            plt.show()

        return res



if __name__ == "__main__":

    room = RoboRoom(roomtype='H', hall_width=1)
    room.add_goal(.5, .5)

    plt.imshow(room.map + room.goal_map, cmap='Greys_r')
    plt.show()