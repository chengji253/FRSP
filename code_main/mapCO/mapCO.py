# 生成主要地图 用于避碰 map collision avoidance

class mapCo:

    def __init__(self):
        self.obstacles = None
        self.resolution = None
        self.boundary = None
        self.boundary_obs = None

    def init_mapCo_one(self, mapInfo, resolution):

        self.resolution = resolution

        x_size = mapInfo.x_size / self.resolution
        y_size = mapInfo.y_size / self.resolution

        self.boundary = [[0, x_size], [0, y_size]]

        self.init_obs(mapInfo)

    def init_obs(self, mapInfo):
        self.boundary_obs = self.generate_obs_boundary(self.boundary)

        # obs [x, y, radius]
        self.obstacles = self.generate_obs_mapInfo(mapInfo)

        self.obstacles = self.obstacles + self.boundary_obs

    def generate_start_pos(self, height_list, width_list):
        pos = []
        for h in height_list:
            for w in width_list:
                pos.append([w, h])
        return pos

    def generate_goal_pos(self, height_list, width_list):
        pos = []
        for h in height_list:
            for w in width_list:
                pos.append([w, h])
        return pos

    def generate_obs_one(self):
        # generate specific obstacles in map
        # 多个obs
        obs = []
        obs_size = 0.5
        # height = [6, 9, 12]
        height = [12, 15, 18]
        for x in range(2, 12, 2):
            obs.append([x, height[0], obs_size])
        for x in range(3, 15, 2):
            obs.append([x, height[1], obs_size])
        for x in range(2, 12, 2):
            obs.append([x, height[2], obs_size])
        return obs

    def generate_obs_boundary(self, boundary):
        x_left = boundary[0][0]
        x_right = boundary[0][1]
        width = x_right - x_left

        y_down = boundary[1][0]
        y_up = boundary[1][1]
        height = y_up - y_down

        rect_center = [x_left-0.5, height/2]
        rect_length = height
        rect_width = 1
        circle_size = 1

        circles1 = self.fill_rectangle_with_circles(rect_center, rect_width, rect_length, circle_size)

        rect_center = [x_right + 0.5, height/2]
        rect_length = height
        rect_width = 1
        circle_size = 1
        circles2 = self.fill_rectangle_with_circles(rect_center, rect_width, rect_length, circle_size)

        rect_center = [(x_left + x_right)/2, y_down - 0.5]
        rect_length = 1
        rect_width = width + 1
        circle_size = 1
        circles3 = self.fill_rectangle_with_circles(rect_center, rect_width, rect_length, circle_size)

        rect_center = [(x_left + x_right)/2, y_up + 0.5]
        rect_length = 1
        rect_width = width + 1
        circle_size = 1
        circles4 = self.fill_rectangle_with_circles(rect_center, rect_width, rect_length, circle_size)

        return circles1 + circles2 + circles3 + circles4
        # return circles1 + circles2

    def fill_rectangle_with_circles(self, rect_center, rect_length, rect_width, circle_size):
        circles = []
        radius = circle_size / 2

        # 计算矩形的四个角
        top_left = [rect_center[0] - rect_length / 2, rect_center[1] + rect_width / 2]
        bottom_right = [rect_center[0] + rect_length / 2, rect_center[1] - rect_width / 2]

        # 在矩形内部填充圆形
        x = top_left[0] + radius
        while x + radius <= bottom_right[0]:
            y = top_left[1] - radius
            while y - radius >= bottom_right[1]:
                circles.append([x, y, radius])
                y -= circle_size
            x += circle_size

        return circles

    def generate_obs_mapInfo(self, mapInfo):
        circles_all = []
        for i in range(mapInfo.num_obs_rect):
            if i == 0:
                continue
            rect = mapInfo.obstacles_rect[i]
            left = rect[0]
            right = rect[1]
            rect_center = [(left[0] + right[0]) / 2 / self.resolution, (left[1] + right[1]) / 2 / self.resolution]
            rect_width = (right[0] - left[0]) / self.resolution
            rect_length = (right[1] - left[1]) / self.resolution
            circle_size = 1
            circles_n = self.fill_rectangle_with_circles(rect_center, rect_width, rect_length, circle_size)
            circles_all += circles_n
        return circles_all

