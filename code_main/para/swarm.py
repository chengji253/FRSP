import logging
import numpy as np
import copy
import math


class Swarm:

    def __init__(self):

        self.robots_num = None
        self.pos_all = None
        self.goal_pos = None

        self.start_ave_pos = None
        self.goal_ave_pos = None

        # 机器人当前所在的node idx
        self.previous_node = None
        self.current_node = None
        self.current_edge = None

        self.goal_cell = None
        self.current_cell = None

        self.goal_node = None
        self.start_node = None

        # The current robot needs to go to advance according to des_node and des_pos
        self.des_node = None
        self.des_pos = None
        # The subsequent series of des node and pos that the robot needs to go to
        self.des_path_node = None
        self.des_path_pos = None

        self.V_all = None
        self.v_max = None

        self.robot_exist = None
        self.reach_dis = 2
        self.reach_dis_goal = 3

        self.robot_radius = 0.4

        self.v_is_0_count = None

        self.Y_pos_reduce = None
        self.Y_pos_reduce_num = None

        self.V_all_store = []
        self.pos_all_store = []

        self.resolution = 10

    def store_pos_v_data(self):
        self.V_all_store.append(copy.deepcopy(self.V_all))
        self.pos_all_store.append(copy.deepcopy(self.pos_all))

    def para_set_no_planner(self):
        # 设置无人机的path路径
        self.current_node = [24] * self.robots_num
        self.start_node = [24] * self.robots_num
        self.goal_node = [25] * self.robots_num
        self.des_pos = self.goal_pos

        self.des_path_pos = []
        self.des_path_node = []
        for i in range(self.robots_num):
            pos = [self.goal_pos[i]]
            self.des_path_pos.append(pos)
            node = [self.goal_node]
            self.des_path_node.append(node)

    def set_start_goal_node(self, start_idx, goal_idx, start_cell, goal_cell):

        self.start_node = [start_idx] * self.robots_num
        self.goal_node = [goal_idx] * self.robots_num

        self.current_node = [start_idx] * self.robots_num
        self.previous_node = [start_idx] * self.robots_num

        self.current_cell = [start_cell] * self.robots_num
        self.goal_cell = [goal_cell] * self.robots_num

        self.Y_pos_reduce_num = [0] * self.robots_num

    def para_set_planner_Other_planner(self):
        des_pos = []
        for i in range(self.robots_num):
            pos = self.des_path_pos[i][0]
            des_pos.append(pos)
        self.des_pos = des_pos

    def para_set_planner(self):
        des_pos = []
        des_node = []
        for i in range(self.robots_num):
            pos = self.des_path_pos[i][0]
            des_pos.append(pos)
            node = self.des_path_node[i][0]
            des_node.append(node)
        self.des_pos = des_pos
        self.des_node = des_node

        for i in range(self.robots_num):
            self.des_path_pos[i][-1] = self.goal_pos[i]

    def del_des_path_first(self):
        # Delete the first node and pos
        for i in range(self.robots_num):
            if len(self.des_path_node[i]) > 1:
                del self.des_path_node[i][0]
                del self.des_path_pos[i][0]

    def update_state_idx_cell(self, idx):
        # 机器人的state需要更新
        # 检查是否到达当前的des
        # 如果到达更新所有的des current
        if self.des_node is None:
            # no planner--des to goal
            self.current_node[idx] = self.goal_node[idx]
        else:
            self.current_node[idx] = copy.copy(self.des_node[idx])
            if len(self.des_path_node[idx]) > 1:
                edge_head = copy.copy(self.des_path_node[idx][0])
                # print("len 1=" + str(len(self.des_path_node[0])))
                # print("len 2=" + str(len(self.des_path_pos[0])))
                del self.des_path_node[idx][0]
                del self.des_path_pos[idx][0]
                # print("idx=" + str(idx))
                # print("des_node=" + str(self.des_path_node[idx][0]))
                # print("des_pos=" + str(self.des_pos[idx][0]) + ',' + str(self.des_pos[idx][0]))
                self.des_pos[idx] = copy.copy(self.des_path_pos[idx][0])
                self.des_node[idx] = copy.copy(self.des_path_node[idx][0])
                edge_tail = copy.copy(self.des_path_node[idx][0])
                self.current_edge[idx, :] = np.array([edge_head, edge_tail])
                # print("robot " + str(idx) + "=" + str(self.current_edge[idx][0]) + "," + str(self.current_edge[idx][1]))

    def update_state_idx(self, idx):
        # The robot's state needs to be updated
        # Check if the current des is reached
        # If reached, update all des current
        if self.des_node is None:
            # no planner--des to goal
            self.current_node[idx] = self.goal_node[idx]
        else:
            self.current_node[idx] = copy.copy(self.des_node[idx])
            self.previous_node[idx] = copy.copy(self.des_node[idx])

            if len(self.des_path_node[idx]) > 1:
                edge_head = copy.copy(self.des_path_node[idx][0])
                # print("len 1=" + str(len(self.des_path_node[0])))
                # print("len 2=" + str(len(self.des_path_pos[0])))
                del self.des_path_node[idx][0]
                del self.des_path_pos[idx][0]
                # print("idx=" + str(idx))
                # print("des_node=" + str(self.des_path_node[idx][0]))
                # print("des_pos=" + str(self.des_pos[idx][0]) + ',' + str(self.des_pos[idx][0]))
                self.des_pos[idx] = copy.copy(self.des_path_pos[idx][0])
                self.des_node[idx] = copy.copy(self.des_path_node[idx][0])
                edge_tail = copy.copy(self.des_path_node[idx][0])
                self.current_edge[idx, :] = np.array([edge_head, edge_tail])
                # print("robot " + str(idx) + "=" + str(self.current_edge[idx][0]) + "," + str(self.current_edge[idx][1]))

    def forward_pos_state(self, i, time_step):
        # if self.robot_exist[i] is True:
        self.pos_all[i][0] += self.V_all[i][0] * time_step
        self.pos_all[i][1] += self.V_all[i][1] * time_step

        # The robot is walking back. It may be being carried away by other robots.
        if self.V_all[i][1] < -1.5:
            self.Y_pos_reduce_num[i] += 1
            if self.Y_pos_reduce_num[i] >= 10:
                self.Y_pos_reduce[i] = True
                # print("robot " + str(i) + "Y pos reduce")
                # print("V=" + str(self.V_all[i][1]))
        else:
            self.Y_pos_reduce_num[i] = 0
            self.Y_pos_reduce[i] = False

    def judge_reach_des(self, i):

        if self.des_pos[i][0] == self.goal_pos[i][0] and self.des_pos[i][1] == self.goal_pos[i][1]:
            # des is goal
            if self.reach(self.pos_all[i], self.des_pos[i], self.reach_dis_goal):
                self.robot_exist[i] = False
                self.V_all[i] = [0, 0]
                return True
            else:
                return False
        else:
            # des is not goal
            if self.reach(self.pos_all[i], self.des_pos[i], self.reach_dis):
                return True
            else:
                return False

    def judge_robot_all_exist(self):
        return all(x is False for x in self.robot_exist)

    def find_dead_lock(self, V, i, around_no_robots):
        # Find the robot in deadlock
        # If there are no robots around and the speed is always zero, it is considered to be in deadlock
        # Keep the judgment of whether there are obs around
        norm_x = np.linalg.norm([V[i][0], V[i][1]])
        if norm_x <= 0.2 and around_no_robots[i] is True and self.robot_exist[i] is True:
            self.v_is_0_count[i] += 1
            if self.v_is_0_count[i] >= 3:
                # print("robot " + str(i) + " is dead lock !")
                return True
        else:
            self.v_is_0_count[i] = 0
            return False

    def deal_with_dead_lock(self, i):
        x_gap = self.des_pos[i][0] - self.pos_all[i][0]
        y_gap = self.des_pos[i][1] - self.pos_all[i][1]

        if abs(x_gap) >= abs(y_gap):
            if y_gap > 0:
                self.V_all[i][1] = 0.5
            else:
                self.V_all[i][1] = -0.5
        elif abs(x_gap) < abs(y_gap):
            if x_gap > 0:
                self.V_all[i][0] = 0.5
            else:
                self.V_all[i][0] = -0.5
        else:
            pass

        if self.v_is_0_count[i] >= 15:
            if abs(x_gap) >= abs(y_gap):
                if y_gap > 0:
                    self.pos_all[i][1] += -0.1
                else:
                    self.pos_all[i][1] += 0.1
            elif abs(x_gap) < abs(y_gap):
                if x_gap > 0:
                    self.pos_all[i][0] += -0.1
                else:
                    self.pos_all[i][0] += 0.1
            else:
                pass

    def compute_ave_start_goal_pos(self):
        self.start_ave_pos = self.compute_ave_pos(self.pos_all)
        self.goal_ave_pos = self.compute_ave_pos(self.goal_pos)

    def compute_ave_pos(self, pos_list):
        sum_x = 0
        sum_y = 0
        for pos in pos_list:
            sum_x += pos[0]
            sum_y += pos[1]
        num = len(pos_list)
        return [sum_x / num, sum_y / num]

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

    def reach(self, p1, p2, bound=0.2):
        if self.distance(p1, p2) < bound:
            return True
        else:
            return False

    def distance(self, pose1, pose2):
        """ compute Euclidean distance for 2D """
        return math.sqrt((pose1[0] - pose2[0]) ** 2 + (pose1[1] - pose2[1]) ** 2)

    def init_swarm_2(self):
        self.robots_num = 2
        self.robot_exist = [True] * self.robots_num
        self.v_is_0_count = [0] * self.robots_num
        self.Y_pos_reduce = [False] * self.robots_num

        self.current_edge = np.zeros((self.robots_num, 2), dtype=int)

        self.pos_all = [[8, 20], [10, 20]]
        self.goal_pos = [[8, 73], [10, 73]]

        self.V_all = [[0, 0] for i in range(len(self.pos_all))]
        self.v_max = [5.0 for i in range(len(self.pos_all))]


    def init_swarm_10(self):
        self.robots_num = 10
        self.robot_exist = [True] * self.robots_num
        self.v_is_0_count = [0] * self.robots_num
        self.Y_pos_reduce = [False] * self.robots_num
        self.current_edge = np.zeros((self.robots_num, 2), dtype=int)

        start_h_list = list(range(10, 10 + 5))
        goal_h_list = list(range(75, 75 + 5))
        start_h_list = [x - 1 for x in start_h_list]
        goal_h_list = [x + 1 for x in goal_h_list]
        width_list = [14, 15]

        self.pos_all = self.generate_start_pos(start_h_list, width_list)
        self.goal_pos = self.generate_goal_pos(goal_h_list, width_list)

        self.V_all = [[0, 0] for i in range(len(self.pos_all))]
        self.v_max = [5.0 for i in range(len(self.pos_all))]


    def init_swarm_30(self):
        self.robots_num = 30
        self.robot_exist = [True] * self.robots_num
        self.v_is_0_count = [0] * self.robots_num
        self.Y_pos_reduce = [False] * self.robots_num
        self.current_edge = np.zeros((self.robots_num, 2), dtype=int)
        start_h_list = list(range(10, 10 + 10))
        goal_h_list = list(range(75, 75 + 10))
        start_h_list = [x - 1 for x in start_h_list]
        goal_h_list = [x + 1 for x in goal_h_list]
        width_list = list(range(20, 20 + 3))
        self.pos_all = self.generate_start_pos(start_h_list, width_list)
        self.goal_pos = self.generate_goal_pos(goal_h_list, width_list)

        self.V_all = [[0, 0] for i in range(len(self.pos_all))]
        self.v_max = [3.0 for i in range(len(self.pos_all))]

    def init_swarm_50(self):
        self.robots_num = 50
        self.robot_exist = [True] * self.robots_num
        self.v_is_0_count = [0] * self.robots_num
        self.Y_pos_reduce = [False] * self.robots_num
        self.current_edge = np.zeros((self.robots_num, 2), dtype=int)

        start_h_list = list(range(10, 10 + 10))
        goal_h_list = list(range(68, 68 + 10))
        start_h_list = [x - 1 for x in start_h_list]
        goal_h_list = [x + 1 for x in goal_h_list]
        width_list = list(range(20, 20 + 5))

        self.pos_all = self.generate_start_pos(start_h_list, width_list)
        self.goal_pos = self.generate_goal_pos(goal_h_list, width_list)

        self.V_all = [[0, 0] for i in range(len(self.pos_all))]
        self.v_max = [3.0 for i in range(len(self.pos_all))]

    def init_swarm_80(self):
        self.robots_num = 80
        self.robot_exist = [True] * self.robots_num
        self.v_is_0_count = [0] * self.robots_num
        self.Y_pos_reduce = [False] * self.robots_num

        self.current_edge = np.zeros((self.robots_num, 2), dtype=int)
        start_h_list = list(range(3, 3 + 20))
        goal_h_list = list(range(68, 68 + 20))
        start_h_list = [x - 1 for x in start_h_list]
        goal_h_list = [x + 1 for x in goal_h_list]
        width_list = list(range(20, 20 + 4))
        self.pos_all = self.generate_start_pos(start_h_list, width_list)
        self.goal_pos = self.generate_goal_pos(goal_h_list, width_list)

        self.V_all = [[0, 0] for i in range(len(self.pos_all))]
        self.v_max = [3.0 for i in range(len(self.pos_all))]

    def init_swarm_100(self):
        self.robots_num = 100
        self.robot_exist = [True] * self.robots_num
        self.v_is_0_count = [0] * self.robots_num
        self.Y_pos_reduce = [False] * self.robots_num

        self.current_edge = np.zeros((self.robots_num, 2), dtype=int)
        start_h_list = list(range(10, 10 + 10))
        goal_h_list = list(range(68, 68 + 10))
        start_h_list = [x - 1 for x in start_h_list]
        goal_h_list = [x + 1 for x in goal_h_list]
        width_list = list(range(21, 21 + 10))
        self.pos_all = self.generate_start_pos(start_h_list, width_list)
        self.goal_pos = self.generate_goal_pos(goal_h_list, width_list)

        self.V_all = [[0, 0] for i in range(len(self.pos_all))]
        self.v_max = [3.0 for i in range(len(self.pos_all))]


    def init_swarm_150(self):
        self.robots_num = 150
        self.robot_exist = [True] * self.robots_num
        self.v_is_0_count = [0] * self.robots_num
        self.Y_pos_reduce = [False] * self.robots_num
        self.current_edge = np.zeros((self.robots_num, 2), dtype=int)
        start_h_list = list(range(10, 10 + 10))
        goal_h_list = list(range(68, 68 + 10))
        start_h_list = [x - 1 for x in start_h_list]
        goal_h_list = [x + 1 for x in goal_h_list]
        width_list = list(range(20, 20 + 15))
        self.pos_all = self.generate_start_pos(start_h_list, width_list)
        self.goal_pos = self.generate_goal_pos(goal_h_list, width_list)

        self.V_all = [[0, 0] for i in range(len(self.pos_all))]
        self.v_max = [3.0 for i in range(len(self.pos_all))]


    def init_swarm_200(self):
        self.robots_num = 200
        self.robot_exist = [True] * self.robots_num
        self.v_is_0_count = [0] * self.robots_num
        self.Y_pos_reduce = [False] * self.robots_num
        self.current_edge = np.zeros((self.robots_num, 2), dtype=int)
        start_h_list = list(range(2, 2 + 20))
        goal_h_list = list(range(68, 68 + 20))
        start_h_list = [x - 1 for x in start_h_list]
        goal_h_list = [x + 1 for x in goal_h_list]
        width_list = list(range(20, 20 + 10))
        self.pos_all = self.generate_start_pos(start_h_list, width_list)
        self.goal_pos = self.generate_goal_pos(goal_h_list, width_list)

        self.V_all = [[0, 0] for i in range(len(self.pos_all))]
        self.v_max = [3.0 for i in range(len(self.pos_all))]


    def init_swarm_250(self):
        self.robots_num = 252
        self.robot_exist = [True] * self.robots_num
        self.v_is_0_count = [0] * self.robots_num
        self.Y_pos_reduce = [False] * self.robots_num
        self.current_edge = np.zeros((self.robots_num, 2), dtype=int)
        start_h_list = list(range(5, 5 + 12))
        goal_h_list = list(range(70, 70 + 12))
        start_h_list = [x - 1 for x in start_h_list]
        goal_h_list = [x + 1 for x in goal_h_list]
        width_list = list(range(15, 15 + 21))
        self.pos_all = self.generate_start_pos(start_h_list, width_list)
        self.goal_pos = self.generate_goal_pos(goal_h_list, width_list)

        self.V_all = [[0, 0] for i in range(len(self.pos_all))]
        self.v_max = [3.0 for i in range(len(self.pos_all))]


    def init_swarm_300(self):
        self.robots_num = 300
        self.robot_exist = [True] * self.robots_num
        self.v_is_0_count = [0] * self.robots_num
        self.Y_pos_reduce = [False] * self.robots_num
        self.current_edge = np.zeros((self.robots_num, 2), dtype=int)
        start_h_list = list(range(2, 2 + 20))
        goal_h_list = list(range(68, 68 + 20))
        start_h_list = [x - 1 for x in start_h_list]
        goal_h_list = [x + 1 for x in goal_h_list]
        width_list = list(range(20, 20 + 15))
        self.pos_all = self.generate_start_pos(start_h_list, width_list)
        self.goal_pos = self.generate_goal_pos(goal_h_list, width_list)

        self.V_all = [[0, 0] for i in range(len(self.pos_all))]
        self.v_max = [3.0 for i in range(len(self.pos_all))]


    def init_swarm_350(self):
        self.robots_num = 350
        self.robot_exist = [True] * self.robots_num
        self.v_is_0_count = [0] * self.robots_num
        self.Y_pos_reduce = [False] * self.robots_num
        self.current_edge = np.zeros((self.robots_num, 2), dtype=int)
        start_h_list = list(range(2, 2 + 14))
        goal_h_list = list(range(68, 68 + 14))
        start_h_list = [x - 1 for x in start_h_list]
        goal_h_list = [x + 1 for x in goal_h_list]
        width_list = list(range(10, 10 + 25))
        self.pos_all = self.generate_start_pos(start_h_list, width_list)
        self.goal_pos = self.generate_goal_pos(goal_h_list, width_list)

        self.V_all = [[0, 0] for i in range(len(self.pos_all))]
        self.v_max = [3.0 for i in range(len(self.pos_all))]


    def init_swarm_400(self):
        self.robots_num = 400
        self.robot_exist = [True] * self.robots_num
        self.v_is_0_count = [0] * self.robots_num
        self.Y_pos_reduce = [False] * self.robots_num
        self.current_edge = np.zeros((self.robots_num, 2), dtype=int)
        start_h_list = list(range(2, 2 + 20))
        goal_h_list = list(range(68, 68 + 20))
        start_h_list = [x - 1 for x in start_h_list]
        goal_h_list = [x + 1 for x in goal_h_list]
        width_list = list(range(10, 10 + 20))

        self.pos_all = self.generate_start_pos(start_h_list, width_list)
        self.goal_pos = self.generate_goal_pos(goal_h_list, width_list)

        self.V_all = [[0, 0] for i in range(len(self.pos_all))]
        self.v_max = [3.0 for i in range(len(self.pos_all))]


    def init_swarm_450(self):
        self.robots_num = 450
        self.robot_exist = [True] * self.robots_num
        self.v_is_0_count = [0] * self.robots_num
        self.Y_pos_reduce = [False] * self.robots_num
        self.current_edge = np.zeros((self.robots_num, 2), dtype=int)
        start_h_list = list(range(2, 20))
        goal_h_list = list(range(68, 86))
        start_h_list = [x - 1 for x in start_h_list]
        goal_h_list = [x + 1 for x in goal_h_list]
        width_list = list(range(3, 3 + 25))
        self.pos_all = self.generate_start_pos(start_h_list, width_list)
        self.goal_pos = self.generate_goal_pos(goal_h_list, width_list)

        self.V_all = [[0, 0] for i in range(len(self.pos_all))]
        self.v_max = [3.0 for i in range(len(self.pos_all))]


    def init_swarm_500(self):
        self.robots_num = 500
        self.robot_exist = [True] * self.robots_num
        self.v_is_0_count = [0] * self.robots_num
        self.Y_pos_reduce = [False] * self.robots_num
        self.current_edge = np.zeros((self.robots_num, 2), dtype=int)
        start_h_list = list(range(3, 3 + 20))
        goal_h_list = list(range(68, 68 + 20))
        start_h_list = [x - 1 for x in start_h_list]
        goal_h_list = [x + 1 for x in goal_h_list]
        width_list = list(range(8, 8 + 25))
        self.pos_all = self.generate_start_pos(start_h_list, width_list)
        self.goal_pos = self.generate_goal_pos(goal_h_list, width_list)

        self.V_all = [[0, 0] for i in range(len(self.pos_all))]
        self.v_max = [3.0 for i in range(len(self.pos_all))]

