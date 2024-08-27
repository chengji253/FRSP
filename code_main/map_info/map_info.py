import copy
import time
import math
from matplotlib.patches import Circle
import networkx as nx
import logging
from .bcd import *
from scipy.ndimage import binary_dilation
from pathfinding.dijkstra import Dijkstra



class MapInfo:

    def __init__(self, swarm, pic_name, resolution):

        self.pic_name = pic_name
        self.swarm = swarm

        self.x_size = None
        self.y_size = None
        self.obstacles = None

        self.num_obs_rect = None
        self.obstacles_rect = None
        self.path_all = None

        self.resolution = resolution

        self.number_cells = None

        self.map_all = None
        self.map_01 = None

        self.start_idx = None
        self.end_idx = None

        self.cell_dict = {}

        self.cell_info = {}
        self.delete_cell_list = None

        self.dijkstra_result = {}

        self.node_all = {}
        self.edge_all = {}

        self.node_to_goal = {}
        self.start_node = None
        self.end_node = None

        self.distance = 1
        self.num_grids = 3
        self.safety_dis = self.swarm.robot_radius*1.2
        self.dijkstra = Dijkstra()

    def init_main(self):
        logging.info("Map decomposition and network construction !")
        self.read_map()
        self.extract_info()
        self.construct_cell_structure()
        self.deal_with_resolution()
        self.construct_flow_network_one()
        self.construct_dijkstra()
        self.update_para()

    def init_main_one(self):
        logging.info("Map decomposition and network construction !")
        self.read_map()
        self.extract_info()
        self.construct_cell_structure()
        self.delete_small_cell()
        self.deal_with_resolution()
        self.construct_flow_network_two()
        self.add_edge_for_delete_cell()
        self.construct_dijkstra()
        self.update_para()

    def update_para(self):
        self.swarm.current_node = np.ones((self.swarm.robots_num, 2), dtype=int) * self.start_node

    def deal_with_resolution(self):
        for k, value in self.cell_info.items():
            value['left_up'] = [value['left_up'][0] / self.resolution, value['left_up'][1] / self.resolution]
            value['left_down'] = [value['left_down'][0] / self.resolution, value['left_down'][1] / self.resolution]
            value['right_up'] = [value['right_up'][0] / self.resolution, value['right_up'][1] / self.resolution]
            value['right_down'] = [value['right_down'][0] / self.resolution, value['right_down'][1] / self.resolution]

    def discrete_points_between(self, p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        points = []

        num = max(abs(y2 - y1), abs(x2 - x1))
        dx = (x2 - x1) / num
        dy = (y2 - y1) / num

        for i in range(num + 1):
            x = x1 + i * dx
            y = y1 + i * dy
            points.append((int(x), int(y)))
        return points

    def generate_discrete_points(self, path):
        discrete_points = []
        for i in range(len(path) - 1):
            discrete_points.extend(self.discrete_points_between(path[i], path[i + 1]))
        return discrete_points

    def calculate_distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def calculate_path_length(self, path):
        total_length = 0
        for i in range(len(path) - 1):
            total_length += self.calculate_distance(path[i], path[i + 1])
        return total_length

    def find_pass_node(self, cell_idx_set, node_idx):
        node_list = []
        for i in range(len(cell_idx_set) - 1):
            cell_down = cell_idx_set[i]
            cell_up = cell_idx_set[i + 1]
            for key, node in self.node_all.items():
                if 'cell_up_set' in node:
                    cell_up_set = node['cell_up_set']
                    cell_down_set = node['cell_down_set']
                    if cell_down in cell_down_set and cell_up in cell_up_set:
                        if node_idx != key:
                            node_list.append(key)
                        break
        return node_list

    def find_cell_down_node(self, cell_idx):
        node_list = []
        for key, node in self.node_all.items():
            if cell_idx in node['cell_up_set']:
                node_list.append(copy.copy(key))
        return node_list

    def find_cell_up_node(self, cell_idx):
        node_list = []
        for key, node in self.node_all.items():
            if cell_idx in node['cell_down_set']:
                node_list.append(copy.copy(key))
        return node_list

    def path_set_search(self):
        # Based on the cell where the current drone is located, find all nodes on the upper boundary of the cell
        # Find the lower boundary node of the goal cell
        # Use the Dijkstra algorithm to connect these nodes as a path
        path_all = {}
        for i in range(self.swarm.robots_num):
            current_cell = self.swarm.current_cell[i]

            if current_cell == self.swarm.goal_cell[i]:
                p1 = self.swarm.goal_node[i]
                p2 = self.swarm.goal_node[i]
                p3 = self.swarm.goal_node[i]
                first_edge = str(p1) + ',' + str(p2)
                second_edge = str(p2) + ',' + str(p3)
                dic = {'path_node': [p1], 'path_length': 1,
                       'start_node': p1, 'end_node': p1, 'drone_idx': i,
                       'first_edge': first_edge, 'second_edge': second_edge}
                idx = len(path_all)
                path_all[idx] = dic
                continue

            node_list = self.find_cell_up_node(current_cell)

            # If the current cell has no upper boundary node, keep the previous des node unchanged
            if len(node_list) == 0:
                # logging.info("node list is [] !")
                path_node = copy.copy(self.swarm.des_path_node[i])
                s_i = copy.copy(self.swarm.previous_node[i])
                g_i = copy.copy(self.swarm.goal_node[i])

                if len(path_node) >= 3:
                    p1 = path_node[0]
                    p2 = path_node[1]
                    p3 = path_node[2]
                    first_edge = str(p1) + ',' + str(p2)
                    second_edge = str(p2) + ',' + str(p3)
                    dic = {'path_node': path_node, 'path_length': 1,
                           'start_node': s_i, 'end_node': g_i, 'drone_idx': i,
                           'first_edge': first_edge, 'second_edge': second_edge}
                    idx = len(path_all)
                    path_all[idx] = dic
                elif len(path_node) == 2:
                    p1 = path_node[0]
                    p2 = path_node[1]
                    first_edge = str(p1) + ',' + str(p2)
                    second_edge = str(p1) + ',' + str(p2)
                    dic = {'path_node': path_node, 'path_length': 1,
                           'start_node': s_i, 'end_node': g_i, 'drone_idx': i,
                           'first_edge': first_edge, 'second_edge': second_edge}
                    idx = len(path_all)
                    path_all[idx] = dic
                continue

            # Find the lower boundary node of the goal cell
            goal_neighbor_node_list = self.find_cell_down_node(self.swarm.goal_cell[i])
            for cur_ner_node in node_list:
                for goal_ner_node in goal_neighbor_node_list:
                    s_i = cur_ner_node
                    mid_i = goal_ner_node
                    g_i = self.swarm.goal_node[i]
                    path_node, path_length = self.find_path_mid(s_i, g_i, mid_i)
                    path_length = self.revise_path_length_cell(i, g_i, path_node, path_length)

                    if math.isnan(path_length) or math.isinf(path_length):
                        path_length = 999999999

                    if len(path_node) >= 3:
                        p1 = path_node[0]
                        p2 = path_node[1]
                        p3 = path_node[2]
                        first_edge = str(p1) + ',' + str(p2)
                        second_edge = str(p2) + ',' + str(p3)
                        dic = {'path_node': path_node, 'path_length': path_length,
                               'start_node': s_i, 'end_node': g_i, 'drone_idx': i,
                               'first_edge': first_edge, 'second_edge': second_edge}
                        idx = len(path_all)
                        path_all[idx] = dic
                    elif len(path_node) == 2:
                        p1 = path_node[0]
                        p2 = path_node[1]
                        first_edge = str(p1) + ',' + str(p2)
                        second_edge = str(p1) + ',' + str(p2)
                        dic = {'path_node': path_node, 'path_length': path_length,
                               'start_node': s_i, 'end_node': g_i, 'drone_idx': i,
                               'first_edge': first_edge, 'second_edge': second_edge}
                        idx = len(path_all)
                        path_all[idx] = dic
                    else:
                        pass

        self.path_all = path_all
        # logging.info("path_all_len=" + str(len(self.path_all)))
        return path_all

    def node_dis_dij(self, node1, node2):
        node_set = self.dijkstra.adjacency_list[node1]
        for n in node_set:
            if n[0] == node2:
                dis = n[1]
                return dis

    def shared_edge_to_path(self):
        self.shared_first_edge = {}
        self.shared_second_edge = {}
        first_edge_set = set()
        second_edge_set = set()

        for idx, value in self.path_all.items():
            first_edge = value['first_edge']
            second_edge = value['second_edge']
            first_edge_set.add(first_edge)
            second_edge_set.add(second_edge)

        for edge in first_edge_set:
            path_idx_set = set()
            for path_idx, value in self.path_all.items():
                if edge == value['first_edge']:
                    path_idx_set.add(path_idx)
            self.shared_first_edge[edge] = path_idx_set

        for edge in second_edge_set:
            path_idx_set = set()
            for path_idx, value in self.path_all.items():
                if edge == value['second_edge']:
                    path_idx_set.add(path_idx)
            self.shared_second_edge[edge] = path_idx_set

    def revise_path_length_cell(self, k, g_k, path_node, path_length):
        # Fix the path length for each drone
        if len(path_node) >= 2:
            # The first node of the path
            first_node = path_node[0]
            # The second to last node
            second_to_last_node = path_node[-2]
            # Calculate the distance between the second to last node and the target node
            dis2 = self.node_dis_dij(second_to_last_node, g_k)

            current_pos = self.swarm.pos_all[k]
            goal_pos = self.swarm.goal_pos[k]

            # current_pos = [x * self.resolution for x in current_pos]
            # goal_pos = [x * self.resolution for x in goal_pos]

            # The position of the first node
            first_pos = self.node_all[first_node]['pos']
            # The second to last node pos
            second_last_pos = self.node_all[second_to_last_node]['pos']

            # The distance between the current pos and the first node
            dis1_n = self.swarm.distance(current_pos, first_pos)
            # The distance between the target pos and the second-to-last node
            dis2_n = self.swarm.distance(goal_pos, second_last_pos)

            # Subtract the distance between nodes to correct the actual distance
            path_length = path_length - dis2 + dis1_n + dis2_n
            return path_length
        else:
            return path_length

    def construct_dijkstra(self):
        graph_edges = []
        for edge, info in self.edge_all.items():
            node_start, node_end = map(int, edge.split(','))
            dis = round(info['dis'], 2)
            e = [node_start, node_end, dis]
            e_re = [node_end, node_start, dis]
            graph_edges.append(e)
            graph_edges.append(e_re)
        self.dijkstra.init_node_edges(graph_edges)

    def find_path_mid(self, start_node, end_node, mid_node):
        # According to the current edges, the paths directly to the goal are combined into a set
        # The path from the starting node to the middle node and the path
        # from the middle node to the goal point are combined

        name1 = (start_node, mid_node)
        if name1 in self.dijkstra_result:
            path1 = self.dijkstra_result[name1]['path']
            dis1 = self.dijkstra_result[name1]['dis']
        else:
            path1, dis1 = self.dijkstra.shortest_path(start_node, mid_node)
            dic_n = {'path': path1, 'dis': dis1}
            self.dijkstra_result[name1] = dic_n

        name2 = (mid_node, end_node)
        if name2 in self.dijkstra_result:
            path2 = self.dijkstra_result[name2]['path']
            dis2 = self.dijkstra_result[name2]['dis']
        else:
            path2, dis2 = self.dijkstra.shortest_path(mid_node, end_node)
            dic_n = {'path': path2, 'dis': dis2}
            self.dijkstra_result[name2] = dic_n

        path1 = list(path1)
        path2 = list(path2)
        path1.pop()

        path = path1 + path2
        dis = dis1 + dis2

        return path, dis

    def draw_mapInfo(self):
        self.display_separate_cell(self.bcd_out_im, self.bcd_out_cells)
        self.draw_network()

    def draw_network(self):
        G = nx.Graph()
        for node, data in self.node_all.items():
            G.add_node(node, pos=data['pos'])
        for node, data in self.node_start_end.items():
            G.add_node(node, pos=data['pos'])

        for edge, capacity in self.edge_all.items():
            node_start, node_end = map(int, edge.split(','))
            G.add_edge(node_start, node_end, capacity=capacity)

        pos = nx.get_node_attributes(G, 'pos')
        edge_labels = {(u, v): d['capacity'] for u, v, d in G.edges(data=True)}

        plt.figure(figsize=(8, 6))
        # nx.draw(G, pos, with_labels=True, node_color='skyblue', node_size=100, edge_color='k')
        nx.draw(G, pos, with_labels=True, node_color='skyblue', node_size=30, edge_color='k', width=0.2,
                style='dashed')

        # nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, label_pos=0.4)  # 调整这里的label_pos值

        # Add circles for each node
        # print()
        # self.node_all
        ax = plt.gca()  # Get current axes
        for i in range(len(self.node_all)):
            node_pos_set = self.node_all[i]['node_option_pos']
            for j in range(len(node_pos_set)):
                pos = node_pos_set[j]
                radius = self.swarm.robot_radius
                circle = Circle((pos[0], pos[1]), radius, fill=False, edgecolor='r', linewidth=2)
                ax.add_patch(circle)
        plt.axis('equal')  # Set equal scaling by changing axis limits
        plt.show()

    def delete_small_cell(self):
        # delete cells that the gap between y_up and y_down is to small

        self.delete_cell_list = set()
        for i in range(len(self.cell_info)):
            cell_n = self.cell_info[i]
            left_up = cell_n['left_up']
            left_down = cell_n['left_down']

            if (left_up[1] - left_down[1]) <= 5:
                self.delete_cell_list.add(i)

    def add_edge_for_delete_cell(self):
        # when some sell is deleted, there need to add some edges
        for cell_n in self.delete_cell_list:
            up_cell_set = self.cell_info[cell_n]['up_node']
            for up_cell in up_cell_set:
                node_list1 = self.find_node_with_up_cell(cell_n)
                node_list2 = self.find_node_with_down_cell(up_cell)
                self.add_edges_by_list(node_list1, node_list2)

    def add_edges_by_list(self, node_list1, node_list2):
        for i in node_list1:
            for j in node_list2:
                ca_min = max(self.node_all[i]['capacity'], self.node_all[j]['capacity'])
                dis = self.list_distance(self.node_all[i]['pos'], self.node_all[j]['pos'])
                edge = str(i) + ',' + str(j)
                edge_capacity = ca_min * (dis / (2*self.safety_dis))

                d_n = {'capacity': edge_capacity, 'dis': dis}
                self.edge_all[edge] = d_n

    def find_node_with_up_cell(self, cell_n):
        node_list = []
        for node, node_info in self.node_all.items():
            if 'cell_up_set' in node_info:
                cell_set = node_info['cell_up_set']
                if cell_n in cell_set:
                    node_list.append(node)
        return node_list

    def find_node_with_down_cell(self, cell_n):
        node_list = []
        for node, node_info in self.node_all.items():
            if 'cell_down_set' in node_info:
                cell_set = node_info['cell_down_set']
                if cell_n in cell_set:
                    node_list.append(node)
        return node_list

    def compute_edge_capacity(self, i, j):
        pos_i = self.node_all[i]['pos']
        pos_j = self.node_all[j]['pos']
        ca_min = min(self.node_all[i]['capacity'], self.node_all[j]['capacity'])
        dis = self.list_distance(self.node_all[i]['pos'], self.node_all[j]['pos'])

        x1, y1 = pos_i
        x2, y2 = pos_j
        vector_x = x2 - x1
        vector_y = y2 - y1
        angle = math.atan2(vector_y, vector_x)
        sin_value = math.sin(angle)

        capacity_edge = int(sin_value*dis/(2*self.safety_dis)*ca_min)
        if capacity_edge < 1:
            capacity_edge = 1
        return capacity_edge

    def construct_flow_network_one(self):
        # Build a network of nodes and edges
        # Find adjacent edges and add nodes
        for i in range(len(self.cell_info)):

            cell_n = self.cell_info[i]
            down_node = cell_n['down_node']
            up_node = cell_n['up_node']

            up_edge = [cell_n['left_up'], cell_n['right_up']]
            if bool(up_node):
                for cell in up_node:
                    up_cell = self.cell_info[cell]
                    up_cell_down_edge = [up_cell['left_down'], up_cell['right_down']]
                    self.add_new_node_one(cell_n, up_edge, up_cell, up_cell_down_edge)

        # Find the connection between nodes, establish edges and establish capacity for edges
        for i in range(len(self.node_all)):
            # cell_set_i = self.node_all[i]['cell_set']
            cell_up_set_i = self.node_all[i]['cell_up_set']
            for cell_i in cell_up_set_i:
                for j in range(len(self.node_all)):
                    # if i >= j:
                    #     continue
                    cell_down_set_j = self.node_all[j]['cell_down_set']
                    for cell_j in cell_down_set_j:
                        if cell_i == cell_j:
                            # if self.node_all[i]['pos'][1] == self.node_all[j]['pos'][1]:
                            #     continue
                            ca_min = max(self.node_all[i]['capacity'], self.node_all[j]['capacity'])
                            dis = self.list_distance(self.node_all[i]['pos'], self.node_all[j]['pos'])
                            edge = str(i) + ',' + str(j)
                            # 储存的是 list [capacity, dis]
                            capacity_n = self.compute_edge_capacity(i, j)
                            if capacity_n == 0:
                                capacity_n = 0.1
                            d_n = {'capacity': capacity_n, 'dis': dis}
                            self.edge_all[edge] = d_n

        # Find which nodes are connected to the starting cell and
        # the ending cell and add the starting and ending points and edges
        s_cell_idx = 0
        g_cell_idx = self.number_cells - 1

        node_sum = len(self.node_all)
        self.start_idx = node_sum
        self.end_idx = node_sum + 1
        self.start_node = node_sum
        self.end_node = node_sum + 1
        self.node_start_end = {}

        # s_pos = self.cell_info[s_cell_idx]['pos']
        s_pos = self.swarm.start_ave_pos
        dic_s = {'pos': s_pos, 'capacity': 1}
        self.node_start_end[node_sum] = dic_s
        # self.node_all[node_sum] = dic_s

        # g_pos = self.cell_info[g_cell_idx]['pos']
        g_pos = self.swarm.goal_ave_pos
        dic_g = {'pos': g_pos, 'capacity': 1}
        self.node_start_end[node_sum + 1] = dic_g
        # self.node_all[node_sum + 1] = dic_g

        for i in range(len(self.node_all)):
            node_cell_set = self.node_all[i]['cell_set']
            for cell in node_cell_set:
                ca = self.node_all[i]['capacity']
                if cell == s_cell_idx:
                    edge = str(node_sum) + ',' + str(i)
                    dis = self.list_distance(self.node_all[i]['pos'], self.node_start_end[node_sum]['pos'])
                    d_n = {'capacity': ca, 'dis': dis}
                    self.edge_all[edge] = d_n
                if cell == g_cell_idx:
                    edge = str(i) + ',' + str(node_sum + 1)
                    dis = self.list_distance(self.node_all[i]['pos'], self.node_start_end[node_sum + 1]['pos'])
                    d_n = {'capacity': ca, 'dis': dis}
                    self.edge_all[edge] = d_n

    def construct_flow_network_two(self):
        # Build a network of nodes and edges
        # Find adjacent edges and add nodes
        for i in range(len(self.cell_info)):

            if i in self.delete_cell_list:
                continue

            cell_n = self.cell_info[i]
            down_node = cell_n['down_node']
            up_node = cell_n['up_node']

            up_edge = [cell_n['left_up'], cell_n['right_up']]
            if bool(up_node):
                for cell in up_node:
                    up_cell = self.cell_info[cell]
                    up_cell_down_edge = [up_cell['left_down'], up_cell['right_down']]
                    self.add_new_node_one(cell_n, up_edge, up_cell, up_cell_down_edge)

        for i in range(len(self.node_all)):
            cell_up_set_i = self.node_all[i]['cell_up_set']
            for cell_i in cell_up_set_i:
                for j in range(len(self.node_all)):
                    # if i >= j:
                    #     continue
                    cell_down_set_j = self.node_all[j]['cell_down_set']
                    for cell_j in cell_down_set_j:
                        if cell_i == cell_j:
                            # if self.node_all[i]['pos'][1] == self.node_all[j]['pos'][1]:
                            #     continue
                            ca_min = max(self.node_all[i]['capacity'], self.node_all[j]['capacity'])
                            dis = self.list_distance(self.node_all[i]['pos'], self.node_all[j]['pos'])
                            edge = str(i) + ',' + str(j)
                            capacity_n = self.compute_edge_capacity(i, j)
                            if capacity_n == 0:
                                capacity_n = 0.1
                            d_n = {'capacity': capacity_n, 'dis': dis}
                            self.edge_all[edge] = d_n

        # Find which nodes are connected to the starting cell and
        # the ending cell and add the starting and ending points and edges
        s_cell_idx = 0
        g_cell_idx = self.number_cells - 1

        node_sum = len(self.node_all)
        self.start_idx = node_sum
        self.end_idx = node_sum + 1
        self.start_node = node_sum
        self.end_node = node_sum + 1
        self.node_start_end = {}

        # s_pos = self.cell_info[s_cell_idx]['pos']
        s_pos = self.swarm.start_ave_pos
        dic_s = {'pos': s_pos, 'capacity': 1}
        self.node_start_end[node_sum] = dic_s
        # self.node_all[node_sum] = dic_s

        # g_pos = self.cell_info[g_cell_idx]['pos']
        g_pos = self.swarm.goal_ave_pos
        dic_g = {'pos': g_pos, 'capacity': 1}
        self.node_start_end[node_sum + 1] = dic_g
        # self.node_all[node_sum + 1] = dic_g

        for i in range(len(self.node_all)):
            node_cell_set = self.node_all[i]['cell_set']
            for cell in node_cell_set:
                ca = self.node_all[i]['capacity']
                if cell == s_cell_idx:
                    edge = str(node_sum) + ',' + str(i)
                    dis = self.list_distance(self.node_all[i]['pos'], self.node_start_end[node_sum]['pos'])
                    d_n = {'capacity': ca, 'dis': dis}
                    self.edge_all[edge] = d_n
                if cell == g_cell_idx:
                    edge = str(i) + ',' + str(node_sum + 1)
                    dis = self.list_distance(self.node_all[i]['pos'], self.node_start_end[node_sum + 1]['pos'])
                    d_n = {'capacity': ca, 'dis': dis}
                    self.edge_all[edge] = d_n

    def list_distance(self, l1, l2):
        vector1 = np.array([l1[0], l1[1]])
        vector2 = np.array([l2[0], l2[1]])
        result_vector = vector1 - vector2
        dis = np.linalg.norm(result_vector)
        return dis

    def calculate_discrete_positions(self, p1, p2, b):
        """
        Compute and return the coordinates of discrete positions on a line segment.
        :return: A list containing the coordinates of all discrete positions,
        each of which is also a coordinate of the form (x, y).
        """
        distance = np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

        positions_count = int(distance // b)

        discrete_positions = []
        for i in range(1, positions_count):
            if i == 0:
                continue
            t = i * b / distance
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            discrete_positions.append((x, y))

        return discrete_positions

    def overlap_segment(self, n_p1, n_p2, m_p1, m_p2):
        x1 = n_p1[0]
        y1 = n_p1[1]

        x2 = n_p2[0]

        x3 = m_p1[0]

        x4 = m_p2[0]

        if x1 > x2:
            x1, x2 = x2, x1
        if x3 > x4:
            x3, x4 = x4, x3

        start = max(x1, x3)
        end = min(x2, x4)

        if start <= end:
            return [start, y1], [end, y1]
        else:
            return None

    def add_new_node_one(self, cell_n, cell_n_edge, cell_m, cell_m_edge):
        # After judgment, add a new node to enter
        # Additional information required: adjacent cell capacity nearby location

        dis_1 = cell_n_edge[1][0] - cell_n_edge[0][0]
        dis_2 = cell_m_edge[1][0] - cell_m_edge[0][0]

        cell_n_p1 = [cell_n_edge[1][0], cell_n_edge[1][1]]
        cell_n_p2 = [cell_n_edge[0][0], cell_n_edge[0][1]]

        cell_m_p1 = [cell_m_edge[1][0], cell_m_edge[1][1]]
        cell_m_p2 = [cell_m_edge[0][0], cell_m_edge[0][1]]

        # logging.info("cell_n idx=" + str(cell_n['idx']))
        # logging.info(cell_n['pos'])
        #
        # logging.info("cell_m idx=" + str(cell_m['idx']))
        # logging.info(cell_m['pos'])

        if dis_1 <= 0 or dis_2 <= 0:
            # pass
            logging.warning("add node-error 1")
        if cell_n_edge[0][1] != cell_n_edge[1][1] or cell_m_edge[0][1] != cell_m_edge[1][1]:
            # pass
            logging.warning("add node-error 2")

        over_p1, over_p2 = self.overlap_segment(cell_n_p1, cell_n_p2, cell_m_p1, cell_m_p2)
        node_option_pos = self.calculate_discrete_positions(over_p1, over_p2, 2*self.safety_dis)
        node_option_num = len(node_option_pos)

        partition = 4

        node_option_pos_n = []
        number = 0
        for i in range(node_option_num):
            p = node_option_pos[i]
            node_option_pos_n.append(p)
            number += 1

            if number == partition and i != node_option_num - 1:
                node_option_num_n = len(node_option_pos_n)
                node_pos_n = self.compute_ave_pos(node_option_pos_n)
                # add new node
                idx = len(self.node_all)

                cell_idx_set = set()
                cell_idx_set.add(cell_n['idx'])
                cell_idx_set.add(cell_m['idx'])
                cell_up_set = set()
                cell_down_set = set()
                cell_up_set.add(cell_m['idx'])
                cell_down_set.add(cell_n['idx'])
                c_info = {'pos': node_pos_n, 'capacity': node_option_num_n, 'cell_set': cell_idx_set,
                          'cell_up_set': cell_up_set, 'cell_down_set': cell_down_set,
                          'node_idx': idx, 'node_option_pos': node_option_pos_n, 'node_option_num': node_option_num_n}
                self.node_all[idx] = c_info

                # clear info
                node_option_pos_n = []
                number = 0

            elif i == node_option_num - 1:
                node_option_num_n = len(node_option_pos_n)
                node_pos_n = self.compute_ave_pos(node_option_pos_n)
                # add new node
                idx = len(self.node_all)

                cell_idx_set = set()
                cell_idx_set.add(cell_n['idx'])
                cell_idx_set.add(cell_m['idx'])
                cell_up_set = set()
                cell_down_set = set()
                cell_up_set.add(cell_m['idx'])
                cell_down_set.add(cell_n['idx'])
                c_info = {'pos': node_pos_n, 'capacity': node_option_num_n, 'cell_set': cell_idx_set,
                          'cell_up_set': cell_up_set, 'cell_down_set': cell_down_set,
                          'node_idx': idx, 'node_option_pos': node_option_pos_n, 'node_option_num': node_option_num_n}
                self.node_all[idx] = c_info

                # clear info
                node_option_pos_n = []
                number = 0

    def compute_ave_pos(self, node_option_pos):
        x = 0
        y = 0
        for pos in node_option_pos:
            x += pos[0]
            y += pos[1]
        x = x / len(node_option_pos)
        y = y / len(node_option_pos)

        return (x, y)

    def add_new_node(self, cell_n, cell_n_edge, cell_m, cell_m_edge):
        # After judgment, add a new node to enter
        # Additional information required: adjacent cell capacity nearby location

        dis_1 = cell_n_edge[1][0] - cell_n_edge[0][0]
        dis_2 = cell_m_edge[1][0] - cell_m_edge[0][0]

        cell_n_p1 = [cell_n_edge[1][0], cell_n_edge[1][1]]
        cell_n_p2 = [cell_n_edge[0][0], cell_n_edge[0][1]]

        cell_m_p1 = [cell_m_edge[1][0], cell_m_edge[1][1]]
        cell_m_p2 = [cell_m_edge[0][0], cell_m_edge[0][1]]

        if dis_1 <= 0 or dis_2 <= 0:
            print("add node-error 1")
        if cell_n_edge[0][1] != cell_n_edge[1][1] or cell_m_edge[0][1] != cell_m_edge[1][1]:
            logging.warning("add node-error 2")
        if dis_2 > dis_1:
            # Establish nodes with dis_1 as the center
            node_pos = [(cell_n_edge[1][0] + cell_n_edge[0][0]) / 2, cell_n_edge[0][1]]
            capacity = int(dis_1 / (2*self.safety_dis))
            node_option_pos = self.calculate_discrete_positions(cell_n_p1, cell_n_p2, 2*self.safety_dis)
            node_option_num = len(node_option_pos)
        else:
            node_pos = [(cell_m_edge[1][0] + cell_m_edge[0][0]) / 2, cell_m_edge[0][1]]
            capacity = int(dis_2 / (2*self.safety_dis))
            node_option_pos = self.calculate_discrete_positions(cell_m_p1, cell_m_p2, 2*self.safety_dis)
            node_option_num = len(node_option_pos)

        idx = len(self.node_all)
        cell_idx_set = set()
        cell_idx_set.add(cell_n['idx'])
        cell_idx_set.add(cell_m['idx'])

        cell_up_set = set()
        cell_down_set = set()
        cell_up_set.add(cell_m['idx'])
        cell_down_set.add(cell_n['idx'])

        c_info = {'pos': node_pos, 'capacity': capacity, 'cell_set': cell_idx_set,
                  'cell_up_set': cell_up_set, 'cell_down_set': cell_down_set,
                  'node_idx': idx, 'node_option_pos': node_option_pos, 'node_option_num': node_option_num}
        self.node_all[idx] = c_info

    def extract_info(self):
        self.x_size = self.map_all.shape[0]
        self.y_size = self.map_all.shape[1]

        self.cell_dict = {index - 1: value for index, value in self.cell_dict.items() if index != 0}
        self.number_cells = self.number_cells - 1

        for i in range(self.number_cells):
            dic_n = {}
            area_i = len(self.cell_dict[i])
            dic_n['area'] = area_i

            # First find the upper and lower boundaries of the
            # y-axis, then find the left and right boundaries of the x-axis
            y_up, y_down = self.find_up_down_boundary_cell(self.cell_dict[i])

            left_up, left_down, right_up, right_down = self.find_left_right_boundary(y_up, y_down, self.cell_dict[i])
            node1 = [left_up, y_up]
            node2 = [left_down, y_down]
            node3 = [right_up, y_up]
            node4 = [right_down, y_down]

            dic_n['left_up'] = node1
            dic_n['left_down'] = node2
            dic_n['right_up'] = node3
            dic_n['right_down'] = node4
            # dic_n['pos'] = [(x_left + x_right) / 2 / self.resolution,
            #                 (left_up + left_down + right_up + right_down) / 4 / self.resolution]
            dic_n['pos'] = [(left_up + left_down + right_up + right_down) / 4 / self.resolution
                , (y_up + y_down) / 2 / self.resolution, ]
            dic_n['idx'] = i

            self.cell_info[i] = dic_n

    def construct_cell_structure(self):
        # Establish connections between cells
        # Find which nodes are connected to the upper and lower boundaries
        for i in range(0, self.number_cells):
            # print("cell=" + str(i))
            cell_n = self.cell_dict[i]
            cell_info_n = self.cell_info[i]

            left_up = cell_info_n['left_up']
            left_down = cell_info_n['left_down']
            right_up = cell_info_n['right_up']
            right_down = cell_info_n['right_down']

            # print(left_up)
            # print(left_down)
            # print(right_up)
            # print(right_down)

            # Traverse the lower boundary to find which cells are adjacent
            down_y = left_down[1]
            left_x = left_down[0]
            right_x = right_down[0]
            down_node = set()
            for x in range(left_x, right_x + 1):
                idx = [x, down_y - 1]
                value = self.map_all[idx[0], idx[1]] - 1
                if value != -1 and value != i:
                    down_node.add(value)
            # Traverse the upper boundary to find which cells are adjacent
            up_y = right_up[1]
            left_x = left_up[0]
            right_x = right_up[0]
            up_node = set()
            for x in range(left_x, right_x + 1):
                idx = [x, up_y + 1]
                value = self.map_all[idx[0], idx[1]] - 1
                if value != -1 and value != i:
                    up_node.add(value)
            self.cell_info[i]['down_node'] = down_node
            self.cell_info[i]['up_node'] = up_node

    def find_up_down_boundary_cell(self, cell_dict):
        # Find the x value of the current left and right boundaries
        y_down = 999999999
        y_up = -100
        for idx in cell_dict:
            if idx[1] <= y_down:
                y_down = idx[1]
            if idx[1] >= y_up:
                y_up = idx[1]
        return y_up, y_down

    def find_left_right_boundary(self, y_up, y_down, cell):
        # Find the left and right y values of the upper and lower boundaries
        left_up = 999999999
        right_up = -100

        left_down = 99999999
        right_down = -100

        if y_up == y_down:
            for idx in cell:
                if idx[1] == y_up:
                    if idx[0] <= left_up:
                        left_up = idx[0]
                    if idx[0] >= right_up:
                        right_up = idx[0]
            right_down = right_up
            left_down = left_up
            return left_up, left_down, right_up, right_down

        for idx in cell:
            if idx[1] == y_up:
                if idx[0] <= left_up:
                    left_up = idx[0]
                if idx[0] >= right_up:
                    right_up = idx[0]
            elif idx[1] == y_down:
                if idx[0] >= right_down:
                    right_down = idx[0]
                if idx[0] <= left_down:
                    left_down = idx[0]
            else:
                pass

        return left_up, left_down, right_up, right_down

    def find_up_down_boundary(self, x_left, x_right, cell):
        # Find the upper and lower y values of the left and right boundaries
        left_up = -1
        left_down = 9999999
        right_up = -1
        right_down = 9999999

        for idx in cell:
            if idx[0] == x_left:
                if idx[1] >= left_up:
                    left_up = idx[1]
                if idx[1] <= left_down:
                    left_down = idx[1]
            elif idx[0] == x_right:
                if idx[1] >= right_up:
                    right_up = idx[1]
                if idx[1] <= right_down:
                    right_down = idx[1]
            else:
                pass

        return left_up, left_down, right_up, right_down

    def find_obs_rectangles(self, map_array):
        nrows, ncols = map_array.shape
        visited = np.zeros_like(map_array, dtype=bool)
        rectangles = []

        def dfs_iterative(x, y, rect):
            stack = [(x, y)]
            while stack:
                x, y = stack.pop()
                if x < 0 or x >= nrows or y < 0 or y >= ncols or visited[x, y] or map_array[x, y] != 0:
                    continue
                visited[x, y] = True
                rect[0] = min(rect[0], x)
                rect[1] = min(rect[1], y)
                rect[2] = max(rect[2], x)
                rect[3] = max(rect[3], y)
                stack.extend([(x + dx, y + dy) for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]])

        for i in range(nrows):
            for j in range(ncols):
                if map_array[i, j] == 0 and not visited[i, j]:
                    rect = [i, j, i, j]  # x_min, y_min, x_max, y_max
                    dfs_iterative(i, j, rect)
                    # Convert to top-left and bottom-right corners
                    rectangles.append([(rect[0], rect[1]), (rect[2], rect[3])])

        return len(rectangles), rectangles

    def plot_matrix_map(self, path):
        plt.figure()

        plt.imshow(self.map_01 == 1, cmap='Greys', origin='lower')

        plt.imshow(self.map_01 == 0, cmap='Accent_r', origin='lower', alpha=0.5)

        path = np.array(path)
        plt.plot(path[:, 1], path[:, 0], color='lime', linewidth=2)

        plt.show()

    def display_separate_cell(self, separate_map, cells):
        fig_new = plt.figure()

        display_img = np.empty([*separate_map.shape, 3], dtype=np.uint8)
        random_colors = np.random.randint(0, 255, [cells, 3])
        for cell_id in range(1, cells):
            display_img[separate_map == cell_id, :] = random_colors[cell_id, :]
        for idx, cell in self.cell_info.items():
            pos = cell['pos']
            name = 'Ce' + str(idx)
            plt.text(pos[0] * self.resolution, pos[1] * self.resolution, name, ha='center', va='center')

        display_img_rotated = np.rot90(display_img)
        display_img_rotated_flipped = np.flipud(display_img_rotated)
        plt.imshow(display_img_rotated_flipped)
        plt.gca().invert_yaxis()
        # plt.imshow(display_img)

    def plot_large_ndarray(self, ndarray):
        fig, ax = plt.subplots(figsize=(14, 8))  # Adjust the figure size as necessary

        # Flatten the array and get unique values and their positions
        unique_values, indices = np.unique(ndarray, return_index=True)
        rows, cols = np.unravel_index(indices, ndarray.shape)

        # Plot each unique number at its position
        for (row, col, value) in zip(rows, cols, unique_values):
            ax.text(col, row, str(value), ha='center', va='center', fontsize=8)

        # Adjust the limits and invert the y-axis
        ax.set_xlim(-1, ndarray.shape[1])
        ax.set_ylim(-1, ndarray.shape[0])
        ax.invert_yaxis()

        # Hide the axes
        ax.xaxis.set_visible(False)
        ax.yaxis.set_visible(False)

        plt.show()

    def read_map(self):
        # Read the original data
        original_map = cv2.imread(self.pic_name)

        # 1's represents free space while 0's represents objects/walls
        if len(original_map.shape) > 2:
            # logging.info("Map image is converted to binary")
            single_channel_map = original_map[:, :, 0]
            _, binary_map = cv2.threshold(single_channel_map, 127, 1, cv2.THRESH_BINARY)

        num_obs, obstacles_rect = self.find_obs_rectangles(binary_map)

        # Call The Boustrophedon Cellular Decomposition function
        bcd_out_im, bcd_out_cells, cell_numbers, cell_boundaries, non_neighboor_cell_numbers = bcd(binary_map)

        self.bcd_out_cells = copy.deepcopy(bcd_out_cells)
        self.bcd_out_im = copy.deepcopy(bcd_out_im)

        self.number_cells = bcd_out_cells
        self.map_all = bcd_out_im

        # self.plot_large_ndarray(self.map_all)

        self.map_01 = copy.deepcopy(bcd_out_im)
        self.map_01[self.map_01 >= 1] = 2
        # Assign the value of 0 to 1
        self.map_01[self.map_01 == 0] = 1
        # Assign all elements with value -1 to 0
        self.map_01[self.map_01 == 2] = 0
        # self.map_01 = self.map_01[self.map_01 != 0] = 1

        dilated_size = 20

        structure_element = np.ones((dilated_size, dilated_size))

        # Expand the obstacle map
        self.dilated_map = binary_dilation(self.map_01, structure=structure_element).astype(self.map_01.dtype)

        # self.plot_matrix_map()
        self.num_obs_rect = num_obs
        self.obstacles_rect = obstacles_rect

        for i in range(bcd_out_cells):
            li_n = []
            self.cell_dict[i] = copy.copy(li_n)
        for idx, element in np.ndenumerate(bcd_out_im):
            # logging.info(f"Index: {idx}, Value: {element}")
            li_index = [idx[0], idx[1]]
            self.cell_dict[element].append(copy.copy(li_index))

