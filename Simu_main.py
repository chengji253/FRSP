import copy
import sys
import os
import numpy as np
import time
import json
from RVO_module import RVO
from mapCO import mapCO
from map_info import map_info
from para import swarm
from flow_planner import flow_main
from visualize import plotPic
import logging
import colorlog
# from vis import visualize_traj_dynamic


def set_debug_mode(debug_mode):
    handler = colorlog.StreamHandler()
    formatter = colorlog.ColoredFormatter(
        '%(log_color)s%(asctime)s - %(levelname)s: %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
        log_colors={
            'DEBUG': 'blue',
            'INFO': 'light_white',
            'WARNING': 'yellow',
            'ERROR': 'red',
            'CRITICAL': 'bold_red',
        }
    )
    handler.setFormatter(formatter)
    logger = colorlog.getLogger()
    if logger.hasHandlers():
        logger.handlers.clear()
    logger.addHandler(handler)

    if debug_mode:
        logger.setLevel(logging.INFO)
    else:
        logger.setLevel(logging.WARNING)


class Simu_main:

    def __init__(self, pic_name, forest_bool):

        if forest_bool is True:
            self.add_name = 'forest_' + pic_name
            self.pic_name = "pic/forest/" + pic_name + ".png"
        else:
            self.add_name = 'maze_' + pic_name
            self.pic_name = "pic/maze/" + pic_name + ".png"

        self.title_name = None
        self.total_time = None
        self.time_step = None
        self.resolution = 10

        self.swarm = swarm.Swarm()
        self.RVO = RVO.Velocity_obstacle()
        self.mapCo = None
        self.mapInfo = None
        self.flow_planner = None
        self.planner_time = 0
        self.planner_run_num = 0

        self.t_r_flow_list = []
        self.average_t_r_flow = None

    def choose_para_swarm(self, number):
        functions = {
            2: self.para_2,
            10: self.para_10,
            30: self.para_30,
            50: self.para_50,
            80: self.para_80,
            100: self.para_100,
            150: self.para_150,
            200: self.para_200,
            250: self.para_250,
            300: self.para_300,
            350: self.para_350,
            400: self.para_400,
            450: self.para_450,
            500: self.para_500
        }
        if number in functions:
            functions[number]()
        else:
            logging.warning(f"No para available for {number}")

        self.RVO.around_no_robots = [False] * self.swarm.robots_num

    def para_2(self):
        self.title_name = 'data/' + self.add_name + '/para_2'
        self.total_time = 30
        self.time_step = 0.01
        self.swarm.init_swarm_2()

    def para_10(self):
        self.title_name = 'data/' + self.add_name + '/para_10'
        self.total_time = 40
        self.time_step = 0.01
        self.swarm.init_swarm_10()

    def para_30(self):
        self.title_name = 'data/' + self.add_name + '/para_30'
        self.total_time = 40
        self.time_step = 0.01
        self.swarm.init_swarm_30()

    def para_50(self):
        self.title_name = 'data/' + self.add_name + '/para_50'
        self.total_time = 50
        self.time_step = 0.01
        # self.swarm.init_swarm_1()
        self.swarm.init_swarm_50()

    def para_80(self):
        self.title_name = 'data/' + self.add_name + '/para_80'
        self.total_time = 60
        self.time_step = 0.01
        # self.swarm.init_swarm_1()
        self.swarm.init_swarm_80()

    def para_100(self):
        self.title_name = 'data/' + self.add_name + '/para_100'
        self.total_time = 50
        self.time_step = 0.01
        # self.swarm.init_swarm_1()
        self.swarm.init_swarm_100()

    def para_150(self):
        self.title_name = 'data/' + self.add_name + '/para_150'
        self.total_time = 50
        self.time_step = 0.01
        # self.swarm.init_swarm_1()
        self.swarm.init_swarm_150()

    def para_200(self):
        self.title_name = 'data/' + self.add_name + '/para_200'
        self.total_time = 50
        self.time_step = 0.01
        self.swarm.init_swarm_200()

    def para_250(self):
        self.title_name = 'data/' + self.add_name + '/para_250'
        self.total_time = 55
        self.time_step = 0.01
        self.swarm.init_swarm_250()

    def para_300(self):
        self.title_name = 'data/' + self.add_name + '/para_300'
        self.total_time = 55
        self.time_step = 0.01
        self.swarm.init_swarm_300()

    def para_350(self):
        self.title_name = 'data/' + self.add_name + '/para_350'
        self.total_time = 55
        self.time_step = 0.01
        self.swarm.init_swarm_350()

    def para_400(self):
        self.title_name = 'data/' + self.add_name + '/para_400'
        self.total_time = 55
        self.time_step = 0.01
        self.swarm.init_swarm_400()

    def para_450(self):
        self.title_name = 'data/' + self.add_name + '/para_450'
        self.total_time = 55
        self.time_step = 0.01
        self.swarm.init_swarm_450()

    def para_500(self):
        self.title_name = 'data/' + self.add_name + '/para_500'
        self.total_time = 55
        self.time_step = 0.01
        self.swarm.init_swarm_500()

    def compute_average_time_result(self):
        array = np.array(self.t_r_flow_list)
        # 计算列的平均值
        self.average_t_r_flow = np.mean(array, axis=0)

    def init_simulation_flow_planner_forest(self):
        self.title_name = self.title_name + "flow"

        self.swarm.compute_ave_start_goal_pos()
        self.mapCo = mapCO.mapCo()
        self.mapInfo = map_info.MapInfo(self.swarm, self.pic_name, self.resolution)
        self.swarm.reach_dis = 2
        self.swarm.reach_dis_goal = 3

        self.flow_planner = flow_main.Flow_planner(self.swarm, self.mapInfo)
        self.flow_planner.k_1 = 1
        self.flow_planner.k_2 = 0.5
        self.flow_planner.k_3 = 0.5
        logging.info("k_1=" + str(self.flow_planner.k_1) + ", k_2=" + str(self.flow_planner.k_2)
                     + ", k_3=" + str(self.flow_planner.k_3))

        t1 = time.time()
        self.mapInfo.init_main_one()
        t2 = time.time()
        self.mapCo.init_mapCo_one(self.mapInfo, self.resolution)
        t3 = time.time()
        self.swarm.set_start_goal_node(self.mapInfo.start_idx, self.mapInfo.end_idx, 0, self.mapInfo.number_cells - 1)
        # self.mapInfo.draw_mapInfo()

        t_r_flow = self.flow_planner_run()
        t4 = time.time()
        self.planner_time += t4 - t3
        self.planner_run_num += 1
        self.t_r_flow_list.append(t_r_flow)
        logging.info("mapInfo_time=" + str(t2 - t1) + "s")
        logging.info("mapCo_time=" + str(t3 - t2) + "s")
        logging.info("flow_planner_first=" + str(t4 - t3) + "s")

    def init_simulation_flow_planner_maze(self):
        self.title_name = self.title_name + "flow"
        self.swarm.compute_ave_start_goal_pos()
        self.swarm.reach_dis = 1
        self.swarm.reach_dis_goal = 3

        self.mapCo = mapCO.mapCo()
        self.mapInfo = map_info.MapInfo(self.swarm, self.pic_name, self.resolution)

        self.flow_planner = flow_main.Flow_planner(self.swarm, self.mapInfo)
        self.flow_planner.k_1 = 1
        self.flow_planner.k_2 = 0.5
        self.flow_planner.k_3 = 0.5

        t1 = time.time()
        self.mapInfo.init_main()
        t2 = time.time()
        self.mapCo.init_mapCo_one(self.mapInfo, self.resolution)
        t3 = time.time()
        self.swarm.set_start_goal_node(self.mapInfo.start_idx, self.mapInfo.end_idx, 0, self.mapInfo.number_cells - 1)

        # self.mapInfo.draw_mapInfo()

        t_r_flow = self.flow_planner_run()
        t4 = time.time()
        self.planner_time += t4 - t3
        self.planner_run_num += 1
        self.t_r_flow_list.append(t_r_flow)
        logging.info("mapInfo_time=" + str(t2 - t1) + "s")
        logging.info("mapCo_time=" + str(t3 - t2) + "s")
        logging.info("flow_planner_first=" + str(t4 - t3) + "s")

    def deal_with_dead_lock(self, i):
        # When the current robot is found to be in a deadlock state,
        # search for a new path based on the current cell state
        current_cell = self.swarm.current_cell[i]
        node_list = self.mapInfo.find_cell_up_node(current_cell)
        g_n = self.swarm.goal_node[i]
        if len(node_list) == 0:
            path_node = [g_n]
        else:
            node_n = node_list[0]
            path_node, path_length = self.mapInfo.find_path_cell_node(node_n, g_n)

        self.swarm.des_path_node[i] = copy.copy(path_node)
        self.set_center_pos_each(i)
        self.flow_planner.position_allocation_greedy_now_each(i)

    def flow_planner_run(self):
        tp1 = time.time()
        self.mapInfo.path_set_search()
        self.mapInfo.shared_edge_to_path()
        self.flow_planner.compute_num_on_edge()
        self.flow_planner.init_para_shared_edge()
        tp2 = time.time()

        des_path_node = self.flow_planner.path_node_selection()

        tp3 = time.time()
        if len(des_path_node) != 0:
            self.swarm.des_path_node = des_path_node
            self.set_center_pos()
            self.flow_planner.local_position_allocation()
            self.swarm.para_set_planner()
        tp4 = time.time()

        t_r = [tp2 - tp1, tp3 - tp2, tp4 - tp3]
        logging.info("Path set search =" + str(tp2 - tp1) + "s")
        logging.info("Path node selection =" + str(tp3 - tp2) + "s")
        logging.info("Local position_allocation =" + str(tp4 - tp3) + "s")
        return t_r

    def set_center_pos_each(self, i):
        path_node = self.swarm.des_path_node[i]
        pos_list = []
        for node in path_node:
            if node == self.swarm.goal_node[i]:
                pos = self.swarm.goal_pos[i]
            elif node == self.swarm.start_node[i]:
                pos = self.mapInfo.node_start_end[node]['pos']
            else:
                pos = self.mapInfo.node_all[node]['pos']
                pos = [pos[0], pos[1]]
            pos_list.append(pos)
        self.swarm.des_path_pos[i] = pos_list

    def set_center_pos(self):
        des_path_pos = []
        for i in range(self.swarm.robots_num):
            path_node = self.swarm.des_path_node[i]
            pos_list = []
            for node in path_node:
                if node == self.swarm.goal_node[i]:
                    pos = self.swarm.goal_pos[i]
                elif node == self.swarm.start_node[i]:
                    pos = self.mapInfo.node_start_end[node]['pos']
                else:
                    pos = self.mapInfo.node_all[node]['pos']
                    pos = [pos[0], pos[1]]
                pos_list.append(pos)
            des_path_pos.append(pos_list)
        self.swarm.des_path_pos = des_path_pos

    def data_to_json(self, file_path, data):
        if os.path.exists(file_path):
            os.remove(file_path)
        with open(file_path, 'w', encoding='utf-8') as json_file:
            # json.dump(data, json_file, ensure_ascii=False, indent=4)
            json.dump(data, json_file, ensure_ascii=False, indent=4)

        logging.info(f"date to {file_path}")
        logging.info("-----")

    def update_cell_state(self, i):
        if self.swarm.robot_exist[i] is False:
            return

        pos_idx = [int(self.swarm.pos_all[i][0] * self.resolution), int(self.swarm.pos_all[i][1] * self.resolution)]

        size_n = self.mapInfo.map_01.shape
        if 0 <= pos_idx[0] < size_n[0] and 0 <= pos_idx[1] < size_n[1]:
            cell_idx = self.mapInfo.map_all[pos_idx[0], pos_idx[1]] - 1
            if cell_idx != -1:
                self.swarm.current_cell[i] = copy.copy(cell_idx)
        else:
            return

    def simulate_main_flow_planner(self):
        # define workspace model
        ws_model = dict()
        ws_model['robot_radius'] = self.swarm.robot_radius
        ws_model['circular_obstacles'] = self.mapCo.obstacles
        # simulation setup
        # total simulation time (s)
        total_time = self.total_time
        # simulation step
        step = self.time_step
        # visualization
        name_n = '/snap%s.png'
        name_n = self.title_name + name_n

        # plotPic.plot_JPS_traj(ws_model, self.swarm, self.mapCo.boundary, self.mapInfo)
        folder_path = os.path.dirname(name_n)
        if os.path.exists(folder_path):
            for filename in os.listdir(folder_path):
                file_path = os.path.join(folder_path, filename)
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.remove(file_path)
        else:
            os.makedirs(folder_path)

        # simulation starts
        t = 0

        while t * step < total_time:

            if t % 10 == 0:
                self.swarm.store_pos_v_data()

            self.compute_V_RVO(ws_model)
            # update position
            if t != 0 and t % 100 == 0:
                tp1 = time.time()
                t_r_flow = self.flow_planner_run()
                tp2 = time.time()
                self.t_r_flow_list.append(t_r_flow)
                self.planner_time += tp2 - tp1
                self.planner_run_num += 1
                logging.info("planner_time=" + str(tp2 - tp1) + "s")
            for i in range(self.swarm.robots_num):
                if self.swarm.find_dead_lock(self.swarm.V_all, i, self.RVO.around_no_robots):
                    self.swarm.deal_with_dead_lock(i)
                self.swarm.forward_pos_state(i, self.time_step)
                self.update_cell_state(i)

                if self.swarm.judge_reach_des(i):
                    self.swarm.update_state_idx(i)
            # ----------------------------------------
            if t % 20 == 0:
                plotPic.visualize_traj_dynamic(ws_model, self.swarm, self.mapCo.boundary, self.mapInfo, time=t * step,
                                               name=name_n % str(t / 10))
                logging.info("t=" + str(t))
            if self.swarm.judge_robot_all_exist():
                plotPic.visualize_traj_dynamic(ws_model, self.swarm, self.mapCo.boundary, self.mapInfo, time=(t) * step,
                                               name=name_n % str((t) / 10))
                break
            t += 1
            # print("t=" + str(t))

        logging.info("final run time_step=" + str(t * step))

    def compute_V_RVO(self, ws_model):
        # compute desired vel to goal
        V_des = self.RVO.compute_V_des(self.swarm.pos_all, self.swarm.des_pos, self.swarm.v_max)
        # compute the optimal vel to avoid collision
        V = self.RVO.RVO_update(self.swarm.pos_all, V_des, self.swarm.V_all, ws_model,
                                self.swarm.robot_exist)

        self.swarm.V_all = copy.copy(V)
        for i in range(self.swarm.robots_num):
            if self.swarm.robot_exist[i] and self.swarm.current_cell[i] == self.swarm.goal_cell[i]:
                self.swarm.V_all[i] = copy.copy(V_des[i])
            # if self.swarm.robot_exist[i] and self.swarm.des_pos[i] == self.swarm.goal_pos[i]:


def run_flow_planner(map_name, num, forest_bool):
    logging.info("run_flow_planner")
    time1 = time.time()
    s1 = Simu_main(map_name, forest_bool)
    s1.choose_para_swarm(num)

    if forest_bool is True:
        s1.init_simulation_flow_planner_forest()
    else:
        s1.init_simulation_flow_planner_maze()

    # s1.mapInfo.draw_mapInfo()

    time2 = time.time()

    s1.simulate_main_flow_planner()
    time3 = time.time()

    init_time = time2 - time1
    run_time = time3 - time2

    s1.compute_average_time_result()

    logging.info("init_time=" + str(init_time) + "s")
    logging.info("run_time=" + str(run_time) + "s")
    logging.info("planner num=" + str(s1.planner_run_num))
    logging.info("planner time sum=" + str(s1.planner_time) + "s")
    logging.info("planner ave time=" + str(s1.planner_time / s1.planner_run_num) + "s")

    data_store = {'init_time': init_time, 'run_time': run_time, 'planner num': s1.planner_run_num,
                  'planner time': s1.planner_time, 'planner ave time': (s1.planner_time / s1.planner_run_num),
                  'planner three sections average time': s1.average_t_r_flow.tolist(),
                  'flow_section_list': s1.t_r_flow_list,
                  'pos_all_time': s1.swarm.pos_all_store, 'V_all_time': s1.swarm.V_all_store}
    file_path = s1.title_name + "/flow" + str(num) + ".json"

    s1.data_to_json(file_path, data_store)


def run_number_all_forest():
    forest_name_list = ["f1"]
    # forest_name_list = ["f1", "f2", "f3", "f4", "f5"]
    # num_list = [500]
    num_list = [500]
    # num_list = [50, 100, 150, 200, 250, 300, 350, 400, 450, 500]
    # num_list = [10, 30, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500]

    # run forest map
    for map_name in forest_name_list:
        for swarm_num in num_list:
            logging.info("map = " + map_name)
            logging.info("swarm number=" + str(swarm_num))
            run_flow_planner(map_name, swarm_num, True)


def run_number_all_maze():
    # maze_name_list = ["m5", "m3", "m4"]
    maze_name_list = ["m1", "m2", "m3", "m4", "m5"]
    # num_list = [100, 150, 200, 250, 300, 350, 400]
    num_list = [10]
    # num_list = [100, 150, 200, 250, 300, 350, 400]
    # num_list = [30, 50, 80, 150, 200, 250, 300, 350, 400]
    # num_list = [50, 100, 150, 200, 250, 300, 350, 400, 450, 500]

    # run maze map
    for map_name in maze_name_list:
        for swarm_num in num_list:
            logging.info("map = " + map_name)
            logging.info("swarm number=" + str(swarm_num))
            run_flow_planner(map_name, swarm_num, False)


set_debug_mode(True)
run_number_all_forest()
# run_number_all_maze()
