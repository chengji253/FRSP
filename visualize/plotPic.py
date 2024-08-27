import matplotlib
import matplotlib.pyplot as pyplot
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.patches import Polygon
import matplotlib.cm as cmx
import matplotlib.colors as colors
import numpy as np
import matplotlib.pyplot as plt


def visualize_traj_dynamic(ws_model, swarm, boundary, mapInfo, time=None, name=None):
    matplotlib.use('Agg')
    X = swarm.pos_all
    U = swarm.V_all
    goal = swarm.goal_pos
    resolution = 10

    figure = pyplot.figure(figsize=(7, 10))
    ax = figure.add_subplot(1, 1, 1)
    cmap = get_cmap(len(X))
    # plot obstacles
    for hole in ws_model['circular_obstacles']:
        srec = matplotlib.patches.Rectangle(
            (hole[0] - hole[2], hole[1] - hole[2]),
            2 * hole[2], 2 * hole[2],
            facecolor='dimgrey',
            fill=True,
            alpha=1)
        ax.add_patch(srec)
        circle = patches.Circle(
            (hole[0], hole[1]),  # 圆心坐标
            hole[2],  # 半径
            facecolor='dimgrey',  # 面颜色
            fill=True,  # 填充
            alpha=1)  # 透明度
        ax.add_patch(circle)
    for i in range(0, len(X)):
        # -------plot car
        if swarm.robot_exist[i] is False:
            continue
        robot = matplotlib.patches.Circle(
            (X[i][0], X[i][1]),
            radius=ws_model['robot_radius'],
            facecolor=cmap(i),
            edgecolor='black',
            linewidth=0.5,
            ls='solid',
            alpha=1,
            zorder=2)
        ax.add_patch(robot)
        # ----------plot velocity
        ax.arrow(X[i][0], X[i][1], U[i][0]*0.5, U[i][1]*0.5, head_width=0.15, head_length=0.1, fc=cmap(i), ec=cmap(i))
        # ax.text(X[i][0] - 0.1, X[i][1] - 0.1, r'$%s$' % i, fontsize=15, fontweight='bold', zorder=3)
        ax.plot([goal[i][0]], [goal[i][1]], '*', color=cmap(i), markersize=5, linewidth=3.0)

        # plot des pos of all drones
        pos = swarm.des_pos[i]
        # pos_n = matplotlib.patches.Rectangle(
        #     (pos[0] - 2*ws_model['robot_radius'], pos[1] - 2*ws_model['robot_radius']),
        #     width=4 * ws_model['robot_radius'],
        #     height=4 * ws_model['robot_radius'],
        #     facecolor='white',
        #     edgecolor='black',
        #     linewidth=0.5,
        #     ls='solid',
        #     alpha=1,
        #     zorder=2)
        # pos_n = matplotlib.patches.Circle(
        #                 (pos[0], pos[1]),
        #                 radius=ws_model['robot_radius'],
        #                 facecolor=cmap(i),
        #                 edgecolor='black',
        #                 linewidth=0.5,
        #                 ls='solid',
        #                 alpha=1,
        #                 zorder=2)
        # ax.add_patch(pos_n)

        # plot path of each robots
        path = swarm.des_path_pos[i]
        path = np.array(path)
        plt.plot(path[:, 0], path[:, 1], color=cmap(i), linewidth=1)

    # plot pos option
    # for key, node_n in mapInfo.node_all.items():
    #     if 'node_option_pos' in node_n:
    #         pos_list = node_n['node_option_pos']
    #         for pos in pos_list:
    #             pos_option = matplotlib.patches.Circle(
    #                 (pos[0], pos[1]),
    #                 radius=ws_model['robot_radius'],
    #                 facecolor=cmap(0),
    #                 edgecolor='black',
    #                 linewidth=0.5,
    #                 ls='solid',
    #                 alpha=1,
    #                 zorder=2)
    #             ax.add_patch(pos_option)

    if time:
        ax.text(0, 94, '$t=%.1f s$' % time, fontsize=20, fontweight='bold')
    # ---set axes ---

    # boundary [[x1,x2], [y1, y2]]
    out_size = 0.5
    ax.set_aspect('equal')
    ax.set_xlim(boundary[0][0] - out_size, boundary[0][1] + out_size)
    ax.set_ylim(boundary[1][0] - out_size, boundary[1][1] + out_size)
    ax.set_xlabel(r'$x (m)$')
    ax.set_ylabel(r'$y (m)$')
    ax.grid(True)
    if name:
        pyplot.savefig(name, dpi=200)
        # pyplot.savefig(name,bbox_inches='tight')
    pyplot.cla()
    pyplot.close(figure)
    return figure


def visualize_traj_dynamic_withoutPath(ws_model, swarm, boundary, mapInfo, time=None, name=None):
    matplotlib.use('Agg')
    X = swarm.pos_all
    U = swarm.V_all
    goal = swarm.goal_pos
    resolution = 10

    figure = pyplot.figure(figsize=(7, 10))
    ax = figure.add_subplot(1, 1, 1)
    cmap = get_cmap(len(X))
    # plot obstacles
    for hole in ws_model['circular_obstacles']:
        srec = matplotlib.patches.Rectangle(
            (hole[0] - hole[2], hole[1] - hole[2]),
            2 * hole[2], 2 * hole[2],
            facecolor='dimgrey',
            fill=True,
            alpha=1)
        ax.add_patch(srec)
        circle = patches.Circle(
            (hole[0], hole[1]),  # 圆心坐标
            hole[2],  # 半径
            facecolor='dimgrey',  # 面颜色
            fill=True,  # 填充
            alpha=1)  # 透明度
        ax.add_patch(circle)
    for i in range(0, len(X)):
        # -------plot car
        if swarm.robot_exist[i] is False:
            continue
        robot = matplotlib.patches.Circle(
            (X[i][0], X[i][1]),
            radius=ws_model['robot_radius'],
            facecolor=cmap(i),
            edgecolor='black',
            linewidth=0.5,
            ls='solid',
            alpha=1,
            zorder=2)
        ax.add_patch(robot)
        # ----------plot velocity
        ax.arrow(X[i][0], X[i][1], U[i][0]*0.5, U[i][1]*0.5, head_width=0.15, head_length=0.1, fc=cmap(i), ec=cmap(i))
        # ax.text(X[i][0] - 0.1, X[i][1] - 0.1, r'$%s$' % i, fontsize=15, fontweight='bold', zorder=3)
        ax.plot([goal[i][0]], [goal[i][1]], '*', color=cmap(i), markersize=5, linewidth=3.0)

        # plot des pos of all drones
        pos = swarm.des_pos[i]
        # pos_n = matplotlib.patches.Rectangle(
        #     (pos[0] - 2*ws_model['robot_radius'], pos[1] - 2*ws_model['robot_radius']),
        #     width=4 * ws_model['robot_radius'],
        #     height=4 * ws_model['robot_radius'],
        #     facecolor='white',
        #     edgecolor='black',
        #     linewidth=0.5,
        #     ls='solid',
        #     alpha=1,
        #     zorder=2)
        # pos_n = matplotlib.patches.Circle(
        #                 (pos[0], pos[1]),
        #                 radius=ws_model['robot_radius'],
        #                 facecolor=cmap(i),
        #                 edgecolor='black',
        #                 linewidth=0.5,
        #                 ls='solid',
        #                 alpha=1,
        #                 zorder=2)
        # ax.add_patch(pos_n)

        # plot path of each robots
        path = swarm.des_path_pos[i]
        path = np.array(path)
        # plt.plot(path[:, 0], path[:, 1], color=cmap(i), linewidth=1)

    # plot pos option
    # for key, node_n in mapInfo.node_all.items():
    #     if 'node_option_pos' in node_n:
    #         pos_list = node_n['node_option_pos']
    #         for pos in pos_list:
    #             pos_option = matplotlib.patches.Circle(
    #                 (pos[0], pos[1]),
    #                 radius=ws_model['robot_radius'],
    #                 facecolor=cmap(0),
    #                 edgecolor='black',
    #                 linewidth=0.5,
    #                 ls='solid',
    #                 alpha=1,
    #                 zorder=2)
    #             ax.add_patch(pos_option)

    if time:
        ax.text(0, 94, '$t=%.1f s$' % time, fontsize=20, fontweight='bold')
    # ---set axes ---

    # boundary [[x1,x2], [y1, y2]]
    out_size = 0.5
    ax.set_aspect('equal')
    ax.set_xlim(boundary[0][0] - out_size, boundary[0][1] + out_size)
    ax.set_ylim(boundary[1][0] - out_size, boundary[1][1] + out_size)
    ax.set_xlabel(r'$x (m)$')
    ax.set_ylabel(r'$y (m)$')
    ax.grid(True)
    if name:
        pyplot.savefig(name, dpi=200)
        # pyplot.savefig(name,bbox_inches='tight')
    pyplot.cla()
    pyplot.close(figure)
    return figure

def plot_JPS_traj(ws_model, swarm, boundary, mapInfo):

    X = swarm.pos_all
    U = swarm.V_all
    goal = swarm.goal_pos
    resolution = 10

    figure = pyplot.figure(figsize=(6, 8))
    ax = figure.add_subplot(1, 1, 1)

    cmap = get_cmap(len(X))
    # plot obstacles
    for hole in ws_model['circular_obstacles']:
        srec = matplotlib.patches.Rectangle(
            (hole[0] - hole[2], hole[1] - hole[2]),
            2 * hole[2], 2 * hole[2],
            facecolor='dimgrey',
            fill=True,
            alpha=1)
        ax.add_patch(srec)
        circle = patches.Circle(
            (hole[0], hole[1]),  # 圆心坐标
            hole[2],  # 半径
            facecolor='dimgrey',  # 面颜色
            fill=True,  # 填充
            alpha=1)  # 透明度
        ax.add_patch(circle)

    for key, path in mapInfo.jps_path_all.items():
        p = np.array(path)
        p = np.array(p)
        plt.plot(p[:, 0]/resolution, p[:, 1]/resolution, linewidth=1)
    # ---set axes ---

    # boundary [[x1,x2], [y1, y2]]
    out_size = 0.5
    ax.set_aspect('equal')
    ax.set_xlim(boundary[0][0] - out_size, boundary[0][1] + out_size)
    ax.set_ylim(boundary[1][0] - out_size, boundary[1][1] + out_size)
    ax.set_xlabel(r'$x (m)$')
    ax.set_ylabel(r'$y (m)$')
    ax.grid(True)

    plt.show()



def visualize_traj_dynamic_1(ws_model, swarm, boundary, mapInfo, time=None, name=None):

    X = swarm.pos_all
    U = swarm.V_all
    goal = swarm.goal_pos
    resolution = 10

    figure = pyplot.figure(figsize=(6, 8))
    ax = figure.add_subplot(1, 1, 1)
    cmap = get_cmap(len(X))
    # plot obstacles
    for hole in ws_model['circular_obstacles']:
        srec = matplotlib.patches.Rectangle(
            (hole[0] - hole[2], hole[1] - hole[2]),
            2 * hole[2], 2 * hole[2],
            facecolor='dimgrey',
            fill=True,
            alpha=1)
        ax.add_patch(srec)
        circle = patches.Circle(
            (hole[0], hole[1]),  # 圆心坐标
            hole[2],  # 半径
            facecolor='dimgrey',  # 面颜色
            fill=True,  # 填充
            alpha=1)  # 透明度
        ax.add_patch(circle)
    # for i in range(0, len(X)):
    #     # -------plot car
    #     if swarm.robot_exist[i] is False:
    #         continue
    #     robot = matplotlib.patches.Circle(
    #         (X[i][0], X[i][1]),
    #         radius=ws_model['robot_radius'],
    #         facecolor=cmap(i),
    #         edgecolor='black',
    #         linewidth=0.5,
    #         ls='solid',
    #         alpha=1,
    #         zorder=2)
    #     ax.add_patch(robot)
    #     # ----------plot velocity
    #     ax.arrow(X[i][0], X[i][1], U[i][0]*0.5, U[i][1]*0.5, head_width=0.15, head_length=0.1, fc=cmap(i), ec=cmap(i))
    #     # ax.text(X[i][0] - 0.1, X[i][1] - 0.1, r'$%s$' % i, fontsize=15, fontweight='bold', zorder=3)
    #     ax.plot([goal[i][0]], [goal[i][1]], '*', color=cmap(i), markersize=5, linewidth=3.0)

    # plot des pos of all drones
    #     pos = swarm.des_pos[i]
    #     pos_n = matplotlib.patches.Circle(
    #                     (pos[0], pos[1]),
    #                     radius=ws_model['robot_radius'],
    #                     facecolor=cmap(i),
    #                     edgecolor='black',
    #                     linewidth=0.5,
    #                     ls='solid',
    #                     alpha=1,
    #                     zorder=2)
    #     ax.add_patch(pos_n)

    # plot pos option
    for key, node_n in mapInfo.node_all.items():
        if 'node_option_pos' in node_n:
            pos_list = node_n['node_option_pos']
            for pos in pos_list:
                pos_option = matplotlib.patches.Circle(
                    (pos[0], pos[1]),
                    radius=ws_model['robot_radius'],
                    facecolor=cmap(0),
                    edgecolor='black',
                    linewidth=0.5,
                    ls='solid',
                    alpha=1,
                    zorder=2)
                ax.add_patch(pos_option)

    if time:
        ax.text(2, 66, '$t=%.1f s$' % time, fontsize=20, fontweight='bold')
    # ---set axes ---

    # boundary [[x1,x2], [y1, y2]]
    out_size = 0.5
    ax.set_aspect('equal')
    ax.set_xlim(boundary[0][0] - out_size, boundary[0][1] + out_size)
    ax.set_ylim(boundary[1][0] - out_size, boundary[1][1] + out_size)
    ax.set_xlabel(r'$x (m)$')
    ax.set_ylabel(r'$y (m)$')
    ax.grid(True)
    if name:
        pyplot.savefig(name, dpi=200)
        # pyplot.savefig(name,bbox_inches='tight')
    pyplot.cla()
    pyplot.close(figure)
    return figure

def get_cmap(N):
    '''Returns a function that maps each index in 0, 1, ... N-1 to a distinct RGB color.'''
    color_norm = colors.Normalize(vmin=0, vmax=N - 1)
    scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv')

    def map_index_to_rgb_color(index):
        return scalar_map.to_rgba(index)

    return map_index_to_rgb_color