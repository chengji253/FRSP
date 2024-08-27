from math import ceil, floor, sqrt
import copy
import numpy
from math import cos, sin, tan, atan2, asin
from math import pi as PI


class Velocity_obstacle:

    def __init__(self):
        self.around_no_robots = None

    def distance(self, pose1, pose2):
        """ compute Euclidean distance for 2D """
        return sqrt((pose1[0] - pose2[0]) ** 2 + (pose1[1] - pose2[1]) ** 2) + 0.001

    def VO_update(self, X, V_des, V_current, ws_model, robot_exist):
        """ compute best velocity given the desired velocity, current velocity and workspace model"""
        ROB_RAD = ws_model['robot_radius'] + 0.05
        V_opt = list(V_current)
        internal_dis = 1
        obs_dis = 1
        for i in range(len(X)):
            if robot_exist[i] is False:
                continue
            vA = [V_current[i][0], V_current[i][1]]
            pA = [X[i][0], X[i][1]]
            RVO_BA_all = []
            for j in range(len(X)):
                if robot_exist[i] is False:
                    continue
                if i != j:
                    vB = [V_current[j][0], V_current[j][1]]
                    pB = [X[j][0], X[j][1]]

                    dist_BA = self.distance(pA, pB)
                    if dist_BA <= internal_dis:
                        # use VO
                        transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]

                        theta_BA = atan2(pB[1] - pA[1], pB[0] - pA[0])
                        if 2 * ROB_RAD > dist_BA:
                            dist_BA = 2 * ROB_RAD
                        theta_BAort = asin(2 * ROB_RAD / dist_BA)
                        theta_ort_left = theta_BA + theta_BAort
                        bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                        theta_ort_right = theta_BA - theta_BAort
                        bound_right = [cos(theta_ort_right), sin(theta_ort_right)]

                        RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2 * ROB_RAD]
                        RVO_BA_all.append(RVO_BA)

            if len(RVO_BA_all) <= 3:
                self.around_no_robots[i] = True
            else:
                self.around_no_robots[i] = False

            for hole in ws_model['circular_obstacles']:
                # hole = [x, y, rad]
                vB = [0, 0]
                pB = hole[0:2]
                transl_vB_vA = [pA[0] + vB[0], pA[1] + vB[1]]
                dist_BA = self.distance(pA, pB)

                if dist_BA <= obs_dis:
                    theta_BA = atan2(pB[1] - pA[1], pB[0] - pA[0])
                    # over-approximation of square to circular
                    OVER_APPROX_C2S = 1.0
                    rad = hole[2] * OVER_APPROX_C2S
                    if (rad + ROB_RAD) > dist_BA:
                        dist_BA = rad + ROB_RAD
                    theta_BAort = asin((rad + ROB_RAD) / dist_BA)
                    theta_ort_left = theta_BA + theta_BAort
                    bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                    theta_ort_right = theta_BA - theta_BAort
                    bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
                    RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad + ROB_RAD]
                    RVO_BA_all.append(RVO_BA)
            vA_post = self.intersect(pA, V_des[i], RVO_BA_all)
            V_opt[i] = vA_post[:]
        return V_opt

    def RVO_update(self, X, V_des, V_current, ws_model, robot_exist):
        """ compute best velocity given the desired velocity, current velocity and workspace model"""
        ROB_RAD = ws_model['robot_radius'] + 0.05
        V_opt = list(V_current)
        internal_dis = 1
        obs_dis = 1
        for i in range(len(X)):
            if robot_exist[i] is False:
                continue
            vA = [V_current[i][0], V_current[i][1]]
            pA = [X[i][0], X[i][1]]
            RVO_BA_all = []
            for j in range(len(X)):
                if robot_exist[j] is False:
                    continue
                if i != j:
                    vB = [V_current[j][0], V_current[j][1]]
                    pB = [X[j][0], X[j][1]]

                    dist_BA = self.distance(pA, pB)
                    if dist_BA <= internal_dis:
                        # use RVO
                        transl_vB_vA = [pA[0] + 0.5 * (vB[0] + vA[0]), pA[1] + 0.5 * (vB[1] + vA[1])]

                        theta_BA = atan2(pB[1] - pA[1], pB[0] - pA[0])
                        if 2 * ROB_RAD > dist_BA:
                            dist_BA = 2 * ROB_RAD
                        theta_BAort = asin(2 * ROB_RAD / dist_BA)
                        theta_ort_left = theta_BA + theta_BAort
                        bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                        theta_ort_right = theta_BA - theta_BAort
                        bound_right = [cos(theta_ort_right), sin(theta_ort_right)]

                        RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2 * ROB_RAD]
                        RVO_BA_all.append(RVO_BA)

            if len(RVO_BA_all) <= 3:
                self.around_no_robots[i] = True
            else:
                self.around_no_robots[i] = False

            for hole in ws_model['circular_obstacles']:
                # hole = [x, y, rad]
                vB = [0, 0]
                pB = hole[0:2]
                transl_vB_vA = [pA[0] + vB[0], pA[1] + vB[1]]
                dist_BA = self.distance(pA, pB)

                if dist_BA <= obs_dis:
                    theta_BA = atan2(pB[1] - pA[1], pB[0] - pA[0])
                    # over-approximation of square to circular
                    OVER_APPROX_C2S = 1.0
                    rad = hole[2] * OVER_APPROX_C2S
                    if (rad + ROB_RAD) > dist_BA:
                        dist_BA = rad + ROB_RAD
                    theta_BAort = asin((rad + ROB_RAD) / dist_BA)
                    theta_ort_left = theta_BA + theta_BAort
                    bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                    theta_ort_right = theta_BA - theta_BAort
                    bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
                    RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad + ROB_RAD]
                    RVO_BA_all.append(RVO_BA)
            vA_post = self.intersect(pA, V_des[i], RVO_BA_all)
            V_opt[i] = vA_post[:]
        return V_opt

    def HRVO_update(self, X, V_des, V_current, ws_model, robot_exist):
        """ compute best velocity given the desired velocity, current velocity and workspace model"""
        ROB_RAD = ws_model['robot_radius'] + 0.05
        V_opt = list(V_current)
        internal_dis = 1
        obs_dis = 1
        for i in range(len(X)):
            if robot_exist[i] is False:
                continue
            vA = [V_current[i][0], V_current[i][1]]
            pA = [X[i][0], X[i][1]]
            RVO_BA_all = []
            for j in range(len(X)):
                if robot_exist[i] is False:
                    continue
                if i != j:
                    vB = [V_current[j][0], V_current[j][1]]
                    pB = [X[j][0], X[j][1]]

                    dist_BA = self.distance(pA, pB)
                    if dist_BA <= internal_dis:

                        theta_BA = atan2(pB[1] - pA[1], pB[0] - pA[0])
                        if 2 * ROB_RAD > dist_BA:
                            dist_BA = 2 * ROB_RAD
                        theta_BAort = asin(2 * ROB_RAD / dist_BA)
                        theta_ort_left = theta_BA + theta_BAort
                        bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                        theta_ort_right = theta_BA - theta_BAort
                        bound_right = [cos(theta_ort_right), sin(theta_ort_right)]

                        # use HRVO
                        dist_dif = self.distance([0.5*(vB[0]-vA[0]),0.5*(vB[1]-vA[1])],[0,0])
                        transl_vB_vA = [pA[0]+vB[0]+cos(theta_ort_left)*dist_dif, pA[1]+vB[1]+sin(theta_ort_left)*dist_dif]

                        RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2 * ROB_RAD]
                        RVO_BA_all.append(RVO_BA)

            if len(RVO_BA_all) <= 3:
                self.around_no_robots[i] = True
            else:
                self.around_no_robots[i] = False

            for hole in ws_model['circular_obstacles']:
                # hole = [x, y, rad]
                vB = [0, 0]
                pB = hole[0:2]
                transl_vB_vA = [pA[0] + vB[0], pA[1] + vB[1]]
                dist_BA = self.distance(pA, pB)

                if dist_BA <= obs_dis:
                    theta_BA = atan2(pB[1] - pA[1], pB[0] - pA[0])
                    # over-approximation of square to circular
                    OVER_APPROX_C2S = 1.0
                    rad = hole[2] * OVER_APPROX_C2S
                    if (rad + ROB_RAD) > dist_BA:
                        dist_BA = rad + ROB_RAD
                    theta_BAort = asin((rad + ROB_RAD) / dist_BA)
                    theta_ort_left = theta_BA + theta_BAort
                    bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                    theta_ort_right = theta_BA - theta_BAort
                    bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
                    RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad + ROB_RAD]
                    RVO_BA_all.append(RVO_BA)
            vA_post = self.intersect(pA, V_des[i], RVO_BA_all)
            V_opt[i] = vA_post[:]
        return V_opt

    def intersect(self, pA, vA, RVO_BA_all):
        # print '----------------------------------------'
        # print 'Start intersection test'
        norm_v = self.distance(vA, [0, 0])
        suitable_V = []
        unsuitable_V = []
        for theta in numpy.arange(0, 2 * PI, 0.1):
            for rad in numpy.arange(0.02, norm_v + 0.02, norm_v / 5.0):
                new_v = [rad * cos(theta), rad * sin(theta)]
                suit = True
                for RVO_BA in RVO_BA_all:
                    p_0 = RVO_BA[0]
                    left = RVO_BA[1]
                    right = RVO_BA[2]
                    dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]
                    theta_dif = atan2(dif[1], dif[0])
                    theta_right = atan2(right[1], right[0])
                    theta_left = atan2(left[1], left[0])
                    if self.in_between(theta_right, theta_dif, theta_left):
                        suit = False
                        break
                if suit:
                    suitable_V.append(new_v)
                else:
                    unsuitable_V.append(new_v)
        new_v = vA[:]
        suit = True
        for RVO_BA in RVO_BA_all:
            p_0 = RVO_BA[0]
            left = RVO_BA[1]
            right = RVO_BA[2]
            dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]
            theta_dif = atan2(dif[1], dif[0])
            theta_right = atan2(right[1], right[0])
            theta_left = atan2(left[1], left[0])
            if self.in_between(theta_right, theta_dif, theta_left):
                suit = False
                break
        if suit:
            suitable_V.append(new_v)
        else:
            unsuitable_V.append(new_v)
        # ----------------------
        if suitable_V:
            # print('Suitable found')
            vA_post = min(suitable_V, key=lambda v: self.distance(v, vA))
            new_v = vA_post[:]
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
        else:
            # print('Suitable not found')
            tc_V = dict()
            for unsuit_v in unsuitable_V:
                tc_V[tuple(unsuit_v)] = 0
                tc = []
                for RVO_BA in RVO_BA_all:
                    p_0 = RVO_BA[0]
                    left = RVO_BA[1]
                    right = RVO_BA[2]
                    dist = RVO_BA[3]
                    rad = RVO_BA[4]
                    dif = [unsuit_v[0] + pA[0] - p_0[0], unsuit_v[1] + pA[1] - p_0[1]]
                    theta_dif = atan2(dif[1], dif[0])
                    theta_right = atan2(right[1], right[0])
                    theta_left = atan2(left[1], left[0])
                    if self.in_between(theta_right, theta_dif, theta_left):
                        small_theta = abs(theta_dif - 0.5 * (theta_left + theta_right))
                        if abs(dist * sin(small_theta)) >= rad:
                            rad = abs(dist * sin(small_theta))
                        big_theta = asin(abs(dist * sin(small_theta)) / rad)
                        dist_tg = abs(dist * cos(small_theta)) - abs(rad * cos(big_theta))
                        if dist_tg < 0:
                            dist_tg = 0
                        tc_v = dist_tg / self.distance(dif, [0, 0])
                        tc.append(tc_v)
                tc_V[tuple(unsuit_v)] = min(tc) + 0.001
            WT = 0.2
            vA_post = min(unsuitable_V, key=lambda v: ((WT / tc_V[tuple(v)]) + self.distance(v, vA)))
        return vA_post

    def in_between(self, theta_right, theta_dif, theta_left):
        if abs(theta_right - theta_left) <= PI:
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        else:
            if (theta_left < 0) and (theta_right > 0):
                theta_left += 2 * PI
                if theta_dif < 0:
                    theta_dif += 2 * PI
                if theta_right <= theta_dif <= theta_left:
                    return True
                else:
                    return False
            if (theta_left > 0) and (theta_right < 0):
                theta_right += 2 * PI
                if theta_dif < 0:
                    theta_dif += 2 * PI
                if theta_left <= theta_dif <= theta_right:
                    return True
                else:
                    return False

    def compute_V_des(self, X, goal, V_max):
        V_des = []
        for i in range(len(X)):
            dif_x = [goal[i][k] - X[i][k] for k in range(2)]
            norm = self.distance(dif_x, [0, 0])
            norm_dif_x = [dif_x[k] * V_max[k] / norm for k in range(2)]
            V_des.append(norm_dif_x[:])
            if self.reach(X[i], goal[i], 0.1):
                V_des[i][0] = 0
                V_des[i][1] = 0
        return V_des

    def reach(self, p1, p2, bound=0.2):
        if self.distance(p1, p2) < bound:
            return True
        else:
            return False
