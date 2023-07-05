import numpy as np
from Data.targets import target3 as target
from spatialmath import SE3
import roboticstoolbox as rtb
from Toolbox.LBR_iiwa_DH import LBR_iiwa

robot = LBR_iiwa()
robot.q = robot.q0

T0 = robot.fkine(robot.q)
T1 = SE3.Trans(target[0][:3])*SE3.Rx(target[0][3])*SE3.Ry(target[0][4])*SE3.Rz(target[0][5])

q0 = robot.q
print('Expected q1: ', robot.ikine_LMS(T1))

# ik_gn_1 = robot.ik_gn(T1)
# ik_lm_chan_1 = robot.ik_lm_chan(T1)
# ik_lm_sugihara_1 = robot.ik_lm_sugihara(T1)
# ik_lm_wampler_1 = robot.ik_lm_wampler(T1)
# ik_nr_1 = robot.ik_nr(T1)
# try: ikine_6s_1 = robot.ikine_6s(T1)
# except Exception as e: ikine_6s_1 = e
# try: ikine_global_1 = robot.ikine_global(T1)
# except Exception as e: ikine_global_1 = e
# ikine_LM_1 = robot.ikine_LM(T1)
# ikine_min_1 = robot.ikine_min(T1)
# try: ikine_mmc_1 = robot.ikine_mmc(T1)
# except Exception as e: ikine_mmc_1 = e
# print(ik_gn_1)
# print(ik_lm_chan_1)
# print(ik_lm_sugihara_1)
# print(ik_lm_wampler_1)
# print(ik_nr_1)
# print(ikine_6s_1)
# print(ikine_global_1)
# print(ikine_LM_1)
# print(ikine_min_1)
# print(ikine_mmc_1)

# ik_gn_2 = robot.ik_gn(T1, q0)
# ik_lm_chan_2 = robot.ik_lm_chan(T1, q0)
# ik_lm_sugihara_2 = robot.ik_lm_sugihara(T1, q0)
# ik_lm_wampler_2 = robot.ik_lm_wampler(T1, q0)
# ik_nr_2 = robot.ik_nr(T1, q0)
# try: ikine_6s_2 = robot.ikine_6s(T1, q0)
# except Exception as e: ikine_6s_2 = e
# try: ikine_global_2 = robot.ikine_global(T1, q0)
# except Exception as e: ikine_global_2 = e
# ikine_LM_2 = robot.ikine_LM(T1, q0)
# ikine_min_2 = robot.ikine_min(T1, q0)
# try: ikine_mmc_2 = robot.ikine_mmc(T1, q0)
# except Exception as e: ikine_mmc_2 = e
# print(ik_gn_2)
# print(ik_lm_chan_2)
# print(ik_lm_sugihara_2)
# print(ik_lm_wampler_2)
# print(ik_nr_2)
# print(ikine_6s_2)
# print(ikine_global_2)
# print(ikine_LM_2)
# print(ikine_min_2)
# print(ikine_mmc_2)

ikine_min_3 = robot.ikine_min(T1, q0=q0, qlim=True, stiffness=5e-1, method='Powell')
print(ikine_min_3)