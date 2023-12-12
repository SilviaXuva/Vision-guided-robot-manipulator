from spatialmath import SO3, SE3
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

world_frame = SE3()
ref = SE3.Trans(1, 1, 1)*SE3.Rx(-np.pi)*SE3.Ry(0)*SE3.Rz(np.pi/2)*SE3.Rz(60,'deg')
point_world_frame = SE3(1, 1, 1)*SE3.Rz(-60, 'deg')

world_frame.plot(frame='W', color='green')
ref.plot(frame='Ref', color='blue')
point_world_frame.plot(frame='Tar', color='red')
plt.show()

rpy = [-np.pi,  0, -np.pi/2]
# Option with SE3
res_1 = point_world_frame*SE3.RPY(rpy)

world_frame.plot(frame='W', color='green')
ref.plot(frame='Ref', color='blue')
point_world_frame.plot(frame='Tar', color='red')
res_1.plot(frame='Res_1', color='green')
plt.show()

# Option without SE3
res_2 = Rotation.from_matrix(np.dot(point_world_frame.R, SE3.RPY(rpy).R)).as_euler('xyz')
res_2 = SE3.Trans(point_world_frame.t)*SE3.RPY(res_2)

world_frame.plot(frame='W', color='green')
ref.plot(frame='Ref', color='blue')
point_world_frame.plot(frame='Tar', color='red')
res_2.plot(frame='Res_2', color='green')
plt.show()

# Option 3
res_3 = Rotation.from_matrix(np.dot(point_world_frame.R, Rotation.from_euler('xyz', rpy).as_matrix())).as_euler('xyz')
res_3 = SE3.Trans(point_world_frame.t)*SE3.RPY(res_3)

world_frame.plot(frame='W', color='green')
ref.plot(frame='Ref', color='blue')
point_world_frame.plot(frame='Tar', color='red')
res_3.plot(frame='Res_3', color='green')
plt.show()

# ====================================================================================================================
from scipy.spatial.transform import Rotation
import numpy as np

# Definindo as matrizes de rotação R1 e R2 (rotações em torno do eixo Z)
R1 = Rotation.from_euler('z', 45, degrees=True)
R2 = Rotation.from_euler('z', 100, degrees=True)

# Calculando a matriz de rotação que leva R1 para R2
R_between = R2.inv() * R1

# Obtendo o ângulo de rotação e o eixo de rotação em radianos
angle = R_between.magnitude()
axis = R_between.as_rotvec() / angle  # Normalizando o vetor do eixo de rotação

# Convertendo para radianos, se necessário
angle_rad = np.radians(angle)

# Se o eixo for diferente do eixo Z, ajusta para o menor ângulo
if not np.allclose(axis, [0, 0, 1]):
    angle_rad = 2 * np.pi - angle_rad if angle_rad > np.pi else angle_rad

print(f"Menor ângulo de rotação em Z: {angle_rad} radianos")

# ====================================================================================================================
from spatialmath import SO3

# Definindo as matrizes de rotação R1 e R2 (rotações em torno do eixo Z)
R1 = SO3.Rx(45, 'deg')*SO3.Rz(60,'deg')  # Rotação de 45 graus em torno do eixo X
R2 = SO3.Rx(100, 'deg')  # Rotação de 100 graus em torno do eixo X

# Calculando a matriz de rotação que leva R1 para R2
R_between = R2.inv() * R1

# Calculando o menor ângulo de rotação em radianos
angle = R_between.angvec()
# ====================================================================================================================

from spatialmath import SO3, SE3
import matplotlib.pyplot as plt
import numpy as np

deg = np.pi/180
rad = 180/np.pi

gripper_rotation = SE3.Rx(0)*SE3.Ry(np.pi)*SE3.Rz(np.pi/2)
gripper_rotation.plot()
plt.show()

T1 = SE3.Rz(90,'deg')
final1 = T1*gripper_rotation
T1.plot(frame='T1', color='green')
final1.plot(frame='ee1', color='red')
plt.show()

final1_1 = final1@SE3(gripper_rotation.inv()@final1)
a = T1*SE3.Trans(1,0,0); a.plot(frame='T1', color='green')
b = final1*SE3.Trans(2,0,0); b.plot(frame='ee1', color='red')
c = final1_1*SE3.Trans(3,0,0); c.plot(frame='ee1_1', color='blue')
plt.show()

T2 = SE3.Rz(60,'deg')
final2 = T2*gripper_rotation
T2.plot(frame='T2', color='green')
final2.plot(frame='ee2', color='red')
plt.show()

final2_1 = (T2*gripper_rotation)@SE3(gripper_rotation.inv()@(T2*gripper_rotation))
a = T2*SE3.Trans(1,0,0); a.plot(frame='T2', color='green')
b = final2*SE3.Trans(2,0,0); b.plot(frame='ee2', color='red')
c = final2_1*SE3.Trans(3,0,0); c.plot(frame='ee2_1', color='blue')
plt.show()
