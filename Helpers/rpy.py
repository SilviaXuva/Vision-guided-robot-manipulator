from spatialmath import SE3
import matplotlib.pyplot as plt
import numpy as np
unit = 'rad'

def encontrar_matriz_de_rotação(R_desejada, R1):
    return np.dot(R_desejada, np.linalg.inv(R1))

R_desejada = SE3.RPY([np.pi,0,0], unit=unit, order='zyx')

row, pitch, yaw = 3.141592654, 2.070680365e-13, -1.570796327
R1 = SE3.RPY(*[row, pitch, yaw], unit = unit, order='zyx')

R_resultado = SE3(np.dot(R1, encontrar_matriz_de_rotação(R_desejada, R1)))

R_desejada.plot(frame='ref', color='blue')
R1.plot(frame='C_W', color='red')
R_resultado.plot(frame='C', color='green')
plt.show()