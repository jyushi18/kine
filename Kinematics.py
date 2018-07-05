import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import pandas as pd


##################################################################################################
#リンク計算用関数
##################################################################################################
l1 = 100
l2 = 98
l3x = 105
l3y = 15
l4x = 113.7
l4z = -12.31
l5 = 50

def trans(x,y,z):
    ans = np.array([[1,0,0,x],
                    [0,1,0,y],
                    [0,0,1,z],
                    [0,0,0,1]])
    return ans


def rot_x(rad):
    ans = np.array([[1,       0,        0,0],
                    [0,np.cos(rad),-np.sin(rad),0],
                    [0,np.sin(rad), np.cos(rad),0],
                    [0,       0,        0,1]])
    return ans

def rot_y(rad):
    ans = np.array([[ np.cos(rad),0,np.sin(rad),0],
                    [        0,1,       0,0],
                    [-np.sin(rad),0,np.cos(rad),0],
                    [        0,0,       0,1]])
    return ans

def rot_z(rad):
    ans = np.array([[np.cos(rad),-np.sin(rad),0,0],
                    [np.sin(rad), np.cos(rad),0,0],
                    [       0,        0,1,0],
                    [       0,        0,0,1]])
    return ans


def Kinematics(rad):
    T_G0 = trans(0,0,l1) @ rot_x(-np.pi/2)
    T_01 = rot_z(rad[0]) @ trans(0,0,l2) @ rot_x(np.pi/2)
    T_12 = rot_z(rad[1]) @ trans(l3x,l3y,0) @ rot_y(np.pi/2)
    T_23 = rot_z(rad[2]) @ rot_y(-np.pi/2)
    T_34 = rot_z(rad[3]) @ trans(l4x,0,l4z) @ rot_y(np.pi/2)
    T_45 = rot_z(rad[4]) @ trans(0,l5,0)

    T_G1 = T_G0 @ T_01
    T_G2 = T_G1 @ T_12
    T_G3 = T_G2 @ T_23
    T_G4 = T_G3 @ T_34

    T_G5 = T_G4 @ T_45

    T = np.array([T_G0,T_G1,T_G2,T_G3,T_G4,T_G5])

    theta = np.zeros([6])
    psi = np.zeros([6])
    phi = np.zeros([6])

    for i in range(6):
        theta[i] = np.arctan2(-T[i,2,0],np.sqrt(np.square(T[i,0,0])+np.square(T[i,1,0])))
        phi[i] = np.arctan2(T[i,1,0],T[i,0,0])
        if theta[i] == 0:
            psi[i] = -T[i,2,0]*np.arctan2(T[i,0,1],T[i,1,1])-T[i,2,0]*phi[i]
        else:
            psi[i] = np.arctan2(T[i,2,1],T[i,2,2])




    return np.array([[T_G0[0,3],T_G0[1,3],T_G0[2,3],theta[0],psi[0],phi[0]],
                    [T_G1[0,3],T_G1[1,3],T_G1[2,3],theta[1],psi[1],phi[1]],
                    [T_G2[0,3],T_G2[1,3],T_G2[2,3],theta[2],psi[2],phi[2]],
                    [T_G3[0,3],T_G3[1,3],T_G3[2,3],theta[3],psi[3],phi[3]],
                    [T_G4[0,3],T_G4[1,3],T_G4[2,3],theta[4],psi[4],phi[4]],
                    [T_G5[0,3],T_G5[1,3],T_G5[2,3],theta[5],psi[5],phi[5]]])




##################################################################################################
#データ読み込み & グラフ準備
##################################################################################################
data = np.loadtxt("JointAngle02.csv",delimiter=",")

fig = plt.figure()
ax = Axes3D(fig)

Point = np.zeros([data.shape[0],3])

##################################################################################################
#初期位置計算
##################################################################################################
theta = np.array([30,0,-90,-60,0])
rad = theta*np.pi/180
Sp = Kinematics(rad)

xyzH = rot_y(-70*np.pi/180)

##################################################################################################
#各関節角でのリンク位置を表示
##################################################################################################
def LinkPlot(j):
    plt.cla() #グラフを消去
    global Point
    global xyzH
    global Sp

    #描画座標ベクトルを表示
    ax.quiver(Sp[5,0],Sp[5,1],Sp[5,2],xyzH[0,0],xyzH[1,0],xyzH[2,0],color='r',length=30, normalize=False)
    ax.quiver(Sp[5,0],Sp[5,1],Sp[5,2],xyzH[0,1],xyzH[1,1],xyzH[2,1],color='g',length=30, normalize=False)
    ax.quiver(Sp[5,0],Sp[5,1],Sp[5,2],xyzH[0,2],xyzH[1,2],xyzH[2,2],color='b',length=30, normalize=False)



    theta = data[j*2]
    #rad = theta*np.pi/180
    S1 = Kinematics(theta) #現在の角度の順運動計算

    xyz = np.zeros([6,4,4])

    #現在の各リンク座標ベクトルを計算
    for i in range(6):
        xyz[i] = rot_z(S1[i,5]) @ rot_y(S1[i,3]) @ rot_x(S1[i,4])

    #現在のリンクを表示
    ax.plot(S1[:,0],S1[:,1],S1[:,2],"-o",ms=3)
    #現在の各リンク座標ベクトルを表示
    for i in range(6):
        ax.quiver(S1[i,0],S1[i,1],S1[i,2],xyz[i,0,0],xyz[i,1,0],xyz[i,2,0],color='r',length=30, normalize=False)
        ax.quiver(S1[i,0],S1[i,1],S1[i,2],xyz[i,0,1],xyz[i,1,1],xyz[i,2,1],color='g',length=30, normalize=False)
        ax.quiver(S1[i,0],S1[i,1],S1[i,2],xyz[i,0,2],xyz[i,1,2],xyz[i,2,2],color='b',length=30, normalize=False)

    Point[j,0] = S1[5,0]
    Point[j,1] = S1[5,1]
    Point[j,2] = S1[5,2]

    #ペン先の通った位置を描画
    ax.plot(Point[:,0],Point[:,1],Point[:,2],"o",ms=1)

    # 軸ラベル
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_xlim([0,200])
    ax.set_ylim([0,200])
    ax.set_zlim([0,200])




ani = animation.FuncAnimation(fig, LinkPlot, interval=1, frames=data.shape[0])
#ani.save("output.gif", writer='imagemagick')
#LinkPlot()
plt.show()
