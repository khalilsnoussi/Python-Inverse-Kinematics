import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class ThreeLinkArm:
    def __init__(self, joint_angles=[0, 0, 0], L=[2, 2, 2]):
        self.L = np.array(L)
        self.update_joints(joint_angles)

    def update_joints(self, joint_angles):
        self.q = joint_angles
        self.forward_kinematics()

    def forward_kinematics(self):
        L, q = self.L, self.q
        self.x = np.cumsum([0, 
                            L[0]*np.cos(q[0]), 
                            L[1]*np.cos(np.sum(q[:2])),
                            L[2]*np.cos(np.sum(q))])
        self.y = np.cumsum([0, 
                            L[0]*np.sin(q[0]), 
                            L[1]*np.sin(np.sum(q[:2])),
                            L[2]*np.sin(np.sum(q))])
        self.z = np.cumsum([0, 
                            L[0]*np.sin(q[0]), 
                            L[1]*np.sin(np.sum(q[:2])),
                            L[2]*np.sin(np.sum(q))])

    def inverse_kinematics(self, x, y, z, iterations=1000, learning_rate=0.01):
        for i in range(iterations):
            e = np.array([x, y, z]) - np.array([self.x[-1], self.y[-1], self.z[-1]])
            J = self.jacobian()
            dq = learning_rate * np.linalg.pinv(J).dot(e)
            self.q += dq
            self.forward_kinematics()

    def jacobian(self):
        L, q = self.L, self.q
        J = [[-L[0]*np.sin(q[0])-L[1]*np.sin(np.sum(q[:2]))-L[2]*np.sin(np.sum(q)),
              -L[1]*np.sin(np.sum(q[:2]))-L[2]*np.sin(np.sum(q)),
              -L[2]*np.sin(np.sum(q))],
             [L[0]*np.cos(q[0])+L[1]*np.cos(np.sum(q[:2]))+L[2]*np.cos(np.sum(q)),
              L[1]*np.cos(np.sum(q[:2]))+L[2]*np.cos(np.sum(q)),
              L[2]*np.cos(np.sum(q))],
             [L[0]*np.sin(q[0])+L[1]*np.sin(np.sum(q[:2]))+L[2]*np.sin(np.sum(q)),
              L[1]*np.sin(np.sum(q[:2]))+L[2]*np.sin(np.sum(q)),
              L[2]*np.sin(np.sum(q))]]
        return np.array(J)

    def plot(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self.x, self.y, self.z)
        ax.scatter(self.x, self.y, self.z, c='r')
        for i in range(len(self.x)):
            ax.text(self.x[i], self.y[i], self.z[i], f'({self.x[i]:.2f}, {self.y[i]:.2f}, {self.z[i]:.2f})')
        ax.set_xlim([-sum(self.L)-1, sum(self.L)+1])
        ax.set_ylim([-sum(self.L)-1, sum(self.L)+1])
        ax.set_zlim([-sum(self.L)-1, sum(self.L)+1])
        plt.show()

# Test
arm = ThreeLinkArm([0, 0, 0])
#arm.plot()
arm.inverse_kinematics(5, 3, 2)
arm.plot()