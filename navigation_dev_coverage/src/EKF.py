import numpy as np
from numpy.ma.core import masked_inside

class jetbotEKF():
    def __init__(self):
        ss = 3
        ms = ss-3
        self.ss = 3
        self.ms = ss-3
        self.x = np.array([0. for i in range(ss)])
        self.H = np.array([[0. for i in range(ss)] for j in range(ms)])
        self.G = np.array([[0. for i in range(ss)] for j in range(ss)])
        self.Q = np.array([[0. for i in range(ss)] for j in range(ss)])
        for i in range(2):
            self.Q[i,i] = 0.1
        self.Q[2,2] = np.pi/10
        for i in range(3,ss):
            self.Q[i,i] = 0.0
        self.R = np.array([[0. for i in range(ms)] for j in range(ms)])
        for i in range(ms):
            self.R[i,i] = 0.1
        self.sigma = 3.0*self.Q
        for i in range(3,self.ss):
            self.sigma[i,i] += 3*self.R[i-3,i-3]
        
        self.landmarks = []

    def dataAssociation(self, l, id):
        addnew = True
        minSoFar = 1000.0
        minMark = -1
        thresh = (0.8*0.8/(0.1))
        for mark in self.landmarks:
            if mark[1] == id:
                a = np.zeros(self.ms)
                b = np.zeros(self.ms)
                a[mark[0]*2] = l[0]
                a[mark[0]*2+1] = l[1]
                b[mark[0]*2] = self.x[mark[0]*2+3]
                b[mark[0]*2+1] = self.x[mark[0]*2+3+1]
                d = np.matmul(np.matmul((a - b).T, np.linalg.inv(self.R)), (a-b))
                if d < minSoFar:
                    minSoFar = d
                    minMark = mark[0]
        
        print("min distance:", minSoFar, "\n")

        if minMark == -1 or minSoFar > thresh:
            addnew = True
            ind = len(self.landmarks)
            self.add_landmark(l, id)
        else:
            ind = minMark
        
        return addnew, ind


    def add_landmark(self, l, id):

        # self.landmarks.append(id)
        self.landmarks.append([len(self.landmarks), id])

        self.ss+=2
        self.ms+=2
        ss = self.ss
        ms = self.ms

        xo = self.x
        self.x = np.array([0. for i in range(ss)])
        for i in range(self.ss-2):
            self.x[i] = xo[i]
        self.x[self.ss-2] = l[0]
        self.x[self.ss-1] = l[1]

        self.Q = np.array([[0. for i in range(ss)] for j in range(ss)])
        for i in range(3):
            self.Q[i,i] = 1.0
        self.R = np.array([[0. for i in range(ms)] for j in range(ms)])
        for i in range(self.ms):
            self.R[i,i] = 0.1

        sigma_o = self.sigma
        self.sigma = np.array([[0. for i in range(ss)] for j in range(ss)])
        for i in range(ss-2):
            for j in range(ss-2):
                self.sigma[i,j] = sigma_o[i,j]
        self.sigma[ss-2,ss-2] = 3*self.R[0,0]
        self.sigma[ss-1,ss-1] = 3*self.R[0,0]

    
    
    def get_G(self, v, t):

        theta = self.x[2]
        self.G = np.array([[0. for i in range(self.ss)] for j in range(self.ss)])
        for i in range(self.ss):
            self.G[i,i] = 1.0
        self.G[0,2] = -1.0*v*np.sin(theta)*t
        self.G[1,2] = 1.0*v*np.cos(theta)*t

        return self.G
    
    def get_H(self,ind):
        theta = self.x[2]
        ms = self.ms
        self.H = np.array([[0. for i in range(self.ss)] for j in range(self.ms)])
        # print(self.H.shape)
        i = ind
        # for i in range(ms/2):
        j = i*2 + 1
        self.H[j,0] = np.sin(theta)
        self.H[j,1] = -1*np.cos(theta)
        self.H[j,2] = -1*(self.x[j-1+3] - self.x[0])*np.cos(theta) - (self.x[j+3] - self.x[1])*np.sin(theta)
        self.H[j,j-1+3] = -1*np.sin(theta)
        self.H[j, j+3] = np.cos(theta)

        self.H[j-1,0] = -1*np.cos(theta)
        self.H[j-1,1] = -1*np.sin(theta)
        self.H[j-1,2] = -1*(self.x[j-1+3] - self.x[0])*np.sin(theta) + (self.x[j+3] - self.x[1])*np.cos(theta)
        self.H[j-1,j-1+3] = np.cos(theta)
        self.H[j-1, j+3] = np.sin(theta)
        # print(self.H.shape)
        return self.H
    
    
    def g(self,final_pose, vt, t):
        self.x[0] = final_pose[0]
        self.x[1] = final_pose[1]
        self.x[2] = final_pose[2]
    
    def h(self, ind):
        z = np.zeros(self.ms)
        theta = self.x[2]
        ms = self.ms
        # for i in range(ms/2):
        j = ind*2
        xi = self.x[j+3]
        yi = self.x[j+3+1]
        z[j] = (xi - self.x[0])*np.cos(theta) + (yi - self.x[1])*np.sin(theta)
        z[j+1] = -1*(xi - self.x[0])*np.sin(theta) + (yi - self.x[1])*np.cos(theta)
        
        return z
    

    def predict(self, final_pose, vt, t):
        
        G = self.get_G(vt, t)
        self.g(final_pose, vt, t)

        # G = self.get_G(vt, t)
        # print("G matrix during predict:: ",G)
        # print("sigma matrix:: ",self.sigma)
        self.sigma = np.matmul(G, self.sigma)
        self.sigma = np.matmul(self.sigma, G.T) + self.Q
        # print("after prediction:", self.x)
    
    def update(self,z, ind):
        H = self.get_H(ind)

        K = np.matmul(np.matmul(H, self.sigma), H.T)
        # print("Kalman gain1 :: ",K)
        K = np.linalg.inv(K + self.R)
        # print("Kalman gain2 :: ",K)
        K = np.matmul(np.matmul(self.sigma, H.T), K)
        # print("Kalman gain3 :: ",K)

        zp = self.h(ind)
        # print("H matrix:: ",H)
        # print("Kalman gain :: ",K)
        # K1 = K.T
        # K1 = np.flip(K1, axis=1)
        # K = K1.T
        # print("Kalman gain :: ",K)
        # print("z:: ",z)
        # print("zp:: ", zp)
        # print("second term:: ", np.matmul(K, (z - zp)))

        self.x = self.x + np.matmul(K, (z - zp))

        self.sigma = np.matmul((np.identity(self.ss) - np.matmul(K, H)), self.sigma)
    








        

