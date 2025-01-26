
from ErrorFuntion import ErrorFunctionInterface
from ErrorFuntion import LaserBasedErrorFunction
from DataDecode import DataDecodeInterface
from DataDecode import DataDecodeOffine

import matplotlib.pyplot as plt
import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.linalg import spsolve
from abc import ABC, abstractmethod


class GraphSlam(ABC):
    def __init__(self):
        self.__errorFunction = None
        self.__b = None
        self.__H = None
    
    @abstractmethod
    def createErrorFunction(self)->ErrorFunctionInterface:
        pass
    
    def formulateBMatrix(self):
        errors = self.__errorFunction.getErrorFunction()
        position = self.__errorFunction.getPosition()
        positionLen = len(position)
        blockWidth = len(position[0])
        self.__b = np.zeros((blockWidth * positionLen, 1))
         
        for element in errors:
            Eij = element['error']
            Aij = element['A']
            Bij = element['B']
            Omega = element['infoMatrix']
            i = element['relative'][0]
            j = element['relative'][1]
            
            bi = Aij.T @ Omega @ Eij
            bj = Bij.T @ Omega @ Eij
            
            self.__b[i*blockWidth: i*blockWidth + blockWidth] += bi
            self.__b[j*blockWidth: j*blockWidth + blockWidth] += bj
            
            
            
    
    def formulateHMatrx(self):
        errors = self.__errorFunction.getErrorFunction()
        position = self.__errorFunction.getPosition()
        positionLen = len(position)
        blockWidth = len(position[0])
        self.__H = np.zeros((blockWidth*positionLen, blockWidth*positionLen))
        
        for element in errors:
            Aij = element['A']
            Bij = element['B']
            Omega = element['infoMatrix']
            i = element['relative'][0]
            j = element['relative'][1]
            
            Hii = Aij.T @ Omega @ Aij
            Hij = Aij.T @ Omega @ Bij
            Hji = Bij.T @ Omega @ Aij
            Hjj = Bij.T @ Omega @ Bij
            
            self.__H[i*blockWidth:i*blockWidth + blockWidth, i*blockWidth:i*blockWidth + blockWidth] += Hii
            self.__H[i*blockWidth:i*blockWidth + blockWidth, j*blockWidth:j*blockWidth + blockWidth] += Hij
            self.__H[j*blockWidth:j*blockWidth + blockWidth, i*blockWidth:i*blockWidth + blockWidth] += Hji
            self.__H[j*blockWidth:j*blockWidth + blockWidth, j*blockWidth:j*blockWidth + blockWidth] += Hjj
            
        self.__H[0:3, 0:3] += np.eye(3)
            
    
    def solveProblem(self):
        self.__errorFunction = self.createErrorFunction()
        position = self.__errorFunction.getPosition()
        
        self.plotPosition(position)
        
        for i in range(10):
            self.formulateBMatrix()
            self.formulateHMatrx() 

            #implement LM algorithm later
            #implement sparse matrix to accelerate the process
            Hsparse = csr_matrix(self.__H)
            deltaX = spsolve(Hsparse, -self.__b)
            #deltaX = - np.linalg.inv(self.__H) @ self.__b #direct inversion is too slow
            position += deltaX.reshape(len(position), len(position[0]))
            self.__errorFunction.updatePosition(position)
            
            self.plotPosition(position)
            
            if np.linalg.norm(deltaX) < 10e-6:
                break
            
            
        print("end of solving")
        plt.show()
        
        
        
    @abstractmethod
    def plotPosition(self, position):
        pass

        
    
class LaserBasedGraphSlam(GraphSlam):
    def __init__(self, dataInterface:DataDecodeInterface):
        self.__dataInterface = dataInterface
        
        
    def createErrorFunction(self):
        #factory method pattern
        #create a concrete error function here
        return LaserBasedErrorFunction(self.__dataInterface)
    
    def plotPosition(self, position):
        x, y, theta = zip(*position)
        plt.rcParams['figure.figsize'] = [15, 15]
        plt.clf()
        plt.plot(x, y, marker='.', linestyle='-', color='b')
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Plot of (x, y) coordinates')
        plt.draw()
        plt.pause(0.1)
        


if __name__ == "__main__":
    offlineData = DataDecodeOffine('data/killian-v.dat', 'data/killian-e.dat')
    laserBasedGrpahSlam = LaserBasedGraphSlam(offlineData)
    laserBasedGrpahSlam.solveProblem()