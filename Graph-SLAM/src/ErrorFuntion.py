
from abc import ABC, abstractmethod
import numpy as np
from DataDecode import DataDecodeInterface
from DataDecode import DataDecodeOffine
import math

class ErrorFunctionInterface(ABC):
    
    @abstractmethod
    def getErrorFunction(self):
        #error function is composed of errors, Jacobian, information matrix
        pass
    
    @abstractmethod
    def getErrors(self):
        #get erros only 
        pass
    
    @abstractmethod
    def getJacobian(self):
        #get Jacobian only
        pass
    
    @abstractmethod
    def getInfoMatrix(self):
        #get infomation matrix only
        pass
    
    @abstractmethod
    def updatePosition(self, position):
        pass
    
    @abstractmethod
    def getPosition(self):
        pass
    


class LaserBasedErrorFunction(ErrorFunctionInterface):
    
    def __init__(self, data: DataDecodeInterface):
        self.__dataInterface  = data
        self.__position = self.__dataInterface.getPosition()
        self.__observation = self.__dataInterface.getObservation()
        self.__errorFunction = []
        self.__setErrorFunction()
        
        
    #set errors, Jacobian and information matrix
    def __setErrorFunction(self):
        for element in self.__observation:
            pairs = element['relative']

            Ti = self.__vector3dToTranformMatrix(self.__position[pairs[0]])
            Tj = self.__vector3dToTranformMatrix(self.__position[pairs[1]])
            Zij = self.__vector3dToTranformMatrix(element['relativePos'])
            
            #compute error
            #error = Zij.inv * (Ti.inv * Tj)
            Eij = self.__TransformMatrixToVector3d(np.linalg.inv(Zij) @ (np.linalg.inv(Ti) @ Tj))
            
            #compute Jacobian
            #Aij and Bij
            Ri = Ti[0:2, 0:2]
            Rz = Zij[0:2, 0:2]
            ti = Ti[0:3, 2]
            tj = Tj[0:3, 2]
            cosThetaI = math.cos(self.__position[pairs[0]][2])
            sinThetaI = math.sin(self.__position[pairs[0]][2])
            dRiTdThetaI = np.array([[-sinThetaI, cosThetaI],
                                   [-cosThetaI, -sinThetaI]])
            Aij = np.zeros((3,3))
            Bij = np.zeros((3,3))
            
            Aij[0:2, 0:2] = -Rz.T @ Ri.T 
            Aij[0:2, 2] = Rz.T @ dRiTdThetaI @ (tj[0:2] - ti[0:2])
            Aij[2,0:3] = [0., 0., -1.]
            
            Bij[0:2, 0:2] = Rz.T @ Ri.T 
            Bij[0:2, 2] = [0., 0.]
            Bij[2, 0:3] = [0., 0., 1.] 
            
            #store data structure
            oneErrorWithJacob = {'relative':pairs,
                                 'error':Eij,
                                 'A': Aij,
                                 'B': Bij,
                                 'infoMatrix': element['infoMatrix']}
            
            self.__errorFunction.append(oneErrorWithJacob)
            
            
    
    def __vector3dToTranformMatrix(self, vector):
        T = np.zeros((3,3))
        T[0,2] = vector[0]
        T[1,2] = vector[1]
        
        cosTheta = math.cos(vector[2])
        sinTheta = math.sin(vector[2])
        T[0,0] = cosTheta
        T[0,1] = -sinTheta
        T[1,0] = sinTheta
        T[1,1] = cosTheta
        T[2,0] = 0.
        T[2,1] = 0.
        T[2,2] = 1.
        
        return T
    
    def __TransformMatrixToVector3d(self, T):
        vector = np.zeros((3,1))
        vector[0] = T[0,2]
        vector[1] = T[1,2]
        vector[2] = math.atan2(T[2,1], T[1,1])
        
        return vector
        
            
    
    #after one iteration of the Guassian-Newton algorithm, need to update x = x + △x
    def updatePosition(self, position:list):
        self.__position = position
        self.__setErrorFunction()
        
    def getPosition(self):
        return self.__position
    
    def getErrorFunction(self):
        return self.__errorFunction
    
    #these three interfaces are not used in this concrete class
    def getErrors(self):
        pass
    
    def getJacobian(self):
        pass
    
    def getInfoMatrix(self):
        pass
    

    
    

#test
if __name__ == "__main__":
    offlineData = DataDecodeOffine('data/killian-v.dat', 'data/killian-e.dat')
    laserErrorFun = LaserBasedErrorFunction(offlineData)