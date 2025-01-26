
from abc import ABC, abstractmethod
import numpy as np

class DataDecodeInterface(ABC):
    
    @abstractmethod
    def getPosition(self):
        pass
    
    @abstractmethod
    def getObservation(self):
        pass
    
    
class DataDecodeOffine(DataDecodeInterface):
    
    def __init__(self, fileInputPath1, fileInputPath2):
        self.__fileInputPath1 = fileInputPath1
        self.__fileInputPath2 = fileInputPath2
        self.__position = []
        self.__observation = []
        self.decodeOfflineData()
        
        
    def decodeOfflineData(self):
        with open(self.__fileInputPath1, 'r') as file:
            for line in file:
                elements = line.strip().split(" ")
                self.__position.append((float(elements[2]), float(elements[3]), float(elements[4])))
        
        with open(self.__fileInputPath2, 'r') as file:
            for line in file:
                elements = line.strip().split(" ") 
                oneElement = {'relative':(int(elements[1]), int(elements[2])), 
                              'relativePos':(float(elements[3]),float(elements[4]),float(elements[5])),
                              'infoMatrix':np.array([[float(elements[6]),  float(elements[7]),  float(elements[10])],
                                                     [float(elements[7]),  float(elements[8]),  float(elements[11])],
                                                     [float(elements[10]), float(elements[11]), float(elements[9])]])}
                self.__observation.append(oneElement)
        
                           
    def getPosition(self):
        return self.__position
    
    def getObservation(self):
        return self.__observation
                
                
# offlineData = DataDecodeOffine('data/killian-v.dat', 'data/killian-e.dat')
# offlineData.decodeOfflineData()
