import csv
import numpy as np
import matplotlib.pyplot as plt
plt.rcParams.update({'font.size': 30})
ScanTimeArray=np.zeros(0)
ImuTimeArray=np.zeros(0)
ImuXArayTmp=np.zeros(0)
ImuYArayTmp=np.zeros(0)
ImuZArayTmp=np.zeros(0)
DISTTmp=np.zeros(0)
DISTTimeTmp=np.zeros(0)
LayerTmp=np.zeros(0)
LayerTimeTmp=np.zeros(0)
LayerFirstTime=np.zeros(0)
with open('/home/rosuser/mrs6xxx_moved.csv', 'r') as csvfile:
    debugreader = csv.reader(csvfile, delimiter=';', quotechar='#')
    for row in debugreader:
        keyword=row[1]
        if('LASESCANTIME' in keyword):
            ScanTimeArray=np.append(ScanTimeArray,row[0])
        elif('ACCX'in keyword):
            ImuTimeArray=np.append(ImuTimeArray,row[0])
            ImuXArayTmp = np.append(ImuXArayTmp, row[2])
        elif('ACCY'in keyword):
            ImuYArayTmp = np.append(ImuYArayTmp, row[2])
        elif('ACCZ'in keyword):
            ImuZArayTmp = np.append(ImuZArayTmp, row[2])
        elif('DIST'in keyword):
            DISTTmp = np.append(DISTTmp, row[2])
            DISTTimeTmp = np.append(DISTTimeTmp, row[0])

#MRS600 TIMING BEGIN
        elif ('LAYER' in keyword):
            LayerTmp = np.append(LayerTmp, row[2]).astype(float)
            LayerTimeTmp = np.append(LayerTimeTmp, row[0]).astype(float)
            if(row[2]=='13.19000000'):
                LayerFirstTime=np.append(LayerFirstTime,row[0]).astype(float)
                print(row[0])
DeltaLayerFirstTime=(LayerFirstTime[1:]-LayerFirstTime[:-1])/1000
DeltaMean=np.mean(DeltaLayerFirstTime)
TimePredictetVal=np.arange(0,DeltaLayerFirstTime.size,1)
TimePredictetVal=TimePredictetVal*DeltaMean
TimePredictionDiff=LayerFirstTime[1:]*1000-LayerFirstTime[0]-TimePredictetVal
LayerFirstTime=(LayerFirstTime-LayerFirstTime[0])/1e6
#plt.plot(LayerFirstTime[:-1],DeltaLayerFirstTime,lael="")
plt.plot(LayerFirstTime[:-1],TimePredictionDiff)
plt.xlabel('Time since scaner start /s')
plt.ylabel('Time betwean first Layers /ms')
plt.title('MRS6xxx timing moved scaner')
plt.legend(loc='upper left')
plt.show()
# MRS600 TIMING END
#ImuMsgCount=np.min([ImuXArayTmp.size,ImuYArayTmp.size,ImuYArayTmp.size])
#ImuDataCombined=np.zeros([ImuMsgCount,4])
#for i in range(0, ImuMsgCount):
#    ImuDataCombined[i,0]=ImuTimeArray[i]
#    ImuDataCombined[i, 1] = ImuXArayTmp[i]
#    ImuDataCombined[i, 2] = ImuYArayTmp[i]
#    ImuDataCombined[i, 3] = ImuZArayTmp[i]
#    if(i>0):
#        DiffArray=ImuDataCombined[i-1,1:]==ImuDataCombined[i,1:]
#        if(DiffArray[0] and DiffArray[1] and DiffArray[2]):
#            print("Identical Imu Data found")
#            print(ImuDataCombined[i-1,1:]==ImuDataCombined[i,1:])
#            print(ImuDataCombined[i-1])
#            print(ImuDataCombined[i])

#DeltaImuTimes=ImuTimeArray[1:].astype(float)-ImuTimeArray[:-1].astype(float)
#DeltaImuTimesMean=np.mean(DeltaImuTimes)
#DeltaImuTimesStd=np.std(DeltaImuTimes)
#DeltaX=DISTTmp[1:].astype(float)-DISTTmp[:-1].astype(float)
#DeltaXT=DISTTimeTmp[1:].astype(float)-DISTTimeTmp[:-1].astype(float)
#AX=DeltaX.astype(float)/(DeltaXT.astype(float)*1e-6)-9.81
#AXT=DISTTimeTmp[1:].astype(float)
#print("Imu Stats: Samples,Mean DeltaT,Sigma Delta T")
#print(DeltaImuTimes.size+1)
#print(DeltaImuTimesMean)
#print(DeltaImuTimesStd)
#DeltaScanTimeArray=ScanTimeArray[1:].astype(float)-ScanTimeArray[:-1].astype(float)
#DeltaScanTimeArrayMean=np.mean(DeltaScanTimeArray)
#DeltaScanTimeArrayStd=np.std(DeltaScanTimeArray)
#print("Laser Stats: Samples,Mean DeltaT,Sigma Delta T")
#print(DeltaScanTimeArray.size+1)
#print(DeltaScanTimeArrayMean)
#print(DeltaScanTimeArrayStd)
#plt.plot(ImuDataCombined[:,0],ImuDataCombined[:,1],label='X acceleration / m/s^2')
#plt.plot(DISTTimeTmp.astype(float),DISTTmp.astype(float),label='X distance /m')
#plt.xlabel('Time since scanner start /µs')
#plt.ylabel('Readings')
#plt.title('IMU vs distance coherence checking')
#plt.legend(loc='upper left')
#plt.show()