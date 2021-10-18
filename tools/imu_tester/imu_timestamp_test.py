'''

'''
import matplotlib
import matplotlib.pyplot as plt
import numpy as np


class ImuTimeStampTester:
  def __init__(self, logName):
        self.logName = logName
 
  def run(self):
  # try to parse rostopic-echo file and search for timestamps
    with open(self.logName, "r") as f:
      data = f.readlines()

    state = 0
    secVal = 0.000;
    nanoVal = 0.0

    timestamps = np.zeros(1)
    firstTime = True
    for line in data:
        words = line.split()
        for word in words:
            if (state == 1):
                secVal = float(word)
                state = 2
            if (state == 3):
                nanoVal = float(word)
                timeStampVal = secVal + 1E-9 * nanoVal
                print("Timestamp: {timestamp}".format(timestamp=timeStampVal))
                if (firstTime):
                    firstTime = False
                    timestamps[0] = timeStampVal
                else:
                    timestamps = np.append(timestamps, timeStampVal)
                state = 0

            if (word == 'secs:'):
             state = 1

            if (word == 'nsecs:'):
             state = 3
        print(words)

    # now we can analyse and plot the result

    deltaT = timestamps[1:] - timestamps[:-1]
    X = np.arange(0, timestamps.size, 1)

    ax = plt.gca()
    plt.ylabel('Zeitdifferenz t(x)-t(x-1)')
    plt.xlabel('relativer Rospaketindex x')
    plt.title('Zeitverhalten der IMU Timestamps nach langen Differenzen')
    ax.grid(which='major', axis='both', linestyle='--')

    for i in range(0, deltaT.size - 1):
      if deltaT[i] > np.mean(deltaT) + 2 * np.std(deltaT):
           print('ERROR INDEX:' + str(i) + ' Timestamp[INDEX-1]:' + str(timestamps[i]) + ' Timestamp[INDEX]:' + str(
                  timestamps[i + 1]))
    plt.plot(deltaT)
    plt.show()
    plt.savefig('imutimestampsmu+2sigma.png', dpi=300)


if __name__ == "__main__":
    filename = "/tmp/imu.txt"  # create such a file by running rostopic echo /imu >/tmp/imu.txt for about 10 secs.

    imuTest = ImuTimeStampTester(filename)
    imuTest.run()
