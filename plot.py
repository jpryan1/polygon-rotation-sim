import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# font = {'family' : 'normal',
#         'weight' : 'bold',
#         'size'   : 22}
import sys

# matplotlib.rc('font', **font)



times = {}

directory = "trials/"
trial = sys.argv[1]

for i in range(1,501):
    
    filename = directory+trial+str(i)+".txt"
    filelines = open(filename, "r").readlines()
    for line in filelines:
        if("Simulating" in line):
            continue
        time = line.split(",")[0]
        mean = float(line.split(",")[1])
        if(time in times):
            times[time].append(mean)
        else:
            times[time] = [mean]


x = []
y = []
stddevs = []
medians = []
for i in range(48):
    time = str(36 + i*2.5)
    if(time[-2:] == ".0"):
        time = time[:-2]
    x.append(float(time))
    if(time in times):
        cur_mean = sum(times[time])/len(times[time])
        cur_stddev = 0
        for j in range(len(times[time])):
            cur_stddev += (times[time][j] - cur_mean)**2
        cur_stddev /= len(times[time])
        stddevs.append(np.sqrt(cur_stddev))
        y.append(cur_mean)
        medians.append(np.median(times[time]))
    else:
        print("no ",time,"found")
        print
        y.append(y[-1])
        stddevs.append(stddevs[-1])


below = []
above = []
for i in range(len(stddevs)):
    below.append(y[i] - stddevs[i])
    above.append(y[i] + stddevs[i])



plt.plot(x,y, '-o', label="Mean")
plt.plot(x,medians, '-o', label="Median")
plt.plot(x,[0 for i in range(len(x))], '--w')
plt.fill_between(x, below, above, color = '#539caf', alpha = 0.4)

plt.xlabel("Time")
plt.ylabel("Mean angular velocity (past 1k iters)")
plt.title("Mean angular velocity vs time - "+trial)
# plt.show()
plt.legend()
plt.savefig(trial+".png")
# plt.show()



