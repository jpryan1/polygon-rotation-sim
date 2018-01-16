import matplotlib.pyplot as plt
import numpy as np

f = open("data.txt","r").readlines()


data = []
err = []
for i in range(0,len(f),2):
	data.append(float(f[i]))
	err.append(float(f[i+1]))


x = [x/10.0 for x in range(75,86,2)]


plt.errorbar(x,data,yerr=err)
two = [2]*len(x)
plt.plot(x,two, '--')
plt.xlabel("Radius")
plt.ylabel("Distance")
plt.title("Average Consecutive Collision Vertex Distance")

plt.show()





