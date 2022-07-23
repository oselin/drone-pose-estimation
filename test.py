from matplotlib import pyplot as plt
import time
 
plt.ion()
fig = plt.figure()
axis = fig.add_subplot(111)
 
for i in range(30000):
    axis.plot(i,i,'o')
    plt.draw()
    if i > 5:
        time.sleep(2)   
plt.close()