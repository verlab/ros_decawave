import serial
import time

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

plt.ion()
fig, ax = plt.subplots()
x, y = [],[]
sc = ax.scatter(x, y)
plt.xlim(-1, 15)
plt.ylim(-1, 15)

plt.draw()
plt.show(block=False)

ser = serial.Serial("/dev/ttyACM1", 115200, timeout=1)
# for i in xrange(3):
# 	ser.write('\n')
# 	time.sleep(0.1)

cc = str(ser.readline())
print("start:", cc)

# ser.write('lec')
# ser.write('\n')
# time.sleep(0.1)

# POS,0,1291,1.11,0.67,0.55,0,x02

for i in range(1000):
	while(ser.inWaiting()==0):
		pass

	cc = str(ser.readline())

	cc_s = cc.split(',')
	if len(cc_s) != 8:
		continue

	if cc_s[0] != "POS":
		continue

	#print(cc_s)
	print("{}) id:{} ({}, {})".format(i, cc_s[2], cc_s[3], cc_s[4]))
	

	x.append(cc_s[3])
	y.append(cc_s[4])
	sc.set_offsets(np.c_[x,y])
	fig.canvas.draw_idle()
	plt.pause(0.001)
	plt.plot()
	#fig.canvas.draw()
	#
	
	#input("Press [enter] to continue.")
	#time.sleep(1)

ser.close()

plt.waitforbuttonpress()
