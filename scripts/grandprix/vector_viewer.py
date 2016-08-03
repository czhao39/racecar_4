#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.cm as cm
import numpy as np
import time

fig = plt.figure()
ax1 = fig.add_subplot(111)

# TODO: optimize using numpy
def animate(i):
    #pullData = open("/home/racecar/racecar-ws/src/racecar_4/scripts/grandprix/vectorData.txt","r").read()
    pullData = open("/home/racecar-4/racecar-ws/src/racecar/racecar_4/scripts/grandprix/vectorData.txt").read()
    dataArray = pullData.split('\n')
    vectors = []
    labels = []
    for eachLine in dataArray:
        if len(eachLine)>1:
            label,u,v = eachLine.split(',')
            vectors.append((u, v))
            labels.append(label)

    vectors = [[int(i) for i in vector] for vector in vectors]
    flatVec = [abs(x) for vector in vectors for x in vector]
    dim = int(max(flatVec))

    ax1.clear()

    ax1.set_xlim([-dim,dim])
    ax1.set_ylim([-dim,dim])
    plt.axhline(0, color='black')
    plt.axvline(0, color='black')
    colors = cm.rainbow(np.linspace(0, 1, len(vectors)-1))
    quivers = []
    for vector, c in zip(vectors[:-1], colors):
        quivers.append(ax1.quiver(0, 0, vector[1], vector[0], angles='xy',scale_units='xy',scale=1, color=c))
    quivers.append(ax1.quiver(0, 0, vectors[-1][1], vectors[-1][0], angles='xy',scale_units='xy',scale=1, color='black'))
    for i, l in enumerate(labels):
        plt.quiverkey(quivers[i], 0.5, 0.1+float(i)/25., 4, l, labelpos='W', fontproperties={'weight': 'bold'})
    plt.draw()
ani = animation.FuncAnimation(fig, animate, interval=1000)

plt.show()
