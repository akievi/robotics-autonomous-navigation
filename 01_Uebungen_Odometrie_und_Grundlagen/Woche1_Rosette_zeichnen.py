import numpy as np
import matplotlib.pyplot as plt

def rosetteMalen():
    n = 0
    xWerte = []
    yWerte = []
    
    while n < 5:
        n = int(input("Geben Sie eine Zahl größer oder gleich 5 ein: "))
    
    winkelschritt = (2 * np.pi) / n
    
    for i in range(n):
        winkel = i * winkelschritt
        x = np.cos(winkel)
        y = np.sin(winkel)
        xWerte.append(x)
        yWerte.append(y)
        
    for i in range(len(xWerte)):
        for j in range(i+1, len(xWerte)):
            plt.plot([xWerte[i], xWerte[j]], [yWerte[i], yWerte[j]])

    plt.show()
    
rosetteMalen()