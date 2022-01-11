import matplotlib.pyplot as plt
x = [1]
y = [1]



fig, (ax1, ax2) = plt.subplots(2)
ax1.scatter(x,y)
ax2.scatter(x,y)

def onclick(event):
    if event.button == 1:
         x.append(event.xdata)
         y.append(event.ydata)
    #clear frame
    ax1.clear()
    ax2.clear()
    ax1.scatter(x,y); #inform matplotlib of the new data
    ax2.scatter(x,y); #inform matplotlib of the new data
    plt.draw() #redraw
    print('redea')


fig.canvas.mpl_connect('button_press_event',onclick)
plt.show()
plt.draw()