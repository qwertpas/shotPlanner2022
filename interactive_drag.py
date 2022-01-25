from numpy import sqrt, cos, tan, sin, radians, degrees, pi, arctan2
import matplotlib.patches as mpatches
from scipy.optimize import fsolve
from scipy.integrate import quad
import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import solve_ivp



g = 9.81
rim_width = 1.2192 #4ft
rim_height = 2.64
cargo_radius = 0.2413/2
# drag_coeff = 0.23 #https://www.chiefdelphi.com/t/galactech-4926-build-blog-2022/398705/5
drag_coeff = 0.50 #https://www.chiefdelphi.com/t/galactech-4926-build-blog-2022/398705/5
cargo_mass = 0.27
air_density = 1.225

cargo_area = pi * cargo_radius**2

def get_speed_func(startpt, endpt):
    x0, y0 = startpt
    x1, y1 = endpt
    return lambda a: sqrt(0.5*g / (y0-y1 + (x1-x0)*tan(a))) * (x1-x0)/cos(a) # not squared

def get_speed_func_squared(startpt, endpt):
    x0, y0 = startpt
    x1, y1 = endpt
    return lambda a: (0.5*g / (y0-y1 + (x1-x0)*tan(a))) * ((x1-x0)/cos(a))**2 #squared in case domain error

def get_ang_speed_space(xpos, ypos, doShow=False):

    f_far_squared = get_speed_func_squared((xpos, ypos), (rim_width/2, rim_height))
    f_near_squared = get_speed_func_squared((xpos, ypos), (-rim_width/2, rim_height))
    f_squared_diff = lambda a: f_far_squared(a) - f_near_squared(a)
    intersection = fsolve(f_squared_diff, radians(85))[0]

    ang_lower_bound = max(intersection, radians(5))
    ang_upper_bound = radians(85)

    f_far = lambda a: sqrt(f_far_squared(a))
    f_near = lambda a: sqrt(f_near_squared(a))
    f_diff = lambda a: f_far(a) - f_near(a)
    area, error = quad(f_diff, ang_lower_bound, ang_upper_bound)

    angles = np.linspace(degrees(ang_lower_bound), degrees(ang_upper_bound), num=50)
    lower_bound_pts = np.vectorize(f_near)(radians(angles))
    upper_bound_pts = np.vectorize(f_far)(radians(angles))

    if doShow:
        print(f'intersection at angle = {degrees(intersection)} degrees')
        print(f'integrating from {ang_lower_bound} to {ang_upper_bound} radians')
        print(f'{area} area')
        plt.figure()
        plt.fill_between(angles, lower_bound_pts, upper_bound_pts)
        plt.xlabel("angle (degrees)")
        plt.ylabel("speed (m/s)")
        plt.xlim([5, 85])
        plt.ylim([0, 15])

    return area, angles, lower_bound_pts, upper_bound_pts

def flight_model(t, s):
    x, vx, y, vy = s
    dx = vx
    dy = vy

    v_squared = vx**2 + vy**2
    v = sqrt(v_squared)

    sin_component = vy/v
    cos_component = vx/v

    Fd = 0.5 * air_density * cargo_area * drag_coeff * v_squared

    Fx = -Fd*cos_component
    Fy = -Fd*sin_component - cargo_mass*g

    dvx = Fx / cargo_mass
    dvy = Fy / cargo_mass
    return [dx, dvx, dy, dvy]

def hit_ground(t, s):
    x, vx, y, vy = s
    return y
hit_ground.terminal = True

def hit_rim(t, s):
    x, vx, y, vy = s
    dist_to_rim = min(x - -rim_width/2, -(y - rim_height)) #positive if cargo is down and to the right of closest rim
    return dist_to_rim + cargo_radius
hit_rim.terminal = True

def passed_rim(t, s):
    x, vx, y, vy = s
    return x - rim_width/2
passed_rim.terminal = True

def try_shot(s0):
    t_span = (0, 5.0)
    solution = solve_ivp(flight_model, t_span, s0, events=[hit_ground, hit_rim, passed_rim], max_step=0.05)

    result = 0 #default is success
    if(solution.y[0][-1] < -rim_width/2):
        result = -1 #undershot
    elif(solution.y[0][-1] > rim_width/2 - cargo_radius):
        result = 1 #overshot

    return result, solution.y[0, :], solution.y[2, :]


x_range = np.arange(-6, -1, 0.1)
y_range = np.arange(0.2, 1.25, 0.1)

area_grid = np.zeros((x_range.size, y_range.size))

for xi in range(x_range.size):
    for yi in range(y_range.size):
        area, angles, lower_bound_pts, upper_bound_pts = get_ang_speed_space(x_range[xi], y_range[yi], doShow=False)
        area_grid[xi][yi] = area * arctan2(rim_width, abs(x_range[xi]))
    # print(x_range[xi])

X, Y = np.meshgrid(x_range, y_range)
mesh_color = area_grid.T

fig, (ax1, ax2) = plt.subplots(2, figsize=(5, 8))
fig.suptitle('A projectile motion simulator for FRC 2022. \n The color gradient shows the size of the allowable error (pitch*yaw*speed) in making the shot', wrap=True)

shoot_state = [-2.8, 2.5, 0.4, 8]

result, traj_x, traj_y = try_shot(shoot_state)
traj = [traj_x, traj_y]


def repaint_ax1():
    ax1.clear()
    ax1.scatter(X, Y, c=mesh_color, marker='s')
    ax1.set_xlim([-6.2, 1])
    ax1.set_ylim([-0.1, 4])
    ax1.set(xlabel='x position (meters)', ylabel='y position (meters)')

    left, bottom, width, height = (-rim_width/2, 0, rim_width, rim_height)
    ax1.add_patch(mpatches.Rectangle((left,bottom), width, height, fill=False, color="gray", linewidth=2))
    ax1.add_patch(mpatches.Rectangle((-(2.72/2-0.34), 0), 2.72-2*0.34, 0.57, fill=False, color="gray", linewidth=2))
    ax1.add_patch(mpatches.Rectangle((-6, -0.5), 7, 0.5, fill=True, color="gray", linewidth=2))
    ax1.set_aspect('equal', adjustable='box')
    ax1.set_title("Drag dot to change shoot position")


    result, traj[0], traj[1] = try_shot(shoot_state)
    color_str = 'red'
    if result == 0:
        color_str = 'green'
    ax1.plot(traj[0], traj[1], color_str)
    ax1.scatter(shoot_state[0], shoot_state[2], c='black')


def repaint_ax2():
    ax2.clear()
    area, angles, lower_bound_pts, upper_bound_pts = get_ang_speed_space(shoot_state[0], shoot_state[2], doShow=False)
    ax2.fill_between(angles, lower_bound_pts, upper_bound_pts, color='green')
    ax2.set(xlabel="angle (degrees)", ylabel="speed (m/s)")
    ax2.set_xlim([5, 85])
    ax2.set_ylim([0, 25])
    ax2.set_title("Drag dot to change shoot angle and speed")

    vx = shoot_state[1]
    vy = shoot_state[3]
    ax2.scatter(degrees(arctan2(vy, vx)), (vx**2 + vy**2)**0.5, c='black')



def onHover(event):
    if event.button == 1: 
        ix, iy = event.xdata, event.ydata

        if event.inaxes is ax1 and -6 < ix < -1 and 0.2 < iy < 1.25:

            shoot_state[0] = ix
            shoot_state[2] = iy
            
            repaint_ax1()
            repaint_ax2()
            plt.draw()

        if event.inaxes is ax2:

            angle = radians(ix)
            speed = iy

            shoot_state[1] = speed * cos(angle)
            shoot_state[3] = speed * sin(angle)

            repaint_ax1()

            repaint_ax2()

            plt.draw()




repaint_ax1()
repaint_ax2()

fig.canvas.mpl_connect('motion_notify_event', onHover)
plt.show()
plt.draw()


