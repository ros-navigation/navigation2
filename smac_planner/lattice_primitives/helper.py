import math

def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def angle_difference(angle_1, angle_2):

    difference = abs(angle_1 - angle_2)
    
    if difference > math.pi:
        # If difference > 180 return the shorter distance between the angles
        difference = 2*math.pi - difference
    
    return difference

# def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
#     """
#     Plot arrow
#     """
#     plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
#               fc=fc, ec=ec, head_width=width, head_length=width)
#     plt.plot(x, y)
#     plt.plot(0, 0)


# def show_trajectory(target, xc, yc, arrow=False):
#     if arrow:
#         plot_arrow(target.x, target.y, target.yaw)
#     plt.plot(xc, yc, "-r")
#     plt.axis("equal")
#     plt.grid(True)
#     plt.show()