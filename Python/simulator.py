import pygame as pg
import numpy as np

WIDTH = 640
HEIGHT = 480

def calculateIK(x_f, y_f, angle_f, lengths):
    """Return array of pivot points calculated from inverse kinematics"""
    global WIDTH
    global HEIGHT
    # flip final angle to be measured CCW
    angle_f *= -1
    x = [0, 0, 0]
    y = [0, 0, 0]
    angles = [0, 0, 0]
    # offset final values by origin point and then calculate
    x[2] = x_f - (WIDTH // 2) + lengths[2]*np.cos(np.deg2rad(angle_f))
    y[2] = y_f - (HEIGHT // 2) + lengths[2]*np.sin(np.deg2rad(angle_f))
    c = np.sqrt(x[2]**2 + y[2]**2)  # dist from origin to x2, y2

    if c > (lengths[0] + lengths[1]):
        return [(-1, -1), (-1, -1), (-1, -1)]
    angles[1] = -np.arctan2(y[2], x[2]) + np.arccos((lengths[1] * lengths[1] - lengths[0] * lengths[0] - c * c) / (-2 * lengths[0] * c))
    # flip angle to choose upper solution
    angles[1] *= -1

    x[1] = lengths[0] * np.cos(angles[1])
    y[1] = lengths[0] * np.sin(angles[1])
    # re-apply offset to all pivot values for display
    for i in range(len(x)):
        x[i] += WIDTH // 2
        y[i] += HEIGHT // 2

    return [(x[0], y[0]), (x[1], y[1]), (x[2], y[2]), (x_f, y_f)]

def calculateFK(angles, lengths):
    """Return array of pivot points calculated from forward kinematics"""
    global WIDTH
    global HEIGHT
    pivots = [(WIDTH//2, HEIGHT//2)]
    # update pivots of later joints
    for i in range(1, len(pivots)):
        piv_x = WIDTH // 2
        piv_y = HEIGHT // 2
        for j in range(i):
            piv_x += lengths[j] * np.cos(np.deg2rad(angles[j]))
            piv_y += lengths[j] * np.sin(np.deg2rad(angles[j]))
        pivots[i] = (piv_x, piv_y)


pg.init()
screen = pg.display.set_mode((WIDTH, HEIGHT))
clock = pg.time.Clock()

lengths = [80, 80, 40]
widths = [20, 15, 10]
radius = [15, 12, 9, 7]
angles = [0, 0, 0]
x_f = 0
y_f = 0
angle_f = 0

# Store pivot positions
pivots = [(WIDTH//2, HEIGHT//2)]
for i in range(1, len(lengths)+1):
    piv_x = pivots[0][0]
    piv_y = pivots[0][1]
    for j in range(i):
        piv_x += lengths[j] * np.cos(np.deg2rad(angles[j]))
        piv_y += lengths[j] * np.sin(np.deg2rad(angles[j]))
    pivots.append((piv_x, piv_y))
prev_pivots = pivots  # to store last working pivots

running = True
mode = "General IK Mode"
while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False

    # Get mouse and keyboard actions
    mousex, mousey = pg.mouse.get_pos()
    keys = pg.key.get_pressed()
    if keys[pg.K_1]:
        mode = "General IK Mode"
    elif keys[pg.K_2]:
        mode = "Fixed Angle IK Mode"
    if keys[pg.K_UP] and mode == "Fixed Angle IK Mode":
        angle_f += 5
    elif keys[pg.K_DOWN] and mode == "Fixed Angle IK Mode":
        angle_f -= 5


    if mode == "General IK Mode":
        # Start with default 180 degree CCW and test other angles to find working one
        angle_f = 180
        pivots = calculateIK(mousex, mousey, angle_f, lengths)
        while pivots[0] == (-1, -1) and angle_f != 540:
            angle_f += 5
            pivots = calculateIK(mousex, mousey, angle_f, lengths)
    elif mode == "Fixed Angle IK Mode":
        pivots = calculateIK(mousex, mousey, angle_f, lengths)

    # display final arm and text
    screen.fill(pg.Color('gray12'))
    font = pg.font.SysFont(None, 24)
    if pivots[0] == (-1, -1):
        for i in range(len(prev_pivots) - 1):
            pg.draw.line(screen, pg.Color("grey48"), prev_pivots[i], prev_pivots[i + 1], widths[i])
            pg.draw.circle(screen, pg.Color("grey"), prev_pivots[i], radius[i])  # Pivot point
        pg.draw.circle(screen, pg.Color("grey"), prev_pivots[len(prev_pivots) - 1], radius[len(prev_pivots) - 1])
        error_text = font.render("Point out of Reachable Range", True, pg.Color("red"))
        screen.blit(error_text, (WIDTH // 2 - 100, HEIGHT // 2 + 50))
    else:
        for i in range(len(pivots) - 1):
            pg.draw.line(screen, pg.Color("blue"), pivots[i], pivots[i + 1], widths[i])
            pg.draw.circle(screen, pg.Color("chartreuse3"), pivots[i], radius[i])  # Pivot point
        pg.draw.circle(screen, (250, 0, 0), pivots[len(pivots) - 1], radius[len(pivots) - 1])
        angle_text = font.render("End Angle: " + str(angle_f % 360), True, pg.Color("white"))
        screen.blit(angle_text, (WIDTH // 2 - 60, HEIGHT // 2 + 50))
        prev_pivots = pivots

    mode_text = font.render("Mode: " + mode, True, pg.Color("white"))
    screen.blit(mode_text, (15, 15))
    pg.display.set_caption('Robotic Arm IK Simulator')
    pg.display.flip()
    clock.tick(30)

pg.quit()