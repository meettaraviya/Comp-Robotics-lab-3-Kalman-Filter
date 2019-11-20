# dt = 0.1
# max_v = 50pi mm/s
# omega = 10pi/9 rad/s

def straight_line(t):
    # travel about 250 mm
    if t < 15:
        return (60,60)
    # then stop
    elif 15 <= t < 20:
        return (0, 0)
    # drive backwards
    elif 20 <= t < 35:
        return (-60, -60)
    else:
        return (0, 0)

def square(t):

    if t < 10:  #drive straight
        return (60,60)
    elif 10 <= t < 15:  #rotate
        return (54, -54)

    elif 15 <= t < 25:  #straight
        return (60,60)
    elif 25 <= t < 30:  #rotate
        return (54, -54)

    elif 30 <= t < 40:  #straight
        return (60,60)
    elif 40 <= t < 45:  # rotate
        return (54, -54)

    elif 45 <= t < 55:  #straight
        return (60,60)
    elif 55 <= t < 60:  # rotate
        return (54, -54)
    else:
        return (0,0)

def circle(t):
    return -60, 60

def no_movement(t):
    return 0, 0

def big_circle(t):
    return 60, 20