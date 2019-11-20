# dt = 0.1
# max_v = 50pi mm/s
# omega = 10pi/9 rad/s

def straight_line(t):
    sequence = []

    for i in range(15):
        # travel about 250 mm
        if i < 15:
            sequence.append((60,60))
        # then stop
        elif 15 <= i < 20:
            sequence.append((0, 0))
        # drive backwards
        elif 20 <= i < 35:
            sequence.append((-60, -60))

    return sequence[t]

def square(t):
    sequence = []

    # travel about 250 mm
    for i in range(100):
        if i < 10:  #drive straight
            sequence.append((60,60))
        elif 10 <= i < 15:  #rotate
            sequence.append((54, -54))

        elif 15 <= i < 25:  #straight
            sequence.append((60,60))        
        elif 25 <= i < 30:  #rotate
            sequence.append((54, -54))

        elif 30 <= i < 40:  #straight
            sequence.append((60,60))
        elif 40 <= i < 45:  # rotate
            sequence.append((54, -54))

        elif 45 <= i < 55:  #straight
            sequence.append((60,60))
        elif 55 <= i < 60:  # rotate
            sequence.append((54, -54))
        else:
            sequence.append((0,0))
    return sequence[t]

def circle(t):
    return -60, 60

def no_movement(t):
    return 0, 0

def big_circle(t):
    return 60, 20