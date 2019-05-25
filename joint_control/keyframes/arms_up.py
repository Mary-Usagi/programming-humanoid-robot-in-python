

def arms_up():
    # Choregraphe bezier export in Python.
    names = list()
    times = list()
    keys = list()


    names.append("LShoulderPitch")
    times.append([1.00000])
    keys.append([[0.08433, [3, -0.33333, 0.00000], [3, 0.33333, 0.00000]]])

    names.append("LShoulderRoll")
    times.append([1.00000])
    keys.append([[1.55390, [3, -0.33333, 0.00000], [3, 0.33333, 0.00000]]])

    names.append("RShoulderPitch")
    times.append([1.00000])
    keys.append([[-0.02757, [3, -0.33333, 0.00000], [3, 0.33333, 0.00000]]])
    
    names.append("RShoulderRoll")
    times.append([1.00000])
    keys.append([[-1.53558, [3, -0.33333, 0.00000], [3, 0.33333, 0.00000]]])
    return names, times, keys
