from UM.Math.Vector import Vector

ScaleFactor = 10

def toODE(input):
    if isinstance(input, Vector):
        return (input.x / ScaleFactor, input.y / ScaleFactor, input.z / ScaleFactor)
    elif hasattr(input, "__len__"):
        result = []
        for i in range(len(input)):
            result.append(input[i] / ScaleFactor)
        return result
    else:
        return input / ScaleFactor

def fromODE(input):
    if hasattr(input, "__len__"):
        if len(input) == 3:
            return Vector(input[0] * ScaleFactor, input[1] * ScaleFactor, input[2] * ScaleFactor)

        result = []

        for i in range(len(input)):
            result.append(input[i] * ScaleFactor)

        return result
    else:
        return input * ScaleFactor

