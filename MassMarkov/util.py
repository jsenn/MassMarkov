import math


def mean(xs):
    assert(len(xs) > 0)
    return sum(xs) / float(len(xs))

def stddev(xs, avg):
    assert(len(xs) >= 2)
    val = 0
    for x in xs:
        diff = x - avg
        val += diff * diff

    return math.sqrt(val / (len(xs) - 1))

