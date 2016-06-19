def cross_vec3(a,b) :
    return [a[1]*b[2] - a[2]*b[1],a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]]


def dot_vec3(a,b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
