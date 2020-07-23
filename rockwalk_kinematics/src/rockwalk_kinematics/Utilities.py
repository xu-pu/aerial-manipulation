import math

def compute_euler(rot):
    """ Computes body eulers suitable to analyze rolling motion:
    Rot(z, \psi)*Rot(z,pi/2)*Rot(y, \theta)*Rot(z, \phi)
    Also take a look at the README.md file.
    """

    if rot[2,2]!= 1 or rot[2,2]!= 1:
        theta = math.atan2(math.sqrt(math.pow(rot[0,2],2) + math.pow(rot[1,2],2)),rot[2,2])
        phi = math.atan2(rot[2,1],-rot[2,0])
        psi = math.atan2(-rot[0,2],rot[1,2])

    elif rot[2,2]== -1:
        theta = math.pi/2
        phi = 0
        psi = math.atan2(rot[0,0], -rot[1,0])

    elif rot[2,2]== 1:
        theta = 0
        phi = 0
        psi = math.atan2(-rot[0,0],rot[1,0])

    return psi, theta, phi
