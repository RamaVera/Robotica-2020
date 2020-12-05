import numpy as np


def RotZ(theta):
    R = np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
    return R


def RotY(theta):
    R = np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
    return R


def rotEuler (phi, theta, psi):
    R = RotZ(phi).dot(RotY(theta).dot(RotZ(psi)))
    return R


def invEuler (R, conf=1):
    if conf == 0:
        raise Exception("conf es el signo de la rotacion, no puede ser nulo")
    #Falta resolver indeterminacion atan(0,0)
    conf = np.sign(conf)
    theta = np.arccos(R[2,2])
    theta = np.arctan2(conf*np.sqrt(1-np.square(R[2,2])),R[2,2])

    phi = np.arctan2(conf*R[1,2],conf*R[0,2])
    psi = np.arctan2(conf*R[2,1],-conf*R[2,0])
    return np.array([phi,theta, psi])


def rotK(k, theta):
    if np.shape(k) != (3,1):
        raise Exception("shape(k) debe ser 3,1")
    k = k/np.sqrt(np.sum(k**2)) #Normalizo para que sea unitario
    R = np.eye(3)*np.cos(theta)+(1-np.cos(theta))*k.dot(k.T)+np.sin(theta)*skew(k)
    return R


def skew(k):
    """Calcula la matriz equivalente al producto vectorial"""
    k1,k2,k3 = k[0,0], k[1,0], k[2,0]
    matriz = np.array([[  0, -k3,  k2],
                       [ k3,   0, -k1],
                       [-k2,  k1,   0]])
    return matriz


def ang2rad(*ang):
    res = []
    for x in ang:
        res.append( x*np.pi/180 )
    return res


def rad2ang(*rad):
    res = []
    for x in rad:
        res.append( x*180/np.pi )
    return res

def shift360(angulo):
    while 0<=angulo<=360:
        if angulo > 360:
            angulo = angulo - 360
        elif angulo < 0:
            angulo = angulo + 360
    return angulo


def cuaternion2KTheta(q):
    g, qx, qy, qz = q
    theta = 2 * np.arccos(g)
    k = np.zeros([3, 1])
    if qx or qy or qz:
        k[0] = qx / np.sin(theta / 2)
        k[1] = qy / np.sin(theta / 2)
        k[2] = qz / np.sin(theta / 2)
    return k, theta