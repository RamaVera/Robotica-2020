import numpy as np
from scipy.spatial.transform import Rotation



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
    return np.array(res)
    # return res


def rad2ang(*rad):
    res = []
    for x in rad:
        res.append( x*180/np.pi )
    return np.array(res)

def shift360(q,angulos = '[0,2pi]'):
    #Paso q a [0,1]
    cdadVueltas = np.floor(q/(2*np.pi))
    q = q-cdadVueltas*2*np.pi #q va de [0,2pi]
    if angulos == '[0,2pi]':
        return q
    elif angulos == '[-pi,pi]':
        q = q*(q<=np.pi)+(q-2*np.pi)*(q>np.pi)
        return q


def cuaternion2KTheta(q):
    g, qx, qy, qz = q
    theta = 2 * np.arccos(g)
    k = np.zeros([3, 1])
    if qx or qy or qz:
        k[0] = qx / np.sin(theta / 2)
        k[1] = qy / np.sin(theta / 2)
        k[2] = qz / np.sin(theta / 2)
    return k, theta

def rmat2rvec(mat):
    R = Rotation.from_dcm(mat)
    return R.as_rotvec()

if __name__=='__main__':
    np.set_printoptions(precision=2)
    ej =1
    if ej==1:
        print "Testing de shift 360"
        q = np.array([0, 2*np.pi-0.01,-2*np.pi+0.01,100,13])
        print "q orig:\n",rad2ang(*q),'deg'
        q = shift360(q,'[0,2pi]')
        print "q obtenido:\n",rad2ang(*q),'deg'
        q = shift360(q,'[-pi,pi]')
        print "q obtenido:\n",rad2ang(*q),'deg'
        print shift360(1.0001*np.pi,'[-pi,pi]')


