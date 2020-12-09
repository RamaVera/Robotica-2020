import numpy as np
from matplotlib import pyplot as plt, image as mpimg

#Problema directo, inverso y jacobiano (no implementado)
####################################
# deberia ser para Cualquier Robot
####################################
class cadenaCinematica(object):
    def __init__(self, modelo):
        self.eslabones = [] #Representa el sistema de referencia de los N eslabones
    @staticmethod
    def help():
        print "R(Z_{i-1}, theta_i)="
        print "[cos(theta_i) -sin(theta_i) 0 0]\n" \
              "[sin(theta_i)  cos(theta_i) 0 0]\n" \
              "[0             0            1 0]\n" \
              "[0             0            0 1]\n"
        print "T(Z_{i-1}, d_i)="
        print "[1 0 0 0  ]\n" \
              "[0 1 0 0  ]\n" \
              "[0 0 1 d_i]\n" \
              "[0 0 0 1  ]\n"
        print "T(x_{i}, a_i)="
        print "[1 0 0 a_i]\n" \
              "[0 1 0 0  ]\n" \
              "[0 0 1 0  ]\n" \
              "[0 0 0 1  ]\n"
        print "R(x_{i}, alpha_i)="
        print "[1 0             0            0]\n" \
              "[0 cos(alpha_i) -sin(alpha_i) 0]\n" \
              "[0 sin(alpha_i)  cos(alpha_i) 0]\n" \
              "[0 0             0            1]\n"
        print "A_{i-1}^i=R(Z_{i-1}, th_i)*T(Z_{i-1}, d_i)*T(x_{i}, a_i)*R(x_{i}, alpha_i)"
        print "[c(th_i) -s(th_i)c(alpha_i)  s(th_i)s(alpha_i) a_i*c(th_i)]\n" \
              "[s(th_i)  c(th_i)c(alpha_i) -c(th_i)s(alpha_i) a_i*s(th_i)]\n" \
              "[0        s(alpha_i)         c(alpha_i)        d_i        ]\n" \
              "[0        0                  0                 1          ]\n"

        plt.imshow(mpimg.imread('scara.jpg'))
        plt.show()
    def getMatrix(self, q, a):
        pass
    def getParameters(self, A):
        pass


def DH2Matrix(a, d, alpha, theta):
    c = np.cos
    s = np.sin
    M = np.array([[c(theta), -s(theta) * c(alpha), s(theta) * s(alpha), a * c(theta)],
                  [s(theta), c(theta) * c(alpha), -c(theta) * s(alpha), a * s(theta)],
                  [0, s(alpha), c(alpha), d],
                  [0, 0, 0, 1]])
    return M

#############################################
#                IRB 140
#############################################
def IRB140Directo(q):
    c = np.cos
    s = np.sin
    q1,q2,q3,q4,q5,q6 = q

    #Parametros de DH
    a     = np.array([   70, 360,    0,     0,    0, 0])
    d     = np.array([    0,   0,    0,   380,    0, 0])
    alpha = np.array([-1/2.,   0, 1/2., -1/2., 1/2., 0])*np.pi
    for i in range(a.size):
        DH = DH2Matrix(a[i], d[i], alpha[i], q[i])
        A = DH if i==0 else A.dot(DH)

    #Calculo de configuracion
    conf1 = np.sign(d[3]*s(q2+q3)+a[1]*c(q2)+a[0])
    conf2 = np.sign(c(q3))
    conf3 = np.sign(q5)
    conf = np.array([conf1, conf2, conf3])
    conf[conf==0]=1 #signo(0)=1
    return A, conf


def IRB140Inverso(M, conf, q1_ant = 0, q4_ant = 0):
    #Configuracion de la solucion
    conf1, conf2, conf3 = conf
    if not (conf1**2 ==1 and conf2**2==1 and conf3**2==1):
        raise Exception("Configuracion invalida")
    #Parametros de Denavit y Hartenberg para el ABB IRB140
    a = np.array([70, 360, 0, 0, 0, 0])
    d = np.array([0, 0, 0, 380, 0, 0])
    alpha = np.array([-1 / 2., 0, 1 / 2., -1 / 2., 1 / 2., 0]) * np.pi

    px = M[0,3]
    py = M[1,3]
    pz = M[2,3]

    #Calculo q1
    if px==0 and py == 0:
        print "q1 es singular, cualquier q1 sirve"
        print "elijo q1 ingresado o 0 en su defecto"
        q1 = q1_ant
        s1 = np.sin(q1)
        c1 = np.cos(q1)
    else:
        s1 = conf1 * py / np.sqrt(px ** 2 + py ** 2)
        c1 = conf1 * px / np.sqrt(px ** 2 + py ** 2)
        q1 = np.arctan2(s1,c1)

    # Calculo q3
    s3 = ( (px*c1+py*s1-a[0])**2+pz**2-(d[3]**2+a[1]**2) )/ (2*a[1]*d[3])
    if np.abs(s3)>1:
        raise Exception("Punto no alcanzable en q3")
    c3 = conf2*np.sqrt(1-s3**2)
    if np.abs(c3)<1E-9:
        print "q3 es singular"
    q3 = np.arctan2(s3,c3)

    #Calculo q2
    s2 = ( (px*c1+py*s1-a[0])* d[3]*c3 - pz*(d[3]*s3+a[1]) ) / ((d[3]*c3)**2+(d[3]*s3+a[1])**2)
    c2 = ( (px*c1+py*s1-a[0])* (d[3]*s3+a[1]) + pz*d[3]*c3 ) / ((d[3]*c3)**2+(d[3]*s3+a[1])**2)
    q2 = np.arctan2(s2,c2)

    q = np.array([q1,q2,q3,0,0,0])
    #Calculo de R^3_0
    for i in range(3):
        DH = DH2Matrix(a[i], d[i], alpha[i], q[i])
        R_30 = DH[0:3,0:3] if i == 0 else R_30.dot(DH[0:3,0:3])

    #Calculo de R^6_3
    R_60 = M[0:3,0:3]
    R_63 = R_30.T.dot(R_60) # R^3_0 *R^6_3 = R^6_0

    #Calculo de q5
    c5 = R_63[2,2]
    s5 = conf3*np.sqrt(1-c5**2)
    q5 = np.arctan2(s5,c5)

    #Calculo de q4 y q6
    if np.abs(R_63[0,2])<1E-9 and np.abs(R_63[1,2])<1E-9:
        print "q4 es singular, cualquier q4 sirve"
        print "elijo q4 ingresado o 0 en su defecto"
        q4 = q4_ant
        s46 = R_63[1,0]
        c46 = R_63[0,0]
        q6 = np.arctan2(s46,c46)-q4 #Rotacion sobre Z
    else:
        s4 = conf3*R_63[1,2]
        c4 = conf3*R_63[0,2]
        q4 = np.arctan2(s4,c4)

        s6 =  conf3*R_63[2,1]
        c6 = -conf3*R_63[2,0]
        q6 = np.arctan2(s6,c6)

    q[3:6]=q4,q5,q6
    #q da NaN, y tira error
    q[np.abs(q)<1E-9]=0
    return q
#############################################
#                SCARA
#############################################
def ScaraDirecto(q,a):
    c = np.cos
    s = np.sin
    q1,q2,q3,q4 = q
    a1,a2 = a
    c124 = np.cos(q1+q2+q4)
    s124 = np.sin(q1+q2+q4)
    M = np.array([[c124, -s124, 0, a1*c(q1)+a2*c(q1+q2)],
                  [s124,  c124, 0, a1*s(q1)+a2*s(q1+q2)],
                  [0   ,     0, 1, q3                  ],
                  [0   ,     0, 0, 1                   ]])

    conf = np.sign(q2)+(q2==0)
    return M, conf


def ScaraInverso(M, conf, a):
    px = M[0,3]
    py = M[1,3]
    pz = M[2,3]
    a1,a2 = a
    if not (0<=(a1-a2)**2<=(px**2+py**2) <= (a1+a2)**2):#Alcanzabilidad
        raise Exception("El punto no es alcanzable")
    c2 = (px**2+py**2 - (a1**2+a2**2))/(2*a1*a2)
    s2 = conf*np.sqrt(1-c2**2)
    q2 = np.arctan2(s2,c2)

    s1 = (a2*(py*c2-px*s2)+a1*py)/(px**2+py**2)
    c1 = (a2*(py*s2+px*c2)+a1*px)/(px**2+py**2)
    q1 = np.arctan2(s1,c1)

    q3 = pz

    s124 = M[1,0]
    c124 = M[0,0]
    q4 = np.arctan2(s124, c124)-q1-q2
    q = q1,q2,q3,q4
    return q


