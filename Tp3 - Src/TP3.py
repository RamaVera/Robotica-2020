import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
from Rotaciones import ang2rad, rad2ang
from cadenaCinematica import IRB140Directo

# #############################################
# #                parametros DH
# #############################################
# def DH2Matrix(a, d, alpha, theta):
#     c = np.cos
#     s = np.sin
#     M = np.array([[c(theta), -s(theta) * c(alpha), s(theta) * s(alpha), a * c(theta)],
#                   [s(theta), c(theta) * c(alpha), -c(theta) * s(alpha), a * s(theta)],
#                   [0, s(alpha), c(alpha), d],
#                   [0, 0, 0, 1]])
#     return M
# #############################################
# #                IRB 140
# #############################################
# def skew(k):
#     """Calcula la matriz equivalente al producto vectorial"""
#     k1,k2,k3 = k.flatten()
#     matriz = np.array([[  0, -k3,  k2],
#                        [ k3,   0, -k1],
#                        [-k2,  k1,   0]])
#     return matriz
#
def propagarTerna(q,j,k):
    """Calcula el problema cinematico directo"""
    def DH2Matrix(a, d, alpha, theta):
        c = np.cos
        s = np.sin
        M = np.array([[c(theta), -s(theta) * c(alpha), s(theta) * s(alpha), a * c(theta)],
                      [s(theta), c(theta) * c(alpha), -c(theta) * s(alpha), a * s(theta)],
                      [0, s(alpha), c(alpha), d],
                      [0, 0, 0, 1]])
        return M
    #Parametros de DH
    a     = np.array([   70, 360,    0,     0,    0, 0])
    d     = np.array([    0,   0,    0,   380,    0, 0])
    alpha = np.array([-1/2.,   0, 1/2., -1/2., 1/2., 0])*np.pi
    A=[]
    for i in range(6):
        A.append( DH2Matrix(a[i], d[i], alpha[i], q[i]) )

    invertir = False
    if j>k:
        j,k = k,j
        invertir = True
    res = np.eye(4)
    for i in np.arange(j,k): res = res.dot(A[i])
    if invertir: res = np.linalg.inv(res)
    return res

#
# def jacobianoIRB140(q, terna=0):
#     c = np.cos
#     s = np.sin
#     q1,q2,q3,q4,q5,q6 = q
#
#     #Parametros de DH
#     a     = np.array([   70, 360,    0,     0,    0, 0])
#     d     = np.array([    0,   0,    0,   380,    0, 0])
#     alpha = np.array([-1/2.,   0, 1/2., -1/2., 1/2., 0])*np.pi
#     nEjes = a.size
#
#     #############################################
#     # Calculo las relaciones entre ternas A_{i-1}^i
#     #############################################
#     A=[]    #Relacion entre terna i-1 con la terna i
#     for i in range(nEjes):
#         A.append( DH2Matrix(a[i], d[i], alpha[i], q[i]) )
#     A01 = propagarTerna(A,0,1)
#     for i in range(nEjes):
#         #distancias desde la terna hasta el punto a evaluar
#         pin = propagarTerna(A,i,nEjes)[0:3,[3]] #p_0,n, p_1,n
#         print pin
#
#
#     return A

def jacobianoIRB140(q):
    #Parametros de Denavit y Hartenberg para el ABB IRB140
    a1,a2,a3,a4,a5,a6 = np.array([70, 360, 0, 0, 0, 0])
    d1,d2,d3,d4,d5,d6 = np.array([0, 0, 0, 380, 0, 0])
    alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = np.array([-1 / 2., 0, 1 / 2., -1 / 2., 1 / 2., 0]) * np.pi

    q1,q2,q3,q4,q5,q6 = q
    J = np.zeros([6,6])
    s=np.sin
    c=np.cos
    #terna 3
    # Jv = np.array([[                      0,d4+a2*s(q3),d4],
    #                    [d4*s(q2+q3)+a2*c(q2)+a1,          0, 0],
    #                    [                      0,  -a2*c(q3), 0]])
    # J22_04 = np.array([[ 0,0, s(q5)],
    #                    [-1,0,-c(q5)],
    #                    [ 0,1,     0]])
    #terna 0
    J11 = np.array([[-(a1+a2*c(q2)+d4*s(q2+q3))*s(q1), -(a2*s(q2)-d4*c(q2+q3))*c(q1), d4*c(q1)*c(q2+q3)],
                   [ (a1+a2*c(q2)+d4*s(q2+q3))*c(q1), -(a2*s(q2)-d4*c(q2+q3))*s(q1), d4*s(q1)*c(q2+q3)],
                   [                               0,         -a2*c(q2)-d4*s(q2+q3),      -d4*s(q2+q3)]])
    J12 = np.zeros([3,3])
    J21 = np.array([[0, -s(q1), -s(q1)],
                    [0,  c(q1),  c(q1)],
                    [1,      0,      0]])
    # J22 = np.array([[s(q2+q3)*c(q1), -s(q1)*c(q4)-s(q4)*c(q1)*c(q2+q3), ((-(s(q1)*c(q4)*c(q2+q3)+s(q4)*c(q1))*c(q5)+s(q1)*s(q5)*s(q2+q3))*(s(q1)*s(q5)*c(q4)*c(q2+q3)+s(q1)*s(q2+q3)*c(q5)+s(q4)*s(q5)*c(q1))+(s(q5)*c(q2+q3)+s(q2+q3)*c(q4)*c(q5))*(-s(q5)*s(q2+q3)*c(q4)+c(q5)*c(q2+q3)))/(-s(q1)*s(q4)*c(q5)-s(q5)*s(q2+q3)*c(q1)+c(q1)*c(q4)*c(q5)*c(q2+q3))],
    #                 [s(q2+q3)*s(q1),                                                                                                     -s(q1)*s(q4)*c(q2+q3)+c(q1)*c(q4), s(q1)*s(q5)*c(q4)*c(q2+q3)+s(q1)*s(q2+q3)*c(q5)+s(q4)*s(q5)*c(q1)],
    #                 [      c(q2+q3),                                                                                                                        s(q4)*s(q2+q3), -s(q5)*s(q2+q3)*c(q4)+c(q5)*c(q2+q3)]])
    J22 = np.array([[s(q2+q3)*c(q1), -c(q1)*s(q4)*c(q2+q3)-s(q1)*c(q4), c(q1)*c(q2+q3)*c(q4)*s(q5)+c(q1)*s(q2+q3)*c(q5)-s(q1)*s(q4)*s(q5)],
                    [s(q2+q3)*s(q1), -s(q1)*s(q4)*c(q2+q3)+c(q1)*c(q4), s(q1)*c(q2+q3)*c(q4)*s(q5)+s(q1)*s(q2+q3)*c(q5)+c(q1)*s(q4)*s(q5)],
                    [      c(q2+q3),                    s(q4)*s(q2+q3), c(q2+q3)*c(q5)-s(q2+q3)*c(q4)*s(q5)]])
    return J11,J12,J21,J22
# cos(q1)*sin(q2)*sin(q3)*sin(q4) - cos(q1)*cos(q2)*cos(q3)*sin(q4) - cos(q4)*sin(q1)
def problemaInversoNumerico(poseInicial,poseDeseada):
    pass
#############################################
#                Ejercicios
#############################################
if __name__ == "__main__":
    print "trabajo practico sobre velocidades y singularidades"
    ej =3
    if ej == 1:
        print "posicion extendida"
        qdeg = [0, 0, 90, 0, 0, 0]
        q = ang2rad(*qdeg)
        A, conf = IRB140Directo(q)
        print A
        B = propagarTerna(q,0,6)
        print B

    elif ej == 2:
        print "Obtencion del jacobiano del IRB140"
        qdeg = [0,-90,180,0,90,0]
        # qdeg = [10,90,18,20,90,1]
        qdeg = [0, 0, 90, 0, 0, 0]

        q = ang2rad(*qdeg)
        qdot = np.array([[0, 0, 0, 1, 0, 0]]).T * 2 * np.pi  # w=2pif
        Jv,_,J21,J22 = jacobianoIRB140(q)
        print "Jv"
        print Jv
        # print "J21"
        # print J21
        a1, a2, a3, a4, a5, a6 = np.array([70, 360, 0, 0, 0, 0])
        d1, d2, d3, d4, d5, d6 = np.array([0, 0, 0, 380, 0, 0])
        alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = np.array([-1 / 2., 0, 1 / 2., -1 / 2., 1 / 2., 0]) * np.pi

        q1, q2, q3, q4, q5, q6 = q
        J = np.zeros([6, 6])
        s = np.sin
        c = np.cos
        Jv = np.array([[                      0,d4+a2*s(q3),d4],
                           [d4*s(q2+q3)+a2*c(q2)+a1,          0, 0],
                           [                      0,  -a2*c(q3), 0]])

        A = propagarTerna(q,0,3)[0:3,0:3]
        Jv = A.dot(Jv)
        print "v\n",Jv.dot(qdot[0:3])

        print "J22"
        print J22
        print "w:"
        print (J21.dot(qdot[0:3])+J22.dot(qdot[3:6]))/(2*np.pi)
        J21 = np.array([[-s(q2+q3)*c(q4), s(q4), s(q4)],
                        [-c(q2+q3),0,0],
                        [s(q4)*s(q2+q3), c(q4), c(q4)]])
        J22 = np.array([[ 0,0, s(q5)],
                        [-1,0,-c(q5)],
                        [ 0,1,     0]])
        A=propagarTerna(q,0,4)[0:3,0:3]
        J21 = A.dot(J21)
        J22 = A.dot(J22)
        print J22
        print "w:"
        w = J21.dot(qdot[0:3])+J22.dot(qdot[3:6])
        print w/(2*np.pi)

    elif ej == 3:
        w =np.zeros([3,1])
        print "testing de velocidades calculadas con v = omega*r"
        print "\nPosicion extendida, giro en q1"
        qdot = np.array([[1, 0, 0, 0, 0, 0]]).T*2*np.pi #w=2pif
        q = ang2rad(0,0,90,0,0,0)
        Jv,_,J21,J22 = jacobianoIRB140(q)
        v = Jv.dot(qdot[0:3])
        w = J21.dot(qdot[0:3])+J22.dot(qdot[3:])
        print "w*r = ", qdot[0]*810
        print "vel calculada:\n", v
        print "f=w/2pi calculada:\n", w/(2*np.pi)
        print "\nPosicion extendida, giro en q2"
        qdot = np.array([[0, 1, 0, 0, 0, 0]]).T*2*np.pi #w=2pif
        q = ang2rad(0,0,90,0,0,0)
        Jv,_,J21,J22 = jacobianoIRB140(q)
        v = Jv.dot(qdot[0:3])
        w = J21.dot(qdot[0:3]) + J22.dot(qdot[3:])
        print "w*r = ", -qdot[1]*740
        print "vel calculada:\n", v
        print "f=w/2pi calculada:\n", w / (2 * np.pi)
        print "\nPosicion extendida, giro en q1 y q4"
        qdot = np.array([[1, 0, 0, 1, 0, 0]]).T * 2 * np.pi  # w=2pif
        q = ang2rad(0, 0, 90, 0, 0, 0)
        Jv, _, J21, J22 = jacobianoIRB140(q)
        v = Jv.dot(qdot[0:3])
        w = J21.dot(qdot[0:3]) + J22.dot(qdot[3:])
        print "w*r = ", qdot[0] * 810
        print "vel calculada:\n", v
        print "f=w/2pi calculada:\n", w / (2 * np.pi)
        print "\nPosicion en forma de r, giro en q1"
        qdot = np.array([[1, 0, 0, 0, 0, 0]]).T*2*np.pi #w=2pif
        q = ang2rad(0, -90, 180, 0, 90, 0)
        Jv,_,J21,J22 = jacobianoIRB140(q)
        v = Jv.dot(qdot[0:3])
        w = J21.dot(qdot[0:3]) + J22.dot(qdot[3:])
        print "w*r = ", qdot[0] * 450
        print "vel calculada:\n", v
        print "f=w/2pi calculada:\n", w / (2 * np.pi)
        print "\nPosicion en forma de r, giro en q2"
        qdot = np.array([[0, 1, 0, 0, 0, 0]]).T*2*np.pi #w=2pif
        q = ang2rad(0,-90,180,0,90,0)
        Jv,_,J21,J22 = jacobianoIRB140(q)
        v = Jv.dot(qdot[0:3])
        w = J21.dot(qdot[0:3]) + J22.dot(qdot[3:])
        print "w*r = ", qdot[1]*np.sqrt(360**2+380**2)
        print "vel calculada:\n", v
        print "magnitud:", np.linalg.norm(v)
        print "f=w/2pi calculada:\n", w / (2 * np.pi)

