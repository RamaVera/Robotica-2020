import numpy as np
from scipy.linalg import null_space
import matplotlib.image as mpimg
import sympy as sym
from Rotaciones import ang2rad, rad2ang, rmat2rvec,shift360
from cadenaCinematica import IRB140Directo, IRB140Inverso
from matplotlib import pyplot as plt, animation as animation
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3D

# def null(a, rtol=1e-5):
#     u, s, v = np.linalg.svd(a)
#     rank = (s > rtol*s[0]).sum()
#     return rank, v[rank:].T.copy()
np.set_printoptions(formatter={'float': lambda x: "{0:0.2f}".format(x)})
# np.set_printoptions(precision=2)

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

def jacobianoIRB140(q, block = False):
    #Parametros de Denavit y Hartenberg para el ABB IRB140
    a1,a2,a3,a4,a5,a6 = np.array([70, 360, 0, 0, 0, 0])
    d1,d2,d3,d4,d5,d6 = np.array([0, 0, 0, 380, 0, 0])
    alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = np.array([-1 / 2., 0, 1 / 2., -1 / 2., 1 / 2., 0]) * np.pi

    q1,q2,q3,q4,q5,q6 = q
    J = np.zeros([6,6])
    s=np.sin
    c=np.cos
    #terna 0
    J11 = np.array([[-(a1+a2*c(q2)+d4*s(q2+q3))*s(q1), -(a2*s(q2)-d4*c(q2+q3))*c(q1), d4*c(q1)*c(q2+q3)],
                   [ (a1+a2*c(q2)+d4*s(q2+q3))*c(q1), -(a2*s(q2)-d4*c(q2+q3))*s(q1), d4*s(q1)*c(q2+q3)],
                   [                               0,         -a2*c(q2)-d4*s(q2+q3),      -d4*s(q2+q3)]])
    J12 = np.zeros([3,3])
    J21 = np.array([[0, -s(q1), -s(q1)],
                    [0,  c(q1),  c(q1)],
                    [1,      0,      0]])
    J22 = np.array([[s(q2+q3)*c(q1), -c(q1)*s(q4)*c(q2+q3)-s(q1)*c(q4), c(q1)*c(q2+q3)*c(q4)*s(q5)+c(q1)*s(q2+q3)*c(q5)-s(q1)*s(q4)*s(q5)],
                    [s(q2+q3)*s(q1), -s(q1)*s(q4)*c(q2+q3)+c(q1)*c(q4), s(q1)*c(q2+q3)*c(q4)*s(q5)+s(q1)*s(q2+q3)*c(q5)+c(q1)*s(q4)*s(q5)],
                    [      c(q2+q3),                    s(q4)*s(q2+q3), c(q2+q3)*c(q5)-s(q2+q3)*c(q4)*s(q5)]])
    if block:
        return J11,J12,J21,J22
    else:
        J = np.zeros([6,6])
        J[:3,:3]=J11
        J[:3,3:]=J12
        J[3:,:3]=J21
        J[3:,3:]=J22
        return J

def singularidadesIRB140():
    #Parametros de Denavit y Hartenberg para el ABB IRB140
    a1,a2,a3,a4,a5,a6 = sym.symbols('a1:7')
    d1,d2,d3,d4,d5,d6 = sym.symbols('d1:7')
    q1,q2,q3,q4,q5,q6 = sym.symbols('q1:7')
    c = sym.cos
    s = sym.sin
    ec1 = a1+d4*s(q2+q3)+a2*c(q2)+a1
    ec2 = a2*c(q3)
    ec3 = d4*s(q5)
    r = sym.solve(ec1, q2,q3)
    print r
    return 1

def problemaInversoNumerico(qInicial,poseDeseada,vel=1, iteracionesMax=1000):
    Ts = .1
    epsilon = 1
    q = qInicial
    dp=np.zeros([6,1])#dp=[dx, dphi]
    xd = posed[:3,[3]]
    Rd = posed[:3,:3]
    q = shift360(q, '[-pi,pi]')
    for i in range(iteracionesMax):
        pose,_ = IRB140Directo(q)#Posicion actual
        x = pose[:3,[3]]
        R = pose[:3,:3]
        dx = xd-x
        dR = (R.T.dot(Rd)).T
        dphi = np.reshape(rmat2rvec(dR),[3,1])
        dp[0:3] = dx/np.linalg.norm(dx)*vel*Ts #muevo en direccion
        dp[3:] = dphi
        J = jacobianoIRB140(q)
        Jpinv = np.linalg.pinv(J)
        q=q+Jpinv.dot(dp).flatten()
        q = shift360(q,'[-pi,pi]')
        yield q
        if np.linalg.norm(dx)+np.linalg.norm(dphi)<epsilon:
            # print "FIN"
            break


def graficarTrayectoria(qInicial, poseDeseada, vel=10):
    iteraciones = 2000
    pos = problemaInversoNumerico(qInicial, posed,vel=vel, iteracionesMax=iteraciones)
    registroPos = np.zeros([3, iteraciones])
    for i, q in enumerate(pos):  # todo
        A,conf = IRB140Directo(q)
        x=A[0:3,[3]]
        registroPos[:, [i]] = x
    registroPos = registroPos[:, :i]

    AInit,conf = IRB140Directo(qInicial)
    xInit = AInit[:3,3]
    xd  = posed[0:3,3]
    phiInit = rmat2rvec(AInit[:3,:3])*180/np.pi
    phid = rmat2rvec(posed[:3, :3])*180/np.pi
    phiFin = rmat2rvec(A[:3,:3])*180/np.pi

    pRelevantes = np.array([[0, 0, 0],  # Punto origen de coordenadas
                            xInit,      # Punto de inicio
                            xd]).T  # punto final

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlim([-850, 850])
    ax.set_ylim([-850, 850])

    fig.suptitle("trayectoria\n orientacion inicial: {}\norientacion final: {}\norientacion deseada: {}".format(phiInit,phiFin,phid))
    ax.plot3D(registroPos[0, :], registroPos[1, :], registroPos[2, :], 'b')
    ax.plot3D(pRelevantes[0, :], pRelevantes[1, :], pRelevantes[2, :], 'ro')
    ax.text(pRelevantes[0, 0], pRelevantes[1, 0], pRelevantes[2, 0], "Origen coordenadas", color='black')
    ax.text(pRelevantes[0, 1], pRelevantes[1, 1], pRelevantes[2, 1], "punto inicial", color='black')
    ax.text(pRelevantes[0, 2], pRelevantes[1, 2], pRelevantes[2, 2], "punto deseado", color='black')
    plt.show()
    return q
#############################################
#                Ejercicios
#############################################
if __name__ == "__main__":
    print "trabajo practico sobre velocidades y singularidades"
    ej =5
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
        Jv,_,J21,J22 = jacobianoIRB140(q, True)
        v = Jv.dot(qdot[0:3])
        w = J21.dot(qdot[0:3])+J22.dot(qdot[3:])
        print "w*r = ", qdot[0]*810
        print "vel calculada:\n", v
        print "f=w/2pi calculada:\n", w/(2*np.pi)
        print "\nPosicion extendida, giro en q2"
        qdot = np.array([[0, 1, 0, 0, 0, 0]]).T*2*np.pi #w=2pif
        q = ang2rad(0,0,90,0,0,0)
        Jv,_,J21,J22 = jacobianoIRB140(q, True)
        v = Jv.dot(qdot[0:3])
        w = J21.dot(qdot[0:3]) + J22.dot(qdot[3:])
        print "w*r = ", -qdot[1]*740
        print "vel calculada:\n", v
        print "f=w/2pi calculada:\n", w / (2 * np.pi)
        print "\nPosicion extendida, giro en q1 y q4"
        qdot = np.array([[1, 0, 0, 1, 0, 0]]).T * 2 * np.pi  # w=2pif
        q = ang2rad(0, 0, 90, 0, 0, 0)
        Jv, _, J21, J22 = jacobianoIRB140(q, True)
        v = Jv.dot(qdot[0:3])
        w = J21.dot(qdot[0:3]) + J22.dot(qdot[3:])
        print "w*r = ", qdot[0] * 810
        print "vel calculada:\n", v
        print "f=w/2pi calculada:\n", w / (2 * np.pi)
        print "\nPosicion en forma de r, giro en q1"
        qdot = np.array([[1, 0, 0, 0, 0, 0]]).T*2*np.pi #w=2pif
        q = ang2rad(0, -90, 180, 0, 90, 0)
        Jv,_,J21,J22 = jacobianoIRB140(q, True)
        v = Jv.dot(qdot[0:3])
        w = J21.dot(qdot[0:3]) + J22.dot(qdot[3:])
        print "w*r = ", qdot[0] * 450
        print "vel calculada:\n", v
        print "f=w/2pi calculada:\n", w / (2 * np.pi)
        print "\nPosicion en forma de r, giro en q2"
        qdot = np.array([[0, 1, 0, 0, 0, 0]]).T*2*np.pi #w=2pif
        q = ang2rad(0,-90,180,0,90,0)
        Jv,_,J21,J22 = jacobianoIRB140(q, True)
        v = Jv.dot(qdot[0:3])
        w = J21.dot(qdot[0:3]) + J22.dot(qdot[3:])
        print "w*r = ", qdot[1]*np.sqrt(360**2+380**2)
        print "vel calculada:\n", v
        print "magnitud:", np.linalg.norm(v)
        print "f=w/2pi calculada:\n", w / (2 * np.pi)

    elif ej==4:
        print "Generacion de trayectoria"
        print "Pose de inicio = brazo extendido"
        print "Pose de Llegada = brazo en forma de r"
        qd = ang2rad(0, -90, 180, 0, 90, 0)  # forma de r
        print qd
        posed, confd = IRB140Directo(qd)
        print "pose deseada:\n", posed
        qinit = ang2rad(0, 0, 90, 0, 0, 0)  # brazo extendido
        qObtenida = graficarTrayectoria(qinit,posed)
        AObtenida,conf = IRB140Directo(qObtenida)
        print "pose obtenida:\n", AObtenida
        print "q destino obtenida numericamente\n", qObtenida
        qVerdadero = IRB140Inverso(posed,conf)
        print "q obtenido con resolucion del problema inverso\n", qVerdadero

    elif ej ==5:
        print "Verificacion de varias trayectorias"
        for i in range(3):
            qd = np.random.uniform(-np.pi, np.pi,6)
            qInit = np.random.uniform(-np.pi, np.pi,6)
            posed,confd =  IRB140Directo(qd)
            qObtenida = graficarTrayectoria(qInit,posed)


    elif ej ==6:
        print "Obtengo las q singulares"
        qsing1 = ang2rad( *[-53.33, -58.77, 7.52, 0, 0, 0] )
        singularidadesIRB140()

    elif ej == 7:
        print "\nSingularidades del Brazo"
        A = np.array([[1, 0, 0,   0],
                      [0, 1, 0,   0],
                      [0, 0, 1, 500],
                      [0, 0, 0,   1],
                      ])
        # q= IRB140Inverso(A,[1,1,1],q1_ant=0)
        q = [0.00, -0.86, -0.07, 0.00, 0.93, 0.00]
        print "q:\n",q,"\n"
        J = jacobianoIRB140(q)
        print "J:\n",J
        nullJ = null_space(J, 1E-3)
        qdotrad = nullJ / np.min(nullJ)
        qdotgrad = rad2ang(*qdotrad.flatten())
        print "\nnull(J)[deg]\n", np.reshape(qdotgrad, [6, 1]) / np.min(qdotgrad) * 10

        nullJ = null_space(J.T, 1E-3)
        qdotrad = nullJ / np.min(nullJ)
        qdotgrad = rad2ang(*qdotrad.flatten())
        print "\nnull(J')[deg]\n", np.reshape(qdotgrad, [6, 1])
        #####################
        #####################
        print "\nSingularidades del codo"
        q = [ 0, -90, 90, 0, 45, 0]
        print "q:\n",q,"\n"
        q = ang2rad(*q)
        J = jacobianoIRB140(q)
        print "J\n",J
        nullJ = null_space(J,1E-4)
        qdotrad = nullJ/np.min(nullJ)
        qdotgrad = rad2ang(*qdotrad.flatten())
        print "\nnull(J)[deg]\n",np.reshape(qdotgrad,[6,1])/np.min(qdotgrad)*10

        nullJ = null_space(J.T,1E-4)
        qdotrad = nullJ/np.min(nullJ)
        qdotgrad = rad2ang(*qdotrad.flatten())
        print "\nnull(J')[deg]\n",np.reshape(qdotgrad,[6,1])
        #####################
        #####################
        print "\nSingularidades de la munieca"
        q = [0, -90, 180, 0, 0, -90]
        print "q:\n", q, "\n"
        q = ang2rad(*q)
        J = jacobianoIRB140(q)
        print "J\n", J
        nullJ = null_space(J,1E-3)
        qdotrad = nullJ / np.min(nullJ)
        qdotgrad = rad2ang(*qdotrad.flatten())
        print "\nnull(J)[deg]\n", np.reshape(qdotgrad, [6, 1]) / np.min(qdotgrad) * 10

        nullJ = null_space(J.T,1E-3)
        qdotrad = nullJ / np.min(nullJ)
        qdotgrad = rad2ang(*qdotrad.flatten())
        print "\nnull(J')[deg]\n", np.reshape(qdotgrad, [6, 1])




