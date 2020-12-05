import numpy as np
import sympy as sym
from sympy import init_printing,latex,pprint
from tabulate import tabulate
import time
init_printing(pretty_print=True,use_unicode=True)

#################################################
# Script para realizar el estudio completo de un robot Scara
# Se obtienen las matrices de la cinematica, el Jacobiano de velocidades,
# las matrices de la dinamica, y la S - function para Simulink
class robot(object):
    def __init__(self, modelo, simbolico = True):
        # Nombre del modelo a generar.
        # Si el string esta vacio no se escriben las functiones correpondientes
        self.modelo = modelo
        self.simbolico = simbolico #Guardo los datos de forma simbolica
        self.tablaDH=None
        self.nAxis=None
        self.jointType=None
        self.masaEslabon=None
        self.rgEslabon=None
        self.IgzzEslabon=None
        if modelo == "scara":
            self.parametrosScara()
        elif modelo == "IRB140":
            self.parametrosIRB140()
        # Montaje del robot
        #Direccion de la gravedad
        self.g = np.array([[0, 0, -9.8]]).T # Puesto en el piso
        self.A = [] #conjunto de matrices de rototraslacion que vinculan a una terna con la siguiente
        self._calcKinematics() #Calcula las matrices cinematicas
        print "Cinematica Calculada"
    def help(self):
        print "************************************************\nModelizando robot definido por los parametros DH\n"
        table = tabulate(self.tablaDH,['a', 'd', 'alpha', 'theta'])
        print table

    def parametrosIRB140(self):
        a = np.array([70, 360, 0, 0, 0, 0])
        d = np.array([0, 0, 0, 380, 0, 0])
        alpha = np.array([-1 / 2., 0, 1 / 2., -1 / 2., 1 / 2., 0]) * np.pi
        theta = np.array([0, 0, 0, 0, 0, 0])
        self.tablaDH = np.array([
            a,d,alpha,theta
        ]).T
        self.jointType = np.array([['r', 'r', 'r', 'r', 'r', 'r']]).T
        self.nAxis = self.tablaDH.shape[0]
    def parametrosScara(self):
        # Los parametros DH se almacenan en las columnas de la matriz DH
        # DH = [a d alpha theta]
        DH = np.array([
            [0.2, 0.2, 0, 0],    #a
            [  0,   0, 0, 0],    #d
            [  0,   0, 0, 0],    #alpha
            [  0,   0, 0, 0]]).T #theta
        self.tablaDH = DH
        self.nAxis = self.tablaDH.shape[0]

        # Definicion del tipo de juntas
        self.jointType = np.array([['r', 'r', 'p', 'r']]).T

        # Datos de la dinamica
        self.masaEslabon = np.array([[1.5, 1, 0.5, 0.2]]).T
        self.rgEslabon = np.array([
            [-DH[0, 0]/2, -DH[1, 0]/2, 0, 0],
            [          0,           0, 0, 0],
            [          0,           0, 0, 0]])
        self.IgzzEslabon = np.array([[10 ^ -3],
                                     [10 ^ -4],
                                     [0],
                                     [10 ^ -4]])
    def calcDH(self, a, d, alpha, theta):
        c = np.cos
        s = np.sin
        c = sym.cos
        s = sym.sin
        M = sym.Matrix([[c(theta), -s(theta)*c(alpha),  s(theta)*s(alpha), a*c(theta)],
                        [s(theta),  c(theta)*c(alpha), -c(theta)*s(alpha), a*s(theta)],
                        [       0,           s(alpha),           c(alpha),          d],
                        [       0,                  0,                  0,          1]])
        return M
    def _calcKinematics(self):
        # Calculo el problema cinematico directo
        # Los angulos de los ejes son las variables articulares
        theta=list(sym.symbols('theta1:{}'.format(self.nAxis+1)))#Notacion Matlab, de 1:N+1
        d    =list(sym.symbols('d1:{}'.format(self.nAxis+1)))
        a    =list(sym.symbols('a1:{}'.format(self.nAxis+1)))
        q    =list(sym.symbols('q1:{}'.format(self.nAxis+1)))
        alpha=self.tablaDH[:,2] # La torsipn la voy a trabajar en forma numerica
        #Simplifico las expresiones incorporando los ceros que faltan
        DH = self.tablaDH
        jT = self.jointType
        for i in range(self.nAxis):
            if DH[i,1]==0 and jT[i] == 'r':
                d[i]=0
            if DH[i,0] == 0:
                a[i]=0
            if DH[i,3] == 0 and jT[i] == 'p':
                theta[i]=0
            if jT[i]=='r':
                theta[i] = q[i]
            if jT[i] == 'p':
                d[i] = q[i]
        # Resolucion del problema directo
        # Matrices genericas de rototraslacion
        for i in range(self.nAxis):
            A = self.calcDH(a[i], d[i],alpha[i],theta[i])
            self.A.append( A )

    def calcKinematics(self):
        A = self.A
        if not A:
            raise Exception ( "Primero debe calcular las Ai")
        W = []
        for i in range(self.nAxis):
            if i == 0:
                W.append(A[i])
            else:
                W.append(W[i-1]*A[i])
            W[i]=sym.nsimplify(W[i])
            W[i]=sym.trigsimp(W[i])
        return W

    def escribirModeloCinematico(self):
        #Para escribir la s-function
        filename = ['auto_dirKinematics_', self.modelo]
        # writeKinematicModel(filename, W);

    def propagarTerna(self, j, k):
        """Multiplica las ternas de A, desde la terna j hasta la terna k"""
        invertir = False
        if j > k:
            j, k = k, j
            invertir = True
        res = sym.eye(4)
        for i in np.arange(j, k):
            res = res*self.A[i]
        # res = sym.trigsimp(res)
        res = sym.nsimplify(res,tolerance=1E-10)
        if invertir:
            # res = sym.trigsimp(res)
            res = res.inv()
        return res

    def calcJacobian(self, oJv, pJv, pJw):
        """oJv: terna a la cual se calcula la velocidad de movimiento
        pJv: terna en que se proyectan las velocidades lineales
        pJw: terna en que se proyectan las velocidades rotacionales"""
        J=sym.Matrix(sym.MatrixSymbol('J',6,self.nAxis))
        A = self.A

        for qi in range(self.nAxis):#qi = eje de la terna i
            Aojv = self.propagarTerna(qi, oJv) #del eje hasta la terna final
            Apjv = self.propagarTerna(qi, pJv) #del eje hasta la terna proyectada de velocidad
            Apjw = self.propagarTerna(qi, pJw) #del eje hasta la terna proyectada de rotacion w

            p = Aojv[0:3,3]
            for i in range(3): #Itero por eje x, y, z
                if self.jointType[qi] == 'r':
                    rv = Apjv[0:3, i]
                    J[i, qi] = p.cross(rv)[2]
                    rw = Apjw[0:3, i]
                    J[i + 3, qi] = rw[2]
                else:
                    J[i, qi] = Apjw[2,i]
                    J[i + 3, qi] = 0
        J = sym.nsimplify(J,tolerance=1E-10)
        J = sym.trigsimp(J)
        return J

# if isempty(strfind(path, '../;'))
#     addpath('../')
#     addpath('../src')
# end
#
#
# # Calculo de las matrices de la dinamica
# print 'Calculando las matrices de la dinamica\n'
# calcDynamics()
#
# # Calculo del regresor de la dinamica
# # Opcionalmente puedo indicar los parametros conocidos, asi el regresor
# # queda desensablado en 2: K_kn y K_un
# known_params = [m]
# # Calculo el regresor en funcion de las matrices de las dinamicas
# # simplificadas
# calcRegressor
#
# # Reemplazo los valores conocidos para que la s - function dependa solo
# # de las variables de estado
# print 'Reemplazando valores conocidos en el modelo ... '
# mg_eslabon = rg_eslabon. * repmat(masa_eslabon(), 1, 3)';
# for i=1:N_axis:
#     RgRg = getSkew(rg_eslabon(:, i))*getSkew(rg_eslabon(:, i));
#     Iozz_eslabon(i, 1) = Igzz_eslabon(i) - masa_eslabon(i) * RgRg(3, 3);
#
# DH_vals = DH(:, 2)
# DH_vals(joint_type == 'p') = DH(joint_type == 'p', 4)
# DH_vars = d
# DH_vars(joint_type == 'p') = theta(joint_type == 'p')
#
# Ms = subs(M, [a'         DH_vars(DH_vars~=0)'     m'             mxg'              myg'               mzg'I0zz' ], [DH(:, 1)'   DH_vals(DH_vars~=0)'masa_eslabon'  mg_eslabon(1,:)   mg_eslabon(2,:)    mg_eslabon(3,:)     Iozz_eslabon'])
# Ms = vpa(simplify(Ms))
# Cs = subs(C, [a'         DH_vars(DH_vars~=0)'     m'             mxg'              myg'               mzg'I0zz' ], [DH(:, 1)'   DH_vals(DH_vars~=0)'masa_eslabon'  mg_eslabon(1,:)   mg_eslabon(2,:)    mg_eslabon(3,:)     Iozz_eslabon'])
# Cs = vpa(simplify(Cs))
# Gs = subs(G, [a'         DH_vars(DH_vars~=0)'     m'             mxg'              myg'               mzg'I0zz' ], [DH(:, 1)'   DH_vals(DH_vars~=0)'masa_eslabon'  mg_eslabon(1,:)   mg_eslabon(2,:)    mg_eslabon(3,:)     Iozz_eslabon'])
# Gs = vpa(simplify(Gs))
# print('OK\n')
#
# # Reemplazo los valores conocidos de la cinematica
# # para que la funcion del regresor dependa solo
# # de las posiciones y sus derivadas
# print 'Reemplazando valores conocidos en el regresor ... '
# K_kns = subs(K_kn, [a'       ], [DH(:, 1)'  ])
# K_kns = vpa(simplify(K_kns))
# K_uns = subs(K_un, [a'       ], [DH(:, 1)'  ])
# K_uns = vpa(simplify(K_uns));
# print 'OK\n'
#
# # Escribo el archivo con la dinamica
# if length(model_name) > 0:
#     filename = ['auto_sf_' model_name];
#     fprintf('Generando archivo %s.m S-Function nivel 1 para Simulink ... ', filename);
#     writeDynamicModel(filename, q, q_p, Ms, Cs, Gs);
#     print 'OK\n'
#
#     filename = ['auto_reg_' model_name];
#     fprintf('Generando archivo %s.m Regresor de la dinamica ... ', filename);
#     writeDynamicRegressor(filename, K_uns, K_kns, p_kn, masa_eslabon',q,q_p,q_2p);
#     print 'OK\n'
#
#     filename = ['auto_param_' model_name];
#     fprintf('Generando archivo %s.m Parametros del robot para comprobar identificacion ... ', filename);
#
#     # Valores reales de los parametros a estimar
#     params = subs(p_un, [a'         DH_vars(DH_vars~=0)'     m'             mxg'              myg'               mzg'I0zz                         ' ],                          [DH(:, 1)    '   DH_vals(DH_vars~=0)'    masa_eslabon    '  mg_eslabon(1,:)   mg_eslabon(2,:)    mg_eslabon(3,:)     Iozz_eslabon']);
#     params = vpa(simplify(params))
#     writeParams(filename, DH, p_kn, p_kn_val, p_un, p_un_val)
#     #writeParams(filename, DH, params);
#     print 'OK\n'

if __name__ == "__main__":
    ej = 4
    if ej == 1:
        scara = robot("scara")
        scara.help()
        print('Calculando las matrices de DH de las ternas ...')
        for i in range(len(scara.A)):
            print scara.A[i]
        #todo, no se que hace
        # print('Calculando la cinematica directa ...')
        # print scara.calcKinematics()
        print 'OK\n'

    elif ej == 2:
        scara = robot("scara")
        # Calculo el Jacobiano.
        J=scara.calcJacobian(4, 2,2)
        Jred=sym.Matrix([[J[0:3,:]],[J[5,:]]])
        ecuacion = sym.det(Jred)
        q = sym.Matrix(sym.MatrixSymbol('q', 4, 1))
        singularidades = sym.solve(ecuacion, q[1, 0])
        Jsing1 = Jred.subs(q[1, 0], singularidades[0])
        print "J:"
        pprint(J)
        print "\nJred:"
        pprint (Jred)
        print "\ndet:"
        pprint(ecuacion)
        print "\nsingularidades"
        pprint (singularidades)
        print "\nJing1:"
        pprint(Jsing1)
        print "\nnull(Jred)|_{sing1}:"
        pprint(Jsing1.nullspace())
        print "\nDirecciones que no producen movimiento"
        print Jsing1
        J = scara.calcJacobian(4, 4, 4)
        Jred = sym.Matrix([[J[0:3, :]], [J[5, :]]])
        print "\nJ terna 4:"
        pprint(J)
        print "\nJred:"
        pprint(Jred)
    elif ej == 3:
        IRB140 = robot("IRB140")
        J = IRB140.calcJacobian(6,3,4)
        pprint(J)
        J11 = J[0:3,0:3]
        J12 = J[0:3,3:]
        J21 = J[3:,0:3]
        J22 = J[3:,3:]
        print "J11"
        pprint (J11)
        print "J12"
        pprint (J12)
        print "J21"
        pprint (J21)
        print "J22"
        pprint (J22)
    elif ej == 4:
        IRB140 = robot("IRB140")
        J = IRB140.calcJacobian(6,0,0)
        # pprint(J)
        J11 = J[0:3,0:3]
        J12 = J[0:3,3:]
        J21 = J[3:,0:3]
        J22 = J[3:,3:]
        # print "J11"
        # pprint (J11)
        # print "J12"
        # pprint (J12)
        # print "J21"
        # pprint (J21)
        print "J22"
        pprint (J22)
        print "\n partes:"
        print "J22[0,0]"
        pprint(J22[0,0])
        print "J22[1,0]"
        pprint(J22[1,0])
        print "J22[2,0]"
        pprint(J22[2,0])
        print "J22[0,1]"
        pprint(J22[0,1])
        print "J22[1,1]"
        pprint(J22[1,1])
        print "J22[2,1]"
        pprint(J22[2,1])
        print "J22[0,2]"
        pprint(J22[0,2])
        print "J22[1,2]"
        pprint(J22[1,2])
        print "J22[2,2]"
        pprint(J22[2,2])

