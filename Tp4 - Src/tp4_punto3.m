%% TP4 - Punto 3


function [q,qdot,qddot,sameSegment,POSEActual,Tj] = tp4_punto3(tseg,A,B,C,td,Ts,tacc,qant,qdotant,maxCartesianV)

% Armo DBA(-tacc)
    DBA = B\A;
    RBA = DBA(1:3,1:3);
    pBA = DBA(1:3,4);
% Obtengo angulos de Euler
% sol1=[phi1,theta1,psi1];
    sol1 = getEulerAnglesfromR(RBA,'ZYZ',0);
    phiBA   = sol1(1);
    thetaBA = sol1(2);
    psiBA   = sol1(3);
%Armo ThetaA
    ThetaA = [pBA;thetaBA;psiBA];
% Armo DBC(Tj)
    DBC = B\C;
    RBC = DBC(1:3,1:3);
    pBC = DBC(1:3,4);
% Obtengo angulos de Euler
% sol1=[phi1,theta1,psi1];
    sol1 = getEulerAnglesfromR(RBC,'ZYZ',0);
    phiBC   = sol1(1);
    thetaBC = sol1(2);
    psiBC   = sol1(3);
%Armo ThetaC
    ThetaC = [pBC;thetaBC;psiBC];
%Armo ThetaB
    ThetaB = zeros(size(ThetaC));


% Idem interpolacion joint pero modificado para poner phi
sameSegment = true;
if tseg == -tacc
    setTj(0);
end
    Tj = getTj;

    DA = ThetaA;
    DC = ThetaC;

if tseg >= (Tj - tacc)

%     vq1max = 90;
%     vq2max = 180;
%     vq3max = 1000;
     vq4max = 360;
%     vmax = [vq1max,vq2max,vq3max,vq4max]';

    % Como tengo todo en ms divido vmax por 1000
    vmax = [maxCartesianV(1:3)',deg2rad(vq4max),deg2rad(vq4max)]';
    Tj = max( [max(abs(DA./(vmax / 1000))),2*tacc,td] );
    %Tj = max( [2*tacc,td] );
    Tj = ceil(Tj/Ts)*Ts;
    setTj(Tj);

end

tp = tseg + tacc;
tm = tseg - tacc;
DCT = DC /Tj;
DAT = DA /tacc;

if tseg <= tacc %Zona1

    %ThetaDDot = ( DCT + DAT )/(2*tacc);
    %ThetaDot  = ( DCT*tp + DAT*tm )/(2*tacc);
    Theta     = ( DCT .* tp^2 + DAT .* tm^2)/(4*tacc) ;
    Phi       = ( phiBC - phiBA )*tp/(2*tacc) + phiBA;
    

elseif tseg <= Tj - tacc + Ts %Zona2
    %ThetaDDot = zeros(size(B));
    %ThetaDot  = DCT;
    Theta     = DCT .* tseg;
    Phi       = phiBC;

    if tseg == Tj - tacc + Ts
        sameSegment = false;
    end
else
    disp('Inesperado')
end  

%Armo D
R = getRfromEulerAngles([Phi,Theta(4,1),Theta(5,1)],'ZYZ');

D(1:3,1:3) = R;
D(1:3, 4 ) = Theta(1:3);
D( 4 ,1:3) = [0,0,0];
D( 4 , 4 ) = 1;

%Obtengo Pose actual
POSEActual = B*D;

% A partir del problema cinematico inverso obtengo q
q = Inv_SCARA(POSEActual,1,0,[200,200]);
    
qdot = ( q - qant )/Ts ;
qddot = ( qdot - qdotant )/Ts;

end
