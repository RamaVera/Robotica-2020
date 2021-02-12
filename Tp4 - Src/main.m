%% En este codigo seran invocadas y probadas las funciones 
clear all 
close all
clc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%        Trabajo Practico No.4      %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%      Generacion de Trayectorias   %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%                  2020             %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Punto 1

% Test 1 : Funcionamiento de la funcion -> tp4_punto1.m
% Para testear rellenar q_ref_deg con los valores de q1,q2,q4 en rad y q3
% en mm.
% El test realizara el problema directo para obtener los parametros de
% x0,y0,z0 y obtendra Roll. Se espera que la funcion tp4_punto1 devuelva lo
% mismo que se introdujo en el vector q_ref_deg

%Directo
% q_ref_deg= [q1,q2,q3,q4] <- COMPLETAR CON LO QUE SE DESEE PROBAR

%q_ref_deg=[0,0,-50,0]
%q_ref_deg=[0,0,-50,-90]
q_ref_deg=[0,90,-50,-90]
q_ref_rad=[deg2rad(q_ref_deg(1)),deg2rad(q_ref_deg(2)),-50,deg2rad(q_ref_deg(4))]

% En caso de singularidad el sistema siempre opta por conf=1 (codo +)
[POSE,conf] = Direct_SCARA(q_ref_rad);

%Inverso
x0=POSE(1,4);
y0=POSE(2,4);
z0=POSE(3,4);
Roll = q_ref_deg(1)+ q_ref_deg(2) + q_ref_deg(4) ;

% Test del punto 1
q = tp4_punto1([x0,y0,z0,Roll],conf);    

%% Punto2
clear all
close all
clc

POSE1 = [-200; 200;-100; 0; 1];
POSE2 = [200; 200;-200; 90; 1];

qa = tp4_punto1(POSE1(1:4),POSE1(5));
qb = tp4_punto1(POSE2(1:4),POSE2(5));

% Entiendo como que el robot tiene que tocar el punto por lo que duplico el
% valor de la posicion de los ejes de manera que "se detenga" en los puntos
Qd = [qa',qa',qb',qb',qa',qa'];
k = 0;
k = k + 1 ;qa = Qd(:,k);
k = k + 1 ;qb = Qd(:,k);
k = k + 1 ;qc = Qd(:,k);

Td =  [1000 1000 1000 1000 1000];
a = 0;
a = a + 1 ;td = Td(a);

tacc = 200;

proccessAll = true;
sameSegment = true;
tseg = - tacc;
n = -tacc;

qOb        = [];
qdotOb     = [];         
qddotOb    = [];
N = [];
TjObs = [];

Ts = 10; %ms

while(proccessAll)
      
    [q,qdot,qddot,sameSegment,Tj]=tp4_punto2(tseg,qa,qb,qc,td,Ts,tacc);
    tseg = tseg + Ts ;
    n = n + Ts;
       
    if sameSegment == false
        k = k + 1 ;
        %Cambio de segmento
        if k > length(Qd)
            %No tengo mas segmentos que analizar
            proccessAll = false;
        else
            %Defino nuevo segmento
            qa = q;
            qb = qc;
            qc = Qd(:,k);
            
            a = a + 1;
            td = Td(a);
            
            tseg = -tacc + Ts;
            TjObs  = [TjObs, Tj];
        end
    end

% Ploteo en T real dentro del while para debug
% Solo se plotea la variacion angular con las referencias en el tiempo
% transcurrido. Desventaja: Plotear ralentiza la ejecucion del algoritmo

%     figure(1)
%     subplot(4,1,1)
%         hold on
%         scatter(n,rad2deg(qb(1)),'r.');
%         scatter(n,rad2deg(qc(1)),'b.');
%         scatter(n,rad2deg(q(1)),'kx')
%         ylabel('Grados')
%         %xlabel('ms')
%         %title('q1')
%         
%     subplot(4,1,2)
%         hold on
%         plot(n,rad2deg(qb(2)),'-r');
%         plot(n,rad2deg(qc(2)),'-y');
%         plot(n,rad2deg(q(2)),'xk')
%         ylabel('Grados')
%         %xlabel('ms')
%         %title('q2')
%         
%     subplot(4,1,3)
%         hold on
%         plot(n,qb(3),'xr');
%         plot(n,qc(3),'xy');
%         plot(n,q(3),'xk')
%         ylabel('mm')
%         %xlabel('ms')
%         %title('q3')
%         
%     subplot(4,1,4)
%         hold on
%         plot(n,rad2deg(qb(4)),'xr');
%         plot(n,rad2deg(qc(4)),'xy');
%         plot(n,rad2deg(q(4)),'xk') 
%         ylabel('Grados')
%         %xlabel('ms')
%         %title('q4')
        
% Almaceno la evolucion de las variables de manera de graficarlas
% todas juntas al finalizar el algoritmo. Sin embargo se pueden
% graficar en tiempo real descomentando la seccion anterior

 qOb        = [qOb , q]        ;
 qdotOb     = [qdotOb , qdot]  ;       
 qddotOb    = [qddotOb , qddot];
 N =[N n];

             
end

% Graficos

figure(1)
set(gcf, 'WindowState', 'maximized');
subplot(4,1,1)
    plot(N,rad2deg(qOb(1,:)),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grados')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q1')
subplot(4,1,2)
    plot(N,rad2deg(qOb(2,:)),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grados')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q2')
subplot(4,1,3)
    plot(N,qOb(3,:),'kx')
    plotBorders(TjObs,tacc)
    ylabel('mm')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q3')
subplot(4,1,4)
    plot(N,rad2deg(qOb(4,:)),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grados')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q4')
suptitle('Variacion temporal de los parametros q del Scara')
    
figure(2)
set(gcf, 'WindowState', 'maximized');
subplot(4,1,1)
    plot(N,rad2deg(qdotOb(1,:)*1000),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grados/s')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q1 punto')
subplot(4,1,2)
    plot(N,rad2deg(qdotOb(2,:)*1000),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grados/s')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q2 punto')
subplot(4,1,3)
    plot(N,qdotOb(3,:)*1000,'kx')
    plotBorders(TjObs,tacc)
    ylabel('mm/s')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q3 punto')
subplot(4,1,4)
    plot(N,rad2deg(qdotOb(4,:)*1000),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grado/s')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q4 punto')
suptitle('Variacion temporal de los parametros q punto del Scara')

    
figure(3)
set(gcf, 'WindowState', 'maximized');
subplot(4,1,1)
    plot(N,rad2deg(qddotOb(1,:)*1000^2),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grado/s^2')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q1 2puntos')
subplot(4,1,2)
    plot(N,rad2deg(qddotOb(2,:)*1000^2),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grado/s^2')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q2 2puntos')
subplot(4,1,3)
    plot(N,qddotOb(3,:)*1000^2,'kx')
    plotBorders(TjObs,tacc)
    ylabel('mm/s^2')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q3 2puntos')
subplot(4,1,4)
    plot(N,rad2deg(qddotOb(4,:)*1000^2),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grado/s^2')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q4 2puntos')
suptitle('Variacion temporal de los parametros q dos puntos del Scara')

    
figure(4)
set(gcf, 'WindowState', 'maximized');
for k =1:length(qOb)
    q = qOb(:,k);
    [A,conf]=Direct_SCARA(q);
    px = A(1,4);
    py = A(2,4);
    hold on
    if  abs(px - 200)<0.001 && abs(py - 200)<0.001 
        plot(px,py,'yx','MarkerSize',20)
    elseif abs(px - (-200))<0.001  && abs(py - 200)<0.001   
        plot(px,py,'gx','MarkerSize',20)
    else
        if k < floor(length(qOb)/2)
            p1=plot(px,py,'ro'); %Ida
        else
            p2=plot(px,py,'bo'); %Vuelta
        end
    end
end
    grid on
    title('Trayectoria del origen de la terna 4 (TCP) Scara')
    legend([p1,p2],{'Ida','Vuelta'})


%% Punto 3
clear all
%close all
clc


POSE1 = [-200; 200;-100; 0; 1];
POSE2 = [200; 200;-200; 90; 1];

qa = tp4_punto1(POSE1(1:4),POSE1(5));
qb = tp4_punto1(POSE2(1:4),POSE2(5));

Ja = getJacobianSCARA(qa,[200,200]);
Jb = getJacobianSCARA(qb,[200,200]);

vq1max = 90;
vq2max = 180;
vq3max = 1000;
vq4max = 360;
vmax = [vq1max,vq2max,vq3max,vq4max]';
vmaxrad = [deg2rad(vmax(1:2))',vq3max,deg2rad(vq4max)]';

maxA = Ja *vmaxrad;
maxB = Jb *vmaxrad;
maxCartesianV = abs(min(maxA,maxB));
 


[POSEA,conf] = Direct_SCARA(qa);
[POSEB,conf] = Direct_SCARA(qb);

Qd = {POSEA,POSEA,POSEB,POSEB,POSEA,POSEA};
k = 0;
k = k + 1 ;A = Qd{1};
k = k + 1 ;B = Qd{k};
k = k + 1 ;C = Qd{k};

Td =  [1000 1000 1000 1000 1000];
a = 0;
a = a + 1 ;td = Td(a);

tacc = 200;

proccessAll = true;
sameSegment = true;
tseg = - tacc;
n = -tacc;

qOb        = [];
qdotOb     = [];         
qddotOb    = [];
N = [];
TjObs = [];

Ts = 10; %ms

qant    =  10;
qdotant =  100;

while(proccessAll)
      
    [q,qdot,qddot,sameSegment,POSEActual,Tj]=tp4_punto3(tseg,A,B,C,td,Ts,tacc,qant,qdotant,maxCartesianV);
    tseg = tseg + Ts ;
    n = n + Ts;
       
    qant    =  q;
    qdotant =  qdot;
    
    if sameSegment == false
        k = k + 1 ;
        %Cambio de segmento
        if k > length(Qd)
            %No tengo mas segmentos que analizar
            proccessAll = false;
        else
            %Defino nuevo segmento
            A = POSEActual;
            B = C;
            C = Qd{k};
            
            a = a + 1;
            td = Td(a);
            
            tseg = -tacc + Ts;
            TjObs  = [TjObs, Tj];
        end
    end

% Ploteo en T real dentro del while para debug
% Solo se plotea la variacion angular con las referencias en el tiempo
% transcurrido. Desventaja: Plotear ralentiza la ejecucion del algoritmo

%     figure(1)
%     subplot(4,1,1)
%         hold on
%         scatter(n,rad2deg(qb(1)),'r.');
%         scatter(n,rad2deg(qc(1)),'b.');
%         scatter(n,rad2deg(q(1)),'kx')
%         ylabel('Grados')
%         %xlabel('ms')
%         %title('q1')
%         
%     subplot(4,1,2)
%         hold on
%         plot(n,rad2deg(qb(2)),'-r');
%         plot(n,rad2deg(qc(2)),'-y');
%         plot(n,rad2deg(q(2)),'xk')
%         ylabel('Grados')
%         %xlabel('ms')
%         %title('q2')
%         
%     subplot(4,1,3)
%         hold on
%         plot(n,qb(3),'xr');
%         plot(n,qc(3),'xy');
%         plot(n,q(3),'xk')
%         ylabel('mm')
%         %xlabel('ms')
%         %title('q3')
%         
%     subplot(4,1,4)
%         hold on
%         plot(n,rad2deg(qb(4)),'xr');
%         plot(n,rad2deg(qc(4)),'xy');
%         plot(n,rad2deg(q(4)),'xk') 
%         ylabel('Grados')
%         %xlabel('ms')
%         %title('q4')
        
% Almaceno la evolucion de las variables de manera de graficarlas
% todas juntas al finalizar el algoritmo. Sin embargo se pueden
% graficar en tiempo real descomentando la seccion anterior

 qOb        = [qOb , q']        ;
 qdotOb     = [qdotOb , qdot']  ;       
 qddotOb    = [qddotOb , qddot'];
 N =[N n];

             
end

% Graficos

figure(5)
set(gcf, 'WindowState', 'maximized');
subplot(4,1,1)
    plot(N,rad2deg(qOb(1,:)),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grados')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q1')
subplot(4,1,2)
    plot(N,rad2deg(qOb(2,:)),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grados')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q2')
subplot(4,1,3)
    plot(N,qOb(3,:),'kx')
    plotBorders(TjObs,tacc)
    ylabel('mm')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q3')
subplot(4,1,4)
    plot(N,rad2deg(qOb(4,:)),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grados')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q4')
suptitle('Variacion temporal de los parametros q del Scara')
    
figure(6)
set(gcf, 'WindowState', 'maximized');
subplot(4,1,1)
    plot(N(2:end),rad2deg(qdotOb(1,2:end)*1000),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grados/s')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q1 punto')
subplot(4,1,2)
    plot(N(2:end),rad2deg(qdotOb(2,2:end)*1000),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grados/s')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q2 punto')
subplot(4,1,3)
    plot(N(2:end),qdotOb(3,2:end)*1000,'kx')
    plotBorders(TjObs,tacc)
    ylabel('mm/s')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q3 punto')
subplot(4,1,4)
    plot(N(2:end),rad2deg(qdotOb(4,2:end)*1000),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grado/s')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q4 punto')
suptitle('Variacion temporal de los parametros q punto del Scara')

    
figure(7)
set(gcf, 'WindowState', 'maximized');
subplot(4,1,1)
    plot(N(3:end),rad2deg(qddotOb(1,3:end)*1000^2),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grado/s^2')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q1 2puntos')
subplot(4,1,2)
    plot(N(3:end),rad2deg(qddotOb(2,3:end)*1000^2),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grado/s^2')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q2 2puntos')
subplot(4,1,3)
    plot(N(3:end),qddotOb(3,3:end)*1000^2,'kx')
    plotBorders(TjObs,tacc)
    ylabel('mm/s^2')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q3 2puntos')
subplot(4,1,4)
    plot(N(3:end),rad2deg(qddotOb(4,3:end)*1000^2),'kx')
    plotBorders(TjObs,tacc)
    ylabel('Grado/s^2')
    xlabel(['Ts = ',num2str(Ts),' [ms]'])
    title('Q4 2puntos')
suptitle('Variacion temporal de los parametros q dos puntos del Scara')

    
figure(8)
set(gcf, 'WindowState', 'maximized');
for k =1:length(qOb)
    q = qOb(:,k);
    [A,conf]=Direct_SCARA(q);
    px = A(1,4);
    py = A(2,4);
    hold on
    if  abs(px - 200)<0.0001 && abs(py - 200)<0.0001 
        plot(px,py,'yx','MarkerSize',20)
    elseif abs(px - (-200))<0.0001  && abs(py - 200)<0.0001   
        plot(px,py,'gx','MarkerSize',20)
    else
        if k < floor(length(qOb)/2)
            p1=plot(px,py,'ro'); %Ida
        else
            p2=plot(px,py,'bo'); %Vuelta
        end
    end
end
    grid on
    title('Trayectoria del origen de la terna 4 (TCP) Scara')
    legend([p1,p2],{'Ida','Vuelta'})


%% Funciones Auxiliares para ploteo
 function plotBorders(TjObs,tacc)
    hold on
 	xline(-tacc,'b-.','Tj0-tacc','DisplayName','Tj1');
    xline(0,'-.','Tj0','DisplayName','Tj1');
    xline(tacc,'b-.','Tj0+tacc','DisplayName','Tj1');
    sum =0;
    for k=1:length(TjObs)
        sum = TjObs(k)+ sum;
        xline(sum-tacc,'b-.',['T',num2str(k),'-tacc'],'DisplayName','Tj1');
        xline(sum,'-.',['T',num2str(k)],'DisplayName','Tj1');
        xline(sum+tacc,'b-.',['T',num2str(k),'+tacc'],'DisplayName','Tj1');
    end
    
 end

