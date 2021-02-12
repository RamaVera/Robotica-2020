%% TP4 - Punto 1
% ATENCION: POSE0 tiene el parametro de ROLL en grados. El parametro de
% salida q esta en radianes. Sin embargo se imprime al final la conversion
% en grados para facil chequeo.

function q = tp4_punto1(POSE0,conf)
% Tipo de Robot : Scara
% Parametros
a1 = 200; %mm
a2 = 200; %mm
a = [a1,a2];

% POSE0 = [x0; y0; z0; Roll; conf]^T
% Tanto x0,y0,z0 deben estar en mm
% Roll debe estar en grados
p = POSE0(1:3);
Roll = POSE0(4);

%% Inversa de Scara

%% Obtengo q2    
    if(p(1)^2+p(2)^2 > (a1+a2)^2 || p(1)^2+p(2)^2 < (a1-a2)^2 )
        disp('Punto no alcanzable')
        q = NaN;
        return
    end
    
    c2 = ( p(1)^2+p(2)^2-(a1^2+a2^2) )/(2*a1*a2);
    s2 =  conf(1)*sqrt(1-c2^2);
    
    %%%%%%%%%%%%%%%%%%%%
    q(2) = atan2(s2,c2);
    %%%%%%%%%%%%%%%%%%%%
    
    if(abs(q(2))<10^-10)
        q(2)=0;
    end
%% Obtengo q1
    if( p(1)==0 && p(2)==0 )
        disp('Punto no alcanzable')
        q = NaN;
        return
    end
    
    s1 = ( a2*(p(2)*c2 - p(1)*s2) + a1*p(2) )/(p(1)^2 + p(2)^2);
    c1 = ( a2*(p(2)*s2 + p(1)*c2) + a1*p(1) )/(p(1)^2 + p(2)^2);

    %%%%%%%%%%%%%%%%%%%%%
    q(1) = atan2(s1 ,c1);
    %%%%%%%%%%%%%%%%%%%%

    if(abs(q(1))<10^-10)
        q(1)=0;
    end
%% Obtengo q3
    
    %%%%%%%%%%%%
    q(3) = p(3);
    %%%%%%%%%%%%
    
    if(abs(q(3))<10^-10)
            q(3)=0;
    end
%% Obtengo q4

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    q(4) = deg2rad(Roll) -q(1) -q(2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(abs(q(4))<10^-10)
            q(4)=0;
    end
    
    qgrad = [rad2deg(q(1:2)),q(3),rad2deg(q(4))];
    
    disp('Los valores obtenidos de q (rad) son:')
    disp(q)
    disp('Los valores obtenidos de q (deg) son:')
    disp(qgrad)


%% Analizo topes mecanicos
    
% Analizo topes Mecanicos

if  ( q(2) < deg2rad(-150) ) || ...
    ( q(2) > deg2rad(150)  ) || ...
    ( q(3) < -250 )          || ...
    ( q(3) > -50  )

    disp('El robot llego a su tope mecanico')
    if  ( q(2) < deg2rad(-150) ) 
        disp(['q2 se fija en -150 grados'] )
        q(2) = deg2rad(-150);
    end
    if ( q(2) > deg2rad(150)  ) 
        disp(['q2 se fija en 150 grados'])
        q(2) = deg2rad(150);
    end
    if( q(3) < -250 )
        disp(['q3 se fija en -250 mm'] )
        q(3) = -250;
    end
    if ( q(3) > -50 ) 
        disp(['q3 se fija en -50 mm'] )
        q(3) = -50;
    end
end

end