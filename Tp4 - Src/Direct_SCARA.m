function [POSE,conf] = Direct_SCARA(q)
POSE=eye(4);
%Segun hoja de datos
%a1=120;
%a2=138;
a1 = 200;
a2 = 200;
TablaDH(1,:) = [q(1),0   ,a1, 0];
TablaDH(2,:) = [q(2),0   ,a2, 0];
TablaDH(3,:) = [0   ,q(3),0 , 0];
TablaDH(4,:) = [q(4),0   ,0 , 0];

    for i=1:length(q)
        A_{i} = getAfromTable(TablaDH(i,:));
        POSE=POSE*A_{i};
    end
    POSE(abs(POSE)<10^-10) = 0 ;
    
    conf(1)=sign(q(2));

    for i=1:length(conf)
        if( conf(i) == 0 )
            disp(['Atencion: Singularidad Detectada en q2'])
            conf(i) = 1;
        end
    end
end