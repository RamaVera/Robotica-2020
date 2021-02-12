%% TP4 - Punto 2


function [q,qdot,qddot,sameSegment,Tj] = tp4_punto2(tseg,A,B,C,td,Ts,tacc)
    sameSegment = true;

    if tseg == -tacc
        setTj(0);
    end
          
    Tj = getTj;

    DA = A-B;
    DC = C-B;
    
    if tseg >= (Tj - tacc)
       
        vq1max = 90;
        vq2max = 180;
        vq3max = 1000;
        vq4max = 360;
        vmax = [vq1max,vq2max,vq3max,vq4max]';

        % Como tengo todo en ms divido vmax por 1000
        Tj = max( [max(abs(DA./(vmax / 1000))),2*tacc,td] );
        Tj = ceil(Tj/Ts)*Ts;
        setTj(Tj);
          
    end
   
    tp = tseg + tacc;
    tm = tseg - tacc;
    DCT = DC /Tj;
    DAT = DA /tacc;
    
    if tseg <= tacc
        
        qddot = ( DCT + DAT )/(2*tacc);
        qdot  = ( DCT*tp + DAT*tm )/(2*tacc);
        q     = ( DCT * tp^2 + DAT * tm^2)/(4*tacc) + B;
    
    elseif tseg <= Tj - tacc + Ts
        qddot = zeros(size(B));
        qdot  = DCT;
        q     = DCT * tseg + B;
        
        if tseg == Tj - tacc + Ts
            sameSegment = false;
        end
    else
        disp('Inesperado')
    end
   
end
