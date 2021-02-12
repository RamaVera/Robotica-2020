function q = Inv_SCARA(POSE,conf,Qanterior,dim)

    a1=dim(1);
    a2=dim(2);
    n=POSE(1:end-1,1);
    s=POSE(1:end-1,2);
    a=POSE(1:end-1,3);
    p=POSE(1:end-1,4);
    q1Anterior = Qanterior(1);
  
%% Obtengo q2    
if(p(1)^2+p(2)^2 > (a1+a2)^2 || p(1)^2+p(2)^2 < (a1-a2)^2 )
    disp('Punto no alcanzable')
    q = NaN;
    return
end
    c2 = ( p(1)^2+p(2)^2-(a1^2+a2^2) )/(2*a1*a2);
    s2 =  conf(1)*sqrt(1-c2^2);
q(2) = atan2(s2,c2);
    if(abs(q(2))<10^-10)
        q(2)=0;
    end
%% Obtengo q1
    if( p(1)==0 && p(2)==0 )
        disp('Singularidad detectada, se asigna q1 con q anterior') 
q(1) = q1Anterior;    
    else
        s1 = ( a2*(p(2)*c2 - p(1)*s2) + a1*p(2) )/(p(1)^2 + p(2)^2);
        c1 = ( a2*(p(2)*s2 + p(1)*c2) + a1*p(1) )/(p(1)^2 + p(2)^2);
q(1) = atan2(s1 ,c1);
        if(abs(q(1))<10^-10)
            q(1)=0;
        end
    end
%% Obtengo q3
q(3) = p(3);
    if(abs(q(3))<10^-10)
            q(3)=0;
    end
%% Obtengo q4
    c124 = n(1);
    s124 = n(2);
q(4) = atan2(s124,c124)-q(1)-q(2);
    if(abs(q(4))<10^-10)
            q(4)=0;
    end
end