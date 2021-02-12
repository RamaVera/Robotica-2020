function [sol1,sol2]=getEulerAnglesfromR(R,type,phi_singular)
   
    if(length(R)>3)
        disp('Solo se admite matrices de 3x3');
        return
    end
       
    if (strcmp(type,'ZYZ'))
      if(abs(R(3,3))==1)
      %Si R(3,3)==1 implica que es una rotacion pura en Z y que Ry=I
      theta=0;
      phi=phi_singular;
      psi=atan2(R(2,1),R(1,1))-phi;
      sol1=[phi,theta,psi];
      sol2=NaN;
      else
      theta1=getTheta(R(3,3));
      phi1=getPhi(R(2,3),R(1,3),theta1);
      psi1=getPsi(R(3,2),R(3,1),theta1);
      phi2=getPhi(R(2,3),R(1,3),-theta1);
      psi2=getPsi(R(3,2),R(3,1),-theta1);
      
      sol1=[phi1,theta1,psi1];
      sol2=[phi2,-theta1,psi2];
      end
    elseif (strcmp(type,'ZYX'))
       disp("Sin implementar")
       sol1=NaN;
       sol2=NaN;
    else
        disp('Especifique correctamente el tipo de rotacion de Euler');
        error
    end

end



%%%%%%%%%%%%%%%%%%%
function theta=getTheta(v)
theta=acos(v);
end

function phi=getPhi(u,v,theta)
phi=atan2(u/sign(theta),v/sign(theta));
end

function psi=getPsi(u,v,theta)
psi=atan2(u/sign(theta),-v/sign(theta));
end