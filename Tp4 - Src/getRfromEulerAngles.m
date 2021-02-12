function [R]=getRfromEulerAngles(sol,type)

   if(length(sol)~=3)
        disp('Solo se admite 3 angulos');
        return
   end
   
   phi=sol(1);
   theta=sol(2);
   psi=sol(3);
  
   if (strcmp(type,'ZYZ'))
       R=RotZ(phi)*RotY(theta)*RotZ(psi);  
   elseif (strcmp(type,'ZYX'))
       R=RotZ(phi)*RotY(theta)*RotX(psi);   
   else
        disp('Especifique correctamente el tipo de rotacion de Euler');
        return
   end
end