function R=RotZ(theta)

R=[cos(theta),-sin(theta),0;
   sin(theta),cos(theta) ,0;
   0         ,0          ,1];
R(abs(R)<0.0001)=0;  %Evita que ponga 0.0000 o 3.12e-22 y pone directamente 0

end