function R=RotY(theta)

R=[cos(theta)   ,0 ,sin(theta);
            0   ,1 ,          0;
   -sin(theta)  ,0 ,cos(theta)];
R(abs(R)<0.0001)=0;  %Evita que ponga 0.0000 o 3.12e-22 y pone directamente 0

end