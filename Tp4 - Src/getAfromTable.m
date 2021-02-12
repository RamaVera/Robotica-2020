function A=getAfromTable(rowOfTableDH)
%% IMPORTARTE rowOfTableDH = [theta d a alpha]
% Pasar los angulos siempre en radianes
theta = rowOfTableDH(1);
d = rowOfTableDH(2);
a = rowOfTableDH(3);
alpha = rowOfTableDH(4);

A(1,1)=cos(theta); 
A(1,2)=-sin(theta)*cos(alpha);
A(1,3)=sin(theta)*sin(alpha); 
A(1,4)=a*cos(theta);

A(2,1)=sin(theta); 
A(2,2)=cos(theta)*cos(alpha);
A(2,3)=-cos(theta)*sin(alpha); 
A(2,4)=a*sin(theta);

A(3,1)=0; 
A(3,2)=sin(alpha);
A(3,3)=cos(alpha); 
A(3,4)=d;

A(4,:)=[0,0,0,1];


end