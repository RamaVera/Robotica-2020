function J = getJacobianSCARA(q,a)

J(1,1:4) = [-a(1)*sin(q(1)) - a(2)*sin(q(1)+q(2)) , - a(2)*sin(q(1)+q(2)), 0 , 0];
J(2,1:4) = [ a(1)*cos(q(1)) + a(2)*cos(q(1)+q(2)) ,   a(2)*cos(q(1)+q(2)), 0 , 0];
J(3,1:4) = [ 0 ,0 ,-1,0];
J(4:5,1:4) = zeros(2,4);
J(6,1:4) = [ 1 ,1 ,0,-1];
end