function output = Y(position,velocity,x,y)
%% position,velocity, acceleration, velocity
%% subfunction: generate regression matrix, 2¡Á4
%% biased regression matrix
%% Y(u1,u2,u3,y4) = Y(position,velocity,x,y) = Y(q,dq,x,y)
%% Mx + Cy + g = tau

p = position; p1 = p(1); p2 = p(2); % u1
v = velocity; v1 = v(1); v2 = v(2); % u2
x1 = x(1); x2 = x(2); % u3
y1 = y(1); y2 = y(2); % u4

Y13 = (2*x1+x2)*cos(p2) - (v2*y1+v1*y2+v2*y2)*sin(p2);
Y14 = -(2*x1+x2)*sin(p2) - (v2*y1+v1*y2+v2*y2)*cos(p2);
Y23 = x1*cos(p2) + v2*y1*sin(p2);
Y24 = -x1*sin(p2) + v2*y1*cos(p2);
output = [x1, x2,    Y13, Y14;
           0, x1+x2, Y23, Y24];

end