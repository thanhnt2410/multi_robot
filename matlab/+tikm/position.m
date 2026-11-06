function [x,y] = position(theta,c)
%POSITION Actual path coordinates r(theta), not Eq. (4)'s ellipse parameter.
x=c.A*cos(c.a*theta);
y=c.B*sin(c.b*theta);
end
