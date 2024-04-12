function [Veh] = vehicle (Xest,scale)

if(nargin < 2)
    scale =1.0;
end

%=====================================================================
%   Calculate xy co-ordinates for corners of vehicle triangle
%   Written Matt Rozyn. Started 28th July 2004. Finished .........
%=====================================================================

r1 = (0.25*scale);r2 = (0.125*scale);
% r1 = 150.6;r2 = 150.3;
Veh(1,1) = Xest(1) + r1*cos(Xest(3));     %x1
Veh(1,2) = Xest(2) + r1*sin(Xest(3));     %y1
Veh(2,1) = Xest(1) + r2*cos(Xest(3)+(2/3*pi));  %x2
Veh(2,2) = Xest(2) + r2*sin(Xest(3)+(2/3*pi));  %y2
Veh(3,1) = Xest(1) + r2*cos(Xest(3)+(4/3*pi));  %x3
Veh(3,2) = Xest(2) + r2*sin(Xest(3)+(4/3*pi));  %y3
