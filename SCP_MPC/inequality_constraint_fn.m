function [g] = inequality_constraint_fn()
% Defines safety region for the rocket and will return a positive value if violated.

%assume rectangle bound, axes aligned around launchpad

%passed in params:
%boundaries: [x1,x2,y1,y2]
%tightening parameters: [d1, d2, d3, d4];
%current position [x,y]

g = [x - x2 + d1;
    x1 - x + d2;
    y - y2 + d3;
    y1 - y + d4];

end

