function R = theta2R(theta)
% For coordinate transformation, x2 = R*x1 + t 
% set the counterclockwise as the positive direction.
% the R represents from i th frame to world frame
R = [cos(theta) -sin(theta);
    sin(theta) cos(theta)];
end