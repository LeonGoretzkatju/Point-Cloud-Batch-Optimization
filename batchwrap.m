%% Coding by Yingyu Wang

% function nu = batchwrap(alpha);
%
% 
% batch limits angles to -pi +pi


function nu = batchwrap(alpha)

nn = size(alpha,1);

for i=1:nn

	if (alpha(i) > pi)
		alpha(i) = alpha(i) - 2 * pi;
    elseif (alpha(i) < -pi)
		alpha(i) = alpha(i) + 2 * pi;
    end

end
nu = alpha;
end
