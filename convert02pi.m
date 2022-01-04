function x = convert02pi(theta)
%%Convert rad to [0, 2pi)
while (any(theta<0, 'all') || any(theta >= 2*pi, 'all'))
    theta(theta<0) = theta(theta<0) + 2*pi;
    theta(theta>= 2*pi) = theta(theta>= 2*pi) - 2*pi;
end
x = theta;
end