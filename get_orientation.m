function theta = get_orientation(X1,Y1, X2, Y2)
%Get the vector's orientation in [0,2pi].
%
temp = atan2(Y2-Y1, X2-X1);
if temp < 0
    theta = temp+ 2*pi;
else
    theta = temp;
end
end

