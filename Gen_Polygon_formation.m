function Pos = Gen_Polygon_formation(center,radius,num_of_edges,offset)
    %Generate all positions of vertices for a specified polygon in 2D
    %center: 1x2 array
    %offset unit:rad
N = num_of_edges;
r = radius;
Pos = zeros(N,3);% x y theta
Rot = @(t) [cos(t) -sin(t) ;sin(t) cos(t)];

Pos(1,1:2) = Rot(offset)*[1*r;0];
delta_theta = 2*pi/N;
Pos(:,3) = offset + (pi/2 + delta_theta*(0:N-1));
Pos(:,3) = convert02pi(Pos(:,3)); % Convert to 0 to 2pi

for i=2:N
    Pos(i,1:2) = Rot(delta_theta)*transpose(Pos(i-1,1:2));
end

for i=1:N
    Pos(i,1:2) = center+Pos(i,1:2);
end

    
end
    
    