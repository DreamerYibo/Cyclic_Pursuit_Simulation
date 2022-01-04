%%Cyclic Pursuit test1
clc; clear;
map_size = [100,100];
N = 5; %Number of agents
dt = 0.02; % Update period: unit s
t = 0;
t_max =200;

X = zeros(N,1);
Y = zeros(N,1);
Theta = zeros(N,1);
V = zeros(N,1);
Omega = zeros(N,1);
k_alpha = 1;
% k_r = 1;
k_r = (pi/10)*csc(pi/5) + 0.001;

X = 0.5*map_size(1) + 0.3*map_size(1)*(rand(N,1)-0.5);
Y = 0.5*map_size(1) + 0.3*map_size(1)*(rand(N,1)-0.5);
Theta = 2*pi*rand(N,1); %% 0 to 2*pi

figure(1)
% axis([0, map_size(1), 0, map_size(2)]);
axis equal
hold on
for i=1:N
    plot(X(i),Y(i),'o')
end

figure(2)
title

while t<t_max
    temp1 = 0;
    for i=1:N
        X(i) = X(i)+cos(Theta(i))*V(i)*dt;
        Y(i) = Y(i)+sin(Theta(i))*V(i)*dt;
        Theta(i) = Theta(i)+Omega(i)*dt;
        if (Theta(i) < 0)
            Theta(i) = Theta(i) + 2*pi;
        elseif (Theta(i) > 2*pi)
            Theta(i) = Theta(i) - 2*pi;
        end
        if (i == N)
            V(i) = k_r*norm([X(1)-X(i),Y(1)-Y(i)]);
            temp1 = get_orientation(X(i), Y(i),X(1),Y(1)) - Theta(i);
            if (temp1 > pi)
                temp1 = -(2*pi - temp1);
            elseif (temp1 < -pi)
                temp2 = 2*pi + temp1
            end
            Omega(i) = k_alpha*(temp1);
        else
            V(i) = k_r*norm([X(i+1)-X(i),Y(i+1)-Y(i)]);
            temp1 = get_orientation(X(i), Y(i),X(i+1),Y(i+1)) - Theta(i);
            if (temp1 > pi)
                temp1 = -(2*pi - temp1);
            elseif (temp1 < -pi)
                temp2 = 2*pi + temp1
            end
            Omega(i) = k_alpha*(temp1);
        end
        disp(Theta') % debug
    end
    plot(X,Y,'b.');
    pause(0.01);
    t = t+dt;
end