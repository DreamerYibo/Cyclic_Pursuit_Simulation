%%Cyclic Pursuit using the linear formation control method.
clc; clear;
map_size = [100,100];
N = 5; %Number of agents
dt = 0.05; % Update period: unit s
t = 0;
t_max =20;

Rot = @(t) [cos(t) -sin(t) ;sin(t) cos(t)] ;
Relative_mat = eye(N,N); % Get the relative pos from X and Y
Relative_mat(:,2:end) = Relative_mat(:,2:end) -1*eye(N,N-1);
Relative_mat(N,1) = -1;

Rot = @(t) [cos(t) -sin(t) ;sin(t) cos(t)] ;

X = zeros(N,t_max/dt); % X t_i
Y = zeros(N,t_max/dt);
R = zeros(N,t_max/dt);
Theta = zeros(N,t_max/dt);

V = zeros(N,1);
Omega = zeros(N,1);

X(:,1) = 0.5*map_size(1) + 0.3*map_size(1)*(rand(N,1)-0.5);
Y(:,1) = 0.5*map_size(1) + 0.3*map_size(1)*(rand(N,1)-0.5);
Theta = 2*pi*rand(N,1); %% 0 to 2*pi
R(1:N-1,1) = sqrt((X(2:N,1)-X(1:N-1,1)).^2 + (Y(2:N,1)-Y(1:N-1,1)).^2);
R(N,1) = sqrt((X(1,1)-X(N,1)).^2 + (Y(1,1)-Y(N,1)).^2);

I_N = eye(N,N); %Ajacency mat for the diagraph
A_D = [I_N(:,N), I_N(:,1:N-1)];
Delta_D = I_N;
Lap_D = Delta_D-A_D; % Which is actually the same as Relative_mat
D_D_mod = [eye(N-1,N-1);-ones(1,N-1)]; % D_D_mod*transpose(D_D) = Lap_D

z_ref = zeros(N-1,2); % Reference (Target) relative states.
D_D = -Relative_mat(:,2:N); % Incidence matrix corresponding to one spanning tree graph of the D
Pos = Gen_Polygon_formation(0.5*map_size, 0.1*map_size(1), N, 0);
z_ref(:,1) = transpose(D_D) * Pos(:,1);
z_ref(:,2) = transpose(D_D) * Pos(:,2);

k_gain = 1; %Gain

rgb_list = rand(N,3);
figure(1)
% axis([0, map_size(1), 0, map_size(2)]);
axis equal
hold on
for i=1:N
    temp_h(i) = plot(X(i),Y(i),'x', 'MarkerEdgeColor', rgb_list(i,:), 'linewidth', 10 );
    str_array(i) = "Agent"+num2str(i);
end
legend(temp_h, num2cell(str_array),'AutoUpdate','off');

dxy_new_temp = zeros(2,N)

for k = 2: t_max/dt
    % noise = randn(N,2);
    % ex_t = z_ref(:,1) - transpose(D_D)*(noise(:,1) + X(:,k-1));
    % ey_t = z_ref(:,2) - transpose(D_D)*(noise(:,2) + Y(:,k-1));

    % x_dot = k_gain*D_D*ex_t;
    % y_dot = k_gain*D_D*ey_t;

    x_dot = -k_gain*Lap_D*X(:,k-1) + k_gain*D_D_mod*z_ref(:,1);
    y_dot = -k_gain*Lap_D*Y(:,k-1) + k_gain*D_D_mod*z_ref(:,2);
    if (k==2)
        V = 0; %Rotate only at this step
        Theta(:,k) = atan2(y_dot,x_dot);
        X(:,k) = X(:,k-1);
        Y(:,k) = Y(:,k-1);
    else
        V = sqrt(x_dot.^2 + y_dot.^2);
        if (all(round(V,4) == 0))
            V(:) = 0;
        end
        temp_theta = atan2(y_dot,x_dot);
        Omega = convert02pi(temp_theta - Theta(:,k-1))./dt;

        if (round(Omega,8) ~= 0)
            dx_temp = (1./Omega).*V.*sin(Omega*dt); % Relative to the fixed frame of each agent.
            dy_temp = (1./Omega).*V.*(1-cos(Omega*dt));
            for i=1:N
                dxy_new_temp(:,i) = Rot(Theta(i,k-1))*[dx_temp(i);dy_temp(i)];
            end
            X(:,k) = X(:,k-1) + transpose(dxy_new_temp(1,:));
            Y(:,k) = Y(:,k-1) + transpose(dxy_new_temp(2,:));
        else
            X(:,k) = X(:,k-1)+cos(Theta(:,k-1)).*V.*dt;
            Y(:,k) = Y(:,k-1)+sin(Theta(:,k-1)).*V.*dt;
        end
        Theta(:,k) = temp_theta;
    end
    R(1:N-1,k) = sqrt((X(2:N,k)-X(1:N-1,k)).^2 + (Y(2:N,k)-Y(1:N-1,k)).^2);
    R(N,k) = sqrt((X(1,k)-X(N,k)).^2 + (Y(1,k)-Y(N,k)).^2);
end

figure(2)
hold on
for i=1:N
    temp_h(i) = plot(Theta(i,:), 'Color', rgb_list(i,:), 'linewidth', 2 );
    % str_array(i) = "Agent"+num2str(i);
end
legend(temp_h, num2cell(str_array));

figure(4)
hold on
for i=1:N
    temp_h(i) = plot(R(i,:), 'Color', rgb_list(i,:), 'linewidth', 2 );
end
legend(temp_h, num2cell(str_array));

figure(1)
axis equal
hold on

sleep_factor = 1; % Prevent the bugs of matlab

for k = 1:3*sleep_factor:t_max/dt
    figure(1)
    for i = 1:N
        h = animatedline('Marker','.','MarkerEdgeColor', rgb_list(i,:));
        addpoints(h,X(i,k),Y(i,k));
    end
    figure(1)
    drawnow limitrate nocallbacks
    pause(dt*sleep_factor);
end
drawnow




