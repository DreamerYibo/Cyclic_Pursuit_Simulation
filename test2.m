%%Cyclic Pursuit test2
% Set resolution and error toggles before simulation!
clc; clear;
map_size = [100,100]; % cm
N = 5; %Number of agents
dt = 0.02; % Update period: unit s
t = 0;
t_max =70;

Rot = @(t) [cos(t) -sin(t) ;sin(t) cos(t)] ;
Relative_mat = eye(N,N); % Get the relative pos from X and Y
Relative_mat(:,2:end) = Relative_mat(:,2:end) -1*eye(N,N-1);
Relative_mat(N,1) = -1;

X = zeros(N,t_max/dt);
Y = zeros(N,t_max/dt);
x_measured = zeros(N,1);
y_measured = zeros(N,1);
xy_res = 2000;
xy_error_toggle = 0;
xy_sig = 1e-2*(1+randn(N,1)) % Standard deviation
xy_miu = 1e-3*randn(N,1)

R = zeros(N,t_max/dt);
r_measured = zeros(N,1); % Calculated by the measured x and y.
target_r = 17;
Alpha = zeros(N,t_max/dt);
Beta = zeros(N,t_max/dt);

Theta = zeros(N,1);
theta_measured = zeros(N,1); % Measured Theta's values.
theta_res = 2000;

V = zeros(N,1);
Omega = zeros(N,1);
k_alpha = 1;
% k_r = 1;
k_r = (pi/(2*N))*csc(pi/N);

omega_res = 1000; % the resolution of omega.
omega_error_toggle = 0;% Turn it on if taking the errors of omega of the actuators into account.
omega_sig = 5*1e-2*(1+randn(N,1))
omega_miu = 1e-3*randn(N,1)
v_res = 1000;
v_error_toggle = 0;
v_sig = 5*1e-2*(1+randn(N,1))
v_miu = 1e-3*randn(N,1)

Omega_logged = zeros(N,t_max/dt); % Use this to store the corresponding data.
V_logged = zeros(N,t_max/dt);

rand_flag = false; %Whether to save the data % Do not comment this line

X(:,1) = 0.5*map_size(1) + 0.3*map_size(1)*(rand(N,1)-0.5);
Y(:,1) = 0.5*map_size(1) + 0.3*map_size(1)*(rand(N,1)-0.5);
Theta = 2*pi*rand(N,1); %% 0 to 2*pi
rand_flag = true;
saved_data = [X(:,1),Y(:,1),Theta]; % This would be saved if the agents do not converge to the polygon formation

% Pos = Gen_Polygon_formation(0.5*map_size, 17, N, 0);
% X(:,1) = Pos(1:end,1);
% Y(:,1) = Pos(1:end,2);
% Theta = Pos(1:end,3);

% load("test2_fixed_speed_mess_up.mat");
% X(:,1) = saved_data(:,1);
% Y(:,1) = saved_data(:,2);
% Theta = saved_data(:,3);

R(1:N-1,1) = sqrt((X(2:N,1)-X(1:N-1,1)).^2 + (Y(2:N,1)-Y(1:N-1,1)).^2);
R(N,1) = sqrt((X(1,1)-X(N,1)).^2 + (Y(1,1)-Y(N,1)).^2);

Q = zeros(N,2);
Q_ = zeros(N,2);
Q(:,1) = Relative_mat*X(:,1); % [x1-x2, y1-y2; ....]
Q(:,2) = Relative_mat*Y(:,1);
Alpha(:, 1) = atan2(Q_(:,2), Q_(:,1)) - Relative_mat*Theta;
Beta(:, 1) = -pi +  Relative_mat*Theta;
if (rand_flag == true && N==2) % customize theta when there are 2 agents.
    % Change the position of agent 2 to make alpha_1 = -alpha_2. The W_italic set.
    % r = sqrt(Q(:,1).^2 + Q(:,1).^2); % C
    % temp_dtheta = (Theta(2)-Theta(1));
    % temp_vec = [cos(temp_dtheta)-1; sin(temp_dtheta)-0];
    % temp_pos = Rot(Theta(1))*temp_vec+[X(1,1);Y(1,1)];
    % X(2,1) = temp_pos(1);
    % Y(2,1) = temp_pos(2);

    % Change the k_r
    k_r = pi/4 * rand; % 0~pi/4
    % k_r = 5*pi/4 * rand; 
    % k_r = pi * rand + pi/4; 
end

rgb_list = rand(N,3);
figure(1)
clf(figure(1))
% axis([0, map_size(1), 0, map_size(2)]);
axis equal
hold on
for i=1:N
    temp_h(i) = plot(X(i),Y(i),'x', 'MarkerEdgeColor', rgb_list(i,:), 'linewidth', 10 );
    str_array(i) = "Agent"+num2str(i);
end
legend(temp_h, num2cell(str_array),'AutoUpdate','off');

dxy_new_temp = zeros(2,N);

for k = 2: t_max/dt %%% debug
    temp1 = 0;
    % X(:,k) = X(:,k-1)+cos(Theta).*V.*dt;
    % Y(:,k) = Y(:,k-1)+sin(Theta).*V.*dt;
    %% Use another way to get X and Y (Assume omega and
    %% V does not change in each iter and calculate X and Y by curve)
    if (round(Omega,8) ~= 0)
        dx_temp = (1./Omega).*V.*sin(Omega*dt); % Relative to the fixed frame of each agent.
        dy_temp = (1./Omega).*V.*(1-cos(Omega*dt));
        
        for i=1:N
            dxy_new_temp(:,i) = Rot(Theta(i))*[dx_temp(i);dy_temp(i)];
        end
        
        X(:,k) = X(:,k-1) + transpose(dxy_new_temp(1,:));
        Y(:,k) = Y(:,k-1) + transpose(dxy_new_temp(2,:));
    else
        X(:,k) = X(:,k-1)+cos(Theta).*V.*dt;
        Y(:,k) = Y(:,k-1)+sin(Theta).*V.*dt;
    end
    % delta Omega has minimum value. errors of actuator may accur
    Theta = Theta + Omega*dt; % Actual Theta

    R(1:N-1,k) = sqrt((X(2:N,k)-X(1:N-1,k)).^2 + (Y(2:N,k)-Y(1:N-1,k)).^2); % Actural R 
    R(N,k) = sqrt((X(1,k)-X(N,k)).^2 + (Y(1,k)-Y(N,k)).^2);
    
    x_measured = round(X(:,k), xy_res) + xy_error_toggle*x_measured.*(randn(N,1).*xy_sig+xy_miu);
    y_measured = round(Y(:,k), xy_res) + xy_error_toggle*y_measured.*(randn(N,1).*xy_sig+xy_miu);

    Q(:,1) = Relative_mat*x_measured; % [x1-x2, y1-y2; ....]
    Q(:,2) = Relative_mat*y_measured;
    r_measured = sqrt(Q(:,1).^2+Q(:,2).^2);
    theta_measured = round(Theta, theta_res);
    for i=1:N
        if (i == N)
            Q_(i,:) = transpose(Rot(theta_measured(1)))* transpose(Q(i,:));
        else
            Q_(i,:) = transpose(Rot(theta_measured(i+1)))*transpose(Q(i,:));
        end
    end
    Alpha(:, k) = atan2(Q_(:,2),Q_(:,1)) +pi-Relative_mat*theta_measured;
    Beta(:, k) = -pi +Relative_mat*theta_measured;
    
    % %modified control law 1
    % alpha_0 = (pi - 2*pi/N)/2;
    % d_Alpha = (Alpha(:,k)-Alpha(:,k-1))/dt;
    % V = k_r*r_measured - 0.5*(1./(10*abs(d_Alpha)+1)).*(r_measured-target_r);

    % %modified control law 2
    % alpha_0 = pi/N;
    % d_Alpha = (Alpha(:,k)-Alpha(:,k-1))/dt;
    % % V = target_r*k_alpha*alpha_0/(2*sin(alpha_0))*ones(N,1);
    % V = 4*ones(N,1);

    %modified control law 3
    % alpha_0 = pi/N;
    % temp_sign = sign(convertn1p1pi(Alpha(:,k))); % Guess if the agent is moving clock-wise or counter clock-wise
    % d_Alpha = (convert02pi(Alpha(:,k))-convert02pi(Alpha(:,k-1)))/dt;
    % V_s = target_r*k_alpha*alpha_0/(2*sin(alpha_0))*ones(N,1); % Target V

    % temp_func = exp(-(convertn1p1pi(Alpha(:,k)) - temp_sign*alpha_0).^2); % Like normal distribution
    % V = (1-temp_func).*k_r.*r_measured + temp_func.*V_s;

    %Original one
    V = k_r*r_measured;
    V = round(V,v_res) + v_error_toggle*V.*(randn(N,1).*v_sig+v_miu);

    % disp("alpha");
    % convertn1p1pi(Alpha(:,k));

    % Omega = k_alpha*(convertn1p1pi(Alpha(:,k))) - sign(Omega)*0.1.*((r_measured-target_r)./target_r);
    Omega = k_alpha*(convertn1p1pi(Alpha(:,k)));
    Omega = round(Omega,omega_res)+omega_error_toggle*Omega.*(randn(N,1).*omega_sig + omega_miu); %Assuem that the error is proportional to the value of omega
    
    Omega_logged(:,k) = Omega;
    % d_r = -k_r*( R(1:N-1,k).*cos(Alpha(1:N-1,k)) + R(2:N,k).*cos(Alpha(1:N-1,k)+Beta(1:N-1,k)) );
    % d_r(end+1) = -k_r*( R(N,k).*cos(Alpha(N,k)) + R(1,k).*cos(Alpha(N,k)+Beta(N,k)) );
end

Alpha = convertn1p1pi(Alpha);
Beta = convert02pi(Beta);

if (rand_flag)
    if(any(diff(R(:,end))>0.5) || any((R(:,end)-R(:,end-1)).^2 > 1) || any(R(:,end)>1e5))
        file_name = "test2_"+get_clock_str(clock());
        save(file_name+".mat", 'saved_data');
    end
end

figure(1)

axis equal
hold on

figure(4)
clf(figure(4))
plot(Omega_logged');
title('Omega')

sleep_factor = 3; % Prevent the bugs of matlab

figure(2)
clf(figure(2))
hold on
subplot(3,1,1)
ylabel("r")
subplot(3,1,2)
ylabel("\alpha")
subplot(3,1,3)
ylabel("\beta")
xlabel("t")

for i = 1:N
    subplot(3,1,1)
    hold on
    plot(dt:dt:t_max,R(i,:),'LineWidth',2,'color', rgb_list(i,:))
    subplot(3,1,2)
    hold on
    plot(dt:dt:t_max,Alpha(i,:),'LineWidth',2,'color', rgb_list(i,:))
    subplot(3,1,3)
    hold on
    plot(dt:dt:t_max,Beta(i,:),'LineWidth',2,'color', rgb_list(i,:))
end
hold off

for k = 1:5*sleep_factor:t_max/dt
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


