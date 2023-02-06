%%%% Room temperature control, example in ECC

clear variables
clc


%%%% Parameters

room_height = 3; % [m]
room_width = 3; % [m]
room_shared_length = 4; % [m]

h_air = 54000/3600; % [W /K m^2]
rho_air = 1.166; % [kg /m^3]
Cp_air = 1005; % [J/kg K]
mCp = rho_air * (room_height*room_width*room_shared_length) * Cp_air; % [J/K]

K_wood = 748.8/3600; % [W /K m]
K_concrete = 2743.2/3600; % [W /K m] 
K_rock_wool = 140.76/3600; % [W /K m]

d_wood = 0.005; % [m]
d_concrete = 0.03; % [m]
d_rock_wool = 0; % [m]
UA_12 = room_height*room_shared_length/(2/h_air + d_wood/K_wood + d_concrete/K_concrete + d_rock_wool/K_rock_wool); % [W/K]

d_wood = 0.0025; % [m]
d_concrete = 0.03; % [m]
d_rock_wool = 0; % [m]
UA_23 = room_height*room_shared_length/(2/h_air + d_wood/K_wood + d_concrete/K_concrete + d_rock_wool/K_rock_wool); % [W/K]

d_wood = 0.00; % [m]
d_concrete = 0.02; % [m]
d_rock_wool = 0.0; % [m]
UA_out = room_height*room_shared_length/(2/h_air + d_wood/K_wood + d_concrete/K_concrete + d_rock_wool/K_rock_wool); % [W/K]

A = [-UA_12-UA_out, UA_12, 0; UA_12, -UA_12-UA_23, UA_23; 0, UA_23, -UA_23-UA_out]/mCp;
B_bar = [200*eye(3), 300*eye(3), 350*ones(3,1)]/mCp;


% Faster computations
% A = 10*A; B_bar = 10*B_bar;

% eig(A)


if max(real(eig(A))) > 1e-10
    error('System is not stable.')
end
[n, m] = size(B_bar);

for i = 1:m
%     B_bar(:,i) = B_bar(:,i)*U_bar(i,2); % normalizing B_bar
    if rank(ctrb(A, B_bar(:,i))) < n
        error('System is not normal.')
    end
end
U_bar = [-ones(m,1), ones(m,1)]; % normalizing U_bar

failure = 4;
    
B = B_bar; B(:, failure) = [];
C = B_bar(:, failure);
U = U_bar; U(failure, :) = [];
W = U_bar(failure, :);


%%%%%%%%%%%%%%% Quantitative resilience %%%%%%%%%%%%%%%%%
    
x_goal = zeros(n,1);

plot_times = false;
x_0 = [0.8; 0.7; 0.9];

%%%%%%%%%%%%%%%% Nominal Reach Time %%%%%%%%%%%%%%%%%%%%%

[T_Eaton, x_Eaton, ~] = time_optimal_Eaton(A, B_bar, U_bar, x_0, x_goal);
%%% Lost function, to re-implement
% [T_N_lb, T_N_ub] = nominal_reach_time_bounds(A, B_bar, x_0, U_bar, plot_times, T_Eaton);

%%%%%%%%%%%%% Malfunctioning Reach Time %%%%%%%%%%%%%%%%%%%%

[T_Sakawa, x_Sakawa, ~] = time_optimal_Sakawa(A, B, -C, U, W, x_0, x_goal);
%%% Lost function, to re-implement
% [T_M_lb, T_M_ub] = malfunctioning_reach_time_bounds(A, B, C, x_0, U, W, plot_times, T_Sakawa);

%%%%%%%%%%%%% Resilience %%%%%%%%%%%%%%  

rq = T_Eaton/T_Sakawa
% rq_low = T_N_lb/T_M_ub;
% disp([rq_low, rq])

%%% Lost function, to re-implement
% [max_rq_lb, min_rq_ub] = quantitative_resilience_bound(A, B, C, U, W)




