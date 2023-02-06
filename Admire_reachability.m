% ADMIRE reachability (ECC extended paper)

% ADMIRE simulation parameters
% 
% STATES:
% v      1 velocity (m/s)
% alpha  2 angle of attack (rad)
% beta   3 sideslip angle (rad)
% p      4 body axis roll rate (rad/s)
% q      5 body axis pitch rate (rad/s)
% r      6 body axis yaw rate (rad/s)
% psi    7 heading angle (rad)
% theta  8 pitch attitude (rad)
% phi    9 roll attitude (rad)
% 
% CONTROLS:
%  1 Right Canard, rc (rad)
%  2 Left Canard, lc (rad)
%  3 Right Outboard Elevon, roe (rad)
%  4 Right Inboard Elevon, rie (rad)
%  5 Left Outboard Elevon, lie (rad)
%  6 Left Outboard Elevon, loe (rad)
%  7 Rudder, rud (rad)
%  8 Leading edge flaps (rad)
%  9 Landing Gear (0-1) [0 - gear up, 1 - gear down]
% 10 Thrust Command (0-1)
% 11 Yaw Thrust Vectoring (rad)
% 12 Pitch Thrust Vectoring (rad)

clc
clear variables


%%%%% Model with 9 states, as the effect of the pitch on the velocity is
%%%%% not negligible due to gravity. Position has no effect though
Mach = 0.3; % altitude = 2000 m
A = [-0.0208061645292079,-4.65122160888159,0.374232664859475,-7.99599275680407e-08,-0.302963397090739,7.99600385903432e-08,0,-9.80999427778406,-0.000559972056723979;-0.00191640084378191,-0.781353095364520,0.00901995284143491,-6.87730379372517e-09,0.973942296730210,6.87732146622058e-09,0,-4.88170785196951e-05,-4.81628666417736e-05;0,0,-0.188184960915832,0.117120786225936,-3.65150659580356e-51,-0.982094560134300,3.06190291202048e-50,-9.54617795801398e-100,0.0972490605972614;0,0,-15.4676182512047,-1.50103533020663,-3.73320275124422e-50,0.543143859886488,3.13040770303800e-49,-9.75975720752659e-99,-0.00436574758199082;0.000610808752105219,4.18198173082101,-0.00613314615282962,-3.10670994211385e-05,-0.777772308292197,3.10670994183314e-05,0,8.34897132560321e-06,8.23708432736892e-06;0,0,0.951382199706097,-0.0932100008410710,2.53458516393180e-50,-0.335835015383815,-2.12532922797551e-49,6.62619671903138e-99,0.00296403913425199;0,0,0,0,0,1.00676875459870,0,0,0;0,0,0,0,1,0,0,0,0;0,0,0,1,0,0.116547523509605,0,0,0];
B_bar = [-1.42042676365169,-1.42042676365169,-0.927171070762340,-1.42757288184381,-1.42757288184381,-0.927171070762340,-0.307532766010299,0.448846619230885,-0.703180119671139,6.01815430830781,-15.3048650281407,-50.9506691491745;-0.000549876720983788,-0.000549876720983788,-0.0566965119412683,-0.0909778752323707,-0.0909778752323707,-0.0566965119412683,0.000357863410678141,0.00520822790871878,0.00179181907856105,-0.00699627606405290,0.0178803821307243,-3.04799422687635;-0.00633411409606306,0.00633411409606306,0.00353654046969741,0.0148728219315439,-0.0148728219315439,-0.00353654046969741,0.0441553601137058,-1.37664366754032e-51,4.29199939146374e-101,-1.33812831967177e-150,3.08653189849630,1.88358943903008e-16;0.849549651283458,-0.849549651283458,-5.20922170298741,-4.48930840943736,4.48930840943736,5.20922170298741,3.03540858873301,-1.40744369270775e-50,4.38802546734135e-100,-1.36806663043070e-149,-18.2188628376002,-1.11182578896268e-15;1.54036208811258,1.54036208811258,-1.25649744402598,-2.01079851628958,-2.01079851628958,-1.25649744402598,0.00519059193586529,-0.137932956532161,-0.143274324668438,-0.101401906195261,0.259657357025434,-190.001762258710;-0.435552469139453,0.435552469139453,-0.230910554336885,-0.492831044428952,0.492831044428952,0.230910554336885,-1.83801900441132,9.55556432454017e-51,-2.97916426981405e-100,9.28822144364958e-150,-153.034955897234,-9.33912352796015e-15;0,0,0,0,0,0,0,0,0,0,0,0;0,0,0,0,0,0,0,0,0,0,0,0;0,0,0,0,0,0,0,0,0,0,0,0];

A = round(100*A)/100;
n = length(A(:,1));
v = Mach*343; % Mach * speed of sound in m/s
X_eq_rad = [v; 6.647*pi/180; 0; 0; 0; 0; 0; 6.647*pi/180; 0];
X_eq_deg = [v; 6.647; 0; 0; 0; 0; 0; 6.647; 0];
u_eq = [-0.03; -0.03; 1.5; 1.5; 1.5; 1.5; 0; 0; 0; 0];%*pi/180;

% Remove landing gear and thrust command as they are not symmetric
B_bar(:, 9:10) = [];
U_bar = [-25*pi/180, 25*pi/180;  % right canard
         -25*pi/180, 25*pi/180;  % left canard
         -25*pi/180, 25*pi/180;  % right outboard elevon
         -25*pi/180, 25*pi/180;  % right inboard elevon
         -25*pi/180, 25*pi/180;  % left inboard elevon
         -25*pi/180, 25*pi/180;  % left outboard elevon
         -30*pi/180, 30*pi/180;  % rudder
         -10*pi/180, 10*pi/180;  % leading edge flap
         -2*pi/180,  2*pi/180;   % yaw thrust vectoring
         -2*pi/180,  2*pi/180];  % pitch thrust vectoring

m = length(U_bar(:,2));
% Scaling B_bar to create zonotope
for j = 1:m
    B_bar(:,j) = B_bar(:,j)*U_bar(j,2);
end
B_bar = round(100*B_bar)/100;
     
for failure = 3 % id of the actuator lost

    B = B_bar; B(:, failure) = [];
    C = B_bar(:, failure);
    BU = zonotope(zeros(n,1), B);
    CW = zonotope(zeros(n,1), C);
    if ~in(BU, CW)
        warning('The jet is not resilient to the loss of actuator %i.', failure)
    end
end


%%%% Actuation with delay %%%%
tau = 1;
eAtau_C = expm(A*tau)*C;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%% 2D projection of BU and CW along the axes %%%%%
% for plot_dim = [1;6]% [1,2,3,4,5; 2,3,4,5,6]
%     figure
%     hold on
%     grid on
%     plot(BU, [plot_dim(1), plot_dim(2)], 'b', 'LineWidth',2);
%     plot(CW, [plot_dim(1), plot_dim(2)], 'r', 'LineWidth',2);
%     
%     set(gca,'fontsize', 18);
%     axis_labels(plot_dim);
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% zonotopes must be of full dimension so projected in 6 dim for the
% difference
Z = minkDiff(project(BU, 1:6), project(CW, 1:6)); % approximated Minkowksi difference from CORA package for zonotopes
Z = zonotope([Z.Z; zeros(3, length(Z.Z(1,:)))]);
g = generators(Z); % = Z.Z(:,2:end);

T = 0.2;

% X = [ V, alpha, beta, p, q, r, psi, theta, phi]
x0 = X_eq_rad + [0; 0; 0; 20*pi/180; 0; 0; 0; 0; 0];
x_goal = X_eq_rad;

dt = 0.03;
Phi = expm(A*dt);
EAB = zonotope(zeros(n,1), integral(@(t) expm(A*(dt - t))*g, 0, dt, 'ArrayValued', true));
R = Phi*x0 + EAB;
t = dt;

plot_dim = [9; 4]; % roll rate as a function of roll
figure; hold on;  grid on;
scatter(x_goal(plot_dim(1)), x_goal(plot_dim(2)), 20, 'red', 'filled')
scatter(x0(plot_dim(1)), x0(plot_dim(2)), 20, 'blue', 'filled')
    
while t + dt < T  
    
    plot(R, [plot_dim(1), plot_dim(2)], 'Color', [0 t/T 1-t/T], 'LineWidth',2);
   
    axis_labels(plot_dim);
    
    R = Phi*R + EAB;
    t = t + dt;
    
end
plot(R, [plot_dim(1), plot_dim(2)], 'Color', [0 t/T 1-t/T], 'LineWidth',2);





for plot_dim = [1,3,5,7,9; 2,4,6,8,1]

    figure
    hold on
    grid on
    plot(R, [plot_dim(1), plot_dim(2)], 'b', 'LineWidth',2);
    scatter(x_goal(plot_dim(1)), x_goal(plot_dim(2)), 20, 'red', 'filled')
    axis_labels(plot_dim);
end


if in(R, x_goal)
    fprintf('x_goal is resiliently reachable.\n')
else
    fprintf('x_goal is NOT resiliently reachable.\n')
end


%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%


function axis_labels(plot_dim)

    set(gca,'fontsize', 18);
    switch plot_dim(1) 
        case 1
            xlabel('velocity (m/s)')
        case 2
            xlabel('angle of attack (rad)')
        case 3
            xlabel('sideslip angle (rad)')
        case 4
            xlabel('roll rate (rad/s)')
        case 5
            xlabel('pitch rate (rad/s)')
        case 6
            xlabel('yaw rate (rad/s)')
        case 7
            xlabel('heading angle (rad)')
        case 8
            xlabel('pitch angle (rad)')
        case 9
            xlabel('roll angle (rad)')
        otherwise
            error('plot_dim must be an integer in [1,9].')
    end
    switch plot_dim(2) 
        case 1
            ylabel('velocity (m/s)')
        case 2
            ylabel('angle of attack (rad)')
        case 3
            ylabel('sideslip angle (rad)')
        case 4
            ylabel('roll rate (rad/s)')
        case 5
            ylabel('pitch rate (rad/s)')
        case 6
            ylabel('yaw rate (rad/s)')
        case 7
            ylabel('heading angle (rad)')
        case 8
            ylabel('pitch angle (rad)')
        case 9
            ylabel('roll angle (rad)')
        otherwise
            error('plot_dim must be an integer in [1,9].')
    end
end