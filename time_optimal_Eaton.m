%%% Time optimal strategy by J.H. Eaton
% based on his paper "An Iterative Solution to Time-Optimal Control", 1962
% Solves the optimal transfer problem for a linear system
% dx/dt = A x(t) + B u(t)  from x(0) = x_0  to x_goal
% The input  u in R^m  is component bounded by   2 x m   matrix U 
% Outputs are the optimal time t,
% final state xt (should be = or close to x_goal)
% and eta to reconstitute the optimal control



function [t, xt, eta] = time_optimal_Eaton(A, B, U, x_0, x_goal)

[n,m] = size(B);
u_min = U(:,1);
u_max = U(:,2);


for j = 1:m
    if rank(ctrb(A, B(:,j))) < n
        disp('System is not normal.')
    end
end

max_iter = 100;

% Initialization
eta = (x_goal - x_0)/norm(x_goal - x_0);
K = 1;
t = 0.01;
E = 100000;

iter = 0;
while iter < max_iter && norm(expm(A*t)*E) > 0.01
    
%     E = expm(-A*t)*x_goal - x_0 - integral(@(s) expm(-A*s)*B_bar*sign(B_bar'*expm(-A'*s)*eta), 0, t, 'ArrayValued',true);
    E = expm(-A*t)*x_goal - x_0 - integral(@(s) expm(-A*s)*B*(u_min + (u_max - u_min).*(sign(B'*expm(-A'*s)*eta)+1)/2), 0, t, 'ArrayValued',true);
    nE = norm(E);
    if nE < 1e-10 % exact termination
        break;
    end
    eta = (eta + K*E/nE)/norm(eta + K*E/nE);
    
    E = expm(-A*t)*x_goal - x_0 - integral(@(s) expm(-A*s)*B*(u_min + (u_max - u_min).*(sign(B'*expm(-A'*s)*eta)+1)/2), 0, t, 'ArrayValued',true);
%     E = expm(-A*t)*x_goal - x_0 - integral(@(s) expm(-A*s)*B_bar*sign(B_bar'*expm(-A'*s)*eta), 0, t, 'ArrayValued',true);
    if E'*eta > 0
        t_new = Newton(A, B, U, t, eta, x_goal, x_0);  
        if t_new < t
            error('t is not increasing.')
        else
            t = t_new;
        end
    else
        K = K/2;
    end
    iter = iter + 1;
    if nE > 1e5
        error("Eaton's algorithm is not converging.")
    end
end

xt = x_goal - expm(A*t)*E;

end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Newton's method to find the first zero of the function f = eta'*E located
% after t.
function t = Newton(A, B_bar, U_bar, t_0, eta, x_goal, x_0)

    u_min = U_bar(:,1);
    u_max = U_bar(:,2);

    t = t_0;
    f = 1;
    repeat = 1;
    while abs(f) > 1e-10 && repeat < 10
        E = expm(-A*t)*x_goal - x_0 - integral(@(s) expm(-A*s)*B_bar*(u_min + (u_max - u_min).*(sign(B_bar'*expm(-A'*s)*eta)+1)/2), 0, t, 'ArrayValued',true);
%         E = expm(-A*t)*x_goal - x_0 - integral(@(s) expm(-A*s)*B_bar*sign(B_bar'*expm(-A'*s)*eta), 0, t, 'ArrayValued',true);
        f = eta'*E;
        df = eta'*(-A*expm(-A*t)*x_goal - expm(-A*t)*B_bar*(u_min + (u_max - u_min).*(sign(B_bar'*expm(-A'*t)*eta)+1)/2));
%         df = eta'*(-A*expm(-A*t)*x_goal - expm(-A*t)*B_bar*sign(B_bar'*expm(-A'*t)*eta));
        t = t - f/df;
        repeat = repeat + 1;
    end
    
    % Bisection method when Newton's method failed to converge
    if abs(f) > 1e-5
        t_sup = t_0;
        f = 1;
        while f > 0
            t_sup = t_sup + 0.1;
            E = expm(-A*t_sup)*x_goal - x_0 - integral(@(s) expm(-A*s)*B_bar*(u_min + (u_max - u_min).*(sign(B_bar'*expm(-A'*s)*eta)+1)/2), 0, t_sup, 'ArrayValued',true);
%             E = expm(-A*t_sup)*x_goal - x_0 - integral(@(s) expm(-A*s)*B_bar*sign(B_bar'*expm(-A'*s)*eta), 0, t_sup, 'ArrayValued',true);
            f = eta'*E;
        end
        t_inf = t_sup - 0.1;
        
        for bisection_iter = 1:10
            t_mid = (t_sup + t_inf)/2;
%             E = expm(-A*t_mid)*x_goal - x_0 - integral(@(s) expm(-A*s)*B_bar*sign(B_bar'*expm(-A'*s)*eta), 0, t_mid, 'ArrayValued',true);
            E = expm(-A*t_mid)*x_goal - x_0 - integral(@(s) expm(-A*s)*B_bar*(u_min + (u_max - u_min).*(sign(B_bar'*expm(-A'*s)*eta)+1)/2), 0, t_mid, 'ArrayValued',true);
            f = eta'*E;
            if f > 0
                t_inf = t_mid;
            else
                t_sup = t_mid;
            end
        end
        t = t_mid;
    end
end