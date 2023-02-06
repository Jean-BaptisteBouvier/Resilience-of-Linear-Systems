%%% Malfunctioning reach time
% "Solution of Linear Pursuit Evasion Games" by Yoshiyuki Sakawa
% Computes the malfunctioning reach time to x_goal starting from x_0
% Dynamics dx/dt = A x(t) + B u(t) + C w(t)    x(0) = x_0 

% Outputs:
% T: malfunctioning reach time defined as the time needed to reach x_goal
% from x_0 when u is optimal to minimize T and w is optimal to maximize T
% x_T: final state, should be close to x_goal
% lambda: vector of costates to recreate both optimal inputs

function [T, x_T, lambda] = time_optimal_Sakawa(A, B, C, U, W, x_0, x_goal)


% [n,~] = size(B);
vertices_U = vertices(U);
vertices_W = vertices(W);

integration_step = 0.01;
min_integration_step = 1e-5;
min_gradient_step = 1e-5;

epsilon = 0.01;
lambda = -x_0/norm(x_0);
repeat = 0;
x_T = x_0;
T = 0;

F = lambda'*(x_0 - x_goal);
dt = integration_step; % discrete integration step for F

while repeat < 20 && norm(x_T - x_goal) > epsilon
    
    u_intg = integral(@(t) lambda'*expm(A*t)*B*max_scalar_prod(B'*expm(A'*t)*lambda, vertices_U), 0, T, 'ArrayValued', true);
    v_intg = integral(@(t) lambda'*expm(A*t)*C*max_scalar_prod(C'*expm(A'*t)*lambda, vertices_W), 0, T, 'ArrayValued', true);
    
    %%% Finding T for this lambda
    while F < -epsilon

        T = T + dt;
        
        K = expm(A*T)*B;
        u_intg = u_intg + dt*lambda'*K*max_scalar_prod(K'*lambda, vertices_U);

        L = expm(A*T)*C;
        v_intg = v_intg + dt*lambda'*L*max_scalar_prod(L'*lambda, vertices_W);

        F_new = lambda'*expm(A*T)*x_0 + u_intg - v_intg - lambda'*x_goal;
        % To improve convergence, reduce time step as F gets close to -epsilon
        if 2*F_new - F > -epsilon && dt > min_integration_step
            dt = dt/10;
        end
        F = F_new;
    end
    
    %%% Gradient descent of F over lambda
    gradient_step = 0.5;
    decreased_step = false;
    
    for iter = 1:20
        
        if iter == 1
            grad_F = expm(A*T)*x_0 - x_goal + integral(@(t) expm(A*t)*B*max_scalar_prod(B'*expm(A'*t)*lambda, vertices_U) - expm(A*t)*C*max_scalar_prod(C'*expm(A'*t)*lambda, vertices_W), 0, T, 'ArrayValued', true);
        end
        F_old = F;
        lambda_old = lambda;
        old_grad_F = grad_F;
        
        lambda = lambda - gradient_step*grad_F;
        lambda = lambda/norm(lambda);
        
        grad_F = expm(A*T)*x_0 - x_goal + integral(@(t) expm(A*t)*B*max_scalar_prod(B'*expm(A'*t)*lambda, vertices_U) - expm(A*t)*C*max_scalar_prod(C'*expm(A'*t)*lambda, vertices_W), 0, T, 'ArrayValued', true);
        F = lambda'*grad_F;
        
        if abs((F - F_old)/F_old) < 1e-4 % less than 0.1% of relative progress is made
            gradient_step = 2*gradient_step; % increase step
           
            if decreased_step || gradient_step > 10
                break % stop the descent when step already too big or has been decreased and then re-increased
            end
        end
        
        % Min has been overshot by a too big gradient_step
        while F > F_old && gradient_step > min_gradient_step
            
            gradient_step = gradient_step/2;
            decreased_step = true; % step size has now been decreased
            lambda = lambda_old - gradient_step*old_grad_F;
            lambda = lambda/norm(lambda);
        
            grad_F = expm(A*T)*x_0 - x_goal + integral(@(t) expm(A*t)*B*max_scalar_prod(B'*expm(A'*t)*lambda, vertices_U) - expm(A*t)*C*max_scalar_prod(C'*expm(A'*t)*lambda, vertices_W), 0, T, 'ArrayValued', true);
            F = lambda'*grad_F;
        end
    end
   

    repeat = repeat + 1;
    
    
    [~, X] = ode45(@(t,x) A*x + B*max_scalar_prod(B'*expm(A'*(T-t))*lambda, vertices_U) - C*max_scalar_prod(C'*expm(A'*(T-t))*lambda, vertices_W), [0 T], x_0);
    x_T = X(end,:);
end



end


%%%%%%%%%%%%%%%%%%%%%%%%%%%% Functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Maximize scalar product <a,u> over all u in the set of vertices of U
function max_u = max_scalar_prod(a, V)

max_scalar_prod = -Inf;
for u = V
    sc = a'*u;
    if sc > max_scalar_prod
        max_scalar_prod = sc;
        max_u = u;
    end
end
end

% % Compute F for fmincon
% function F = F_function(lambda, A,B,C,T,x_0,x_goal, vertices_U, vertices_W)
%     
%     u_intg = integral(@(t) expm(A*t)*B*max_scalar_prod(B'*expm(A'*T)*lambda, vertices_U), 0, T, 'ArrayValued', true);
%     v_intg = integral(@(t) expm(A*t)*C*max_scalar_prod(C'*expm(A'*T)*lambda, vertices_W), 0, T, 'ArrayValued', true);
%     grad_F = expm(A*T)*x_0 - x_goal + u_intg - v_intg;
%     F = lambda'*grad_F;
% end
% 
% % Compute F and its gradient for fmincon
% function [F, grad_F] = F_function_and_grad(lambda, A,B,C,T,x_0,x_goal, vertices_U, vertices_W)
%     
%     u_intg = integral(@(t) expm(A*t)*B*max_scalar_prod(B'*expm(A'*T)*lambda, vertices_U), 0, T, 'ArrayValued', true);
%     v_intg = integral(@(t) expm(A*t)*C*max_scalar_prod(C'*expm(A'*T)*lambda, vertices_W), 0, T, 'ArrayValued', true);
%     grad_F = expm(A*T)*x_0 - x_goal + u_intg - v_intg;
%     F = lambda'*grad_F;
% end
% 
% % Norm constraint for the optimization
% function [c, ceq] = norm_constraint(lambda)
% c = [];
% ceq = lambda'*lambda - 1;
% end