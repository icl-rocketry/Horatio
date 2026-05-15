function K_i = computepointgain(x_i, u_i, x_add_i, Qd, Rd, Ts, rocket_config)

%find CT Jacobians via finite differencing
t_i = 0;%why do we need time?
[x_dot, x_add_dot] = dynamics_fn(t_i, x_i, x_add_i, u_i, rocket_config);

A = zeros(length(x_dot), length(x_i));
B = zeros(length(x_dot), length(u_i));

epsilon = 1e-6; %set depending on variable?

for column = 1:length(x_i)
    e_x_vector = zeros(length(x_i),1);
    e_x_vector(column) = epsilon; %for column i, perturb the ith component of the x vector
    A(:,column) = (dynamics_fn(t_i, x_i + e_x_vector, x_add_i, u_i, rocket_config) - dynamics_fn(t_i, x_i - e_x_vector, x_add_i, u_i, rocket_config)) / (2*epsilon);
end

for column = 1:length(u_i)
    e_u_vector = zeros(length(u_i),1);
    e_u_vector(column) = epsilon; %for column i, perturb the ith component of the u vector    
    B(:,column) = (dynamics_fn(t_i, x_i, x_add_i, u_i + e_u_vector, rocket_config) - dynamics_fn(t_i, x_i, x_add_i, u_i - e_u_vector, rocket_config)) / (2*epsilon);
end

%CT to DT:
%A_d = expm(A.*Ts);
%B_d = expm() - could use Van Loan's matrix-exponential method
system_CT = ss(A, B);
system_DT = c2d(system_CT, Ts, 'zoh');    
Ad = system_DT.A;
Bd = system_DT.B;

%Controlability and stability checks:
eig_Ad = eig(Ad);

for i = 1:length(eig_Ad)
    
    if abs(eig_Ad(i)) > 1 %test for DT instability?
        error('linearised system is unstable');
    end

    if rank([eig_Ad(i).*eye(length(x_i)) - Ad, Bd], 1e-8) == length(x_i) %small tolerance 1e-8 
        continue
    else
        error('Linear system does not pass Hautus Lemma for controllability')
    end

end


%solve DAR
[K_i,S_i,P_i] = dlqr(Ad, Bd, Qd, Rd, 0);

%Q and R need to be discrete time cost matrices, how to define? 




end