clear
clc

%'MPC' path gen
N = 1000; nx = 14; nu = 6;          
A_path = zeros(nx,nx,N-1);
B_path = randn(nx,nu,N-1) * 0.1;
for k = 1:N-1
    Araw = randn(nx) * 0.3;
    A_path(:,:,k) = Araw / (max(abs(eig(Araw))) + 0.5);
end
Q = eye(nx); R = eye(nu); S_f = eye(nx);

%K grid gen 
elapsed = 0;
for i = 1:100
    tic;
    K_grid = updateKgrid( A_path, B_path, Q, R, S_f, N, nu, nx);
    elapsed = elapsed + toc;
    %fprintf('Recursion for N=%d, nx=%d, nu=%d: %.3f ms\n', N, nx, nu, elapsed*1e3);
end

meanKgentime = elapsed/100;
meanKgenfreq = 1/meanKgentime;

%interpolation based off of x
elapsed = 0;
x = randn(nx,1);
x_MPC = randn(nx,N);
for i = 1:nx
    x_MPC(3, i) = 200 -  0.5*i; %monotonically decreasing height
end
for i = 1:100
    tic;
    [u_TVLQR_val] = u_TVLQR(x, x_MPC, K_grid);
    elapsed = elapsed + toc;
    %fprintf('Recursion for N=%d, nx=%d, nu=%d: %.3f ms\n', N, nx, nu, elapsed*1e3);
end

meaninterptime = elapsed/100;
meaninterpfreq = 1/meaninterptime;