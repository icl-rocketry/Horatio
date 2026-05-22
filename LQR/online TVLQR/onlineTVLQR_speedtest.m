clear
clc

N = 1000; nx = 14; nu = 6;          
A_path = zeros(nx,nx,N-1);
B_path = randn(nx,nu,N-1) * 0.1;
for k = 1:N-1
    Araw = randn(nx) * 0.3;
    A_path(:,:,k) = Araw / (max(abs(eig(Araw))) + 0.5);
end
Q = eye(nx); R = eye(nu); S_f = eye(nx);

tic;
K_grid = TVLQR_alongMPC_Kgen( A_path, B_path, Q, R, S_f, N, nu, nx);
elapsed = toc;
fprintf('Recursion for N=%d, nx=%d, nu=%d: %.3f ms\n', N, nx, nu, elapsed*1e3);