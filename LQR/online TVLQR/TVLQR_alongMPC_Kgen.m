function [K_grid] = TVLQR_alongMPC_Kgen( A_path, B_path, Q, R, S_f, N, nu, nx)

%find K at each node along path via backward iteration from last point
%terminal cost

K_grid = zeros(nu,nx,N-1); %x 14 dimensional?

%backward recursion from final path point to first
S_kplus1 = S_f;

    for nodeindex = N-1:-1:1

        A_k = A_path(:,:,nodeindex);
        B_k = B_path(:,:,nodeindex);
        
        [K_k, S_k] = LQR_recursionstep(A_k, B_k, S_kplus1, Q, R);

        K_grid(:,:, nodeindex) = K_k;
        S_kplus1 = S_k;
        
    end

end
