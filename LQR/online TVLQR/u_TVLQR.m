function [u_TVLQR_val] = u_TVLQR(x, x_MPC, K_grid)

    %proxy for time along path via height (monotonically decreasing)
    height = x(3);
    h_MPC = x_MPC(3, :);

    k = find(h_MPC >= height, 1, 'last');  %last index above current height, current height and x between nodes k and k+1
    %edge cases:
    if isempty(k), k = 1; end                  
    if k >= length(h_MPC), k = length(h_MPC)-1; end  

    %linearly interpolate K
    K = ((K_grid(:,:,k+1) - K_grid(:,:,k)) / (h_MPC(k+1) - h_MPC(k)))*(height - h_MPC(k)) + K_grid(:,:,k); %add stability term?

    %linearly interpolate reference - need to treat quaternions
    %differently?
    x_ref = ((x_MPC(:, k+1) - x_MPC(:, k)) / (h_MPC(k+1) - h_MPC(k)))*(height - h_MPC(k)) + x_MPC(:, k); %add stability term?

    u_TVLQR_val = - K*(x - x_ref); %add to u_MPC

    %add stability term in case h_MPC(k+1) - h_MPC(k) = 0?
end