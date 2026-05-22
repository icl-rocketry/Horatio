function [u_lqr] = LQR_onlineloop(x_current, x_mpc, fsm_state, gainschedules)

%gainschedules: scheduling grids via gridded interpolant offline and load as a struct

%MPC error
dx = x_current - x_mpc;

%split into cases depending on flight phase
if (fsm_state.current_phase == 1) %coast

    %current scheduling variable values:

    %interpolate
    K = gainschedules.coast(aoa, q_inf); 
    u_lqr = K*dx;

elseif (fsm_state.current_phase == 3) %powered descent

    %current scheduling variable values:
 
    %interpolate
    K = gainschedules.powered(abs_V, m);
    u_lqr = K*dx;

else %during relight / error handling
    u_lqr = 0;
end


end