function [u_mapped] = mapcontrol(u_hf, u_lf, dt_hf, dt_lf, N_hf, N_lf, dt_output, t0_hf)
% Merge higher contoller signals of different frequency and convert to lower controller frequency 

% time step array
outputtime = (t0_hf:dt_output:(t0_hf+N_hf));

% hf: assumes FOH
lowerhf_indexes = floor((outputtime - t0_hf) ./ dt_hf) + 1;
u_mapped_hf = u_hf(:, lowerhf_indexes);

% lf: assume ZOH
t_lf = (0:dt_lf:N_lf);

% obtains lower step side of the lf signal for each output step
lowerlf_indexes = floor((outputtime) ./ dt_lf) + 1;
u_k = u_lf(:,lowerlf_indexes);
t_k = t_lf(lowerlf_indexes);

% obtains higher step side of the lf signal for each output step
upperlf_indexes = ceil((outputtime) ./ dt_lf) + 1;
u_kplus1 = u_lf(:, upperlf_indexes);
t_kplus1 = t_lf(upperlf_indexes);

% maps lf and hf signals to output signal on a different temporal grid
% (note numerical stability addition to prevent NaN collapse
u_mapped_lf = u_k + ((u_kplus1 - u_k) ./ (t_kplus1 - t_k + 1e-12)) .* (outputtime - t_k);

% combine outputs
u_mapped = u_mapped_hf + u_mapped_lf;
end