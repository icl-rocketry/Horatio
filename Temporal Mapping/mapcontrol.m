function [u_mapped] = mapcontrol(u_hf, u_lf, dt_hf, dt_lf, horizon_hf, horizon_lf, dt_output, t0_hf)
%designed to take a row array of control signals, each a column vector 

outputtime = (t0_hf:dt_output:(t0_hf+horizon_hf));

%hf:
lowerhf_indexes = floor((outputtime - t0_hf)./dt_hf) + 1; %indexes of output points corresponding to hf point before them
u_mapped_hf = u_hf(:,lowerhf_indexes);

%lf:
t_lf = (0:dt_lf:horizon_lf);

lowerlf_indexes = floor((outputtime)./dt_lf) + 1;
u_k = u_lf(:,lowerlf_indexes);
t_k = t_lf(lowerlf_indexes);

upperlf_indexes = ceil((outputtime)./dt_lf) + 1;
u_kplus1 = u_lf(:,upperlf_indexes);
t_kplus1 = t_lf(upperlf_indexes);

u_mapped_lf = u_k + ((u_kplus1-u_k)./(t_kplus1 - t_k)).*(outputtime - t_k);

nanindeces = isnan(u_mapped_lf); %for if t_kplus1 = t_k = t, floor and ceil give same point, finding linear interpolation at a lf point
u_mapped_lf(nanindeces) = u_k(nanindeces);

u_mapped = u_mapped_hf + u_mapped_lf;

