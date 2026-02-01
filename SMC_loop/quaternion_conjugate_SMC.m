function [q_star] = quaternion_conjugate_SMC(q)
% obtains quaternion conjugate
    % decompose quaternion into components
    w = q(1); 
    x = q(2); 
    y = q(3); 
    z = q(4);
    
    % define quaternion conjugate
    q_star = [w; -x; -y; -z];
end

