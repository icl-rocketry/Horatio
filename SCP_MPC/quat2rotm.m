function R_BI = quat2rotm(q)
% Calculates the body to inertial axes rotation matrix 
    
    % extract quaternion components
    qx = q(1); 
    qy = q(2); 
    qz = q(3); 
    qw = q(4);
    
    % calculate rotational matrix for body to inertial axes
    R_BI = [1 - 2 * ((qy ^ 2) + (qz ^ 2)), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw);
            2 * (qx * qy + qz * qw), 1 - 2 * ((qx ^ 2) + (qz ^ 2)), 2 * (qy * qz - qx * qw);
            2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * ((qx ^ 2) + (qy ^ 2))];
end

