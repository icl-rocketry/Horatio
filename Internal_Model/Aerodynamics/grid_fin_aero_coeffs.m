function [C_F, C_M] = grid_fin_aero_coeffs(V_B, w_B, q_fs, x_cg, height_gridfin, Sref_gridfin, Sref_rocket, Lref_rocket, R_body, x_gridfin, GF_actuation, rho, aerosplinefits)
% Calculates the aerodynamic ceoff contributions from the gridfins

    % gridfin angles about x axis
    G_angle_1 = 0.0;
    G_angle_2 = (2/3) * pi;
    G_angle_3 = (4/3) * pi;
    G_angle = [G_angle_1, G_angle_2, G_angle_3];

    % create coefficient list
    C_F = zeros(3, 3);
    C_M = zeros(3, 3);

    % Loop through each fin calculation
    for idx = 1:3
        % distance from cg to geometric gentre of gridfin (approximate cp)
        R_centre = R_body + height_gridfin / 2;
    
        % distance of cg to geometric centre of the grid fin
        r_gridfin = [x_gridfin - x_cg; R_centre * sin(G_angle(idx)); R_centre * cos(G_angle(idx))];
    
        % Use transport theorem to get fin velocity
        V_F_B = V_B + cross(w_B, r_gridfin);
        
        % Body -> Hinge rotation matrices
        RBH = [1, 0, 0; 0, cos(G_angle(idx)), -sin(G_angle(idx)); 0, sin(G_angle(idx)), cos(G_angle(idx))];
    
        % Hinge -> Fin rotation matrices
        RHF = [cos(GF_actuation(idx)), sin(GF_actuation(idx)), 0; -sin(GF_actuation(idx)), cos(GF_actuation(idx)), 0; 0, 0, 1];
      
        % Rotating fin velocities into hinge reference
        V_H = RBH* V_F_B;
    
        % Rotating fin velocities into fin reference
        V_F = RHF * V_H;
    
        % Aero section - approximations used so come back to this when we have wind tunnel data
        % for now grid fins are modelled as quasi-axisymetric which is a
        % valid assumption but is not perfectly accurate so change this
        % when we have data for gridfins in beta and alpha
        
        % get velocities
        U = V_F(1);
        V = V_F(2);
        W = V_F(3);
        
        % get transverse total velocity
        V_transverse = sqrt(V ^ 2 + W ^ 2);

        % get total angle of attack and decomposition angle for y and z
        alpha_total = atan2(V_transverse, -U);
        phi_aero = atan2(W, V);

        % obtain dynamic pressure
        qinf = 0.5 * rho * norm(V_F) ^ 2;

        % get aero coeffs
        CL = ppval(aerosplinefits.CLfit, alpha_total * 180/pi);
        CD = ppval(aerosplinefits.CDfit, alpha_total * 180/pi);

        % get L and D
        L = CL * qinf * Sref_gridfin;
        D = CD * qinf * Sref_gridfin;

        % get axial and transverse forces
        axial_dir = sign(U);
        F_axial = - D * axial_dir * cos(alpha_total) + L * axial_dir * sin(alpha_total);
        F_transverse = - L * cos(alpha_total) - D * sin(alpha_total);

        % decompose forces
        Fx = F_axial;
        Fy = F_transverse * cos(phi_aero);
        Fz = F_transverse * sin(phi_aero);
        F_F = [Fx; Fy; Fz];

        % tranform back to body axes
        F_H = transpose(RHF) * F_F;
        F_B = transpose(RBH) * F_H;
        M_B = cross(r_gridfin, F_B);

        % convert back to coeffients and add to the overall matrix
        C_F(:, idx) = F_B ./ (q_fs * Sref_rocket);
        C_M(:, idx) = M_B ./ (q_fs * Sref_rocket * Lref_rocket);
    end 
end