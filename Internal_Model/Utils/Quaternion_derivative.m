function q_dot = Quaternion_derivative(q, w)
% Calculates the quaternion derivative using derived solution for Omega matrix
    
    % extract rotational velocity components
    wx = w(1);
    wy = w(2);
    wz = w(3);

    % obtain analytical omega matrix
    Omega = [0 , wz , -wy, wx; 
            -wz, 0  , wx , wy; 
             wy, -wx, 0  , wz; 
            -wx, -wy, -wz, 0];

    % find quaternoin derivative
    q_dot = 0.5 * (Omega @ q);
end