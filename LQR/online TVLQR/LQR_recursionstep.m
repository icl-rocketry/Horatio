function [K_k, S_k] = LQR_recursionstep(A_k, B_k, S_kplus1, Q, R)

    %find gain:

    K_k = (R + B_k'*S_kplus1*B_k) \ (B_k'*S_kplus1*A_k);

    %compute new S matrix corresponding to gridpoint k via DT difference
    %ricatti equation

    S_k = Q + A_k'*S_kplus1*A_k - (A_k'*S_kplus1*B_k) * K_k;

    %nuance between conjugate and non-conjugate transpose if matrices
    %contain complex numbers? - should all be real anyway

    %need to enforce S_k symmetry?

    


end
