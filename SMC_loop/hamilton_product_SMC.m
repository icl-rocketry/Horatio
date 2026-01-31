function [q_new] = hamilton_product_SMC(qa, qb)
% function for the SMC layer that computes the hamilton product of 2 quaternions.
    wa = qa(1); xa = qa(2); ya = qa(3); za = qa(4);
    wb = qb(1); xb = qb(2); yb = qb(3); zb = qb(4);

    M_qa = [wa, -xa, -ya, -za; 
            xa,  wa, -za,  ya; 
            ya,  za,  wa, -xa; 
            za, -ya,  xa,  wa];
    
    q_new = M_qa * [wb; xb; yb; zb];
end