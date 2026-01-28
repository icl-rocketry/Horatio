function val = f(x)
    val = x - 10;
end 

cvx_begin quiet
        variable x
        minimize( x^2 + x^4 )
        subject to
            f(x) <= 0
cvx_end

disp(x)