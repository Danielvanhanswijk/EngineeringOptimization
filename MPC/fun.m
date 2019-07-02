function cost = fun(x, H, h, const, dim,i)
    disp(i)
        u = x(1:(dim.nu*dim.N));
        cost = 0.5*u'*H*u + h'*u + const;
    
end