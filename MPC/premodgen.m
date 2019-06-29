function [H,h,const,T_state,S_state] = premodgen(LTI,param, dim, weight)

    for i = 0:dim.N-1
            T_output(i*dim.ny+1:(i+1)*dim.ny,:) = LTI.C*LTI.A^i;   
    end

    S_output = zeros(dim.ny*dim.N,dim.nu*dim.N);

    for k = 1:dim.N-1
        for i = 0:k-1
            S_output(k*dim.ny+1:(k+1)*dim.ny, i*dim.nu+1:(i+1)*dim.nu) = LTI.C*LTI.A^(k-1-i)*LTI.B;       
        end
    end

    % State matrices
    for i = 0:dim.N-1
            T_state(i*dim.nx+1:(i+1)*dim.nx,:) = LTI.A^i;   
    end

    S_state = zeros(dim.nx*dim.N,dim.nu*dim.N);

    for k = 1:dim.N-1
        for i = 0:k-1
            S_state(k*dim.nx+1:(k+1)*dim.nx, i*dim.nu+1:(i+1)*dim.nu) = LTI.A^(k-1-i)*LTI.B;       
        end
    end

    Qfbar = LTI.C*weight.Qf*LTI.C';
    weight.Q = LTI.C*weight.Q*LTI.C';
    Qbar = blkdiag(kron(eye(dim.N-1),weight.Q),Qfbar);
    Rbar = kron(eye(dim.N),weight.R);
    
    H = Rbar + S_output'*Qbar*S_output;
    h = (LTI.x0'*T_output'*Qbar*S_output - LTI.Yr'*Qbar*S_output)';
    const = 0.5*(LTI.x0'*T_output'*Qbar*T_output*LTI.x0 + ...
    LTI.x0'*T_output'*LTI.Yr + ...
    LTI.Yr'*Qbar'*T_output*LTI.x0 + LTI.Yr'*LTI.Yr);
end