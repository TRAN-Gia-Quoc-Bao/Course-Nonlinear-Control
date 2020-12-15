function [af, bf, diffeof] = computeLinearization(f, h, x, u)
% Compute the I/O feedback linearization functions, that is a(x) and b(x)
% such that L^r_fh = a(x) + b(x)u, and the associated diffeomorphism.
% the system must be SISO and affine in the control variable.
%
% Arguments :
% f is the vector field (vector of symbolic expressions)
% h symbolic expression defining the output
% x symbolic vector variable
% u symbolic control variable
%
% License : BSD
% author: Olivier Huber <olivier.huber@inria.fr>

L_f_h = h;
done = 0;
i = 0;
n = numel(f);
diffeoFormula(1) = h;

% derive until the control input is found or the system is not I/O linearizable.
while (i<n && ~done)
    % compute the Lie derivative along f
    dh_x = jacobian(L_f_h, x);
    L_f_h = dh_x*f;
    % check whether u is in the expression
    dh_u = symvar(diff(L_f_h, u)*u);
    if numel(dh_u) ~= 0
        done = 1;
    end
    i = i + 1;
    % add to diffemorphism
    diffeoFormula(i+1) = L_f_h;
end

if done
    % symbolic expressions
    b = diff(L_f_h, u);
    a = L_f_h - b*u;
    % create matlab functions to speed up computations
    bf = matlabFunction(b, 'vars', x);
    af = matlabFunction(a, 'vars', x);
    diffeof = matlabFunction(diffeoFormula(1:n), 'vars', x);
else
    disp('The system is not I/O linearizable by static output feedback');
    af = 0;
    bf = 0;
    diffeof = 0;
end

end
