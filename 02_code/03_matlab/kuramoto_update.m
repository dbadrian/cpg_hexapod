function [ theta, nPhi, nr, nr_d, nx, nx_d ] = ...
    kuramoto_update( N, T, ar, ax, W, w, Phi, phi, R, r, r_d, X, x, x_d )
%KURAMOTO_UPDATE Summary of this function goes here
% okay...doing this very explicitly for now as the C implementation will
% not be vectorized either

% tmp
nPhi = [0.0, 0.0, 0.0];
nr = [0.0, 0.0, 0.0];
nr_d = [0.0, 0.0, 0.0];
nx = [0.0, 0.0, 0.0];
nx_d = [0.0, 0.0, 0.0];
theta = [0.0, 0.0, 0.0];

for i=1:N
    Phi_d = W(i);
    for j=1:N
        Phi_d = Phi_d + w(i,j) * r(j) * sin(Phi(j) - Phi(i) - phi(i,j));
    end
    nPhi(i) = Phi(i) + T * Phi_d;
end

for i=1:N
    r_dd = ar * ((ar/4)*(R(i)-r(i))-r_d(i));
    nr_d(i)= r_d(i) + T * r_dd;
    nr(i) = r(i) + T * r_d(i);
end

for i=1:N
    x_dd = ax * ((ax/4)*(X(i)-x(i))-x_d(i));
    nx_d(i)= x_d(i) + T * x_dd;
    nx(i) = x(i) + T * x_d(i);
end

for i=1:N
%     theta(i) = nx(i) + nr(i) * sin(nPhi(i));
    theta(i) = nr(i) * sin(nPhi(i));
end


end

