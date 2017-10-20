% Config Parameters
N = 3; % number of coupled oscillators
w_max = 1; % rad/s
ar = 4;
ax = 4;
R = [20, 40, 40];
X = [0.0, 0.0, 0.0];

sim_time = 120.0; % seconds

T = 0.01; % simulation time per step in euler

% Variables
W = [w_max, w_max, w_max];

w = [
        0, 0, 0;
        10, 0, 0;
        10, 0, 0;
    ];

phi = [
            0, 0, 0;
            -pi/2, 0, 0;
            pi/2, 0, 0;
      ];

% State Variables
Phi = [0.0, 0.0, 0.0];
r = [0.0, 0.0, 0.0];
r_d = [0.0, 0.0, 0.0];
x = [0.0, 0.0, 0.0];
x_d = [0.0, 0.0, 0.0];

% Output
theta = [0.0, 0.0, 0.0];

% Simulation
plt_theta = zeros(sim_time/T, N);
plt_Phi = zeros(sim_time/T, N);
plt_r = zeros(sim_time/T, N);
plt_x = zeros(sim_time/T, N);

idx = 1;
for i=0:T:sim_time-T
    [theta, Phi, r, r_d, x, x_d] = kuramoto_update( N, T, ar, ax, W, w, Phi, phi, R, r, r_d, X, x, x_d );
    plt_theta(idx,:) = theta;
    plt_Phi(idx,:) = Phi;
    plt_r(idx,:) = r;
    plt_x(idx,:) = x;
    idx = idx + 1;
    
    if i == 20
        phi = [
            0, 0, 0;
            pi/2, 0, 0;
            pi/2, 0, 0;
      ]
    end
    
     if i == 40
        phi = [
            0, 0, 0;
            pi/2, 0, 0;
            -pi/2, 0, 0;
      ]
     end
    
     if i == 80
        phi = [
            0, 0, 0;
            -pi/2, 0, 0;
            -pi/2, 0, 0;
      ]
     end
    
     if i == 100
        phi = [
            0, 0, 0;
            pi/2, 0, 0;
            pi/2, 0, 0;
      ]
    end
end

%% PLOTTING
tscl = linspace(0, sim_time, sim_time/T);

figure % opens new figure window
plot(tscl, plt_theta(:,1), tscl, plt_theta(:,2), tscl, plt_theta(:,3))
