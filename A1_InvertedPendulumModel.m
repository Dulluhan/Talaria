%% Dynamic Flywheel Control to Assist the Balance of BiPedal Robots
% Preliminary inverted complex pendulum model
% 1/15/17
% Simon Honigmann & Vincent Yuan

clc; clear all;

%% State Variables
omega_p = 0; % pendulum angular velocity (rad/s)
omega_w = 0; % flywheel angular velocity (rad/s)
X = [omega_p; omega_w];
dX = [0; 0]; % change of state variables evaluated each time step

%% Outputs
theta_p = 0.01; %pendulum angle measured CW from the vertical axis (rad)

%% System Inputs
T_m = 0; % motor torque
g = 9.81; % m/s^2
U = [T_m ; g];

%% System Parameters
I_m = 4E-4; % moment of inertia of the motor (kg m^2)
I_ms = 3E-8; % moment of inertia of the motor shaft - leave as zero if negligible (kg m^2)
m_m = .3; % mass of the motor (kg)

I_piv = 1E-6; % moment of inertia of the pivot bearing - leave as zero if negligible (kg m^2)

I_w = 4E-4; % flywheel moment of inertia about its center of rotation (kg m^2)
m_w = .3; % flywheel mass (kg)
r_w = .1; % distance of flywheel from the pendulum base (m)

I_b = 0; % momemnt of inertia of the pendulum bar (kg m^2)
m_b = .1; % mass of the pendulum bar (kg)
r_b = .05; % radius from pivot to center of pendulum bar

I_p = I_piv + (I_b + m_b*r_b^2) + (I_w + m_w*r_w^2) + (I_m + m_m*r_w^2); % net moment of inertia of the pendulum about its base (kg m^2)

b_p = .01; % pendulum pivot friction
b_w = .01; % flywheel/motor friction

m_eq = m_b + m_w + m_m; % total mass of rotating components
L_eq = (r_b*m_b + r_w*(m_w + m_m))/(m_eq); % COG distance from pivot

T_grav = L_eq*m_eq*sin(theta_p); % torque on pendulum due to it's weight and angle

A = [-b_p/I_p -b_w/I_p; 0 -b_w/I_w];
B = [1/I_p T_grav/I_p; 1/I_w 0];

%% Simulation Variables
t = 0; % time (s)
dt = 0.01; % timestep (s) 

%% Control Variables
p = 0; % position gain
v = 0; % velocity gain
a = 0; % acceleration gain (?)

%% Operation Simulation
while t < 100
    
    dX = A*X + B*U;
    X = dX.*dt + X;
    
    theta_p = X(1)*dt + theta_p; % integrates over time step
    
    % Change input, U, based on control scheme
    T_m = p*theta_p + v*X(1) + a*dX(1);
    U = [T_m ; g];
  
    T_grav = L_eq*m_eq*sin(theta_p);
    B = [1/I_p T_grav/I_p; 1/I_w 0];
    
    P = [r_w*cos(theta_p+pi/2) r_w*sin(theta_p+pi/2)];
    
    % Animation 
    O = [0 0];
    axis(gca,'equal'); % Aspect ratio of the plot
    axis([-1.2*r_w, 1.2*r_w, -1.2*r_w, 1.2*r_w]);
    grid on;
    
    O_circ = viscircles(O,.05*r_w);
    pend = line([O(1) P(1)],[O(2) P(2)]);
    ball = viscircles(P,.1*r_w);
    
    pause(0.001);
    
    delete(pend);
    delete(ball);
    delete(O_circ);
    
    t = t+dt;
end










