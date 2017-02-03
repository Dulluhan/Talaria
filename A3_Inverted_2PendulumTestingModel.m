%% Dynamic Flywheel Control to Assist the Balance of BiPedal Robots
% Preliminary inverted complex pendulum model
% 1/15/17
% Simon Honigmann & Vincent Yuan

function main
clc; clear all;
global g;
global r_w;
%% State Variables
omega_p = 0; % pendulum angular velocity (rad/s)
omega_w = 0; % flywheel angular velocity (rad/s)
X = [omega_p; omega_w];
dX = [0; 0]; % change of state variables evaluated each time step

%% Outputs
theta_p = 0.0; %pendulum angle measured CW from the vertical axis (rad)

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

b_p = 0; %.01; % pendulum pivot friction
b_w = 0; %.01; % flywheel/motor friction


m_eq = m_b + m_w + m_m; % total mass of rotating components
L_eq = (r_b*m_b + r_w*(m_w + m_m))/(m_eq); % COG distance from pivot

T_grav = L_eq*m_eq*sin(theta_p); % torque on pendulum due to it's weight and angle

A = [-b_p/I_p -b_w/I_p; 0 -b_w/I_w];
B = [1/I_p T_grav/I_p; 1/I_w 0];

%% Simulation Variables
t = 0; % time (s)
dt = 0.02; % timestep (s) 

%% Control Variables
p = 0; % position gain
v = 0; % velocity gain
a = 0; % acceleration gain (?)
subplot(2,2,[1 2]);
axis(gca,'equal'); % Aspect ratio of the plot
axis([-1.2*r_w, (1.2*r_w)*2, -1.2*r_w, 1.2*r_w]);
grid on;
hold on; 
ax2 = subplot(2,2,3);
hold on;
ax3 = subplot(2,2,4);
hold on;
linkaxes([ax2 ax3],'xy');

theta_p = 0.01; %initial conditions

%% control group variables
theta_po = theta_p;
x_o = X;
B_o = B;

%% Operation Simulation
while t < 100
    
    %% Control group
    U_o = [0; g];
    dx_o = A*x_o + B_o*U_o;
    x_o = dx_o.*dt + x_o;
    theta_po = x_o(1)*dt + theta_po;
    if theta_p > 2*pi
        theta_p = theta_p - 2*pi;
    elseif theta_p < - 2*pi
        theta_p = theta_p+2*pi;
    end
    po = [r_w*cos(theta_po+pi/2) r_w*sin(theta_po+pi/2)]; %updated cartesian position of pendulum mass center
    T_gravo = L_eq*m_eq*sin(theta_po); %torque generated from gravity? 
    B_o = [1/I_p T_gravo/I_p; 1/I_w 0]; %Input Matrix
    
    %% Controller for test group
    %T_m = abs(theta_p+pi/2)/(pi/2)*reactive_controller(T_grav); %Using T_grav calcualte amount of toque required 
    %T_m = -T_grav + b_w*X(2) + b_p*X(1);
    T_m = 0;
    
    %% Test group
    U = [T_m ; g]; %Input Vector
    dX = A*X + B*U;
    X = dX.*dt + X;
    theta_p = X(1)*dt + theta_p; % integrates over time step
    if theta_p > 2*pi
        theta_p = theta_p - 2*pi;
    elseif theta_p < - 2*pi
        theta_p = theta_p+2*pi;
    end
    p = [r_w*cos(theta_p+pi/2) r_w*sin(theta_p+pi/2)]; %updated cartesian position of pendulum mass center
    T_grav = L_eq*m_eq*sin(theta_p); %torque generated from gravity? 
    if abs(T_grav) < 0.75*10^-3 %10^-4 too much
        T_grav = 0.0;
    end
    B = [1/I_p T_grav/I_p; 1/I_w 0]; %Input Matrix
    
    %% Animation actions
    animate(p,po,theta_p,theta_po,t);
    
    t = t+dt;
    
end
end

function pendulum1(t,g,r_w)

end

%% accerleration tit tat controller with gaussian noise
% acceleration in gs
function Tm = reactive_controller(accel) 
    global g;
    k = 1; %gain of the system
    var = 2.7*0.001; %mg variance for adxl377 accelerometer
    noise = sqrt(var)*rand*3; %model variance for a +/- 3g mems accel chip
    sensor_data = accel + noise;
    Tm = -k*(sensor_data+sqrt(var)*rand*3)*g; 
end

%% feedforward controller will calculate where it wants to be and how much
%it needs to accelerate
function theta = feedforward_controller(theta_p)
    theta_des = pi/2;  %desired position
    theta = 0; 
end

%% Animation Function Extract
function animate(P,p2,theta_p,theta_p2,t)
% Animation 
    global r_w;
    subplot(2,2,[1,2]);
    axis([-1.2*r_w, (1.2*r_w)*2, -1.2*r_w, 1.2*r_w]);
    O = [0 0];
    O2 = [r_w,0];
    p2 = p2+O2;
    %p
    O_circ = viscircles(O,.05*r_w);
    pend = line([O(1) P(1)],[O(2) P(2)]);
    ball = viscircles(P,.1*r_w);
    %p2
    O_circ2 = viscircles(O2,.05*r_w);
    pend2 = line([O2(1) p2(1)],[O2(2) p2(2)]);
    ball2 = viscircles(p2,.1*r_w);
    
    pause(0.001);
    
    delete(pend);
    delete(ball);
    delete(O_circ);
    delete(pend2);
    delete(ball2);
    delete(O_circ2);
    
    subplot(2,2,3);
    plot(t,theta_p,'.r');
    axis ([t-5, t+5, -2*pi, 2*pi]);
    subplot(2,2,4);
    axis ([t-5, t+5, -2*pi, 2*pi]);
    plot(t,theta_p2,'.r');

end











