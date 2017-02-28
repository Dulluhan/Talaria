%% Lagrangian Mechanics Simulation For Inverted Pendulum
% Model of flywheel pendulum as 2DOF RR manipulator
% 2/27/2017

function main
clr; clear all;

%% Fig close end program setup 
global figs;
figs = figure;
finack = onCleanup(@() cleanupf(figs));

%% Joint Variables
theta_p = 0; % pendulum angular velocity (rad/s)
theta_w = 0; % flywheel angular velocity (rad/s)
q = [theta_w; theta_p]; 

%% Global Variable Decalarations 
global g;
global m_eq;
global L_eq; 
global I_w;
global I_p;
global r_w;

g = 9.81; % m/s^2

m_m = .3; % mass of the motor (kg)

I_piv = 1E-6; % moment of inertia of the pivot bearing - leave as zero if negligible (kg m^2)

%% Flywheel Intertial terms 
I_w = 4E-4; % flywheel moment of inertia about its center of rotation (kg m^2)
m_w = .3; % flywheel mass (kg)
r_w = .1; % distance of flywheel from the pendulum base (m)
I_m = 4E-4; % moment of inertia of the motor (kg m^2)

%% Pendulum Inertial Terms 
I_b = 0; % momemnt of inertia of the pendulum bar (kg m^2)
m_b = .1; % mass of the pendulum bar (kg)
r_b = .05; % radius from pivot to center of pendulum bar
I_p = I_piv + (I_b + m_b*r_b^2) + (I_w + m_w*r_w^2) + (I_m + m_m*r_w^2); % net moment of inertia of the pendulum about its base (kg m^2)

%% Friction Loss
b_p = .01; % pendulum pivot friction
b_w = 0.01; % flywheel/motor friction

%% Equivalent Parameters IRT body
m_eq = m_b + m_w + m_m; % total mass of rotating components
L_eq = (r_b*m_b + r_w*(m_w + m_m))/(m_eq); % COG distance from pivot

%% Generate Constant Lagrangian Parameters 
D = [Iw 0; 0 Ip]; % Generate inertial tensor
F = [bw 0; 0 bp]; % Frictional coefficients

sim(D,F,G,q); 

end

%% System Simulation
function sim(U,D,F,q)
    for dt=1:0.01:100
        %q_des = controller(q(2),dt); %Get desired output
        %[q,q_1] = forkin(q_des); %Simulate motor to get actual output
        q_2 = q/(dt)^2;
        q_1 = q/dt;
        G = gen_grav(q(2));
        q = D*q_2+G-F*q_1; %Net torque acting on system
        %how is this resolved in terms of movement?
        
    end
end

%% Generate coordinates for plot
function p = gen_coord(theta)
    global r_w;
    p = [r_w*cos(theta+pi/2) r_w*sin(theta+pi/2)]; 
    %updated cartesian position of pendulum mass center
end

%% Animation Function
function animate(P,p2,theta_p,theta_p2,t)

% Animation 
    global r_w;
    subplot(2,2,[1,2]);
    axis([-1.2*r_w, (1.2*r_w)*3, -1.2*r_w, 1.2*r_w]);
    O = [0 0];
    O2 = [2*r_w,0];
    p2 = p2+O2;
    %p
    viscircles(O,.05*r_w);
    pend = line([O(1) P(1)],[O(2) P(2)]);
    ball = viscircles(P,.1*r_w);
    %p2
    viscircles(O2,.05*r_w);
    pend2 = line([O2(1) p2(1)],[O2(2) p2(2)]);
    ball2 = viscircles(p2,.1*r_w);
    
    pause(0.001);
    
    delete([pend,ball,pend2,ball2]);
    
    subplot(2,2,3);
    plot(t,theta_p,'.r');
    axis ([t-5, t+5, -2*pi, 2*pi]);
    subplot(2,2,4);
    axis ([t-5, t+5, -2*pi, 2*pi]);
    plot(t,theta_p2,'.r');

end

function q_des = controller(x) %x is angular displacement
end

%% Calculates gravitational torque acting on system based on pendulum angle
function grav = gen_grav(theta_p)
global m_eq;
global L_eq;
torque = L_eq*m_eq*sin(theta_p);
grav = [0; torque];
end



