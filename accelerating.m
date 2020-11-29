
n = 100; % # datapoints
theta_deg = linspace(15, 60, n);
theta = deg2rad(theta_deg); %Ramp Angle, radians (0 to 60 deg)
torque_in =  58.16 * 32 / 14 * 0.99; % sync with calcTorque.m
mu = zeros(1, n);
for i = 1 : n
    mu(i) = accelerationLock(torque_in, theta(i));
end
plot(theta_deg, mu)
xlabel("Ramp angle (degrees)")
ylabel("Minimum \mu_r")


%{
Calculates what minimum coefficient of friction is needed to ensure locking
in the wheels, during acceleration. Deceleration can use the same formulae;
engineInput then becomes the effective braking torque going through the
sprocket.

Inputs: 
    engineInput - torque coming in from the engine (T4) [Nm]
    theta - ramp angle [rad]
    x - fraction of weight of vehicle carried by rear wheels. Defaults to
        0.5. 

Returns:
    mu_r - Minimum coefficient of friction on one side, assuming good
    traction (no slipping) on the other side

%}
function [mu_r] = accelerationLock(engineInput, theta, x)
% BEG CONSTANTS ================
% * indicates that value is unknown
% # indicates that value must be synced with other scripts
    rw = 0.2032; % Radius of wheel #
    T_pre = 10; % Preload torque [Nm] #
    mu = 0.09; %Friction coefficient between friction and steel plates
    ro = 0.055; %Outer radius of friction disk * 
    ri = 0.04; %Inner radius of friction disk *
    r5 = 0.06; % Radius of case [m] *#
    if nargin < 3
        x = 0.5; % Fraction of weight of car carried by back wheels
    end
    disk_contacts = 4; % number of contacting surfaces in assembly
    mg = 630 * 4.448; % N
% END CONSTANTS ================
    %w2 = wheelrpm * 2 * pi / 60;
    %w3 = w2;
    %w4 = w2;
    
    T4 = engineInput;
    T45 = T4;
    Fn = T45 * tan(theta) / 2 / r5;
    muprime = mu * (ro + ri) / 2 * disk_contacts;
    Tc = Fn * muprime;
    
    if T_pre > Tc % Preload wins
        Tc = T_pre;
    end
    
    mu_r = 2 / (rw * mg * x) * (T4 / 2 - Tc);
    
end