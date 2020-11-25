
%{
Rt = turnRadius;
V = vehicleVelocity;
theta = RampAngle;
Capital R for road condition
Lowercase r for vehicle parameters
T2 is torque delivered to left wheel
T3 is torque delivered to right wheel
if isFpre = true, the system is unlocked, ramp angle lock not triggered
It's locked if isPreload = false
Assuming that w2 < w3, i.e. Turning to the left
By symmetry if turing to the right, then all 2-variables are right wheel 
and 3-variables are left wheel 
%}

function [T2, T3, isPreload] = calcTorque(Rt, V, theta)
%List of constants:==========   
    width = 2.0; %Change later, width of the vehicle
    rWheel = 0.5; %Radius of wheels
    mu = 0.3; %Friction coefficient between friction and steel plates
    Tpre = 10; % Preload (after running in) [Nm]
    ro = 0.1; %Outer radius of friction disk
    ri = 0.07; %Inner radius of friction disk
    r5 = 0.06; %Radius of the case
    disk_contacts = 4; % number of contacting surfaces in assembly
%==========================
    
    Rin = Rt - width/2;
    Rout = Rt + width/2;
    M1 = [1,1, 2*V/rWheel; Rout, -Rin , 0];
    
    M1rref = rref(M1);
    w2 = M1rref(1, 3);
    w3 = M1rref(2, 3);
    
    w4 = (w2+w3)/2;
    
    T4 = getEngineTorque(w4);
    T4_5 = T4;
    
    M2 = [1,1, T4_5; w2, w3, T4_5*w4];
    M2rref = rref(M2);
    T5_2 = M2rref(1, 3);
    T5_3 = M2rref(2, 3);
    
    isPreload = false;
 
    muPrime = disk_contacts * 2 * mu * (ro^3 - ri^3)/(3 * (ro^2 - ri^2));
  
    
    Fn = T4_5*tan(theta)/(2*r5);  
    T4_2 = muPrime*Fn;
    
    if Tpre > T4_2 
        isPreload = true;
        T4_2 = TPre;
    end
    T4_3 = -T4_2;
    
    T2 = T4_2 + T5_2; 
    T3 = T4_3 + T5_3;
    
end

function T = getEngineTorque(w)
    
    T = 50 * 745.7 / w;
   
end


