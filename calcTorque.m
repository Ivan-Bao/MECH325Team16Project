
%{
Rt = turnRadius;
V = vehicleVelocity;
theta = RampAngle;
Capital R for road condition
Lowercase r for vehicle parameters
T2 is torque delivered to left wheel
T3 is torque delivered to right wheel
if isFpre = true, the system is unlocked, ramp angle lock not triggered
It's locked if isFpre = false
Assuming that w2 < w3, i.e. Turning to the left
By symmetry if turing to the right, then all 2-variables are right wheel 
and 3-variables are left wheel 
%}

function [T2, T3, isFpre] = calcTorque(Rt, V, theta)
%List of constants:==========   
    width = 2.0; %Change later, width of the vehicle
    rWheel = 0.5; %Radius of wheels
    mu = 0.01; %Friction coefficient between friction and steel plates
    Fpre = 1; %Preload force
    ro = 0.05; %Outer radius of friction disk
    ri = 0.01; %Inner radius of friction disk
    r5 = 0.06; %Radius of the case
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
    
    isFpre = true;
    muPrime = 2 * pi * mu * (ro^3 - ri^3)/3;
    Fn = Fpre;
    temp = T4_5*tan(theta)/(2*r5);
    if temp > Fn
        Fn = temp;
        isFpre = false;
    end
    
    T4_2 = muPrime*Fn;
    T4_3 = -muPrime*Fn;
    
    T2 = T4_2 + T5_2; 
    T3 = T4_3 + T5_3;
    
end

function T = getEngineTorque(w)
    
    T = 1/w;
   
end


