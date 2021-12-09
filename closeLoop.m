s = tf('s');

% Define the TF of the plant DC motor
plant = 69.49135/(0.00000015154*s^2+0.00445525*s+1);

% Input the optimized gain parameters
kp = 50;          
ki = 40;
kd = 0;

% Define the TF of PID controller
controller = kp + ki/s + kd*s;

% Compute the complete TF of the system
System = feedback(plant*controller,1)

% List out the transient parameters; tr, ts, overshoot, etc.
stepinfo(System)

% Uncomment to plot the response
%step(System)