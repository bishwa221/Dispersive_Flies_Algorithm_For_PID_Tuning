function fopt = pid_optimize(x)
s = tf('s');

%plant = 69.49135/((0.00000015154*(s^2))+(0.00445525*s)+1);  % 1
%plant = 0.0311/((0.00000816*(s^2))+(0.000144022*s)+0.00096721);   % 2
%plant = 1.634/((0.00649945*(s^2))+(0.3023*s)+1); %3
%plant = 16.13/((1*(s^2))+(0.201*s)+0.0033); %4
%plant = 15/((1.08*(s^2))+(6.1*s)+1.63); %5
plant = 1/(0.222866*(s^2) + 0.77067*s + 1); %7 %also has overshoot


[r,c] = size(x);
fopt = zeros(1,r); 
for i = 1:r
    kp = x(i,1);
    ki = x(i,2);
    kd = x(i,3);
    
    controller = kp + ki/s + kd*s;

% Plotting the response. This might slow the computation
%step(feedback(plant*controller,1))

    dt = 0.01;
    t = 0:dt:1;

    error = 1 - step(feedback(plant*controller,1),t);

% Cost/Fitness function
    %fopt(i) = sum(t'.* (abs(error) *dt)   );  %ITAE 
    %fopt(i) = sum(  (error .* error)*dt);  %ISE
    %fopt(i) = sum(  abs(error) )*dt;   %IAE
    fopt(i) = sum(t'.*((error).^2)*dt); %ITSE

end
end

