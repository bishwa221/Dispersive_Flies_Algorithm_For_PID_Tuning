clc
clear 
%% Initialize Parameters
N = 48; %swarm size
D = 3; %dimension of problem
delta = 0.0045;
maxIter = 100;
f = @pid_optimize;
lb(1,1:D) = 0.01; %lower bound
ub(1,1:D) = 20; %upper bound
id = 0; %initial index of best fly
X = zeros(N,D); %Flies population matrix
fitness(1,1:N) = realmax; %Fitness values of N flies
%bNeighbour = NaN;
c = zeros(maxIter,1);
%% generation of initial random population
for i = 1:N
    for d = 1:D
        X(i,d) = lb(d) + rand()*(ub(d) - lb(d));
    end
end


%% Main Algorithm Body
for itr = 1:maxIter
    
    % evaluate fitness of the flies and find the best
    fitness = f(X); % f = fitness function here
    [sFitness,id] = min(fitness);
    disp(['Iteration: ', num2str(itr),' Best fly index: ', num2str(id), ' Fitness value: ', num2str(sFitness),])
    c(itr) = sFitness;
    % Update each fly individually
    for i = 1:N
       
        % ELITIST STRATEGY (don't update best fly)
        if i == id
            continue;
        end
        
        % FIND BEST NEIGHBOUR FOR EACH FLY ie fly number i.
        left = mod(i-2,N) + 1; %index of left neighbour
        right = mod(i,N) + 1;  %index of right neighbour
        if fitness(right) < fitness(left)
            bNeighbour = right;
        else
            bNeighbour = left;
        end
        
        for d = 1:D
            %DISTURBANCE/RESTART MECHANISM
            if rand()<delta
                X(i,d) = lb(d) + rand()*( ub(d) - lb(d));
                continue;
            end
            
            %UPDATE EQUATION
            X(i,d) = X(bNeighbour,d) + rand()*(X(id,d)-X(i,d));
            
            % OUT OF BOUND CONTROL
            if or( X(i,d)<lb(d) , X(i,d)>ub(d) ) %if out of bounds
                X(i,d) = lb(d) + rand() * ( ub(d) - lb(d) ); %restart within bounds
            end
        end
    end
end

%% EVALUATE FITNESS OF THE FLIES AND FIND THE BEST
fitness = f(X);
[sFitness,id] = min(fitness);

disp(['Final best fitness= ', num2str(sFitness)])
disp(['Final best fly= ', num2str(X(id,:))])

%% PID RESULT DISPLAY
s = tf('s');

% Define the TF of the plant DC motor
%plant = 69.49135/((0.00000015154*(s^2))+(0.00445525*s)+1);  % 1
%plant = 0.0311/((0.00000816*(s^2))+(0.000144022*s)+0.00096721);   % 2
%plant = 1.634/((0.00649945*(s^2))+(0.3023*s)+1); %3
%plant = 16.13/((1*(sy^2))+(0.201*sy)+0.0033); %4
%plant = 15/((1.08*(s^2))+(6.1*s)+1.63); %5
plant = 1/(0.222866*(s^2) + 0.77067*s + 1); %7 %also has overshoot



% Input the optimized gain parameters
kp = X(id,1);          
ki = X(id,2);
kd = X(id,3);

% Define the TF of PID controller
controller = kp + ki/s + kd*s;

% Compute the complete TF of the system
System = feedback(plant*controller,1)

% List out the transient parameters; tr, ts, overshoot, etc.
stepinfo(System)

% Uncomment to plot the response
%step(System)

figure,semilogy(c,'r')
xlim([0 150]);


            
    