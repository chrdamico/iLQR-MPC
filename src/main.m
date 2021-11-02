% ________ Inizialization
clear all
global mu;

% Define all relevant parameters
load_parameters

% Generate MATLAB functions for dynamics, cost and their derivatives
createFunctions

% Initialize a trajectory
% There are as many inputs as timeHorizon
% At the moment we have no better prediction than a null trajectory
stateSuboptimal = zeros(4,timeHorizon+1);
inputSuboptimal = zeros(1,timeHorizon);

% Use this other input for a better initial trajectory
inputSuboptimal = 1*sin(linspace(0,2*pi, timeHorizon));

% Propagate the trajectory
for i=1:(timeHorizon)
   stateSuboptimal(:,i+1) = dynamicsFunc(stateSuboptimal(:,i),inputSuboptimal(i));
   stateSuboptimal(1:2,i+1) = wrap(stateSuboptimal(1:2,i+1));
end

% _________ Main optimization loop
maxNumIterationsILQG = 200;
maxNumIterationsMPC = timeHorizon + 500;

% set initial values to MPC parameters
numIterationMPC = 1;
xActual(:,numIterationMPC) = x_init;
xCurrent = x_init;
uActual = inputSuboptimal;

errorNorm = norm(wrap(xCurrent(1:2)-x_des(1:2)));


while errorNorm > tolErrorNorm
    % reset parameters to be used in iLQG iteration
    terminationCondition = 0;
    numIterationILQG = 1;
    mu = 0;
    
    % iLQG iteration
    while ~terminationCondition && numIterationILQG < maxNumIterationsILQG
        
        disp(['ILQGiter: ', num2str(numIterationILQG)])
        
        % call trajectoryOptimization to obtain optimized trajectory
        [stateOptimized, inputOptimized, terminationCondition] = trajectoryOptimizazion(stateSuboptimal, inputSuboptimal, timeHorizon);

        %terminate ? 
        if mu > muMax
            fprintf('\nMPC iteration number %d: iLQG terminated because of mu > muMax\n',numIterationMPC);
            break;
        end

        
        inputSuboptimal = inputOptimized;
        stateSuboptimal = stateOptimized;


        
 
        if terminationCondition == 1
            fprintf('\nMPC iteration number %d: iLQG terminated after %d iterations with error norm %f\n',numIterationMPC, numIterationILQG,...
                norm(stateOptimized(1:2,end)-x_des(1:2)));
            break;
        end

        numIterationILQG = numIterationILQG+1;
    end
    

    % extract first element from inputOptimized
    uCurrent = inputOptimized(1);
    % apply first element of inputOptimized to actual robot
    xCurrent = dynamicsFunc(xCurrent,uCurrent);
    errorNorm = norm(xCurrent(1:2)-x_des(1:2));
    
    
    % store xCurrent in xActual for final plot
    xActual(:,numIterationMPC+1) = xCurrent;
    uActual(numIterationMPC) = uCurrent;
    
    
    % Update the state: all values are the same shifted to the left
    % A new final equal to the previous one is added, as well as a new final state with the 
    % dynamics of state and input at (n-1)
    newFinalState = dynamicsFunc(stateOptimized(:,end),inputOptimized(end));
    newFinalInput = inputOptimized(end);
    
    % New state vector
    stateSuboptimal = [stateOptimized(:,2:end), newFinalState];
    stateSuboptimal(:,1) = xCurrent;
    
    % New input vector
    inputSuboptimal = [inputOptimized(2:end), newFinalInput];
    inputSuboptimal(1) = uCurrent;
    
    
     % terminate ? 
    if numIterationMPC >= maxNumIterationsMPC
        disp('numIterationMPC >= maxNumIterationsMPC');
        break;
    end 
    
    % update iteration number
    numIterationMPC = numIterationMPC+1; 
end



%% Plot of MPC solution

figure(2)
plot((1:(numIterationMPC))*1, xActual.', 'linewidth',2), hold on
plot((1:(numIterationMPC-1))*1, uActual, 'linewidth',2)
axis([0 1281 -8 8])
title("States and Input executed trajectory", 'interpreter', 'latex', 'FontSize', 14)
legend({'$q_1$','$q_2$','$\dot{q}_1$','$\dot{q}_2$','u'},...
    'interpreter', 'latex', 'FontSize', 14,'Location','southeast','Orientation','horizontal')
%print('statesTrajZERO', '-painters', '-dpdf')


%% Acrobot Motion
num_figure=3;
figure(num_figure)
for i = 1:4:numIterationMPC
    showplot(xActual(:,i), l1, l2, num_figure)
    grid on
    %print(sprintf('./images/acrobotZEROinput%04d',int32((i+4)/4)),'-dpng', '-r240')
    pause(0.001)
end 

