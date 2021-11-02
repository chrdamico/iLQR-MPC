function [stateOptimized, inputOptimized, terminationCondition] = trajectoryOptimizazion(stateSuboptimal, inputSuboptimal, timeHorizon)
% This function performs one complete iteration of the iLQG optimization (forward and backward pass)
% It provides the optimized trajectories and the terminationCondition 
% terminationCondition = 1 means that iLQG has reached convergence

global line_search_parameters;
global mu;
global cost;
global dmu;
global muFactor;
global muMin;
global zMin;
global tolFun;


kFfw = zeros(1,timeHorizon);
kFbk = zeros(4,timeHorizon);

Vx = fcxFunc(stateSuboptimal(:,end)').';
Vxx = fcxxFunc(stateSuboptimal(:,end)').';


for i=timeHorizon:-1:1
    % perform backward step to obtain feedbak and open loop gains, deltaV
    % and derivatives of the Value
    [kFfw(i), kFbk(:,i), deltaV, Vx, Vxx] = backwardStep(stateSuboptimal(:,i).', inputSuboptimal(i), Vx, Vxx);
end

fwdPassDone = 0;

for alpha_line_search = line_search_parameters
    [stateNew, inputNew,cost_new] = forwardPass(stateSuboptimal, inputSuboptimal, kFfw, kFbk, timeHorizon, alpha_line_search);
    % compute the difference of the cost WRT teh previous iteration
    dcost    = sum(cost(:)) - sum(cost_new(:));
    % compute the expected reduction of the cost
    expected = -alpha_line_search*(deltaV(1) + alpha_line_search*deltaV(2));
    if expected > 0
        z = dcost/expected;
    else
        z = sign(dcost);
        warning('non-positive expected reduction: should not occur');
    end
    if (z > zMin)
        % accept the forward pass in case condition on ratio between actual
        % and expected reduction in cost is met
        fwdPassDone = 1;
        break;
    end
end

terminationCondition = 0;

if fwdPassDone == 1
    % decrease mu to try faster convergence rate
    dmu   = min(dmu / muFactor, 1/muFactor);
    mu    = mu * dmu * (dmu > muMin);
    
    % accept improvements
    cost = cost_new;
    stateOptimized = stateNew;
    inputOptimized = inputNew;
    if dcost < tolFun
        fprintf('\nSUCCESS: cost change < tolFun\n');
        terminationCondition = 1;
    end
else
    % increase mu because there has been no improvement (NO STEP)
    dmu  = max(dmu * muFactor, muFactor);
    mu   = max(mu * dmu, muMin);
    % discard changes
    stateOptimized = stateSuboptimal;
    inputOptimized = inputSuboptimal;
end
    



