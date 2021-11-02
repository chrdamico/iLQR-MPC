function [xOpt, uOpt,cost_new] = forwardPass(xSubopt, uSubopt, kFfw, kFbk, timeHorizon,alpha_line_search)
% This function performs the forward pass of iLQG
% It computes the new trajectories and the new value of the cost
% It uses the old trajectories, the computed gains, timeHorizon and a candidate value
% for alpha_line_search

xOpt = xSubopt;
uOpt = uSubopt;
cost_new = zeros(1,timeHorizon+1);

% Optimal trajectory correction propagation
for i=1:timeHorizon % deve essere di lenght(xSubopt)-1 per non sforare
   % xOpt(:,1) % colonna
   
   % Optimal input update
   uOpt(i) = uSubopt(i) + alpha_line_search*kFfw(i) + kFbk(:,i).'*([wrap(xOpt(1:2,i)-xSubopt(1:2,i)); xOpt(3:4,i)-xSubopt(3:4,i)]);
   % New computed cost
   cost_new(i) = runningCostFunc(xOpt(:,i)',uOpt(i));
   % Optimal state propagation with new input
   xOpt(:,(i+1)) = dynamicsFunc(xOpt(:,i), uOpt(i));
end

cost_new(end) = finalCostFunc(xOpt(:,end).');