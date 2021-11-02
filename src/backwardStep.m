function [kFfw, kFbk, deltaV, Vx, Vxx] = backwardStep(xCurrent, uCurrent, VxNext, VxxNext)
% This function performs the backward step of the iLQG 
% given current x, u and derivatives of the values it returns Vx, Vxx,
% feedback and open loop gains ( previous step )

global mu;


% Compute loss derivatives
lx = lxFunc(xCurrent);
lxx = lxxFunc(xCurrent);

lu = luFunc(uCurrent);
luu = luuFunc(uCurrent);

% Compute dynamics derivatives
fx = fxFunc(xCurrent, uCurrent);
fu = fuFunc(xCurrent, uCurrent);

deltaV = [0 0];


% Update value approximation
Qx = lx.' + fx.'*VxNext;
Qu = lu.' + fu.'*VxNext;
Qxx = lxx + fx.'*VxxNext*fx;
Quu = luu + fu.'*(VxxNext+mu*eye(size(VxxNext)))*fu;
Qux =  fu.'*(VxxNext+mu*eye(size(VxxNext)))*fx;


% Update feedback terms
kFfw = -Qu/Quu;
kFbk = -Qux/Quu;


% Update value derivatives approximations
deltaV = deltaV + [kFfw.'*Qu  0.5*kFfw.'*Quu*kFfw];
Vx = Qx + kFbk.'*Quu*kFfw + kFbk.'*Qu + Qux.'*kFfw;
Vxx = Qxx + kFbk.'*Quu*kFbk + kFbk.'*Qux + Qux.'*kFbk;

