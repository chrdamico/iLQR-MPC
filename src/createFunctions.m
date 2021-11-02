% Run this to create the new nameFunc. All the relevant stuff in
% load_parameters has been moved here. The only exceptions are l1 and l2
% which are needed to plot stuff AND dT


%%______________ Function Data

% Desired goal
x_des = [pi,0,0,0]';

% Time discretization [s]
global dT
dT=0.005;


% Inertia [kg*m^2]
I1 = 0.7;
I2 = 0.3;

% Mass [kg]
m1 = 1;
m2 = 0.5;

% lenght [m]
l1 = 1;
l2 = 0.5;

% CoM position [m]
lc1 = 0.4;
lc2 = 0.2;

% Gravity acceleration [m/s^2]
g = 9.8;


% Dynamical model parameters
a1 = I1 + m1*lc1^2 + I2 + m2*(l1^2+lc2^2);
a2 = m2*l1*lc2;
a3 = I2 + m2*lc2^2;
a4 = g*(m1*lc1 + m2*l1);
a5 = g*m2*lc2;

% Cost parameters
alphax = 1;
alphau = 1;

% Loss Weights
alpha = 1;
velWeight = 0.1;
stateWeight = 0.01;

stateWeightFinal = 100;

inputWeight = 0.001;

W = sym(eye(4)); 
W(3,3) = velWeight; 
W(4,4) = W(3,3);





%% ___________ Generation

% Loss and derivatives
x = sym('x', [4,1]);
syms x1 x2 x3 x4;
u = sym('u');

% definition of the runningCost
error = x-x_des;
error(1:2) = [wrap(x(1)-x_des(1));wrap(x(2)-x_des(2))];
runningCost = stateWeight*sqrt(error.'*W*error+alpha^2)-alpha +inputWeight*alpha^2*(cosh(u/alpha)-1); 

rC = matlabFunction(runningCost, 'vars', {[x(1), x(2), x(3), x(4)], u}, 'file', 'runningCostFunc','Optimize',false);

% definition of the finalCost
finalCost = stateWeightFinal*sqrt(error.'*W*error+alpha^2)-alpha;
fC = matlabFunction(finalCost, 'vars', {[x(1), x(2), x(3), x(4)]}, 'file', 'finalCostFunc','Optimize',false);

% running cost derivatives
lxSym = gradient(runningCost, x).';
lx = matlabFunction(lxSym, 'vars', {[x(1), x(2), x(3), x(4)]}, 'file', 'lxFunc');


luSym = gradient(runningCost, u).';
lu = matlabFunction(luSym, 'file', 'luFunc');

lxxSym = hessian(runningCost, x);
lxx = matlabFunction(lxxSym, 'vars', {[x(1), x(2), x(3), x(4)]}, 'file', 'lxxFunc');

luuSym = hessian(runningCost, u);
luu = matlabFunction(luuSym, 'file', 'luuFunc');

%final derivatives
fcxSym = gradient(finalCost, x).';
fcx = matlabFunction(fcxSym, 'vars', {[x(1), x(2), x(3), x(4)]}, 'file', 'fcxFunc');

fcxxSym = hessian(finalCost, x);
fcxx = matlabFunction(fcxxSym, 'vars', {[x(1), x(2), x(3), x(4)]}, 'file', 'fcxxFunc');

% Note: lux not present as it is 0

% dynamicsFunc

q1dd = (a3*(-u - a4*sin(x(1)) + a2*(x(3) + 2*x(3)*x(4) + x(4)^2)*sin(x(2)))+ ... 
 a2*cos(x(2))*(-u + a2*x(3)*sin(x(2)) + a5*sin(x(1) + x(2))))/((a1-a3)*a3-a2^2*cos(x(2))^2);

q2dd = ((a1 + 2*a2*cos(x(2)))*(u - a2*x(3)*sin(x(2)) - a5*sin(x(1) + x(2))) + (-a3 - ...
    a2*cos(x(2)))*(-a4*sin(x(1)) + a2*x(4)*(2*x(3) + x(4))*sin(x(2)) - ...
    a5*sin(x(1)+ x(2))))/((a1-a3)*a3-a2^2*cos(x(2))^2);

f = [x(1) + x(3)*dT; x(2)+x(4)*dT; ...
     x(3) + q1dd*dT;  x(4) + q2dd*dT];

 % Note the ";" to make it accept a vertical vector!
dynamicsFunc = matlabFunction(f, 'vars', {[x(1); x(2); x(3); x(4)], u}, 'file','dynamicsFunc.m');

% Derivatives of dynamics

fxSym = jacobian(f, x);
fx = matlabFunction(fxSym, 'vars', {[x(1), x(2), x(3), x(4)], u}, 'file', 'fxFunc');

fuSym = jacobian(f, u);
fu = matlabFunction(fuSym, 'vars', {[x(1), x(2), x(3), x(4)], u}, 'file', 'fuFunc');

    
    
