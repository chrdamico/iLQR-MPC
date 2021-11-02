% Time discretization [s]
global dT
dT = 0.005;

% array of alpha coefficients
global line_search_parameters;
line_search_parameters = 10.^linspace(0,-3,11);

%  ______Backwards pass regularization parameters
global muMin;
muMin = 10^(-6);
muMax = 1e10;

global mu;
mu = 0;

global dmu;
dmu = 1;

global muFactor;
muFactor = 2;

global zMin;
zMin = 0;

global tolFun;
tolFun = 1e-4;

global cost;
cost = 0;

% lenght  of the robot arm (used in the drawing) [m]
l1 = 1;
l2 = 0.5;


% TimeHorizon
timeHorizon = 1000;



% Initial state 
x_init = [0;0;0;0];


% Set desired target configuration
x_des = [pi;0;0;0];


% Error tolerance to arrival at the target
tolErrorNorm = 0.01;


