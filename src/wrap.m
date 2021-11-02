function ang_out=wrap(ang_in)

% wraps an angle so that it is contained
% in [-limit,limit)

limit = pi;

ang_out= mod(ang_in+limit,2*limit)-limit;