function []=showplot(state, l1, l2, num_figure)
% Plotting is slow if done this way. I'd have to read how
% animations/backend is done to make it faster. I think this should be okay
% for no so that we can proceed

figure(num_figure),clf
hold on  

% Origin of the first link
x_pos = 0;
y_pos = 0;

% Generalized coordinates from the state
q1 = state(1);
q2 = state(2);

% Positin of the end of the first and second link
x1_pos=x_pos+l1*sin(q1);
y1_pos=y_pos-l1*cos(q1);

x2_pos=x1_pos+l2*sin(q2+q1);
y2_pos=y1_pos-l2*cos(q2+q1);


% First joint's circle
ph=plot(x_pos,y_pos,'go');
set(ph,'LineWidth',4)

% Second join'ts circle
ph=plot(x1_pos,y1_pos,'bo');
set(ph,'LineWidth',4)

% Links
lh=line([x_pos  x1_pos],[y_pos y1_pos]);
set(lh,'LineWidth',3)

lh=line([x1_pos  x2_pos],[y1_pos y2_pos]);
set(lh,'LineWidth',3)

% Axis setup
ws=l1+l2;
axis([-ws-0.5 ws+0.5 -ws-0.5 ws+0.5])			
axis('square')
hold off