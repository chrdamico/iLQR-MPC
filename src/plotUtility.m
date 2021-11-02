%% Plot of iLQG evolution (copy this in main after input and state Suboptimal instruction to plot 
% the evolution of the iLQG solution at every iteration

% figure(1)
% 
% plot(stateOptimized.', 'linewidth',2), hold on
% plot(inputOptimized, 'linewidth',2)
% axis([0 1000 -11 11])
% title("Iteration "+numIterationILQG, 'interpreter', 'latex', 'FontSize', 14)
% legend({'$q_1$','$q_2$','$\dot{q}_1$','$\dot{q}_2$','u'}, 'interpreter', 'latex', 'FontSize', 14,'Location','southeast','Orientation','horizontal')
% %print("ZEROstatesILQG"+numIterationILQG,'-dpng','-r300')
% pause(0.01)
% clf

%% _________ Below here is some code that was used to generate the animations _________________

%% Repeat last frame states

for i=35:50
figure(1)
        plot(stateOptimized.', 'linewidth',2), hold on
        plot(inputOptimized, 'linewidth',2)
        axis([0 1000 -11 11])
        title("Iteration 34", 'interpreter', 'latex', 'FontSize', 14)
        legend({'$q_1$','$q_2$','$\dot{q}_1$','$\dot{q}_2$','u'}, 'interpreter', 'latex', 'FontSize', 14,'Location','southeast','Orientation','horizontal')
        % print("ZEROstatesILQG"+i,'-dpng', '-r300')
        clf
end

%% Repeat last frame acrobot

for i = 295:310
    showplot(xActual(:,end), l1, l2, num_figure)
    grid on
   % print(sprintf('./images/acrobotZEROinput%04d',i),'-dpng', '-r240')

end 