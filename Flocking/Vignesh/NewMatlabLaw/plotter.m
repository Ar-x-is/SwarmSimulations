%% Preliminaries
size(y)
data=zeros(length(t),6*N+1);
data(:,1)=t;
data(:,2:end)=y;
writematrix(data,'data11.csv');
%% Plot Trajectories
plot(y(:,1:6:end),y(:,2:6:end),'-')
axis('equal')
%legend('1','2','3','4')
%% Plot final shape
plot(y(end,1:6:end),y(end,2:6:end),'o')
axis('equal')
% %% Plot Time Signals
% plot(t,y(:,5:6:end)*180/pi)
% %% Plot
% xlim([-1000,2000])
% ylim([-8000,1000])
% %% Distances
% ds=zeros(length(t),1);
% i=1;
% while i<=length(t)
%     ds(i)=norm(y(i,1:2)-y(i,7:8));
%     i=i+1;
% end
% size(ds)
% plot(t,ds)