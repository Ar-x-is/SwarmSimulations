%% Wind
wind_lim=[-5;5];
wind=[sample(wind_lim);sample(wind_lim)];
wind=[0;0];
%% Mission Parameters
%wps=[[0;0],[5000;0],[0;200]];
%wps=[[0;0],[10000;0],[10000;1000],[0;1000],[0;2000],[10000;2000],[10000;3000],[0;3000],[0;4000],[10000;4000],[10000;5000],[0;5000],[0;6000],[10000;6000],[10000;7000],[0;7000],[0;8000],[10000;8000],[10000;9000],[0;9000],[0;10000],[10000;10000],[12000;12000]];
wps=[[0;0],[0;10000]]
v_nav=20;
v_nav_lim=5;
loiter_radius=1000;
[~,ts]=reference_trajectory(0,wps,v_nav,loiter_radius)
%t_span=[0,ts(end)+2*pi*loiter_radius/v_nav]
%tspan=linspace(0,500,10000);
tspan=[0,350];
%% UAV Parameters
tp_lim=[0;2];
%tp=sample(tp_lim)
tp=2;

ta_lim=[0;2];
%ta=sample(ta_lim)
ta=2;
max_speed_lim=[45;45];
min_speed_lim=[8;8];
v_lim_ult=[8,45];
%v_lim=[v_nav-v_nav_lim;v_nav+v_nav_lim];
v_lim=[10,40];
%v_lim=[10,40]
phi_max_lim=[25;45]*pi/180;
%phi_max=sample(phi_max_lim)
phi_max=30*pi/180;

max_a_lim=[4;4];
min_a_lim=[-1;-1];
a_lim=[-1;4];
%a_lim=[-1,4]
%%
function x=sample(lim)
x=lim(1)+rand()*(lim(2)-lim(1));
end