%Particle Soup (2D)
%Leif Wesche

clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.0333;        %Set Time Step (seconds)
run_time=25;    %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[1, 1, 2.8, 1, 1, 3, 1, 1, 2.2, 2, 1, 2, 2.5, 1, 3, 1];    %Particle Mass
qi=[1, 1.7, 1, 0.8, 1, 2, 1, 1.2, 1, 1.9, 2.2, 1, 1.5, 1, 1.6, 1];       %Charge (Positive Only)
q_variance=0;                             %Max Random Charge Variance
xi=[-4, -4, -4, -4, -2, -2, -2, -2, 2, 2, 2, 2, 4, 4, 4, 4];           %Initial X Position
yi=[-4, -2, 2, 4, -4, -2, 2, 4, -4, -2, 2, 4, -4, -2, 2, 4];         %Initial Y Position
vxi=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];            %Initial X Velocity
vyi=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];            %Initial Y Velocity

q_box=100;   %Total Distributed Charge of Borders
res=240;      %Total Number of Discrete Border Points


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Math  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

q_box=q_box/4; res=res/4;
q0=q_box/res;
x0=[linspace(-10, 10, res), 10*ones(1,res), linspace(10, -10, res), -10*ones(1,res)];
y0=[-10*ones(1,res), linspace(-10, 10, res), 10*ones(1,res), linspace(10, -10, res)];
xy0=[x0; y0];

qmax=[qi, q0]; qmax=max(qmax); qmax=qmax+q_variance; 
qmin=[qi, q0]; qmin=min(qmin); qmin=qmin-q_variance; 
mmax=max(m);

n=length(m);

x=xi;
y=yi;
xy=[x; y];
vx=vxi;
vy=vyi;
v=[vx; vy];
F=[];
fig=figure;
set(gcf, 'Position', [0, 0, 1920, 1080]);
for i=t
    tc=round(i);
    %Apply Charge Variance
    if mod(i,0.25) == 0
        for j=[1:n]
        var=(rand-0.5)*q_variance;
        q(j)=qi(j)+var;
        end
    end
    
    [x, y, vx, vy] = Particle_Dynamics_2D(m, q, x, y, vx, vy, xy0, q0, i, dt);

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Animation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    plot(x0, y0, 'o', 'markeredgecolor', [.01 .88 1]*q0/qmax, 'markerfacecolor', [.01 .88 1]*q0/qmax, 'linewidth', 3); hold on
    axis([-10, 10, -10, 10])
    for i=[1:n]  
    plot(x(i), y(i), 'o', 'markeredgecolor',  [.01 .95 1]*q(i)/qmax, 'markerfacecolor', [.01 .95 1]*q(i)/qmax, 'linewidth', (m(i)*30/mmax)); hold on
    end
    grid on
    title(['Particle Soup (2D)      ', 'Time Elapsed:' num2str(tc), 'Sec'])
    hold off
    pause(dt)
%     F=[F, getframe(fig)];
end

% v=VideoWriter('Particle_Soup_2D.avi','Uncompressed AVI');
% open(v)
% writeVideo(v,F)
% close(v)