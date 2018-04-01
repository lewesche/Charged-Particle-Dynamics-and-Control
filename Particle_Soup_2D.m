%Particle Soup (2D)
%Leif Wesche

clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.01;        %Set Time Step (seconds)
run_time=50;    %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[0.5, 3, 0.5, 1.5, 0.5, 0.5, 0.5,1,1];    %Particle Mass
qi=[1.5, 2.5, 1, 2, 1, 1.5, 1, 3, 2];       %Charge (Positive Only)
q_variance=0.5;                             %Max Random Charge Variance
xi=[7, 0, -4, 2, -1, 2, 3, 6, 9];           %Initial X Position
yi=[0, 0, 2, -6, -5, 1, -2, -9, 9];         %Initial Y Position
vxi=[0, 0, 0, 0, 0, 0, 0, 0, 0];            %Initial X Velocity
vyi=[0, 0, 0, 0, 0, 0, 0, 0, 0];            %Initial Y Velocity

q_box=80;   %Total Distributed Charge of Borders
res=20;      %Total Number of Discrete Border Points


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

for i=t
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
    title('Particle Soup (2D) by Leif Wesche')
    hold off
    pause(dt)
    
end

