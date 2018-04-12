%Particle Soup (2D)
%Leif Wesche

clear all
close all
clc
addpath('C:\Users\ASUS\Desktop\Github\Particle Soup')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.0333/2;        %Set Time Step (seconds)
run_time=20;    %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[1, 1, 1, 1]/2;              %Particle Mass
qi=[1, 1, 1, 1];             %Charge (Positive Only)
q_variance=0;       %Max Random Charge Variance
xi=[6, 6, -6, -6];             %Initial X Position
yi=[6, -6, -6, 6];             %Initial Y Position
vxi=[0, -1, 0, 1]*3;            %Initial X Velocity
vyi=[-1, 0, 1, 0]*3;            %Initial Y Velocity

q_box=140;    %Total Distributed Charge of Borders
res=88;      %Total Number of Discrete Border Points
b=10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Math  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

q_box=q_box/4; res=res/4;
x0=[linspace(-b, b, res), b*ones(1,res), linspace(b, -b, res), -b*ones(1,res)];
y0=[-b*ones(1,res), linspace(-b, b, res), b*ones(1,res), linspace(b, -b, res)];
xy0=[x0; y0];
q0=q_box/res*ones(1, length(x0));

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
fig=figure('Position', [0, 0, 1920, 1080]);
for i=t
    tc=round(i);
    %Apply Charge Variance
    if mod(i,0.25) == 0
        for j=[1:n]
        var=(rand-0.5)*q_variance;
        q(j)=qi(j)+var;
        end
    end
    
    [x, y, vx, vy, KE, PE, E] = Particle_Dynamics_2D(m, q, x, y, vx, vy, xy0, q0, i, dt);

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Animation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    for i=[1:length(q0)]
    plot(x0, y0, 'o', 'markeredgecolor', [.01 .88 1]*q0(i)/qmax, 'markerfacecolor', [.01 .88 1]*q0(i)/qmax, 'linewidth', 3); hold on
    end
    axis([-b, b, -b, b])
    for i=[1:n]  
    plot(x(i), y(i), 'o', 'markeredgecolor',  [.01 .95 1]*q(i)/qmax, 'markerfacecolor', [.01 .95 1]*q(i)/qmax, 'linewidth', (m(i)*20/mmax)); hold on
    end
    grid on
    title(['Particle Soup (2D)      ', 'KE=', num2str(KE), '      PE=', num2str(PE), '      E=', num2str(E), '      Time Elapsed:' num2str(tc), 'Sec'])
    set(gca, 'Color', 'k', 'GridColor', 'w');
    hold off
    pause(dt)
%     F=[F, getframe(fig)];
end

% v=VideoWriter('Particle_Soup_2D_1.avi','Uncompressed AVI');
% open(v)
% writeVideo(v,F)
% close(v)