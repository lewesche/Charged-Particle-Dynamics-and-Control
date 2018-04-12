%Particle Soup (3D)
%Leif Wesche

clear all
close all
clc
addpath('C:\Users\ASUS\Desktop\Github\Particle Soup')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.0333/2;        %Set Time Step (seconds)
run_time=50;    %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[1, 1, 1, 1.5, 0.75];        %Particle Mass
qi=[1, 1, 1.25, 1.5, 1];       %Charge (Positive Only)
q_variance=0;         %Max Random Charge Variance
xi=[-0.5, 0, 3, -2, 1];          %Initial X Position
yi=[0, 0.5, 0, -1, 3];            %Initial Y Position
zi=[-0.5, 0.5, -1, 3, 1];            %Initial Z Position
vxi=[1, -1, 0, 0, 1];          %Initial X Velocity
vyi=[1, -1, 1, 0, 0];          %Initial Y Velocity
vzi=[1, -1, 0, -1, -1];          %Initial Z Velocity

q_box=60;       %Total Distributed Charge of Borders
res=36;         %Total Number of Discrete Border Points
b=4;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Math  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

q_box=q_box/4; res=res/4;
q0=q_box/res;

[xyz0] = Square_Border_Geometry_3D(b, res);

qmax=[qi, q0]; qmax=max(qmax); qmax=qmax+q_variance; 
qmin=[qi, q0]; qmin=min(qmin); qmin=qmin-q_variance; 
mmax=max(m);

n=length(m);

xyz=[xi; yi; zi];
vxyz=[vxi; vyi; vzi];

for i=t
    tc=round(i);
    %Apply Charge Variance
    if mod(i,0.25) == 0
        for j=[1:n]
        var=(rand-0.5)*q_variance;
        q(j)=qi(j)+var;
        end
    end
    
    [xyz, vxyz] = Particle_Dynamics_3D(m, q, xyz, vxyz, xyz0, q0, i, dt);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Animation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    scatter3(xyz0(1,:), xyz0(2,:), xyz0(3,:), 'o', 'markeredgecolor', [.01 .88 1]*q0/qmax, 'markerfacecolor', [.01 .88 1]*q0/qmax, 'linewidth', 3); hold on
    axis([-b, b, -b, b, -b, b])
    for j=[1:n]  
    scatter3(xyz(1,j), xyz(2,j), xyz(3,j), 'o', 'markeredgecolor',  [.01 .95 1]*q(j)/qmax, 'markerfacecolor', [.01 .95 1]*q(j)/qmax, 'linewidth', (m(j)*20/mmax)); hold on
    end
    grid on; 
    view([30+10*i, 30+10*i])
    title(['Particle Soup (3D)      ', 'Time Elapsed:' num2str(tc), 'Sec'])
    set(gca, 'Color', 'k', 'GridColor', 'w');
    hold off
    pause(dt)
    
end

