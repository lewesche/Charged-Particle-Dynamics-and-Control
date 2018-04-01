%Particle Soup (1D)
%Leif Wesche

clear all
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.01;            %Set Time Step (seconds)
run_time=20;        %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[1, 1, 2, 0.75, 1];       %Particle Mass
qi=[1, 2, 1, 0.75, 1];      %Charge (Positive Only)
q_variance=0.5;             %Max Random Charge Variance
xi=[2, -3, 1, 4, -1];       %Initial X Position
vxi=[2, -1, 0, 0, 0];       %Initial X Velocity

ql=2;       %Left End Bounary Charge
qr=2;       %Right End Boundary Charge
b=10;       %Boundary Size

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Math  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

qmax=[qi, ql, qr]; qmax=max(qmax); qmax=qmax+q_variance; 
qmin=[qi, ql, qr]; qmin=min(qmin); qmin=qmin-q_variance; 
mmax=max(m);

x=xi;
vx=vxi;

n=length(m);

for i=t
    %Apply Charge Variance
    if mod(i,0.25) == 0
        for j=[1:n]
        var=(rand-0.5)*q_variance;
        q(j)=qi(j)+var;
        end
    end
    %Calculate Dynamics
    [x, vx]=Particle_Dynamics_1D(m, q, x, vx, b, ql, qr, i, dt);

   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Animation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    
   plot(-b/2, 0, 'o', 'markeredgecolor', [.01 .88 1]*ql/qmax, 'markerfacecolor', [.01 .88 1]*ql/qmax, 'linewidth', 4); hold on
   plot(b/2, 0, 'o', 'markeredgecolor', [.01 .88 1]*qr/qmax, 'markerfacecolor', [.01 .88 1]*qr/qmax, 'linewidth', 4); hold on
   axis([-b/2, b/2, -1, 1])  
   for i=[1:n]  
   plot(x(i), 0, 'o', 'markeredgecolor',  [.01 .88 1]*q(i)/qmax, 'markerfacecolor', [.01 .88 1]*q(i)/qmax, 'linewidth', (m(i)*20/mmax)); hold on
   end
   grid on
   hold off
   pause(dt)
end









