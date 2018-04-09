%Particle Soup (1D)
%Leif Wesche

clear all
close all
clc
addpath('C:\Users\ASUS\Desktop\Github\Particle Soup')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.0333/2;            %Set Time Step (seconds)
run_time=25;        %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[1.5, 1, 1.25];       %Particle Mass
qi=[1.5, 2, 1];      %Charge (Positive Only)
q_variance=0;             %Max Random Charge Variance
xi=[2, -3, 0];       %Initial X Position
vxi=[1, -2, -2];       %Initial X Velocity

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
F0=zeros(size(m));
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
    %Calculate Dynamics
    [x, vx, KE, PE, ET]=Particle_Dynamics_1D(m, q, x, vx, b, ql, qr, i, dt);

   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Animation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    
   plot(-b/2, 0, 'o', 'markeredgecolor', [.01 .88 1]*ql/qmax, 'markerfacecolor', [.01 .88 1]*ql/qmax, 'linewidth', 4); hold on
   plot(b/2, 0, 'o', 'markeredgecolor', [.01 .88 1]*qr/qmax, 'markerfacecolor', [.01 .88 1]*qr/qmax, 'linewidth', 4); hold on
   axis([-b/2, b/2, -1, 1])  
   for i=[1:n]  
   plot(x(i), 0, 'o', 'markeredgecolor',  [.01 .88 1]*q(i)/qmax, 'markerfacecolor', [.01 .88 1]*q(i)/qmax, 'linewidth', (m(i)*20/mmax)); hold on
   end
   grid on
   title(['Particle Soup (1D)       KE=', num2str(KE), '       PE=', num2str(PE),   '       E=', num2str(ET) '       Time Elapsed:' num2str(tc), 'Sec'])
   hold off
   pause(dt)
   %Save Video
%    F=[F, getframe(fig)];
end

% v=VideoWriter('Particle_Soup_1D_1.avi','Uncompressed AVI');
% open(v)
% writeVideo(v,F)
% close(v)




