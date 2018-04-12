%Particle Soup (2D) Control
%Leif Wesche

clear all
close all
clc
addpath('C:\Users\ASUS\Desktop\Github\Particle Soup')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.0333/2;        %Set Time Step (seconds)
run_time=15;         %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[1];              %Particle Mass
qi=[5];             %Charge (Positive Only)
q_variance=0.5;     %Max Random Charge Variance
xi=[0];             %Initial X Position
yi=[0];             %Initial Y Position
vxi=[0];            %Initial X Velocity
vyi=[0];           %Initial Y Velocity

q0_max=45;          %Total Distributed Charge of Borders
res=44;             %Total Number of Discrete Border Points
b=8;

x_desired=[-1];     %Desired X Position
y_desired=[2];      %Desired Y Position
Kp=3.5;               %PID Proportional Gain
Kd=2.5;             %PID Derivative Gain
Ki=1.5;             %PID Integral Gain

opt=1;              %Only Use closest actuators? 1=yes, 0=no
                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Math  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

res=res/4;
x0=[linspace(-b, b, res), b*ones(1,res), linspace(b, -b, res), -b*ones(1,res)];
y0=[-b*ones(1,res), linspace(-b, b, res), b*ones(1,res), linspace(b, -b, res)];
xy0=[x0; y0];
d0=(2*b)/(res-1)/opt; %Distance Between Border Charges
if d0>=1.7778
    d0=1.7778;
end

qmax=[qi, q0_max]; qmax=max(qmax); qmax=qmax+q_variance; 
qmin=[qi, q0_max]; qmin=min(qmin); qmin=qmin-q_variance; 
mmax=max(m);

n=length(m);

x=xi;
y=yi;
xy=[x; y];
vx=vxi;
vy=vyi;
v=[vx; vy];
E=[x-x_desired; y-y_desired];
x_desired_prev=[0]; y_desired_prev=[0];         v_desired=[0,0];
F=[];
fig=figure('Position', [0, 0, 1920, 1080]);
%Plot Path
xpath=tan(sin(t)+cos(t));
ypath=tan(sin(t)-cos(t));

for i=t
    tc=round(i);
    %Calculate Desired Position
    x_desired=tan(sin(i)+cos(i));
    y_desired=tan(sin(i)-cos(i));

%     %Calculate Desired Position
%     if mod(i/dt, 2*60) == 0
%     x_desired=randi([-3, 3]);
%     y_desired=randi([-3, 3]);
%     end
    
    %Apply Charge Variance
    if mod(i,0.25) == 0
        for j=[1:n]
        var=(rand-0.5)*q_variance;
        q(j)=qi(j)+var;
        end
    end
    %Calculate Error
    e=[x-x_desired; y-y_desired];
    E=[E, e];
    %Calculate Velocity of Desired Position
    v_desired(1)=diff([x_desired_prev, x_desired])/dt;
    v_desired(2)=diff([y_desired_prev, y_desired])/dt; 
    x_desired_prev=x_desired; y_desired_prev=y_desired; 
    
    %Implement PID Control Law
    if i==0
        int=[0, 0];
    else
    if i/dt <= 5
        int(1)=Ki*trapz([0:dt:i],E(1, 2:end));
        int(2)=Ki*trapz([0:dt:i],E(2, 2:end));
    else
        int(1)=Ki*trapz([(i-5*dt):dt:i],E(1, end-5:end));
        int(2)=Ki*trapz([(i-5*dt):dt:i],E(2, end-5:end));
    end
    end
    pid(1)=Kp*e(1) + Kd*(vx-v_desired(1)) +int(1);
    pid(2)=Kp*e(2) + Kd*(vy-v_desired(2)) +int(2);
    
    
    
    %Prioritize Closest Actuators
    for p=[1:length(xy0)/4]
        qb(p)=-pid(2) * (y-y0(p))^2 / (1+500*exp(-d0*7/(opt*abs(x-x0(p)))));
        qr(p)=pid(1) * (x-x0(p+res))^2 / (1+500*exp(-d0*7/(opt*abs(y-y0(p+res)))));
        qt(p)=pid(2) * (y-y0(p+2*res))^2 / (1+500*exp(-d0*7/(opt*abs(x-x0(p+2*res)))));
        ql(p)=-pid(1) * (x-x0(p+3*res))^2 / (1+500*exp(-d0*7/(opt*abs(y-y0(p+3*res)))));
    end
    q0=[qb, qr, qt, ql];
    
    %Limit Actuator Output
    q0=Actuator_Limits(q0, q0_max);
    q0(abs(q0)<0.02)=0; %Turn Off Low Output Actuators
    
    %Turn Off Actuators at Desired Position
    if abs(mean(q0)) <=0.02 && abs(e(1))<=0.004 && abs(e(2))<0.004
        q0=zeros(size(q0)); e(1)=0; e(2)=0;
    end

    [x, y, vx, vy, KE] = Particle_Dynamics_2D(m, q, x, y, vx, vy, xy0, q0, i, dt);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Animation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    for i=[1:length(q0)]
    if  q0(i)>=0  
       plot(x0(i), y0(i), 'o', 'markeredgecolor', ([.01 .88 1]*q0(i)/qmax).^(1/4), 'markerfacecolor', ([.01 .88 1]*q0(i)/qmax).^(1/4), 'linewidth', 5); hold on
    else 
       plot(x0(i), y0(i), 'o', 'markeredgecolor', (abs([1 0.01 0.01]*q0(i)/qmax)).^(1/4), 'markerfacecolor', (abs([1 0.01 0.01]*q0(i)/qmax)).^(1/4), 'linewidth', 5); hold on
    end
    end
    plot(x_desired, y_desired, 'wx', 'linewidth', 3);  plot(xpath, ypath, 'w--')
    axis([-10, 10, -10, 10])
    for i=[1:n]  
        plot(x(i), y(i), 'o', 'markeredgecolor',  ([.01 .95 1]*q(i)/qmax).^(1/4), 'markerfacecolor', ([.01 .95 1]*q(i)/qmax).^(1/4), 'linewidth', (m(i)*10/mmax)); hold on
    end
    axis([-b, b, -b, b]) 
    grid on
    title(['Particle Soup (2D)      ', '(X,Y)_{Desired}=(', num2str(round([x_desired, y_desired], 2)), ')      e_{xy}=(', num2str([round(e(1),2), round(e(2),2)]), ')      KE=', num2str(KE), '      Time Elapsed:' num2str(tc), 'Sec'])
    set(gca, 'Color', 'k', 'GridColor', 'w')
    hold off
    pause(dt)
%    F=[F, getframe(fig)];    
end

% v=VideoWriter('Particles_Controlled_2D_5.avi','Uncompressed AVI');
% v.FrameRate = 60;
% open(v)
% writeVideo(v,F)
% close(v)


%% 

figure
plot(1,1, 'wo')
set(gca, 'color', 'k')
