%Particle Soup (3D) Control
%Leif Wesche

clear all
close all
clc
addpath('C:\Users\ASUS\Desktop\Github\Particle Soup')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.0333;        %Set Time Step (seconds)
run_time=20;    %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[1];        %Particle Mass
qi=[1];       %Charge (Positive Only)
q_variance=0;         %Max Random Charge Variance
xi=[1];          %Initial X Position
yi=[1.5];            %Initial Y Position
zi=[-2];            %Initial Z Position
vxi=[0];          %Initial X Velocity
vyi=[-0.75];          %Initial Y Velocity
vzi=[1];          %Initial Z Velocity

q0_max=30;          %Total Distributed Charge of Borders
res=40;             %Total Number of Discrete Border Points
b=5;

x_desired=[0];     %Desired X Position
y_desired=[0];      %Desired Y Position
z_desired=[0];
Kp=4.5;               %PID Proportional Gain
Kd=2;             %PID Derivative Gain
Ki=2;             %PID Integral Gain

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Math  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

q0=0; res=res/4; d0=2*b/(res-1);

[xyz0] = Square_Border_Geometry_3D(b, res);

qmax=[qi, q0, q0_max]; qmax=max(qmax); qmax=qmax+q_variance; 
qmin=[qi, q0, -q0_max]; qmin=min(qmin); qmin=qmin-q_variance; 
mmax=max(m);

n=length(m);

xyz=[xi; yi; zi];
vxyz=[vxi; vyi; vzi];
xyz_desired=[x_desired; y_desired; z_desired];
E=[xyz-xyz_desired];
xyz_desired_prev=[0; 0; 0]; v_desired=[0;0;0];


F=[];
[sx, sy, sz]=sphere(15);
fig=figure('Position', [0, 0, 1080, 1080], 'color', [0, 0, 0]);
for i=t
    tc=round(i);
    
        %Calculate Desired Position
    if mod(i/dt, 2*60) == 0
    xyz_desired=[randi([-4, 4]); randi([-4, -4]); randi([-4, 4])];
    end
    
    %Apply Charge Variance
    if mod(i,0.25) == 0
        for j=[1:n]
        var=(rand-0.5)*q_variance;
        q(j)=qi(j)+var;
        end
    end
    
    %Calculate Error
    e=[xyz-xyz_desired];
    E=[E, e];
    
    %Calculate Velocity of Desired Position
    v_desired=[diff([xyz_desired_prev'; xyz_desired'])/dt]';
    xyz_desired_prev=xyz_desired;
    
    %Calculate Error Integral
    for j=1:3
        if i==0
            int=[0, 0, 0];
        else
        if i/dt <= 5
            int(j)=Ki*trapz([0:dt:i],E(j, 2:end));
        else
            int(j)=Ki*trapz([(i-5*dt):dt:i],E(j, end-5:end));
        end
        end
        pid(j)=Kp*e(j) + Kd*(vxyz(j)-v_desired(j)) +int(j);
    end
    [pid(1), pid(2), pid(3)]
    %Implement PID Control Law
    for p=[1:length(xyz0)/6]
        qleft(p)=-pid(1) *abs((xyz(1)-b))^3 ;%/ (1+500*exp(-d0*7/(norm([xyz(2)-xyz0(2,p), xyz(3)-xyz0(3,p)]))));
        qright(p)=pid(1) *abs((xyz(1)+b))^3 ;%/ (1+500*exp(-d0*7/(norm([xyz(2)-xyz0(2,(p+res)), xyz(3)-xyz0(3,(p+res))]))));
        qback(p)=-pid(2) *abs((xyz(2)-b))^3 ;%/ (1+500*exp(-d0*7/(norm([xyz(1)-xyz0(1,(p+2*res)), xyz(3)-xyz0(3,(p+2*res))]))));
        qfront(p)=pid(2) *abs((xyz(2)+b))^3 ;%/ (1+500*exp(-d0*7/(norm([xyz(1)-xyz0(1,(p+3*res)), xyz(3)-xyz0(3,(p+3*res))]))));
        qbottom(p)=-pid(3)*abs((xyz(3)-b))^3 ;%/ (1+500*exp(-d0*7/(norm([xyz(1)-xyz0(1,(p+4*res)), xyz(2)-xyz0(2,(p+4*res))]))));
        qtop(p)=pid(3) *abs((xyz(3)+b))^3 ;%/ (1+500*exp(-d0*7/(norm([xyz(1)-xyz0(1,(p+5*res)), xyz(2)-xyz0(2,(p+5*res))]))));
    end
    
    q0=[qleft, qright, qback, qfront, qbottom, qtop];
    
    %Limit Actuator Output
    q0=Actuator_Limits(q0, q0_max);
    q0(abs(q0)<0.05)=0; %Turn Off Low Output Actuators
    
        %Turn Off Actuators at Desired Position
    if abs(mean(q0)) <=1 && abs(e(1))<=0.04 && abs(e(2))<0.004 && abs(e(3))<0.004
        q0=zeros(size(q0)); e(1)=0; e(2)=0; e(3)=0;
    end
    
    [xyz, vxyz] = Particle_Dynamics_3D(m, q, xyz, vxyz, xyz0, q0, i, dt);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Animation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    hold on; axis([-b-.2, b+.2, -b-.2, b+.2, -b-.2, b+.2])
    for j=[1:length(xyz0)]
        if q0(j)>=0
            s=surf(xyz0(1,j)+sx/7, xyz0(2,j)+sy/7, xyz0(3,j)+sz/7);
            s.FaceColor=([.01 .88 1]*q0(j)/qmax).^(1/2);  s.EdgeColor = 'none';
        end
        if q0(j)<0
            s=surf(xyz0(1,j)+sx/7, xyz0(2,j)+sy/7, xyz0(3,j)+sz/7);
            s.FaceColor=([1 0.01 0.01]*q0(j)/qmin).^(1/2);  s.EdgeColor = 'none';
        end
    end
    for j=[1:n]  
        s=surf(xyz(1,j)+sx*m(j)/mmax/2.5, xyz(2,j)+sy*m(j)/mmax/2.5, xyz(3,j)+sz*m(j)/mmax/2.5); 
        s.FaceColor=([.01 .88 1]*q(j)/qmax).^(1/4);  s.EdgeColor = 'none';
    end
    plot3(xyz_desired(1), xyz_desired(2), xyz_desired(3), 'wx', 'linewidth', 3)
    grid on;
    camlight; lighting phong; view([30+10*i, 30+10*i])
    title(['Particle Soup (3D)      ', '(X,Y,Z)_{Desired}=(', num2str(round([xyz_desired]', 2)), ')      e_{xyz}=(', num2str(round(e', 2)),  '      Time Elapsed:' num2str(tc), 'Sec'], 'color', 'w')
    xlabel('x', 'color', 'w'); ylabel('y', 'color', 'w'); zlabel('z', 'color', 'w');
    set(gca, 'Color', 'k', 'GridColor', 'w', 'XAxisLocation', 'origin'); hold off
    pause(dt); clf
    
end
    
    
    
   
