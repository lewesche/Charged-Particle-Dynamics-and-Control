%Particle Soup (3D) Control
%Leif Wesche

clear all
close all
clc
addpath('C:\Users\ASUS\Desktop\Github\Particle Soup')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Inputs  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dt=0.0333;        %Set Time Step (seconds)
run_time=60;    %Set Run Time (seconds)
t=[0:dt:run_time];    

m=[2];            %Particle Mass
qi=[2];           %Charge (Positive Only)
q_variance=0;     %Max Random Charge Variance
xi=[0];           %Initial X Position
yi=[0];           %Initial Y Position
zi=[-7];           %Initial Z Position
vxi=[0];          %Initial X Velocity
vyi=[0];          %Initial Y Velocity
vzi=[0];          %Initial Z Velocity

q0_max=50;          %Max Distributed Charge of Border Points
res=52;            %Total Number of Discrete Border Points
b=9;

x_desired=[0];     %Desired X Position
y_desired=[0];     %Desired Y Position
z_desired=[0];
Kp=4;               %PID Proportional Gain
Kd=1.75;             %PID Derivative Gain
Ki=1.5;             %PID Integral Gain

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



% LORENZE
sigma = 6;
beta = 8/4;
rho = 40;
f = @(tau,Lor) [-sigma*Lor(1) + sigma*Lor(2); rho*Lor(1) - Lor(2) - Lor(1)*Lor(3); -beta*Lor(3) + Lor(1)*Lor(2)];
[tau,Lor] = ode45(f,[0:1/30/5:12],[1 1 1]);     % Runge-Kutta 4th/5th order ODE solver
Lor_path=[Lor(:,1)'; Lor(:,2)'-6; Lor(:,3)'-40+5]/4.7;


 

path_desired=[];

F=[];
[sx, sy, sz]=sphere(15);
fig=figure('Position', [0, 0, 1080, 1080], 'color', [0, 0, 0]);
for i=t
    tc=round(i);
    
        %Calculate Desired Random Position
%     if round(mod(round(i, 2), 3), 1) == 0
%     xyz_desired=[randi([-5, 5]); randi([-5, 5]); randi([-5, 5])];
%     end
    
    xyz_desired=Lor_path(:,round((i+1/30)*30));
    path_desired=[path_desired, xyz_desired];
    
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
            int(j)=Ki*trapz([(i-5*dt):dt:i],E(j, end-5:end)).*sqrt(abs(e(j)));
        end
        end
        pid(j)=Kp*e(j) + Kd*(vxyz(j)-v_desired(j)) +int(j);
    end
    
    %Implement PID Control Law
    for p=[1:length(xyz0)/6]
        qleft(p)=-pid(1) *abs((xyz(1)+b))^1 / (1+500*exp(-d0*7/(norm([xyz(2)-xyz0(2,p), xyz(3)-xyz0(3,p)]))));
        qright(p)=pid(1) *abs((b-xyz(1)))^1 / (1+500*exp(-d0*7/(norm([xyz(2)-xyz0(2,(p+res^2)), xyz(3)-xyz0(3,(p+res^2))]))));
        qback(p)=-pid(2) *abs((xyz(2)+b))^1 / (1+500*exp(-d0*7/(norm([xyz(1)-xyz0(1,(p+2*res^2)), xyz(3)-xyz0(3,(p+2*res^2))]))));
        qfront(p)=pid(2) *abs((b-xyz(2)))^1 / (1+500*exp(-d0*7/(norm([xyz(1)-xyz0(1,(p+3*res^2)), xyz(3)-xyz0(3,(p+3*res^2))]))));
        qbottom(p)=-pid(3)*abs((xyz(3)+b))^1 / (1+500*exp(-d0*7/(norm([xyz(1)-xyz0(1,(p+4*res^2)), xyz(2)-xyz0(2,(p+4*res^2))]))));
        qtop(p)=pid(3) *abs((b-xyz(3)))^1 / (1+500*exp(-d0*7/(norm([xyz(1)-xyz0(1,(p+5*res^2)), xyz(2)-xyz0(2,(p+5*res^2))]))));
    end
    
    q0=10*[qleft, qright, qback, qfront, qbottom, qtop];
    
    %Limit Actuator Output
    q0=Actuator_Limits(q0, q0_max);
    q0(abs(q0)<0.05)=0; %Turn Off Low Output Actuators
    

        %Turn Off Actuators at Desired Position
    if abs(e(1))<=0.04 && abs(e(2))<0.004 && abs(e(3))<0.004 && norm(vxyz)<0.05
        q0=zeros(size(q0)); e(1)=0; e(2)=0; e(3)=0;
    end
    
    [xyz, vxyz] = Particle_Dynamics_3D(m, q, xyz, vxyz, xyz0, q0, i, dt);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  Animation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    %3D Plot  
    hold on; axis([-b-.2, b+.2, -b-.2, b+.2, -b-.2, b+.2])
    for j=[1:length(xyz0)]
        if q0(j)>=0
            s=surf(xyz0(1,j)+sx/7, xyz0(2,j)+sy/7, xyz0(3,j)+sz/7);
            s.FaceColor=([.01 .88 1]*q0(j)/qmax).^(1);  s.EdgeColor = 'none';
        end
        if q0(j)<0
            s=surf(xyz0(1,j)+sx/7, xyz0(2,j)+sy/7, xyz0(3,j)+sz/7);
            s.FaceColor=([1 0.01 0.01]*q0(j)/qmin).^(1);  s.EdgeColor = 'none';
        end
    end
    for j=[1:n]  
        s=surf(xyz(1,j)+sx*m(j)/mmax/2.5, xyz(2,j)+sy*m(j)/mmax/2.5, xyz(3,j)+sz*m(j)/mmax/2.5); 
        s.FaceColor=([.01 .88 1]*q(j)/qmax).^(1/4);  s.EdgeColor = 'none';
    end
    plot3(xyz_desired(1), xyz_desired(2), xyz_desired(3), 'wx', 'linewidth', 3)
    plot3(path_desired(1,:), path_desired(2,:), path_desired(3,:), 'w', 'linewidth', 1) %Plot Path
    grid on;
    camlight; lighting phong; 
    view([-90+90*cos(i/14+pi/4), 90*cos(i/14+pi/4)])
    title(['Particle Soup (3D)      ', '(X,Y,Z)_{Desired}=(', num2str(round([xyz_desired]', 2)), ')      e_{xyz}=(', num2str(round(e', 2)),  ')      Time Elapsed:' num2str(tc), 'Sec'], 'color', 'w')
    %xlabel('x', 'color', 'w'); ylabel('y', 'color', 'w'); zlabel('z', 'color', 'w');
    grid off
    axis off
    set(gca, 'Color', 'k', 'GridColor', 'w', 'XAxisLocation', 'origin'); hold off
    pause(dt); 
    
    
    %EZ Plot
%     hold on; axis([-b-.2, b+.2, -b-.2, b+.2, -b-.2, b+.2])
%     for j=[1:length(xyz0)]
%         if q0(j)>=0
%             plot3(xyz0(1,j), xyz0(2,j), xyz0(3,j), 'o', 'color', ([.01 .88 1]*q0(j)/qmax).^(2), 'linewidth', 5)
%         end
%         if q0(j)<0
%             plot3(xyz0(1,j), xyz0(2,j), xyz0(3,j), 'o', 'color', abs(([1 0.01 0.01]*q0(j)/qmax).^(2)), 'linewidth', 5)
%         end
%     end
%     for j=[1:n]  
%         plot3(xyz(1,j), xyz(2,j), xyz(3,j), 'bo', 'linewidth', 8)
%     end
%     plot3(xyz_desired(1), xyz_desired(2), xyz_desired(3), 'wx', 'linewidth', 3)
%     grid on;
%     camlight; lighting phong; 
%     %view([90,0])
%     view([30+10*i, 30+10*i])
%     title(['Particle Soup (3D)      ', '(X,Y,Z)_{Desired}=(', num2str(round([xyz_desired]', 2)), ')      e_{xyz}=(', num2str(round(e', 2)),  '      Time Elapsed:' num2str(tc), 'Sec'], 'color', 'w')
%     xlabel('x', 'color', 'w'); ylabel('y', 'color', 'w'); zlabel('z', 'color', 'w');
%     set(gca, 'Color', 'k', 'GridColor', 'w', 'XAxisLocation', 'origin'); hold off
%     pause(dt); clf

    F=[F, getframe(fig)]; clf
end
    %%
v=VideoWriter('Particle_Control_3D_18.avi','Uncompressed AVI');
open(v)
writeVideo(v,F)
close(v)
    
%% Example Paths
clc
close all

%Toroid
% a=3; b=3; n=20; u=t(end)/(2*pi);
% path_desired=[(a+b*cos(n*t/u)).*cos(t/u); (a+b*cos(n*t/u)).*sin(t/u); b*sin(n*t/u)];

%Double Sphere
% path_desired=[3*abs(sin(t/9.5)).*cos(2*t); 3*abs(sin(t/9.5)).*sin(2*t); t/run_time*8-4];

%Pokey Thing
% path_desired=[4*cos(2*t).^3; 4*sin(2*t).^3; 8*t/t(end)-4];

%Lorenze
sigma = 6;
beta = 8/4;
rho = 40;
f = @(tau,Lor) [-sigma*Lor(1) + sigma*Lor(2); rho*Lor(1) - Lor(2) - Lor(1)*Lor(3); -beta*Lor(3) + Lor(1)*Lor(2)];
[tau,Lor] = ode45(f,[0:1/30/5:12],[1 1 1]);     % Runge-Kutta 4th/5th order ODE solver
Lor_path=[Lor(:,1)'; Lor(:,2)'-6; Lor(:,3)'-40+5]/4.7;

path_desired=Lor_path;


plot3(path_desired(1,:), path_desired(2,:), path_desired(3,:))



