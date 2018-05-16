%Particle Dynamics (3D) Function
%Leif Wesche

function [xyz, vxyz, E] = Particle_Dynamics_3D(m, q, xyz, vxyz, xyz0, q0, i, dt)

    n=length(m);

    F=zeros(n,3); PE=0;
    
    for j=[1:n]
        %Calculate Force from Boundaries
        for p=[1:length(xyz0)] 
            r=[xyz(:,j)-xyz0(:,p)]';
            F(j,:)=F(j,:) +  (q(j)*q0(p))/(norm(r)^3) * r;
        end
        k=[1:n];
        k(j)=[];
        %Calculate Force from Particles
        for k=k
            r=[xyz(:,j)-xyz(:,k)]';
            F(j,:)=F(j,:) +  (q(j)*q(k))/(norm(r)^3) * r;
        end
    end
    F=F';
        %Calculate Acceleration, Velocity, Position
        for j=[1:n]
            a(:,j)=F(:,j)./m(n);
        end
        vxyz=vxyz+a.*dt;
        xyz=xyz+vxyz.*dt;
end


