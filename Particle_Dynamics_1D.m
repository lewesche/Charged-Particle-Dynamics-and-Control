%Particle Soup (1D) Dynamics Function
%Leif Wesche

function [x, vx] = Particle_Dynamics_1D(m, q, x, vx, b, ql, qr, i, dt)

n=length(m);

    for j=[1:n]
        %Calculate Force From Boundaries
        rl=[(x(j)+b/2)];    rr=[(x(j)-b/2)];
        F(j)=(q(j)*ql)/(norm(rl)^3)*rl + (q(j)*qr)/(norm(rr)^3)*rr;        
        k=[1:n];
        k(j)=[];
        %Calculate Force from Particles
        for k=k
            r=[(x(j)-x(k))];
            F(j)=F(j) + (q(j)*q(k))/(norm(r)^3)*r;
        end
        a(j)=F(j)/m(j);
        vx(j)=vx(j)+a(j)*dt;
        x(j)=x(j)+vx(j)*dt;
    end
end