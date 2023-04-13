                    %% Inverse Kinematics of RRR+wrist structure

function Theta=RRR_ikine(H)

    a2=10; d1=10; a3=15; d6=7;
    d=0;
%     dh=[0 d1 0 pi/2
%                    0 0 a2 0
%                    0 0 a3 0
%                    0 0 0 -pi/2
%                    0 0 0 pi/2
%                    0 d6 0 0];

    R=H(1:3,1:3);
    o=H(1:end-1,end);
    oc=o-d6*R*[0;0;1];
    xc=oc(1); yc=oc(2); zc=oc(3);
    
    % atan2(y,x)

    r=sqrt(xc^2+yc^2-d^2);
    s=zc-d1;
    
%     r^2 + s^2
    
    D=(r^2+s^2-a2^2-a3^2)/(2*a2*a3);
%     cosT3 = (cosT3 - 1) / 2
   
    t1=atan2(yc,xc);
    t3=atan2(sqrt(1-D^2),D);
    t2=atan2(s,r)-atan2(a3*sin(t3),a2+a3*cos(t3));

    theta4Y=-cos(t1)*sin(t2+t3)*R(1,3)-sin(t1)*sin(t2+t3)*R(2,3)+cos(t2+t3)*R(3,3);
    theta4X=cos(t1)*cos(t2+t3)*R(1,3)+sin(t1)*cos(t2+t3)*R(2,3);

    t4=atan2(theta4Y,theta4X);

    theta5X=sin(t1)*R(1,3)-cos(t1)*R(2,3);
    theta5Y=sqrt(1-theta5X^2);

    t5=atan2(theta5Y,theta5X);

    theta6Y=sin(t1)*R(1,2)-cos(t1)*R(2,2);
    theta6X=-sin(t1)*R(1,1)+cos(t1)*R(2,1);

    t6=atan2(theta6Y,theta6X);
    
    Theta=[t1 t2 t3 t4 t5 t6];

end