                            %% D-H function
                            % input: D-H params
                            % output Homogenous matrix and A
function [H,A]=DH(dh)

    m=size(dh,1); % number of rows
    theta=dh(:,1);
    d=dh(:,2);
    a=dh(:,3);
    alpha=dh(:,4);
    
    A{1,m}=[];
    H=eye(4);
    
    for i=1:m
          A{i} = R(theta(i),'z')*T(d(i),'z')*T(a(i),'x')*R(alpha(i),'x');
          H=H*A{i}; 
    end

end