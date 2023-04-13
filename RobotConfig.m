                       %% Robot Configuration
                       % using robotic toolbox

function Robot=RobotConfig(dh)
    
    for i=1:size(dh,1)
        L(i)= Link(dh(i,:));
    end
    Robot=SerialLink(L);
end