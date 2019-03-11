% ***
% Given a series of SE matrices T, where T{1} is location of the base, 
% joint types j [...], simulate(...) creates a 3D representation of the robot
% j: 'u' = universal, 'p' = prismatic, 'c' = cylindrical

function out = simulate(T,j)
%% define visual paramters here
axis([-5 5 -5 5 -.2 5])
base_thickness = .2;
base_rad = 1;
link_thick = [.3,.15,.15,.1,.05];
cw = 4;
%% functional
[X,Y,Z] = cylinder(base_rad,100);
X = X + T{1}(1,4);
Y = Y + T{1}(2,4);
Z = Z + T{1}(3,4);
hold on
a = surf(X,Y,-Z*base_thickness);
a.EdgeAlpha = 0;
a.FaceColor = [.5 .5 .5];
a = fill(X(:),Y(:),[.5 .5 .5]);

for i = 1:length(T)-1
    [X,Y,Z] = cylinder(link_thick(i),cw);
    for j = 1:cw+1
        out = rotmat('z',pi/4)*[X(1,j);Y(1,j);zeros(size(Z(1,j)))];
        out = T{i}(1:3,4)+out;      
        X(1,j) = out(1);
        Y(1,j) = out(2);
        Z(1,j) = out(3);
        out = rotmat('z',pi/4)*[X(2,j);Y(2,j);zeros(size(Z(2,j)))];
        out = T{i+1}(1:3,4) + out;
        X(2,j) = out(1);
        Y(2,j) = out(2);
        Z(2,j) = out(3);
    end
    a = surf(X,Y,Z);
    a.FaceColor = [.7 .7 .8];
end

end