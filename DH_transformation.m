function DH = DH_transformation(alpha, a, d, theta)
%% Tony Su

% construct transformation matrix between joints based on DH parameters
% alpha - rotate about x axis
% a - translate along x axis
% d - translate along z axis
% theta - rotate about z axis
% DH - transformation matrix 4x4
% Here goes your implementation

% Simply following transformation matrix equation
DH = [cosd(theta), -sind(theta), 0, a; ...
    sind(theta)*cosd(alpha), cosd(theta)*cosd(alpha), -sind(alpha), -sind(alpha)*d; ...
    sind(theta)*sind(alpha), cosd(theta)*sind(alpha), cosd(alpha), cosd(alpha)*d; ...
    0 0 0 1];

end