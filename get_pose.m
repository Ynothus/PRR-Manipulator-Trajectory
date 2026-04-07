function pose = get_pose(DH, q)
% get the pose at given joint configuration q using transformation matrix DH
% - q is nx1 vector, where n is the number of joints
% - DH is a 4x4 transformation matrix from the end-effector to the base frame
% - pose is 6x1 vector, including 3x1 postion and 3x1 orientation

q_sym = symvar(DH);                 % get symbolic joint variables
T = double(subs(DH, q_sym, q'));    % get the T matrix
position = T(1:3, 4);               % get position
K = R2K(T(1:3, 1:3));               % get orientation
pose = [position; K];               % get 6x1 vector including position and orientation
end

function K = R2K(R)
% convert R matrix to K*theta representation
theta = acos((R(1, 1)+R(2,2)+R(3,3)-1)/2);
K_hat = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)]./(2*sin(theta));
K = theta*K_hat;
end