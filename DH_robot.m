function [DH_tf, DH_list] = DH_robot(DH_parameters)
% create transformation matrix using DH table
% DH_parameters - nx4, where n is the number of frames
% DH_tf - 4x4 homogeneous transformation matrix from end-effector to base frame
% DH_list - nx1 cell, each cell includes a 4x4 transformation matrix between neighboring frames
% Here goes your implementation

    % Initalizing variables
    TEnd = eye(4);
    Trans = {};
    numFrames = size(DH_parameters, 1);
    % Matrix multiplying each transformation to find end effector to base
    % transformation matrix and indexing each transformation
    for i = 1:numFrames
        Trans{i,1} = DH_transformation(DH_parameters(i,1), DH_parameters(i,2), DH_parameters(i,3), DH_parameters(i,4));
        TEnd = TEnd * Trans{i,1};
    end
    DH_tf = TEnd;
    DH_list = Trans;
    for linkIndex = 1:size(DH_list, 1)
        disp(['Frame ', num2str(linkIndex), ' to frame ', num2str(linkIndex-1)]);
        disp('DH parameters (alpha, a, d, theta):');
        disp(DH_parameters(linkIndex, :));
        disp('DH transformation:');
        disp(DH_list{linkIndex, 1});
    end
    disp('DH transformation (End-effector to base):');
    disp(DH_tf);
end
