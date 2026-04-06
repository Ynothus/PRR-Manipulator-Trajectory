 function [Jv, Jw] = DH_jacobian(DH_parameters, joint_types)
    numFrames = size(DH_parameters,1);
    numJoints = size(joint_types,1);
    [~, TList] = DH_robot(DH_parameters);
    T_j_i = 1;
    Jv_i = sym(zeros(3, numJoints));
    Jw_i = Jv_i;
    zList = sym(zeros(3, numFrames));
    oList = zList;
    
    % Parsing Z-axes and origin position from each transformation
    for j = 1:numFrames
        T_j = cell2mat(TList(j));
        T_j_i = T_j_i * T_j;
        zList(:,j) = T_j_i(1:end-1,3);
        oList(:,j) = T_j_i(1:end-1,4);
    
    end
    % Calculating and indexing each Jacobian term
    for i = 1:numJoints
        if joint_types(i,1) == 1
            % Set Jwi = z_i-1
            Jw_i(:,i) = zList(:,i);
            % Set Jvi = that cross product thing
            Jv_i(:,i) = cross(zList(:,i), (oList(:, numFrames) - oList(:,i)));
        else
            % Set Jwi = 0_3x1
            Jw_i(:,i) = [0; 0; 0];
            % Set Jvi = z_i-1
            Jv_i(:,i) = zList(:,i);
    
        end
    end
    
    Jv = Jv_i;
    Jw = Jw_i;
    
    
    % print the Jacobian for linear and angular velocities
    disp('Jacobian for linear velocity (Jv):')
    disp(Jv);
    disp('Jacobian for angular velocity (Jw):')
    disp(Jw);
end