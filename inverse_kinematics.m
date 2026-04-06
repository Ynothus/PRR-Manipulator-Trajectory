function q_estimate = inverse_kinematics(DH, Jacob, target_pose, q_initial, mask)
% Inputs:
%   - DH is a 4x4 symbolic matrix, which depends only on joint variables
%   - Jacob is a 6xn symbolic matrix, which depends only on joint variables
%   - target_pose is a 6x1 vector, including position and orientation
%   - q_initial is a nx1 vector, where n is the number of joints
%   - mask is 6x1 vector, indicting control postion, orientation, or both (default)
% Outputs:
%   - q_estimate is a nx1 vector, where n is the number of joints

    % Helpful local functions
    
    % Euler Angles -> Rotation Matrix 
    function R = eul2rotm_zyx(rpy)
        rx = rpy(1);
        ry = rpy(2);
        rz = rpy(3);
    
        cx = cos(rx); sx = sin(rx);
        cy = cos(ry); sy = sin(ry);
        cz = cos(rz); sz = sin(rz);
    
        R = [ cz*cy,  cz*sy*sx - sz*cx,  cz*sy*cx + sz*sx;
              sz*cy,  sz*sy*sx + cz*cx,  sz*sy*cx - cz*sx;
               -sy,        cy*sx,              cy*cx       ];
    end
    % Rotation Matrix -> Angle Axis Vectors
    function e_ori = rotm2angvec(R)
        val = (trace(R) - 1) / 2;
        val = max(min(val, 1), -1);
        theta = acos(val);
            k = [R(3,2) - R(2,3);
                 R(1,3) - R(3,1);
                 R(2,1) - R(1,2)] / (2 * sin(theta));
            e_ori = theta * k;
    end

% here goes your implementation
    % Initialization
    max_iter = 3000; % Max iterations
    tol = 1e-6; % Error Tolerance
    n = length(q_initial);
    joint_vars = sym('q', [n, 1], 'real');

    % Symbolic matrix -> matlab function
    DH_func = matlabFunction(DH,   'Vars', {joint_vars});
    J_func = matlabFunction(Jacob, 'Vars', {joint_vars});

    % Mask setup
    mask = mask(:);
    S = diag(mask);
    active_idx = find(mask);
    S_reduced = S(active_idx, :);

    % Extract Target Position and Orientation
    target_pos = target_pose(1:3);
    target_rpy = target_pose(4:6);

    % Converts target euler angles to rotation matrix
    R_target = eul2rotm_zyx(target_rpy);

    % Main Iterative IK loop solver 
    q = q_initial(:);
    for iter = 1:max_iter
        % Evaluate forward kinematics numerically
        T_current = DH_func(q);
        
        % Current position
        pos_current = T_current(1:3, 4);
        R_current   = T_current(1:3, 1:3);

        % Position error
        e_pos = target_pos(:) - pos_current(:);

        % Orientation error
        R_err = R_target * R_current';
        % Rotation Matrix -> Angle-Axis vector
        e_ori = rotm2angvec(R_err);

        % Full 6x1 error
        e_full = [e_pos; e_ori];

        % Apply mask
        e = S_reduced * e_full;

        % Check convergence
        if norm(e) < tol
            fprintf('IK converged in %d iterations (error = %.2e)\n', iter, norm(e));
            q_estimate = q;
            return;
        end

        % Evaluate Jacobian numerically
        J_full = J_func(q);

        % Apply mask to Jacobian
        J = S_reduced * J_full;

        
        % delta_q = J^-1(q') * e
        dq = J \ e;

        % Update joint angles: q = q' + Δq
        q = q + dq;
    end

    warning('IK did not converge after %d iterations. Final error: %.4e', max_iter, norm(e));
    q_estimate = q;
    
% print the joint configuration and corresponding end-effector pose
disp('q_estimates:');
disp(q_estimate);

disp('target pose and reached pose:');
disp([target_pose, pos_current]);
end
