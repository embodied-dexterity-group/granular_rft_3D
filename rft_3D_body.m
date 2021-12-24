function [F_total, M_total] = rft_3D_body( body, origin, orientation, vel, ang_vel, rft_coeff, TR )   
%RFT_3D_BODY Outputs the forces and torques on a body intruding through granular media
%   Inputs: body geometry (struct containing body.r plate centroids (nx3), body.n plate normals (nx3),
%   and body.A plate areas (nx1)), body position, angular orientation, translational velocity, 
%   angular velocity, RFT scaling coefficient (optional), STL representation (optional)
%   
%   Outputs: Force vector [N], Moment vector [N-cm]
%   All 3D vectors (e.g. position, velocity) are row vectors

% Note about angle definitions:
% Orientation: 3-2-x Euler angles
% Angular velocity: relative to body corotational basis

    if nargin < 6 || isempty(rft_coeff)
        rft_coeff = 1;
    end
    
    % define orientation parameters
    psi = orientation(1);           % about E3, betweeen 0 and 2pi
    theta = orientation(2);         % down from E3, between 0 and pi
    R1 = [cos(psi), sin(psi), 0; 
         -sin(psi), cos(psi), 0; 
                 0,        0, 1];
    R2 = [cos(theta), 0, -sin(theta); 
                   0, 1,           0; 
          sin(theta), 0,  cos(theta)]; 
    R = R2 * R1;

    % rotate position & normal vectors of all faces based on current orientation
    r_rot_vec = (R.' * body.r')';
    depth = origin(3) + r_rot_vec(:,3);
    n_rot_vec = (R.' * body.n')';
    
    % calculate velocity of each plate element based on body velocities
    n_elements = size(body.r, 1);
    v_vec = vel + cross(repmat(ang_vel, n_elements, 1), r_rot_vec); 
    
    % normalize velocity & normal vectors
    v_norm_vec = v_vec ./ vecnorm(v_vec, 2, 2);
    n_norm_vec = n_rot_vec ./ vecnorm(n_rot_vec, 2, 2);
    
    % logical conditions that determine whether or not to count force on element
    leading_edge = dot(n_norm_vec, v_norm_vec, 2) > 0;      % plate's normal vector has a component in the penetration direction, so it maintains contact forces within sand grains
    intruding = depth < 0;                                  % plate is below sand surface
    include = leading_edge & intruding;
    
    % isolate the elements that satisfy this condition for calculation
    n_inc = n_norm_vec(include,:);
    v_inc = v_norm_vec(include,:);

    % generate basis for RFT decomposition
    % edge case: if plate is roughly horizontal, use v to define e2 -> stick to 2D RFT model
    e2_vec = (abs(n_inc(:,3)) >= 1 - 1e-3) .* ((v_inc+[1e-5 0 0]) .* [1 1 0]) ./ vecnorm((v_inc+[1e-5 0 0]) .* [1 1 0], 2, 2) + ...  % edge case
             (abs(n_inc(:,3)) <  1 - 1e-3) .* (n_inc .* [1 1 0]) ./ vecnorm(n_inc .* [1 1 0], 2, 2);                                 % nominal case
    e1_vec = cross(e2_vec, repmat([0,0,1], sum(include), 1));
    
    % decompose velocity vector
    v1_vec = dot(v_inc, e1_vec, 2) .* e1_vec;
    v23_vec = v_inc - v1_vec;
    
    % calculate 2D RFT parameters
    beta_vec =   atan2(  n_inc(:,3), dot(  n_inc, e2_vec, 2)) + pi/2;
    gamma_vec = -atan2(v23_vec(:,3), dot(v23_vec, e2_vec, 2));
    
    %%%%%%%%%%% START: MODELS FROM EMPIRICAL DATA %%%%%%%%%%%
    % NOTE: The following equations were arbitrarily created based on a 
    % loose fitting to 2D Data in Li et al. 2013 Terradynamics. DO NOT RELY
    % ON; this is simply an example of a force mapping to enable the code 
    % to run and will not produce accurate results. Please implement or
    % measure your own 2D RFT coefficients. 

    % 2D alpha components, these functions are dummy functions
    aZ = (.6 * sin(gamma_vec) + .4) .* cos(beta_vec);
    aX = 0.25 * cos(gamma_vec) .* (-cos(2*beta_vec) + 1);
      
    % 3D alpha component, uses model from 2021 3D RFT
    % simplest form: alpha_y = alpha_x(beta=0, gamma=0)
    % linear fit form:
    aY = -0.0015 * beta_vec * pi/180 + 0.2035;

    % calculate scaling factors due to decomposition of velocity into 
    % tangential (v1) and normal (v23) directions
    vt = vecnorm(v1_vec');
    ct_fit = [0.440850096954369, 3.62263982880971, 1.60910808139526, 0.832324700111401];
    f1 =  ct_fit(1) * ( tanh(ct_fit(2)*vt - ct_fit(3)) +  tanh(ct_fit(3))) / ct_fit(4);
    
    vn = vecnorm(v23_vec');    
    cn_fit = [1.99392673405210, 1.61146827229181, 0.973746396532650, 5.81083560370960];
    f23 = cn_fit(1) * (atanh(cn_fit(2)*vn - cn_fit(3)) + atanh(cn_fit(3))) / cn_fit(4);
    %%%%%%%%%%%% END: MODELS FROM EMPIRICAL DATA %%%%%%%%%%%%
    
    % calculate forces and return vectors to inertial frame
    F1 =  -f1' .* aY .* e1_vec .* sign(dot(v_inc, e1_vec, 2));
    F2 = -f23' .* aX .* e2_vec;
    F3 =  f23' .* aZ .* [0,0,1];
    F_i = F1 + F2 + F3;
    
    % superposition: sum force from each plate element to calculate total force & moment
    Fi_mat = (-depth(include)) * rft_coeff .* body.A(include) .* F_i;
    F_total = sum(Fi_mat);

    Mi_mat = cross(r_rot_vec(include,:), Fi_mat); % moment is calculated about body origin
    M_total = sum(Mi_mat);
    
    % if the STL is an input, animate the trajectory of the intruder;
    % the color of each plate element indicates the magnitude of stress on it
    if nargin == 7
        figure(9);
        % extend force matrix to include elements with no force applied
        F_plot = zeros(size(body.n)); F_plot(include,:) = Fi_mat;

        % plot STL geometry
        s = trisurf(TR.ConnectivityList, TR.Points(:,1)+origin(1), TR.Points(:,2)+origin(2), TR.Points(:,3)+origin(3),...
                    vecnorm(F_plot') ./ body.A'); % color elements by force/area magnitude
        alpha(s, 0.8); colorbar; 
        xlabel('x');    ylabel('y');    zlabel('z');
        xlim([-4,4]);   ylim([-4,4]);   zlim([-10,1]); axis equal;
        rotate(s, [0 0 1], psi * 180/pi, origin);
        rotate(s, [-sin(psi) cos(psi) 0], theta * 180/pi, origin);
        title(strcat('yaw:  ', num2str(round(psi*180/pi,0)), ', pose:  ', num2str(round(theta*180/pi,0))))  
    end  
end
