%% GRANULAR RESISTIVE FORCE THEORY IMPLEMENTATION FOR 3D TRAJECTORIES
%
% Laura Treers:  ltreers@berkeley.edu
% Cyndia Cao: cyndia_cao@berkeley.edu
% Hannah Stuart: hstuart@berkeley.edu
% Embodied Dexterity Group, UC Berkeley Mechanical Engineering
%
% IEEE Robotics and Automation Letters ( Volume: 6, Issue: 2 )
% 03 February 2021
% https://ieeexplore.ieee.org/document/9345981
% 
% This file shows two of the example bodies and trajectories that are
% explored in the paper. We use the function "stlread" to import meshes of
% the bodies that are exported from CAD. Then we process the body geometry
% and pass it to the function that calculates the forces on the body, "rft_3d_body".
%
% NOTE: we do not include a formulation for 2D RFT in this code. We have
% inserted a stand-in function to demonstrate the behavior of the code, 
% but it is not tailored to any specific media. Users of this code should, 
% for their own granular media, obtain coefficients for alpha_x and alpha_z
% as a function of intrusion angle and orientation. 

% See "A Terradynamics of Legged Locomotion on Granular Media", Li et al, 
% Science 339, 1408 (2013) for one example of 2D RFT implementation.


%% Oscillation example
clear;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IMPORT BODY GEOMETRY
% We import an STL in units of mm, convert it to cm, and translate the body
% so that it rotates about the point [0,0,0]
TR0 = stlread('ellipsoid-exp.stl');
point_of_rotation = [9.6, 9.6, 18.05]; % translation vector, based on STL geometry
centered_vertices = (TR0.Points - point_of_rotation) / 10; % mm -> cm
TR = triangulation(TR0.ConnectivityList, centered_vertices);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CALCULATE GEOMETRY OF EACH PLATE
n_vec = faceNormal(TR); 				% normal vector of each plate element

r_vec = zeros(size(n_vec)); 			% centroid of each plate element
for i = 1:size(n_vec,1)
    vertices = TR.ConnectivityList(i,:);
    p1 = TR.Points(vertices(1),:);
    p2 = TR.Points(vertices(2),:);
    p3 = TR.Points(vertices(3),:);
    r_vec(i,:) = mean([p1; p2; p3]);
end

A_vec = zeros(size(n_vec,1),1); 		% area of each plate element
for plate = 1:size(n_vec,1)
    connected_vertices = TR.ConnectivityList(plate,:);
    r1 = TR.Points(connected_vertices(1),:);
    r2 = TR.Points(connected_vertices(2),:);
    r3 = TR.Points(connected_vertices(3),:);
    A_vec(plate) = norm(cross(r2-r1, r3-r1)) / 2;
end

body.r = r_vec; body.n = n_vec; body.A = A_vec;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DEFINE OSCILLATION TRAJECTORY
t_vec = (0:0.1:12)'; 	% time vector [s]
amp = 9 * pi/180; 		% amplitude [deg -> rad]
omega = (5/4) * pi; 	% frequency [rad/s]
vel = [0, 0, -0.5]; 	% penetration velocity [cm/s]
pos_init = [0, 0, 1.8]; % initial position [cm]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CALCULATE FORCES AND MOMENTS ON BODY DUE TO INTRUSION TRAJECTORY
F_vec = zeros(numel(t_vec), 3); 	% forces [N]
M_vec = zeros(numel(t_vec), 3); 	% moments [N-cm]

zeta = 0.33; % RFT scaling coefficient specific to our glass beads

for ti = 1:length(t_vec)
    pos_origin = pos_init + vel * t_vec(ti); 			% constant velocity translation
    pos_theta = amp * sin(omega * t_vec(ti)); 			% sinusoidal angular position
    vel_theta = amp * omega * cos(omega * t_vec(ti)); 
    
    % TR is an optional argument that can be used to display an animation
    % of the forces on the intruder. The color of each element indicates
    % the magnitude of force on each individual plate.
    % If no zeta is given, zeta = 1 will be used
    [F, M] = rft_3D_body(body, pos_origin, [0, pos_theta], vel, [0, vel_theta, 0], zeta, TR);
    F_vec(ti,:) = F; M_vec(ti,:) = M;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOT CALCULATED FORCES AND MOMENTS
depth = -10 * (pos_init(3) + vel(3) * t_vec);

% plot all 3 components of force
figure; hold on; set(gca,'fontsize', 18,'DefaultLineLineWidth',2)
plot(depth, F_vec(:,1), 'r')
plot(depth, F_vec(:,2), 'b')
plot(depth, F_vec(:,3), 'k')
legend({'F_x', 'F_y', 'F_z'},'Location','northwest')
title('Forces on intruding body - Oscillation')
xlabel('Depth [mm]'); ylabel('Force [N]')

% plot all 3 components of moment
figure; hold on; set(gca, 'fontsize', 18, 'DefaultLineLineWidth', 2)
plot(depth, M_vec(:,1), 'r')
plot(depth, M_vec(:,2), 'b')
plot(depth, M_vec(:,3), 'k')
legend({'M_x', 'M_y', 'M_z'}, 'Location', 'northwest')
title('Moments on intruding body - Oscillation')
xlabel('Depth [mm]'); ylabel('Moment [N-cm]')


%% Circumnutation example
clear; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% IMPORT BODY GEOMETRY
% We import an STL in units of mm, convert it to cm, and translate the body
% so that it rotates about the point [0,0,0]
TR0 = stlread('root-tip.stl');
point_of_rotation = [0, 0, 35.5];
centered_vertices = (TR0.Points - point_of_rotation) / 10; % mm -> cm
TR = triangulation(TR0.ConnectivityList, centered_vertices);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CALCULATE GEOMETRY OF EACH PLATE
n_vec = faceNormal(TR); 					% normal vector of each plate element

r_vec = zeros(size(n_vec)); 				% centroid of each plate element
for i = 1:size(n_vec,1)
    vertices = TR.ConnectivityList(i,:);
    p1 = TR.Points(vertices(1),:);
    p2 = TR.Points(vertices(2),:);
    p3 = TR.Points(vertices(3),:);
    r_vec(i,:) = mean([p1; p2; p3]);
end

A_vec = zeros(size(n_vec,1), 1); 			% area of each plate element
for plate = 1:size(n_vec,1)
    connected_vertices = TR.ConnectivityList(plate,:);
    r1 = TR.Points(connected_vertices(1),:);
    r2 = TR.Points(connected_vertices(2),:);
    r3 = TR.Points(connected_vertices(3),:);
    A_vec(plate) = norm(cross(r2-r1, r3-r1)) / 2;
end

body.r = r_vec; body.n = n_vec; body.A = A_vec;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DEFINE CIRCUMNUTATION TRAJECTORY
body_pose = 10 * pi/180; 					% body pitch from vertical [deg -> rad]
helical_pitch = 4; 							% pitch of helical motion [cm]
omega = 15 * pi/180; 						% rotation rate [rad/s]
rot_period = 2*pi / omega; 					% time to complete one rotation [s]
vel =[0, 0, -helical_pitch / rot_period]; 	% penetration rate [cm/s] (vertical)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CALCULATE FORCES AND MOMENTS ON BODY DUE TO INTRUSION TRAJECTORY
t_vec = (0:0.5:50)'; % time vector [s]
F_vec = zeros(numel(t_vec), 3);
M_vec = zeros(numel(t_vec), 3);

zeta = 0.33; % RFT scaling coefficient specific to our glass beads

for ti = 1:length(t_vec)
	% constant velocity motion
    pos_origin = vel * t_vec(ti); 
	pos_yaw = omega * t_vec(ti); 
    
    % TR is an optional argument that can be used to display an animation
    % of the forces on the intruder. The color of each element indicates
    % the magnitude of force on each individual plate.
    % If no zeta is given, zeta = 1 will be used
    [F, M] = rft_3D_body(body, pos_origin, [pos_yaw, body_pose], vel, [0 0 omega], zeta, TR);
    F_vec(ti,:) = F; M_vec(ti,:) = M;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOT CALCULATED FORCES AND MOMENTS
depth = -10 * vel(3) * t_vec;

% all 3 components of force
figure; hold on; set(gca, 'fontsize', 18, 'DefaultLineLineWidth', 2)
plot(depth, F_vec(:,1), 'r')
plot(depth, F_vec(:,2), 'b')
plot(depth, F_vec(:,3), 'k')
legend({'F_x', 'F_y', 'F_z'}, 'Location', 'northwest')
title('Forces on intruding body - Circumnutation')
xlabel('Depth [mm]'); ylabel('Force [N]')

% lateral vs vertical forces
figure; hold on; set(gca, 'fontsize', 18, 'DefaultLineLineWidth', 2)
plot(depth, sqrt( F_vec(:,1).^2 + F_vec(:,2).^2 ), 'r')
plot(depth, F_vec(:,3), 'k')
legend({'F_{lateral}', 'F_z'}, 'Location', 'northwest')
title('Forces on intruding body - Circumnutation')
xlabel('Depth [mm]'); ylabel('Force [N]')
xlim([0,50])

% all 3 components of moment
figure; hold on; set(gca, 'fontsize', 18, 'DefaultLineLineWidth', 2)
plot(depth, M_vec(:,1), 'r')
plot(depth, M_vec(:,2), 'b')
plot(depth, M_vec(:,3), 'k')
legend({'M_x', 'M_y', 'M_z'}, 'Location', 'southwest')
title('Moments on intruding body - Circumnutation')
xlabel('Depth [mm]'); ylabel('Moment [N-cm]')
xlim([0,50])
