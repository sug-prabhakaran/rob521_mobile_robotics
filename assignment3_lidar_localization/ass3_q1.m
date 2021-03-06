% =========
% ass3_q1.m
% =========
%
% This assignment will introduce you to the idea of first building an
% occupancy grid then using that grid to estimate a robot's motion using a
% particle filter.
% 
% There are two questions to complete (5 marks each):
%
%    Question 1: code occupancy mapping algorithm 
%    Question 2: see ass3_q2.m
%
% Fill in the required sections of this script with your code, run it to
% generate the requested plot/movie, then paste the plots into a short report
% that includes a few comments about what you've observed.  Append your
% version of this script to the report.  Hand in the report as a PDF file
% and the two resulting AVI files from Questions 1 and 2.
%
% requires: basic Matlab, 'gazebo.mat'
%
% T D Barfoot, January 2016
%
clear all;

% set random seed for repeatability
rng(1);

% ==========================
% load the dataset from file
% ==========================
%
%    ground truth poses: t_true x_true y_true theta_true
% odometry measurements: t_odom v_odom omega_odom
%           laser scans: t_laser y_laser
%    laser range limits: r_min_laser r_max_laser
%    laser angle limits: phi_min_laser phi_max_laser
%
load gazebo.mat;

% =======================================
% Question 1: build an occupancy grid map
% =======================================
%
% Write an occupancy grid mapping algorithm that builds the map from the
% perfect ground-truth localization.  Some of the setup is done for you
% below.  The resulting map should look like "ass2_q1_soln.png".  You can
% watch the movie "ass2_q1_soln.mp4" to see what the entire mapping process
% should look like.  At the end you will save your occupancy grid map to
% the file "occmap.mat" for use in Question 2 of this assignment.

% allocate a big 2D array for the occupancy grid
ogres = 0.05;                   % resolution of occ grid
ogxmin = -7;                    % minimum x value
ogxmax = 8;                     % maximum x value
ogymin = -3;                    % minimum y value
ogymax = 6;                     % maximum y value
ognx = (ogxmax-ogxmin)/ogres;   % number of cells in x direction
ogny = (ogymax-ogymin)/ogres;   % number of cells in y direction
oglo = zeros(ogny,ognx);        % occupancy grid in log-odds format
ogp = zeros(ogny,ognx);         % occupancy grid in probability format

% precalculate some quantities
numodom = size(t_odom,1);
npoints = size(y_laser,2);
angles = linspace(phi_min_laser, phi_max_laser,npoints);
dx = ogres*cos(angles);
dy = ogres*sin(angles);

% interpolate the noise-free ground-truth at the laser timestamps
t_interp = linspace(t_true(1),t_true(numodom),numodom);
x_interp = interp1(t_interp,x_true,t_laser);
y_interp = interp1(t_interp,y_true,t_laser);
theta_interp = interp1(t_interp,theta_true,t_laser);
omega_interp = interp1(t_interp,omega_odom,t_laser);

% set up the plotting/movie recording
vid = VideoWriter('ass3_q1.avi');
open(vid);
figure(1);
clf;
pcolor(ogp);
colormap(1-gray);
shading('flat');
axis equal;
axis on;
M = getframe;
writeVideo(vid,M);

% loop over laser scans (every fifth)
for t=1:5:989 %:size(t_laser,1)
    
    % convert current pose coords to pixels
    [x_pixel, y_pixel] = DistToPixel(x_interp(t), y_interp(t));

    % loop over every laser reading
    for i=1:npoints

        % if laser is below min range or NaN, skip over
        if y_laser(t,i) < r_min_laser || isnan(y_laser(t,i))
            continue
        end
        
        % calculate pixel indexes for laser
        [x_idxs, y_idxs] = RayTrace(x_pixel, y_pixel, theta_interp(t), ...
            angles(i), y_laser(t,i));

        % iterate over each cell index pair
        for j=1:size(x_idxs, 2)
           
            % skip over any instances where index is outside map
            if y_idxs(j) > 180 || x_idxs(j) > 300 || y_idxs(j) == 0 || ...
                    x_idxs(j) == 0
                continue
            end 
            % update log-odds
            if j < (size(x_idxs,2))
                % all cells except last cell are set as free
                oglo(y_idxs(j),x_idxs(j)) = oglo(y_idxs(j),x_idxs(j)) - 0.55;
            else
                % last cell is set as occupied
                oglo(y_idxs(j),x_idxs(j)) = oglo(y_idxs(j),x_idxs(j)) + 2;
            end
                
            % update probability array from log-odd data
            %ogp(y_idxs(j),x_idxs(j)) = ProbFromLogOdds( ...
            %    oglo(y_idxs(j),x_idxs(j)));
        end
    end

    % Update probability array from log-odd array
    ogp = ProbFromLogOdds(oglo);

    
    % ------end of your occupancy grid mapping algorithm-------

    % draw the map
    clf;
    pcolor(ogp);
    colormap(1-gray);
    shading('flat');
    axis equal;
    axis on;
    
    % draw the robot
    hold on;
    x = (x_interp(t)-ogxmin)/ogres;
    y = (y_interp(t)-ogymin)/ogres;
    th = theta_interp(t);
    r = 0.15/ogres;
    r2= y_laser(t,320);
    set(rectangle( 'Position', [x-r y-r 2*r 2*r], 'Curvature', [1 1]), ...
        'LineWidth',2,'FaceColor',[0.35 0.35 0.75]);
    set(plot([x x+r*cos(th)]', [y y+r2*sin(th)]', 'r-'),'LineWidth',2);
    
    % save the video frame
    M = getframe;
    writeVideo(vid,M);
    
    pause(0.1);
    
end

close(vid);
print -dpng ass3_q1.png

%save occmap.mat ogres ogxmin ogxmax ogymin ogymax ognx ogny oglo ogp;

