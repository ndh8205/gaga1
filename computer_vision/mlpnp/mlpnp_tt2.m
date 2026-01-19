clc; clear all; close all;
addpath(genpath('C:\Users\USER\Desktop\relative2'));

%% Initialise the gravitational constant and planet radius.
Mu = 3.98600441500000e+05; % Earth gravitational parameter [km^3/s^2]
R_e = 6.37813630000000e+03; % Earth radius [km]
J2 = 0.001082627; % Second zonal harmonic coefficient [-]

param.orbit.Mu = Mu;
param.orbit.R_e = R_e;
param.orbit.J2 = J2;

%% Chief [A] satellite orbital elements
e_A = 0; % Eccentricity [-]
a_A = 6892.137; % Semi-major axis [km]
h_A = sqrt( a_A*( 1 - e_A^2 ) * Mu ); % Specific angular momentum [km^2/s]
i_A = deg2rad( 97.45 ); % Inclination [rad]
RAAN_A = deg2rad( 185.898 ); % Right ascension of ascending node [rad]
omega_A = deg2rad( 90 ); % Argument of periapsis [rad]
theta_A = deg2rad( 0 ); % True anomaly [rad]

[r_A, v_A] = coe2rv( [ h_A, e_A, RAAN_A, i_A, omega_A, theta_A ], Mu );
x_A = [ r_A; v_A ];

%% Deputy [B] - LVLH frame initial value - GCO formation
n = sqrt( Mu/a_A^3 ); % Mean motion [rad/s]
T_Op = 2 * pi/n; % Orbital period [s]

% GCO initial LVLH state
p_pi_B = deg2rad( 0 ); % Phase angle [rad]
r_init_B = 0.15; % Initial radius [km]

x_0_B = r_init_B/2 * sin( p_pi_B );
x_0_dot_B = r_init_B * n/2 * cos( p_pi_B );
y_0_B = 2 * x_0_dot_B/n;
y_0_dot_B = -2 * n * x_0_B;
z_0_B = sqrt(3) * x_0_B;
z_0_dot_B = sqrt(3) * x_0_dot_B;


r_0_B = [ x_0_B; y_0_B; z_0_B ];
v_0_B = [ x_0_dot_B; y_0_dot_B; z_0_dot_B ];

%% ECI coordinate transformation
hB_I = cross( r_A, v_A );

i_I = r_A/norm( r_A );
k_I = hB_I/norm( hB_I );
j_I = cross( k_I, i_I );

R_L2I = [ i_I, j_I, k_I ];
R_I2L = R_L2I';

% Deputy state ECI
w_r_B = [ -n * r_0_B(2); n * r_0_B(1); 0 ];
r_B = r_A + ( R_L2I * r_0_B );
v_B = v_A + ( R_L2I * ( v_0_B + w_r_B ) );

x_B = [ r_B; v_B ];

%% Main dynamics loop using ode78
dtdtdt = 0.2; % 0.2 sec interval
tt = 1 * T_Op; % 1 orbital period
dt = dtdtdt;

% Camera measurement interval
camera_interval = 10; % 10 sec interval

% ode78 solve
u = [ 0; 0; 0 ];
w_I = [ 0; 0; 0 ];
tspan = 0 : dt : tt;
opts = odeset('Reltol',1e-12,'AbsTol',1e-12,'Stats','on');

% J2 propagation
fprintf('Simulating orbital dynamics...\n');
[t1, x_A_out] = ode78(@(t, X) orbit_propagation_j2(t, X, u, w_I, param), tspan, x_A, opts);
[t2, x_B_out] = ode78(@(t, X) orbit_propagation_j2(t, X, u, w_I, param), tspan, x_B, opts);

%% ECI -> LVLH coordinate transformation
r_A_I = x_A_out(:,1:3)';
v_A_I = x_A_out(:,4:6)';
r_B_I = x_B_out(:,1:3)';
v_B_I = x_B_out(:,4:6)';

% LVLH transformation
r_B_L = zeros( 3, length(t1) );
v_B_L = zeros( 3, length(t1) );

for i = 1 : length(t1)
    r_hat = r_A_I(:,i)/norm(r_A_I(:,i));
    rcv = cross(r_A_I(:,i), v_A_I(:,i));
    h_hat = rcv/norm(rcv);
    t_hat = cross(h_hat, r_hat);
    R_L2I_set = [r_hat, t_hat, h_hat];
    R_I2L_set = R_L2I_set';
    
    % Relative position and velocity
    del_r_gco = r_B_I(:,i) - r_A_I(:,i);
    r_B_L(:,i) = R_I2L_set * del_r_gco;

    del_v_gco = v_B_I(:,i) - v_A_I(:,i);
    omega = cross( r_A_I(:,i), v_A_I(:,i) )/( norm(r_A_I(:,i))^2 );
    v_B_L(:,i) = R_I2L_set * (del_v_gco - cross(omega, del_r_gco));
end

LVLH_pos = r_B_L;
LVLH_vel = v_B_L;

fprintf('Orbit calculation complete.\n');

%% Calculate orbital plane normal vector
[coeff, ~, ~] = pca(LVLH_pos');
orbit_normal_vec = coeff(:,3);

if orbit_normal_vec(3) < 0 
    orbit_normal_vec = -orbit_normal_vec; 
end

%% Initialize camera parameters and load Point Cloud
fprintf('Initializing camera parameters...\n');
camera_params = [];

% Load Point Cloud from MAT file
mat_filename = 'chief_pointcloud.mat';
if exist(mat_filename, 'file')
    fprintf('Loading Point Cloud: %s\n', mat_filename);
    load(mat_filename);

    % Convert to km and 3xN matrix
    all_points_km = points' / 1000;
    total_points = size(all_points_km, 2);

    % Downsample for performance
    max_display_points = 500;
    if total_points > max_display_points
        fprintf('Downsampling points: %d -> %d\n', total_points, max_display_points);
        step = floor(total_points / max_display_points);
        point_idx = 1:step:total_points;
        point_idx = point_idx(1:min(max_display_points, length(point_idx)));
    else
        point_idx = 1:total_points;
    end

    num_points = length(point_idx);
    initial_pointcloud = all_points_km(:, point_idx);
    fprintf('Display points: %d\n', num_points);

    % Center point cloud
    cloud_center = mean(initial_pointcloud, 2);
    initial_pointcloud = initial_pointcloud - cloud_center;
    fprintf('Point cloud centered at: [%.3f, %.3f, %.3f] km\n', cloud_center);
else
    fprintf('Warning: Point Cloud file not found. Creating default points...\n');
    [X,Y,Z] = meshgrid(linspace(-0.05, 0.05, 5));
    initial_pointcloud = [X(:)'; Y(:)'; Z(:)'];
    num_points = size(initial_pointcloud, 2);
end

%% Real-time simulation with camera measurements
fprintf('Starting camera measurement simulation...\n');
fprintf('  - Camera: %.1f sec interval (~%d measurements expected)\n', camera_interval, floor(tt/camera_interval));

% Chief rotation parameters (defined in LVLH)
rotation_rate_dps = 0.3; % deg/sec
rotation_axis_vector = [1; 1; 0];
rotation_rate_rad = deg2rad(rotation_rate_dps);
rotation_axis = rotation_axis_vector / norm(rotation_axis_vector);

% Previous quaternions for continuity
q_I2L_prev = [1; 0; 0; 0];
q_I2A_prev = [1; 0; 0; 0];
q_I2B_prev = [1; 0; 0; 0];
q_L2A_prev = [1; 0; 0; 0];
q_L2B_prev = [1; 0; 0; 0];

% Sensor data storage
sensor_data = struct();
camera_count = 0;

for k = 1:length(t1)
    current_time = t1(k);
    
    % --- Quaternion calculations ---
    % q_I2L: Inertial -> LVLH
    r_hat = r_A_I(:,k)/norm(r_A_I(:,k));
    h_vec = cross(r_A_I(:,k),v_A_I(:,k));
    h_hat = h_vec/norm(h_vec);
    t_hat = cross(h_hat, r_hat);
    DCM_L2I = [r_hat, t_hat, h_hat];
    DCM_I2L = DCM_L2I';
    q_I2L = DCM2Quat(DCM_I2L);
    
    % q_L2A: LVLH -> Chief body (rotating in LVLH)
    if norm(rotation_axis_vector) > 1e-6
        theta = rotation_rate_rad * current_time;
        q_L2A = [cos(theta/2); sin(theta/2) * rotation_axis];
        q_L2A = q_L2A / norm(q_L2A);
    else
        q_L2A = [1; 0; 0; 0];
    end
    
    % q_L2B: LVLH -> Deputy body (no-roll alignment)
    q_L2B = CalculateAttitudeQuat_NoRoll(LVLH_pos(:,k), [0;0;0], orbit_normal_vec);
 
    % Ensure continuity
    if k > 1
        q_I2L = EnsQuatCont(q_I2L, q_I2L_prev);
        q_L2A = EnsQuatCont(q_L2A, q_L2A_prev);
        q_L2B = EnsQuatCont(q_L2B, q_L2B_prev);
    end
    q_L2A = q_L2A / norm(q_L2A);
    q_L2B = q_L2B / norm(q_L2B);

    q_I2A = q2q_mult( q_L2A, q_I2L );
    q_I2B = q2q_mult( q_L2B, q_I2L );

    if k > 1
        q_I2A = EnsQuatCont(q_I2A, q_I2A_prev);
        q_I2B = EnsQuatCont(q_I2B, q_I2B_prev);
    end    

    q_I2A = q_I2A / norm(q_I2A);
    q_I2B = q_I2B / norm(q_I2B);

    % --- Angular velocities ---
    omega_LI_I = h_vec / (r_A_I(:,k).' * r_A_I(:,k));
    omega_LI_L = DCM_I2L * omega_LI_I;
    
    % Calculate angular velocities for state vectors
    if k > 1
        q_dotB = (q_I2B - q_I2B_prev) / dt;
        q_dotA = (q_I2A - q_I2A_prev) / dt;
        
        Xi = GetXiMatrix(q_I2B);
        omega_B_I_inB = 2 * Xi' * q_dotB;

        Xi = GetXiMatrix(q_I2A);
        omega_A_I_inA = 2 * Xi' * q_dotA;
    else
        omega_B_I_inB = [0; 0; 0];
        omega_A_I_inA = [0; 0; 0];
    end

    R_L2B = GetDCM_QUAT(q_L2B);
    R_B2L = R_L2B';
    omega_B_L_inL = R_B2L * omega_B_I_inB - omega_LI_L;
    
    % --- State vectors ---
    Xk_deputy = [r_B_I(:,k); v_B_I(:,k); q_I2B; omega_B_I_inB];
    Xk_chief  = [r_A_I(:,k); v_A_I(:,k); q_I2A; omega_A_I_inA];
    
    % Store results
    sensor_data(k).time = current_time;
    sensor_data(k).LVLH_pos = LVLH_pos(:,k);
    sensor_data(k).q_L2A = q_L2A;
    sensor_data(k).q_L2B = q_L2B;
    sensor_data(k).DCM_L2B = R_L2B;
    sensor_data(k).q_I2B = q_I2B;    
    sensor_data(k).q_I2A = q_I2A;
    
    % Update previous values
    q_I2L_prev = q_I2L;
    q_I2A_prev = q_I2A;
    q_I2B_prev = q_I2B;
    q_L2A_prev = q_L2A;
    q_L2B_prev = q_L2B;

    % --- Camera sensor measurement (at intervals) ---
    if exist('initial_pointcloud', 'var') && ~isempty(initial_pointcloud)
        if mod(current_time, camera_interval) < dt || k == 1
            [z_camera_meas, z_camera_true, camera_params] = measure_sensor_camera( ...
                Xk_deputy, Xk_chief, initial_pointcloud, camera_params, orbit_normal_vec, dt);
            
            sensor_data(k).z_measured = z_camera_meas;
            sensor_data(k).z_true = z_camera_true;
            camera_count = camera_count + 1;
        else
            sensor_data(k).z_measured = [];
            sensor_data(k).z_true = [];
        end
    end
end
fprintf('Sensor measurement complete.\n');
fprintf('  - Camera: %d measurements\n', camera_count);

%% Color definitions
colors = struct('radial', [0.8500, 0.3250, 0.0980], ...
                'intrack', [0, 0.4470, 0.7410], ...
                'crosstrack', [0.4660, 0.6740, 0.1880], ...
                'chief', [0.3010, 0.7450, 0.9330], ...
                'deputy', [0.6350, 0.0780, 0.1840], ...
                'points', [0, 0.7, 1], ...
                'true_proj', [0, 0.8, 0.2], ...
                'meas_proj', [1, 0.2, 0.2], ...
                'sat_body_x', [1, 0, 0], ...
                'sat_body_y', [0, 1, 0], ...
                'sat_body_z', [0, 0, 1]);

time_hours = t1 / 3600;
time_min = t1 / 60;

%% MLPnP Pose Estimation vs Ground Truth Analysis
if exist('initial_pointcloud', 'var') && ~isempty(initial_pointcloud) && camera_count > 0
    
    fprintf('\n========================================\n');
    fprintf('=== MLPnP Ground Truth Fix Verification ===\n');
    fprintf('========================================\n');
    
    % Extract camera measurements
    measurement_data = {};
    meas_count = 0;

    for k = 1:length(sensor_data)
        if ~isempty(sensor_data(k).z_measured)
            meas_count = meas_count + 1;
            measurement_data{meas_count}.time = sensor_data(k).time;
            measurement_data{meas_count}.z_measured = sensor_data(k).z_measured;
            measurement_data{meas_count}.LVLH_pos = sensor_data(k).LVLH_pos;
            measurement_data{meas_count}.q_L2A = sensor_data(k).q_L2A;
            measurement_data{meas_count}.q_L2B = sensor_data(k).q_L2B;
        end
    end

    K = camera_params.intrinsic.K_ideal;
    pixel_noise_std = camera_params.noise.pixel_std;
    
    % Storage
    mlpnp_angle_errors = [];
    mlpnp_trans_errors = [];
    mlpnp_repr_errors = [];
    success_count = 0;

    % Process measurements
    for idx = 1:meas_count
        meas = measurement_data{idx};
        z_meas = meas.z_measured;
        visibility = z_meas(3, :);
        vis_idx = find(visibility > 0.5);
        
        if length(vis_idx) < 6, continue; end
        
        if length(vis_idx) > 30
            step = floor(length(vis_idx) / 30);
            vis_idx = vis_idx(1:step:end);
        end
        
        points_chief = initial_pointcloud(:, vis_idx);
        pixels_used = z_meas(1:2, vis_idx);
        
        % ========================================
        % Ground Truth: C→A (standard convention)
        % ========================================
        DCM_L2A = GetDCM_QUAT(meas.q_L2A);
        [R_L2C, ~] = GetCameraFromQuat(meas.q_L2B, meas.LVLH_pos, orbit_normal_vec);
        
        % Rotation: C→A
        R_true = DCM_L2A * R_L2C';  % (L→A) * (C→L) = C→A
        
        % Translation: C→A
        % Method 1: Using inverse formula
        R_A2C = R_L2C * DCM_L2A';   % A→C rotation
        % t_A2C = -R_L2C * meas.LVLH_pos;  % A→C translation
        % t_true = -R_true * t_A2C;   % C→A translation
        t_true = DCM_L2A * meas.LVLH_pos;    % C→A (simplified)
        
        % MLPnP
        try
            [R_est, t_est] = mlpnp_with_covariance(points_chief, pixels_used, K, pixel_noise_std^2 * eye(2));
            
            % Error calculation
            R_error = R_true' * R_est;  % Should be ~Identity
            angle_error = acos(min(1, max(-1, (trace(R_error)-1)/2))) * 180/pi;
            trans_error = norm(t_est - t_true) * 1000;
            
            % Reprojection
            repr_err = [];
            for i = 1:length(vis_idx)
                P_cam = R_est \ (points_chief(:,i) - t_est);
                if P_cam(3) > 0
                    p_proj = K * P_cam / P_cam(3);
                    repr_err(end+1) = norm(p_proj(1:2) - pixels_used(:,i));
                end
            end
            
            success_count = success_count + 1;
            mlpnp_angle_errors(end+1) = angle_error;
            mlpnp_trans_errors(end+1) = trans_error;
            mlpnp_repr_errors(end+1) = rms(repr_err);
            
            % First measurement verification
            if success_count == 1
                fprintf('\n--- First Measurement Verification ---\n');
                fprintf('Trace(R_error): %.4f (should be ~3.0)\n', trace(R_error));
                fprintf('Det(R_error):   %.4f (should be ~1.0)\n', det(R_error));
                fprintf('||R_err - I||:  %.4e (should be ~0)\n\n', norm(R_error - eye(3), 'fro'));
                % === Issue 2: predict_camera_measurement 검증 ===
                x_test.r_rel_L = meas.LVLH_pos;
                x_test.q_B2A = meas.q_L2A;
                x_test.q_L2B = meas.q_L2B;
                x_test.q_C2B = [1;0;0;0];
                
                z_old = predict_camera_measurement_OLD(x_test, orbit_normal_vec);
                z_new = predict_camera_measurement(x_test, orbit_normal_vec);
                
                fprintf('\n--- Issue 2 Verification ---\n');
                fprintf('Translation (OLD): [%.6f, %.6f, %.6f] km\n', z_old(5:7));
                fprintf('Translation (NEW): [%.6f, %.6f, %.6f] km\n', z_new(5:7));
                fprintf('Translation (TRUE): [%.6f, %.6f, %.6f] km\n', t_true);
                fprintf('Error OLD: %.1f m\n', norm(z_old(5:7) - t_true)*1000);
                fprintf('Error NEW: %.1f m\n\n', norm(z_new(5:7) - t_true)*1000);
            end

            % === 추가: 중간 측정 테스트 ===
            if success_count == round(meas_count/2)
                fprintf('\n--- Issue 2: t=middle ---\n');
                
                x_test.r_rel_L = meas.LVLH_pos;
                x_test.q_B2A = meas.q_L2A;
                x_test.q_L2B = meas.q_L2B;
                x_test.q_C2B = [1;0;0;0];
                
                z_old = predict_camera_measurement_OLD(x_test, orbit_normal_vec);
                z_new = predict_camera_measurement(x_test, orbit_normal_vec);
                
                % Ground truth (이 시점)
                DCM_L2A_mid = GetDCM_QUAT(meas.q_L2A);
                [R_L2C_mid, ~] = GetCameraFromQuat(meas.q_L2B, meas.LVLH_pos, orbit_normal_vec);
                R_true_mid = DCM_L2A_mid * R_L2C_mid';
                t_true_mid = DCM_L2A_mid * meas.LVLH_pos;
                q_true_mid = DCM2Quat(R_true_mid);
                
                % Rotation 에러
                q_err_old = q2q_mult(z_old(1:4), inv_q(q_true_mid));
                q_err_new = q2q_mult(z_new(1:4), inv_q(q_true_mid));
                angle_old = 2 * acos(min(1, abs(q_err_old(1)))) * 180/pi;
                angle_new = 2 * acos(min(1, abs(q_err_new(1)))) * 180/pi;
                
                fprintf('Time: %.1f s\n', meas.time);
                fprintf('Chief rotation: %.1f deg\n', meas.time * 0.3);
                fprintf('Rotation error OLD: %.2f deg\n', angle_old);
                fprintf('Rotation error NEW: %.2f deg\n', angle_new);
                fprintf('Translation error OLD: %.1f m\n', norm(z_old(5:7) - t_true_mid)*1000);
                fprintf('Translation error NEW: %.1f m\n', norm(z_new(5:7) - t_true_mid)*1000);
            end
            
        catch ME
            continue;
        end
    end
    
    % Results
    fprintf('=== Results ===\n');
    fprintf('Success: %d/%d\n', success_count, meas_count);
    if success_count > 0
        fprintf('Rotation:     mean=%.3f°, std=%.3f°, max=%.3f°\n', ...
                mean(mlpnp_angle_errors), std(mlpnp_angle_errors), max(mlpnp_angle_errors));
        fprintf('Translation:  mean=%.2fm, std=%.2fm, max=%.2fm\n', ...
                mean(mlpnp_trans_errors), std(mlpnp_trans_errors), max(mlpnp_trans_errors));
        fprintf('Reprojection: mean=%.3fpx, std=%.3fpx\n', ...
                mean(mlpnp_repr_errors), std(mlpnp_repr_errors));
        
        % Pass/Fail
        if mean(mlpnp_angle_errors) < 1.0
            fprintf('\n✓ GROUND TRUTH FIX VERIFIED (angle < 1°)\n');
        else
            fprintf('\n✗ STILL HAS ISSUES (angle > 1°)\n');
        end
    end
end

function z = predict_camera_measurement(x, orbit_normal_vec)
    q_B2A = x.q_B2A;
    q_L2B = x.q_L2B;
    r_rel_L = x.r_rel_L;

    q_L2A = q2q_mult(q_B2A, q_L2B);
    q_L2A = q_L2A / norm(q_L2A);
    R_L2A = GetDCM_QUAT(q_L2A);
    [R_L2C, ~] = GetCameraFromQuat(q_L2B, r_rel_L, orbit_normal_vec);
    
    % Rotation: C→A
    R_C2L = R_L2C';
    R_C2A = R_L2A * R_C2L;
    q_C2A = DCM2Quat(R_C2A);
    q_C2A = q_C2A / norm(q_C2A);
    
    % Translation
    t_C2A = R_L2A * r_rel_L;
    
    z = [q_C2A; t_C2A];
end