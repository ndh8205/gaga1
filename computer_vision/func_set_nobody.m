
%% MLPnP with Covariance Estimation and 3-Sigma Bounds
function [R_est, t_est, cov_pose] = mlpnp_with_covariance(points3D, pixels, K, pixel_noise_std)
    % MLPnP with covariance estimation
    % pixel_noise_std: 픽셀 측정 노이즈 표준편차 (default: 0.5 pixels)
    
    if nargin < 4
        pixel_noise_std = 0.5;  % pixels
    end
    
    % Standard MLPnP
    bearings = mlpnp_compute_bearings(pixels, K);
    nullspaces = mlpnp_nullspace(bearings);
    [A, is_planar, eigenRot, ~] = mlpnp_design_matrix(points3D, nullspaces);
    [R_init, t_init] = mlpnp_solve_improved(A, is_planar, points3D, bearings, nullspaces, eigenRot);
    
    % Gauss-Newton with Jacobian extraction
    [R_est, t_est, J_final] = mlpnp_gauss_newton_with_jacobian(R_init, t_init, points3D, bearings, nullspaces, 5);
    
    % Covariance estimation
    % Fisher Information Matrix
    H = J_final' * J_final;
    
    % Regularization for numerical stability
    lambda = 1e-10;
    H_reg = H + lambda * eye(size(H));
    
    % Covariance matrix (6x6: 3 rotation + 3 translation)
    try
        cov_pose = inv(H_reg) * pixel_noise_std^2;
    catch
        % If singular, use pseudo-inverse
        cov_pose = pinv(H_reg) * pixel_noise_std^2;
    end
end

%% Modified Gauss-Newton that returns Jacobian
function [R_refined, t_refined, J_final] = mlpnp_gauss_newton_with_jacobian(R_init, t_init, points3D, bearings, nullspaces, max_iter)
    if nargin < 6
        max_iter = 5;
    end
    
    % Rodriguez parameterization
    omega = rot2rodrigues(R_init);
    x = [omega; t_init];
    
    eps_p = 1e-5;
    stop = false;
    iter = 0;
    J_final = [];
    
    while iter < max_iter && ~stop
        [r, J] = compute_residuals_jacobian_analytical(x, points3D, nullspaces);
        
        JtJ = J' * J;
        Jtr = J' * r;
        
        dx = JtJ \ Jtr;
        
        if max(abs(dx)) > 5.0 || min(abs(dx)) > 1.0
            break;
        end
        
        dl = J * dx;
        if max(abs(dl)) < eps_p
            stop = true;
            x = x - dx;
            J_final = J;  % Save final Jacobian
        else
            x = x - dx;
        end
        
        iter = iter + 1;
    end
    
    if isempty(J_final)
        [~, J_final] = compute_residuals_jacobian_analytical(x, points3D, nullspaces);
    end
    
    omega_refined = x(1:3);
    t_refined = x(4:6);
    R_refined = rodrigues2rot(omega_refined);
end

%% Analytical Jacobian computation
function [r, J] = compute_residuals_jacobian_analytical(x, points3D, nullspaces)
    omega = x(1:3);
    t = x(4:6);
    R = rodrigues2rot(omega);
    
    N = size(points3D, 2);
    r = zeros(2*N, 1);
    J = zeros(2*N, 6);
    
    for i = 1:N
        % Camera coordinates
        p_cam = R * points3D(:,i) + t;
        p_norm = norm(p_cam);
        p_unit = p_cam / p_norm;
        
        % Residuals
        ns = nullspaces{i};
        r(2*i-1) = ns(:,1)' * p_unit;
        r(2*i)   = ns(:,2)' * p_unit;
        
        % Jacobian w.r.t normalized point
        dp_dp_cam = (eye(3) - p_unit * p_unit') / p_norm;
        
        % Jacobian w.r.t rotation (Rodriguez)
        dp_cam_domega = -R * skew_sym(points3D(:,i));
        
        % Jacobian w.r.t translation
        dp_cam_dt = eye(3);
        
        % Chain rule
        dr_domega = [ns(:,1)'; ns(:,2)'] * dp_dp_cam * dp_cam_domega;
        dr_dt = [ns(:,1)'; ns(:,2)'] * dp_dp_cam * dp_cam_dt;
        
        J(2*i-1:2*i, 1:3) = dr_domega;
        J(2*i-1:2*i, 4:6) = dr_dt;
    end
end


%% MLPnP with Uncertainty Visualization (추가 섹션)
if success_count > 0
    fprintf('\n=== MLPnP Uncertainty Analysis ===\n');
    
    % Process subset with covariance
    cov_rot_errors = [];
    cov_trans_errors = [];
    mlpnp_times_cov = [];
    
    pixel_noise = camera_params.noise.pixel_std;  % 0.5 pixels
    
    for idx = 1:min(50, meas_count)  % Process first 50 for efficiency
        meas = measurement_data{idx};
        
        vis_idx = find(meas.z_measured(3,:) > 0.5);
        if length(vis_idx) < 6
            continue;
        end
        
        if length(vis_idx) > 30
            step = floor(length(vis_idx) / 30);
            vis_idx = vis_idx(1:step:end);
            vis_idx = vis_idx(1:min(30, length(vis_idx)));
        end
        
        points = initial_pointcloud(:, vis_idx);
        pixels = meas.z_measured(1:2, vis_idx);
        
        try
            % MLPnP with covariance
            [R_est, t_est, cov_pose] = mlpnp_with_covariance(points, pixels, K, pixel_noise);
            
            % Extract standard deviations
            std_rot = sqrt(diag(cov_pose(1:3, 1:3)));     % rad
            std_trans = sqrt(diag(cov_pose(4:6, 4:6)));   % km
            
            % Convert to meaningful units
            std_rot_deg = rad2deg(std_rot);               % deg
            std_trans_m = std_trans * 1000;                % m
            
            % Store
            cov_rot_errors(:, end+1) = std_rot_deg;
            cov_trans_errors(:, end+1) = std_trans_m;
            mlpnp_times_cov(end+1) = meas.time;
            
        catch
            % Skip if failed
        end
    end
    
    if ~isempty(mlpnp_times_cov)
        time_cov_min = mlpnp_times_cov / 60;
        
        % Create uncertainty visualization figure
        figure('Name', 'MLPnP Uncertainty Analysis', 'Position', [150, 100, 1400, 800]);
        
        % --- Rotation uncertainty with 3-sigma bounds ---
        subplot(2, 3, 1);
        % Find corresponding actual errors
        [~, ia, ib] = intersect(round(mlpnp_times*10), round(mlpnp_times_cov*10));
        actual_rot_errors = mlpnp_angle_errors(ia);
        
        plot(time_cov_min, actual_rot_errors, 'b-', 'LineWidth', 1.5);
        hold on;
        % 3-sigma bounds
        upper_bound = 3 * mean(cov_rot_errors(1,:));
        lower_bound = -3 * mean(cov_rot_errors(1,:));
        fill([time_cov_min, fliplr(time_cov_min)], ...
             [upper_bound*ones(size(time_cov_min)), ...
              fliplr(lower_bound*ones(size(time_cov_min)))], ...
             'r', 'FaceAlpha', 0.2, 'EdgeColor', 'r', 'LineStyle', '--');
        
        % 1-sigma bounds
        upper_1sig = mean(cov_rot_errors(1,:));
        lower_1sig = -mean(cov_rot_errors(1,:));
        plot(time_cov_min, upper_1sig*ones(size(time_cov_min)), 'g--', 'LineWidth', 1);
        plot(time_cov_min, lower_1sig*ones(size(time_cov_min)), 'g--', 'LineWidth', 1);
        
        grid on;
        xlabel('Time [min]');
        ylabel('Rotation Error [deg]');
        title('Rotation Error with 3σ Bounds');
        legend('Actual Error', '3σ bounds', '1σ bounds', 'Location', 'best');
        ylim([-4*upper_bound, 4*upper_bound]);
        
        % --- Translation uncertainty with 3-sigma bounds ---
        subplot(2, 3, 2);
        actual_trans_errors = mlpnp_trans_errors(ia);
        
        plot(time_cov_min, actual_trans_errors, 'g-', 'LineWidth', 1.5);
        hold on;
        % 3-sigma bounds
        upper_bound = 3 * mean(cov_trans_errors(1,:));
        lower_bound = -3 * mean(cov_trans_errors(1,:));
        fill([time_cov_min, fliplr(time_cov_min)], ...
             [upper_bound*ones(size(time_cov_min)), ...
              fliplr(lower_bound*ones(size(time_cov_min)))], ...
             'r', 'FaceAlpha', 0.2, 'EdgeColor', 'r', 'LineStyle', '--');
        
        % 1-sigma bounds
        upper_1sig = mean(cov_trans_errors(1,:));
        lower_1sig = -mean(cov_trans_errors(1,:));
        plot(time_cov_min, upper_1sig*ones(size(time_cov_min)), 'b--', 'LineWidth', 1);
        plot(time_cov_min, lower_1sig*ones(size(time_cov_min)), 'b--', 'LineWidth', 1);
        
        grid on;
        xlabel('Time [min]');
        ylabel('Translation Error [m]');
        title('Translation Error with 3σ Bounds');
        legend('Actual Error', '3σ bounds', '1σ bounds', 'Location', 'best');
        ylim([-4*upper_bound, 4*upper_bound]);
        
        % --- Uncertainty evolution ---
        subplot(2, 3, 3);
        plot(time_cov_min, cov_rot_errors(1,:), 'r-', 'LineWidth', 1.5); hold on;
        plot(time_cov_min, cov_rot_errors(2,:), 'g-', 'LineWidth', 1.5);
        plot(time_cov_min, cov_rot_errors(3,:), 'b-', 'LineWidth', 1.5);
        grid on;
        xlabel('Time [min]');
        ylabel('1σ Uncertainty [deg]');
        title('Rotation Uncertainty Evolution');
        legend('ω_x', 'ω_y', 'ω_z', 'Location', 'best');
        
        subplot(2, 3, 4);
        plot(time_cov_min, cov_trans_errors(1,:), 'r-', 'LineWidth', 1.5); hold on;
        plot(time_cov_min, cov_trans_errors(2,:), 'g-', 'LineWidth', 1.5);
        plot(time_cov_min, cov_trans_errors(3,:), 'b-', 'LineWidth', 1.5);
        grid on;
        xlabel('Time [min]');
        ylabel('1σ Uncertainty [m]');
        title('Translation Uncertainty Evolution');
        legend('t_x', 't_y', 't_z', 'Location', 'best');
        
        % --- Consistency check (NEES) ---
        subplot(2, 3, 5);
        nees_values = [];
        for i = 1:length(ia)
            if i <= size(cov_rot_errors, 2)
                error_vec = [deg2rad(actual_rot_errors(i)); actual_trans_errors(i)/1000];
                cov_diag = [cov_rot_errors(1,i)^2; cov_trans_errors(1,i)^2/1e6];
                nees = error_vec(1)^2/cov_diag(1) + error_vec(2)^2/cov_diag(2);
                nees_values(end+1) = nees;
            end
        end
        
        if ~isempty(nees_values)
            plot(time_cov_min(1:length(nees_values)), nees_values, 'k-', 'LineWidth', 1.5);
            hold on;
            yline(chi2inv(0.95, 2), 'r--', 'LineWidth', 1.5);
            yline(chi2inv(0.68, 2), 'g--', 'LineWidth', 1.5);
            grid on;
            xlabel('Time [min]');
            ylabel('NEES');
            title('Normalized Estimation Error Squared');
            legend('NEES', '95% bound', '68% bound', 'Location', 'best');
        end
        
        % --- Statistical consistency ---
        subplot(2, 3, 6);
        % Check how many errors fall within bounds
        within_1sig_rot = sum(abs(actual_rot_errors) <= mean(cov_rot_errors(1,:))) / length(actual_rot_errors) * 100;
        within_3sig_rot = sum(abs(actual_rot_errors) <= 3*mean(cov_rot_errors(1,:))) / length(actual_rot_errors) * 100;
        within_1sig_trans = sum(abs(actual_trans_errors) <= mean(cov_trans_errors(1,:))) / length(actual_trans_errors) * 100;
        within_3sig_trans = sum(abs(actual_trans_errors) <= 3*mean(cov_trans_errors(1,:))) / length(actual_trans_errors) * 100;
        
        categories = categorical({'Rot 1σ', 'Rot 3σ', 'Trans 1σ', 'Trans 3σ'});
        actual_percentages = [within_1sig_rot, within_3sig_rot, within_1sig_trans, within_3sig_trans];
        theoretical = [68.3, 99.7, 68.3, 99.7];
        
        bar(categories, [actual_percentages; theoretical]');
        grid on;
        ylabel('Percentage [%]');
        title('Statistical Consistency Check');
        legend('Actual', 'Theoretical', 'Location', 'best');
        ylim([0, 110]);
        
        sgtitle('MLPnP Uncertainty Analysis with 3σ Bounds', 'FontSize', 14, 'FontWeight', 'bold');
        
        % Print statistics
        fprintf('\nUncertainty Statistics:\n');
        fprintf('  Rotation 1σ: %.3f deg (avg)\n', mean(cov_rot_errors(1,:)));
        fprintf('  Translation 1σ: %.1f m (avg)\n', mean(cov_trans_errors(1,:)));
        fprintf('\nConsistency:\n');
        fprintf('  Rotation within 1σ: %.1f%% (theory: 68.3%%)\n', within_1sig_rot);
        fprintf('  Rotation within 3σ: %.1f%% (theory: 99.7%%)\n', within_3sig_rot);
        fprintf('  Translation within 1σ: %.1f%% (theory: 68.3%%)\n', within_1sig_trans);
        fprintf('  Translation within 3σ: %.1f%% (theory: 99.7%%)\n', within_3sig_trans);
    end
end