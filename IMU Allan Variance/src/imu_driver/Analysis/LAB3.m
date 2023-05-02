bag = rosbag("C:\Users\teju1\OneDrive\Documents\RSN\Francis\group_stationary_data.bag");

bag_select = select(bag,'Topic','/imu');
msgSt = readMessages(bag_select,'DataFormat','struct');

%standardize time
timePoints_secs = cellfun(@(m) double(m.Header.Stamp.Sec), msgSt);
timePoints_nsecs = cellfun(@(m) double(m.Header.Stamp.Nsec), msgSt);
time_combined = timePoints_secs + timePoints_nsecs/1e9;
time = time_combined - time_combined(1);

%angular_velocity
angular_velocity_x = cellfun(@(m) double(m.IMU.AngularVelocity.X), msgSt);
angular_velocity_y = cellfun(@(m) double(m.IMU.AngularVelocity.Y), msgSt);
angular_velocity_z = cellfun(@(m) double(m.IMU.AngularVelocity.Z), msgSt);

quaternions = [angular_velocity_x,angular_velocity_y, angular_velocity_z];

%linear_acceleration
linear_acceleration_x = cellfun(@(m) double(m.IMU.LinearAcceleration.X), msgSt);
linear_acceleration_y = cellfun(@(m) double(m.IMU.LinearAcceleration.Y), msgSt);
linear_acceleration_z = cellfun(@(m) double(m.IMU.LinearAcceleration.Z), msgSt);

linear_acceleration = [linear_acceleration_x,linear_acceleration_y, linear_acceleration_z];

compute_ADEV(angular_velocity_x, 'Angular Velocity X');
compute_ADEV(angular_velocity_y, 'Angular Velocity Y');
compute_ADEV(angular_velocity_z, 'Angular Velocity Z');

compute_ADEV(linear_acceleration_x, 'Linear Acceleration X');
compute_ADEV(linear_acceleration_y, 'Linear Acceleration Y');
compute_ADEV(linear_acceleration_z, 'Linear Acceleration Z');


% plot allan variance
figure;
[avar, tau] = compute_AVAR(linear_acceleration_x);
loglog(tau, avar,'DisplayName','Linear acc-x')
hold on
[avar, tau] = compute_AVAR(linear_acceleration_y);
loglog(tau, avar,'DisplayName','Linear acc-y')
hold on
[avar, tau] = compute_AVAR(linear_acceleration_z);
loglog(tau, avar,'DisplayName','Linear acc-z')
hold on
str = sprintf('Allan Variance for Accelerometer');
title(str)
xlabel('\tau')
ylabel('\sigma^2(\tau)')
legend;

figure;
[avar, tau] = compute_AVAR(angular_velocity_x);
loglog(tau, avar,'DisplayName','Angular Velo-x')
hold on
[avar, tau] = compute_AVAR(angular_velocity_y);
loglog(tau, avar,'DisplayName','Angular Velo-y')
hold on
[avar, tau] = compute_AVAR(angular_velocity_z);
loglog(tau, avar,'DisplayName','Angular Velo-z')
hold on
str = sprintf('Allan Variance for Gyroscope');
title(str)
xlabel('\tau')
ylabel('\sigma^2(\tau)')
legend;

%Compute Allan Deviation
function [] = compute_ADEV(x, st)
    t0 = 1/40; % Freq=40Hz
    theta = cumsum(x, 1)*t0;

    maxNumM = 100;
    L = size(theta, 1);
    maxM = 2.^floor(log2(L/2)); 
    m = logspace(log10(1), log10(maxM), maxNumM).';
    m = ceil(m); % m must be an integer.
    m = unique(m); % Remove duplicates.
    
    tau = m*t0;
    
    avar = zeros(numel(m), 1);
    for i = 1:numel(m)
        mi = m(i);
        avar(i,:) = sum( ...
            (theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
    end
    avar = avar ./ (2*tau.^2 .* (L - 2*m));
    
    adev = sqrt(avar);
    
    %Angle Random Walk
    slope = -0.5;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));
    
    % Find the y-intercept of the line.
    b = logadev(i) - slope*logtau(i);
    % Determine the angle random walk coefficient from the line.
    logN = slope*log(1) + b;
    N = 10^logN;
    tauN = 1;
    lineN = N ./ sqrt(tau);
    %Rate Random Walk
    slope = 0.5;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));
    
    % Find the y-intercept of the line.
    b = logadev(i) - slope*logtau(i);
    
    % Determine the rate random walk coefficient from the line.
    logK = slope*log10(3) + b;
    K = 10^logK;
    tauK = 3;
    lineK = K .* sqrt(tau/3);
    %Bias Instability
    slope = 0;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));
    
    % Find the y-intercept of the line.
    b = logadev(i) - slope*logtau(i);
    
    % Determine the bias instability coefficient from the line.
    scfB = sqrt(2*log(2)/pi);
    logB = b - log10(scfB);
    B = 10^logB;
    tauB = tau(i);
    lineB = B * scfB * ones(size(tau));
    %PLOT
    tauParams = [tauN, tauK, tauB];
    params = [N, K, scfB*B];
    figure
    loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
        tauParams, params, 'o')
    str = sprintf('Allan Deviation with Noise Parameters for %s', st);
    title(str)
    xlabel('\tau')
    ylabel('\sigma(\tau)')
    legend('$\sigma (rad/s)$', '$\sigma_N ((rad/s)/\sqrt{Hz})$', ...
        '$\sigma_K ((rad/s)\sqrt{Hz})$', '$\sigma_B (rad/s)$', 'Interpreter', 'latex')
    text(tauParams, params, {'N', 'K', '0.664B'})
    grid on
    axis equal
end

%Compute Allan Variance
function [avar,tau] = compute_AVAR(x)
    t0 = 1/40; % Freq=40Hz
    theta = cumsum(x, 1)*t0;

    maxNumM = 100;
    L = size(theta, 1);
    maxM = 2.^floor(log2(L/2)); 
    m = logspace(log10(1), log10(maxM), maxNumM).';
    m = ceil(m); % m must be an integer.
    m = unique(m); % Remove duplicates.
    
    tau = m*t0;
    
    avar = zeros(numel(m), 1);
    for i = 1:numel(m)
        mi = m(i);
        avar(i,:) = sum( ...
            (theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
    end
    avar = avar ./ (2*tau.^2 .* (L - 2*m));
    
    adev = sqrt(avar);
    
    %Angle Random Walk
    slope = -0.5;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));
    
    % Find the y-intercept of the line.
    b = logadev(i) - slope*logtau(i);
    % Determine the angle random walk coefficient from the line.
    logN = slope*log(1) + b;
    N = 10^logN;
    tauN = 1;
    lineN = N ./ sqrt(tau);
    %Rate Random Walk
    slope = 0.5;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));
    
    % Find the y-intercept of the line.
    b = logadev(i) - slope*logtau(i);
    
    % Determine the rate random walk coefficient from the line.
    logK = slope*log10(3) + b;
    K = 10^logK;
    tauK = 3;
    lineK = K .* sqrt(tau/3);
    %Bias Instability
    slope = 0;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));
    
    % Find the y-intercept of the line.
    b = logadev(i) - slope*logtau(i);
    
    % Determine the bias instability coefficient from the line.
    scfB = sqrt(2*log(2)/pi);
    logB = b - log10(scfB);
    B = 10^logB;
    tauB = tau(i);
    lineB = B * scfB * ones(size(tau));    
end
