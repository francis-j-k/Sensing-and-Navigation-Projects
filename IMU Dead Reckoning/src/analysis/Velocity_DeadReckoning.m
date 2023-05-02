bag = rosbag ('D:\Northeastern University\Sem 1\Robot Sensing and Navigation\LAB4\2022-10-27-19-39-11.bag');
bsel2 = select(bag,'Topic','/gps');
msgStructs2 = readMessages(bsel2,'DataFormat','struct');
bsel = select(bag,'Topic','/imu');
msgStructs = readMessages(bsel,'DataFormat','struct');
acceleration_x = cellfun(@(m) double(m.IMU.LinearAcceleration.X),msgStructs);
acceleration_y = cellfun(@(m) double(m.IMU.LinearAcceleration.Y),msgStructs);
acceleration_z = cellfun(@(m) double(m.IMU.LinearAcceleration.Z),msgStructs);
magneticField_x = cellfun(@(m) double(m.MagField.MagneticField_.X),msgStructs);
magneticField_y = cellfun(@(m) double(m.MagField.MagneticField_.Y),msgStructs);
magneticField_z = cellfun(@(m) double(m.MagField.MagneticField_.Z),msgStructs);
omega_x = cellfun(@(m) double(m.IMU.AngularVelocity.X),msgStructs);
omega_y = cellfun(@(m) double(m.IMU.AngularVelocity.Y),msgStructs);
omega_z = cellfun(@(m) double(m.IMU.AngularVelocity.Z),msgStructs);
easting = cellfun(@(m) double(m.UTMEasting),msgStructs2);
northing = cellfun(@(m) double(m.UTMNorthing),msgStructs2);
gps_time_sec = cellfun(@(m) double(m.Header.Stamp.Sec),msgStructs2);
gps_time_nanosec = cellfun(@(m) double(m.Header.Stamp.Nsec),msgStructs2);
gps_time = double(gps_time_sec + ( gps_time_nanosec * 10^(-9)));
gps_time = gps_time - gps_time(1);
imu_time_sec = cellfun(@(m) double(m.Header.Stamp.Sec),msgStructs);
imu_time_nanosec = cellfun(@(m) double(m.Header.Stamp.Nsec),msgStructs);
imu_time = double(imu_time_sec + ( imu_time_nanosec * 10^(-9)));
imu_time = imu_time - imu_time(1);
orientation_x = cellfun(@(m) double(m.IMU.Orientation.X),msgStructs);
orientation_y = cellfun(@(m) double(m.IMU.Orientation.Y),msgStructs);
orientation_z = cellfun(@(m) double(m.IMU.Orientation.Z),msgStructs);
orientation_w = cellfun(@(m) double(m.IMU.Orientation.W),msgStructs);

quat = [orientation_w orientation_x orientation_y orientation_z];
eulZYX_rad = quat2eul(quat);
yaw = eulZYX_rad (:,1);
pitch = eulZYX_rad (:,2);
roll = eulZYX_rad (:,3);

% velocity from raw acceleration data
imu_velocity = cumtrapz(imu_time,acceleration_x);

%velocty from gps
gps_velocity = zeros(length(easting),1);
distance = zeros(length(easting),1);
gps_dist_time =  zeros(length(easting),1);
for i = 2 : length(easting)
    gps_velocity(1) =  sqrt((easting(2)-easting(1))^2+(northing(2)-northing(1))^2);   
    distance(i-1) = sqrt((easting(i)-easting(i-1))^2+(northing(i)-northing(i-1))^2);
    gps_dist_time(i-1) = abs(gps_time(i) - gps_time(i-1));
    if gps_dist_time(i-1) ==0
        gps_velocity(i) = gps_velocity(i-1);
    else
        gps_velocity(i) = (distance(i-1)/gps_dist_time(i-1));  
    end
end
gps_time = sort(gps_time);
figure;
plot(imu_time,imu_velocity,"DisplayName","Velocity from raw acceleration x");
hold on
plot(gps_time,gps_velocity,"DisplayName","Velocity from gps");
xlabel("Time in s");
ylabel("Velocity in m/s")
title("Velocity from raw acceleration x and gps")
legend;

% correction of acceleration data
% low_pass filter on accelerometer
a = 0.992;
filtered_acc = zeros(length(acceleration_x),1);
filtered_acc(1) = acceleration_x(1);
for i = 2 : length(acceleration_x)    
    filtered_acc(i) = (1-a)*acceleration_x(i) + a * filtered_acc(i-1);
end

% find constant acceleration and zero velocity intervals
rate_change_acc_x = diff(filtered_acc);
count = 0;
static_period = [];
for i = 2 : length(rate_change_acc_x)
    if abs(rate_change_acc_x(i)) < 0.001
        count = count +1;
    else
        if count > 1*40
            k1 = int16(imu_time(i-1-count));
            k2 = int16(imu_time(i-1));
            a = find(gps_time>k1,1,'first');
            b = find(gps_time>k2,1,'first')-1;
            if gps_velocity(a:b,1) < 0.7
                static_period = [static_period,i-1-count];
                static_period = [static_period,i-1];
            end
        end
        count = 0;
    end
end

num_static_period = length(static_period);
for i = 2 : (num_static_period)
    avg = mean(filtered_acc(static_period(i-1):static_period(i)));
    if i==2
        start = 1;
    else
        start = static_period(i-1);
    end   
    if i < num_static_period -1
        end_ = static_period(i);
    else
        end_ = length(filtered_acc);
    end
    for j = start : end_
        corr_acc(j) = filtered_acc(j)- avg;
    end
end

% velocity from corrected
vel_new = cumtrapz(imu_time,corr_acc);
vel_new(end) = 0;
gps_velocity(length(gps_velocity)) =0;

figure;
plot(imu_time,acceleration_x,"DisplayName","raw acceleration data");
hold on;
plot(imu_time,filtered_acc,"DisplayName","Low Pass Filter acceleration");
hold on
plot(imu_time,corr_acc,"DisplayName","corrected acceleration");
hold on
xlabel("Time in s");
ylabel("Acceleration in m/s^2");
title("Forward Acceleration plot")
legend

figure;
plot(gps_time,gps_velocity,"DisplayName","Velocity from GPS");
hold on;
plot(imu_time,vel_new,"DisplayName","Velocity from accelerometer");
xlabel("Time in secs");
ylabel("Velocity in m/s");
title("Forward velocity plot")
legend

% Dead_Reckoning
scale_matrix = [0.8760 0.0243;0.0243 0.9952];
offset_magx = -0.1604;
offset_magy = 0.0197;
offseted_magx = magneticField_x - offset_magx;
offseted_magy = magneticField_y - offset_magy;
calibrated_mag =  (scale_matrix*[offseted_magx,offseted_magy]')';

magnetic_yaw= (atan2(-calibrated_mag(:,2),calibrated_mag(:,1)));
raw_mag_yaw = atan2(-magneticField_y,magneticField_x);
unwrapped_mag_yaw = unwrap(magnetic_yaw);
gyro_yaw = cumtrapz(imu_time,omega_z)+ magnetic_yaw(1);
wrapped_gyro_yaw = wrapToPi(gyro_yaw);

LPF_mag= lowpass(unwrapped_mag_yaw,0.001,40);
HPF_gyro = highpass(gyro_yaw,0.01,40);
a_c = 0.991;
filtered_yaw = a_c*LPF_mag + (1-a_c)*HPF_gyro;

% figure;
% plot(imu_time,unwrap(yaw),"Display Name","IMU Yaw");
% hold on;
% plot(imu_time,(filtered_yaw),"DisplayName","Complimentary Filter Yaw");
% hold on;
% legend;


%fwd_vel from part 2
figure;
imu_disp = cumtrapz(imu_time,vel_new); %IMU Displacement
plot(imu_time,imu_disp,"red","DisplayName","IMU Displacement");
hold on;
gps_disp = cumtrapz(gps_time,gps_velocity);
plot(gps_time,gps_disp,"blue","DisplayName","GPS Displacement");
title("Plot Displacement vs time");
xlabel("Displacement in m");
ylabel("Time in s");
legend;

% compare w*X_dot and acc_y
X_dot_dot = corr_acc;
% X_dot = cumtrapz(imu_time, X_dot_dot);
X_dot = vel_new;
for  i = 1 : length(omega_z)
    Y_dot_dot_calculated(i) = omega_z(i)* vel_new(i) ;
end

figure;
plot(imu_time,Y_dot_dot_calculated,"Displayname","Ω.x dot");
hold on;
plot(imu_time,acceleration_y,"Displayname","Y dot dot observed");
title("Plot of Ω.x dot vs Y dot dot obs");
xlabel("Acceleration in m/s^2");
ylabel("Time in s");
legend;

Vn =[];
Ve= [];
yaw_after = wrapToPi(filtered_yaw);

for i = 1:length(yaw_after)
    yaw_angle = yaw_after(i); %yaw from magnetometer
    Vn(i) = X_dot(i) * cos(yaw_angle);
    Ve(i) = X_dot(i) * sin(yaw_angle);
end

x_e = (cumtrapz(imu_time, Ve))';
x_n = (cumtrapz(imu_time, Vn))';

% starting_point_correction for magnetometer
utm_easting = easting - easting(1);
utm_northing = northing - northing(1);

figure;
plot(x_e, x_n,"red", utm_easting, utm_northing,"blue");
title("Plot of IMU and GPS trajectory Before adjustment");
xlabel("UTM Northing");
ylabel("UTM Easting");
legend("IMU trajectory","GPS trajectory");

% Heading correction for magnetometer
GPS_slope = (utm_northing(25)-utm_northing(1))/(utm_easting(25) - utm_easting(1));
imu_slope = (x_n(2200)-x_n(1))/(x_e(2200)-x_e(1));

GPS_inc = atan(GPS_slope) + pi();
imu_inc = atan(imu_slope) + pi();

theta_head = GPS_inc - imu_inc;
R_head = [cos(theta_head) -sin(theta_head); sin(theta_head) cos(theta_head)];

R_x_e = (R_head * [x_e,x_n]')';

figure;
plot(R_x_e(:,1), R_x_e(:,2),"red", utm_easting, utm_northing,"blue");
title("Plot of IMU and GPS trajectory After adjustment");
xlabel("UTM Northing");
ylabel("UTM Easting");
legend("IMU trajectory","GPS trajectory");
