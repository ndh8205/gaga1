% star_simulator_main.m
% Star Simulator - MATLAB Version
% Converted from Brian Catraguna's Python code
% Using Hipparcos Catalog

clear; clc;

%% User Input
ra0 = input('Enter the right ascension angle in degrees: ');
de0 = input('Enter the declination angle in degrees: ');
roll0 = input('Enter the roll angle in degrees: ');

% Convert to radians
ra = deg2rad(ra0);
de = deg2rad(de0);
roll = deg2rad(roll0);

%% Sensor Parameters
myu = 1.12e-6;  % length/pixel (meters)
f = 0.00304;    % Focal length (meters)
l = 3280;       % Resolution length (pixels)
w = 2464;       % Resolution width (pixels)

%% Calculate FOV
FOVy = rad2deg(2 * atan((myu*w/2) / f));
FOVx = rad2deg(2 * atan((myu*l/2) / f));

fprintf('Resolution: %d x %d\n', l, w);
fprintf('FOV: %.1f x %.1f degrees\n', FOVx, FOVy);

%% Create Rotation Matrix
M = create_M_matrix(ra, de, roll);
fprintf('\nMatrix M:\n');
disp(M);

% Check if orthogonal
M_transpose = M';
M_inverse = inv(M);
if isequal(round(M_transpose, 5), round(M_inverse, 5))
    fprintf('Matrix M is orthogonal\n');
else
    warning('Matrix M is not orthogonal');
end

%% Load Star Catalogue
fprintf('\nReading CSV file...\n');
opts = detectImportOptions('filtered_catalogue/Hipparcos_Below_6.0.csv');
opts.VariableNamingRule = 'preserve';
star_catalogue = readtable('filtered_catalogue/Hipparcos_Below_6.0.csv', opts);

% Extract columns (use original column names)
star_id = star_catalogue.('Star ID');
ra_stars = star_catalogue.RA;
de_stars = star_catalogue.DE;
magnitudes = star_catalogue.Magnitude;

%% Search for stars within FOV
R = sqrt(deg2rad(FOVx)^2 + deg2rad(FOVy)^2) / 2;
alpha_start = ra - R/cos(de);
alpha_end = ra + R/cos(de);
delta_start = de - R;
delta_end = de + R;

% Find stars in range
star_within_ra = (alpha_start <= ra_stars) & (ra_stars <= alpha_end);
star_within_de = (delta_start <= de_stars) & (de_stars <= delta_end);
stars_in_fov = star_within_ra & star_within_de;

fprintf('Stars in FOV: %d\n', sum(stars_in_fov));

% Filter stars
ra_i = ra_stars(stars_in_fov);
de_i = de_stars(stars_in_fov);
mag_i = magnitudes(stars_in_fov);

%% Convert to sensor coordinates
star_sensor_coords = zeros(length(ra_i), 3);
for i = 1:length(ra_i)
    dir_vector = [cos(ra_i(i))*cos(de_i(i));
                  sin(ra_i(i))*cos(de_i(i));
                  sin(de_i(i))];
    star_sensor_coords(i, :) = (M_transpose * dir_vector)';
end

%% Convert to image coordinates
star_loc = zeros(length(ra_i), 2);
for i = 1:length(ra_i)
    x = f * (star_sensor_coords(i,1) / star_sensor_coords(i,3));
    y = f * (star_sensor_coords(i,2) / star_sensor_coords(i,3));
    star_loc(i, :) = [x, y];
end

pixel_per_length = 1 / myu;

%% Convert to pixel coordinates
pixel_coords = [];
filtered_mag = [];

for i = 1:size(star_loc, 1)
    x1 = star_loc(i, 1);
    y1 = star_loc(i, 2);

    x1pixel = round(pixel_per_length * x1);
    y1pixel = round(pixel_per_length * y1);

    % Check if within bounds
    if abs(x1pixel) > l/2 || abs(y1pixel) > w/2
        continue;
    end

    pixel_coords = [pixel_coords; x1pixel, y1pixel];
    filtered_mag = [filtered_mag; mag_i(i)];
end

fprintf('Stars to draw: %d\n', length(filtered_mag));

%% Create background image
background = zeros(w, l);

% Draw stars
for i = 1:length(filtered_mag)
    x = round(l/2 + pixel_coords(i,1));
    y = round(w/2 - pixel_coords(i,2));

    % Calculate star properties
    mag = abs(filtered_mag(i) - 7);
    radius = round((mag/9)*5 + 2);
    color = round((mag/9)*155 + 100);

    % Draw circle
    background = draw_circle(background, x, y, radius, color);
end

%% Add noise
noise = randi([0, 50], w, l);
background = uint8(0.9 * background + 0.1 * noise);

%% Display
figure;
imshow(background);
title(sprintf('RA=%.1f°, DEC=%.1f°, Roll=%.1f°', ra0, de0, roll0));

% Save
filename = sprintf('ra%.1f_de%.1f_roll%.1f.png', ra0, de0, roll0);
imwrite(background, filename);
fprintf('\nSaved: %s\n', filename);
