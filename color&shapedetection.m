clear all; clc; close all;

% Connect to CoppeliaSim
vrep = remApi('remoteApi');
vrep.simxFinish(-1);  % close any previous connections
id = vrep.simxStart('127.0.0.1', 19000, true, true, 5000, 5);

if id < 0
    disp('Failed to connect to CoppeliaSim.');
    vrep.delete;
    return;
else
    fprintf('Connected to CoppeliaSim on port 19000.\n');
end

% Get vision sensor handle
[~, camHandle] = vrep.simxGetObjectHandle(id, 'visionSensor', vrep.simx_opmode_oneshot_wait);

fprintf('Waiting for objects on conveyor belt...\n');

while true
    % Check proximity signal (imageReady from CoppeliaSim)
    [~, imageReady] = vrep.simxGetIntegerSignal(id, 'imageReady', vrep.simx_opmode_oneshot);
    
    if imageReady == 1
        fprintf('Object detected. Capturing image...\n');
        
        % Capture image
        [~, ~, img] = vrep.simxGetVisionSensorImage2(id, camHandle, 0, vrep.simx_opmode_oneshot_wait);
        
        % Reset signal so same object is not detected again
        vrep.simxSetIntegerSignal(id, 'imageReady', 0, vrep.simx_opmode_oneshot);
        
        % Convert image to HSV
        hsv_img = rgb2hsv(img);
        hue = hsv_img(:,:,1);
        sat = hsv_img(:,:,2);
        val = hsv_img(:,:,3);

        % Mask non-object background
        object_pixels = (sat > 0.4 & val > 0.2);
        mean_hue = mean(hue(object_pixels));

        % Color detection
        color = 'Unknown';
        if (mean_hue < 0.05 || mean_hue > 0.95)
            color = 'Red';
            mask = (hue < 0.05 | hue > 0.95) & object_pixels;
        elseif (mean_hue >= 0.25 && mean_hue <= 0.45)
            color = 'Green';
            mask = (hue > 0.25 & hue < 0.45) & object_pixels;
        elseif (mean_hue >= 0.55 && mean_hue <= 0.7)
            color = 'Blue';
            mask = (hue > 0.55 & hue < 0.7) & object_pixels;
        else
            mask = object_pixels;
        end

        fprintf('Detected Color: %s | Mean Hue: %.2f\n', color, mean_hue);

        % Apply mask to isolate object
        masked_img = img;
        for ch = 1:3
            temp = masked_img(:,:,ch);
            temp(~mask) = 0;
            masked_img(:,:,ch) = temp;
        end

        gray_img = rgb2gray(masked_img);

        % Edge detection
        sobel_edges = edge(gray_img, 'Sobel');
        laplacian = fspecial('laplacian', 0.5);
        laplacian_img = imfilter(double(gray_img), laplacian, 'replicate');
        laplacian_edges = abs(laplacian_img) > 15;
        edges = sobel_edges | laplacian_edges;

        % Clean edges
        edges = imclose(edges, strel('disk', 2));
        edges = imfill(edges, 'holes');
        edges = bwareaopen(edges, 300);

        % Shape detection
        [B, L] = bwboundaries(edges, 'noholes');
        stats = regionprops(L, 'BoundingBox');

        figure; imshow(img); title('📐 Shape Detection'); hold on;

        for k = 1:length(B)
            boundary = B{k};
            if length(boundary) < 20
                continue;
            end

            k_hull = convhull(boundary(:,2), boundary(:,1));
            hull_points = [boundary(k_hull,2), boundary(k_hull,1)];
            approx = reducepoly(hull_points, 0.02);
            num_vertices = size(approx, 1);

            x_coords = approx(:,1);
            y_coords = approx(:,2);
            w = max(x_coords) - min(x_coords);
            h = max(y_coords) - min(y_coords);
            aspect_ratio = w / h;

            shape = 'Unknown';
            if num_vertices <= 8 && aspect_ratio > 0.85 && aspect_ratio < 1.2
                shape = 'Cube';
            elseif num_vertices <= 8 && (aspect_ratio <= 0.85 || aspect_ratio >= 1.2)
                shape = 'Cuboid';
            elseif num_vertices > 8 && aspect_ratio > 0.8 && aspect_ratio < 1.2
                shape = 'Cylinder';
            end

            fprintf('Vertices: %d | AR: %.2f | Shape: %s\n', num_vertices, aspect_ratio, shape);

            plot(boundary(:,2), boundary(:,1), 'g', 'LineWidth', 1.5);
            plot(approx(:,1), approx(:,2), 'r*-', 'LineWidth', 2);
            bbox = stats(k);
            text(bbox.BoundingBox(1), bbox.BoundingBox(2)-10, ...
                sprintf('%s (%s)', shape, color), 'Color', 'black', ...
                'FontSize', 12, 'FontWeight', 'bold');
        end

        % Wait until object leaves the sensor range
        fprintf('⏳ Waiting for object to move away...\n');
        while true
            [~, imageReady] = vrep.simxGetIntegerSignal(id, 'imageReady', vrep.simx_opmode_oneshot);
            if imageReady == 0
                fprintf('Object moved. Ready for next.\n\n');
                break;
            end
            pause(0.1);
        end
    end

    pause(0.1);  % Optional: limit CPU usage
end

% Disconnect when needed
% vrep.simxFinish(id);
% vrep.delete;
