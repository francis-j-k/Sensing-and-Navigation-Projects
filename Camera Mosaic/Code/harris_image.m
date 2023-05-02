buildingDir = 'D:\NEU\images\RSN_calib\Ruggles_15';
buildingScene = imageDatastore(buildingDir);

% Display images to be stitched.
montage(buildingScene.Files)

% Read the first image from the image set.
I = readimage(buildingScene,1);

k = [2914.68790  0 1968.07400; 0 2913.50519 1122.85275; 0 0 1];

radialDistortion = [0.00900 -0.02413 0]; 
tangentialDistortion = [0.00049 -0.00126];

cameraParams = cameraParameters("K",k,"RadialDistortion",radialDistortion, "TangentialDistortion", tangentialDistortion);

% J2 = undistortImage(I,cameraParams,'OutputView','valid');
J = undistortImage(I,cameraParams,'OutputView','full');

% Initialize features for I(1)
grayImage = im2gray(J);
[y,x,m] = harris_filter(grayImage);
[features, points] = extractFeatures(grayImage,[x,y]);

% Initialize all the transformations to the identity matrix. Note that the
% projective transformation is used here because the building images are fairly
% close to the camera. For scenes captured from a further distance, you can use
% affine transformations.
numImages = numel(buildingScene.Files);
tforms(numImages) = projtform2d;



% Initialize variable to hold image sizes.
imageSize = zeros(numImages,2);

% Iterate over remaining image pairs
for n = 1:numImages
    pointsPrevious = points;
    featuresPrevious = features;
        
    % Read I(n).
    I = readimage(buildingScene, n);
    J = undistortImage(I,cameraParams,'OutputView','full');
    
    % Convert image to grayscale.
    grayImage = im2gray(J);    
    
    % Save image size.
    imageSize(n,:) = size(grayImage);
    
    % Detect and extract SURF features for I(n).
    harris_filter(grayImage);
end