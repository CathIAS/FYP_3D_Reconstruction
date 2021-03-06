% Auto-generated by cameraCalibrator app on 19-Feb-2016
%-------------------------------------------------------


% Define images to process
imageFileNames = {'/home/liuwx/Documents/calibration_r/IMG_2509.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2510.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2511.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2512.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2513.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2514.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2515.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2516.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2517.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2518.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2519.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2520.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2521.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2522.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2523.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2524.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2525.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2526.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2527.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2528.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2529.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2530.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2531.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2532.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2533.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2534.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2535.JPG',...
    '/home/liuwx/Documents/calibration_r/IMG_2536.JPG',...
    };

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Generate world coordinates of the corners of the squares
squareSize = 70;  % in units of 'mm'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'mm', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', []);

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams, 'CameraCentric');

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
originalImage = imread(imageFileNames{1});
undistortedImage = undistortImage(originalImage, cameraParams);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')
