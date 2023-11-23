root_img = '/media/yanhao/8tb1/00_data_TII/island_localization/frames/mav0/cam0/data/';

% distortion_coeff = [
%                 -0.11106452180588008,
%                 0.17858247476424663,
%                 -0.0006877836355554949,
%                 -0.00013122812418964706];


intrinsicMatrix = [897.4764841120259    0      516.5816593053271;
                     0       900.131314815222  377.03405845062315;
                     0         0         1  ];
distortionCoefficients = [-0.11106452180588008  0.17858247476424663  -0.0006877836355554949  -0.00013122812418964706 0];


imageSize = [751 1026];

intrinsics = cameraIntrinsicsFromOpenCV(intrinsicMatrix, ...
                                       distortionCoefficients,imageSize);

I = imread([root_img, '1683701610299491504.png']);

J = undistortImage(I,intrinsics);
imshowpair(I,J,"montage");
title("Original Image (left) vs. Corrected Image (right)");