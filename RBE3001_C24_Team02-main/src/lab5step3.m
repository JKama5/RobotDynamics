function [center_arr, color_arr] = lab5step3(camera)
plotFlag = false; % set to true to see incremental photo/pipeline process


%   camera = Camera();
%   imgCB = camera.getImage();
%     [xL, xR, yB, yT, imagePoints] = findCornerExtremes(imgCB);
xL = camera.cb_extremes{1};
xR = camera.cb_extremes{2};
yB = camera.cb_extremes{3};
yT = camera.cb_extremes{4};
imagePoints = camera.cb_extremes{5};

%     disp("Press any key once ready to take a snapshot of the board");
%     pause;

img = camera.getImage();
pause(1);

close all;
%% Image Pipeline
% 1) ignore area outside checkerboard
% 2) segment colored balls using threshold determined in HSV space
% 3) apply median filter to reduce noise
% 4) find the (x,y) pixel coord using regionprops()

if(plotFlag) figure; imshow(img); end

% first, ignore anything other than the checkerboard
ROI = false(size(img,1),size(img,2)); % binary checkerboard mask
ROI(yB:yT,xL:xR) = true;
maskedCB = img;
% apply mask to original image
for i=1:3
    maskedCB(:,:,i) = img(:,:,i).*uint8(ROI);
end

if(plotFlag)
    figure; imshow(maskedCB);
    hold on;
    plot(imagePoints(:,1), imagePoints(:,2), 'ro'); % plots cb coords
    % plot([x1 x2 x3 x4],[y1 y2 y3 y4],'ro'); % plots all approximated corners
    plot([xL xR],[yB yT],'ro'); % plots extremes
    hold off; 
end

% second, segment the colored balls using a threshold
[BWR, imgRedMask] = red_mask(maskedCB);
% figure; imshow(BWR);
[BWG, imgGrnMask] = green_mask(maskedCB);
% figure; imshow(BWG);
[BWO, imgRngMask] = orange_mask(maskedCB);
% figure; imshow(BWO);
[BWY, imgYelMask] = yellow_mask(maskedCB);
% figure; imshow(BWY);

% BW_ALL = BWR | BWG | BWO | BWY;
% figure; imshow(BW_ALL);

% cell order: red, green, orange, yellow
BW = {BWR BWG BWO BWY};
medFilt = cell(1,4);
% third, apply the median filter to reduce noise
for i = 1:4
    medFilt{i} = medfilt2(BW{i});
     %figure; imshow(medFilt{i});
end

% for i = 1:4
%     medFilt{i} = medfilt2(medFilt{i});
%      figure; imshow(medFilt{i});
% end

% med = medfilt2(BW_ALL);
% figure; imshow(med);

color_vars = cell(1,4);
% finally, find the (x,y) pixel coord using regionprops()
for i = 1:4
    color_vars{i} = regionprops(medFilt{i},'Centroid','BoundingBox','Area');
end

% extract centers and box positions from color_vars
buffer = 0;
center_arr = 0;
color_arr = 0;
positions = 0;
color_names = ['r', 'g', 'o', 'y'];
label_names = {'Red' 'Green' 'Orange' 'Yellow'};
label_str = cell(1);
for k = 1:4
    if ~isempty(color_vars{k})
        color_cnt = size(color_vars{k},1); % count how many of same color balls
        for i = 1:color_cnt
            test=color_vars{k}(i).Area;
            if color_vars{k}(i).Area<250
                continue
            end
            center_arr(i+buffer,1:2) = color_vars{k}(i).Centroid;
            color_arr(i+buffer) = color_names(k);
            positions(i+buffer,1:4) = color_vars{k}(i).BoundingBox;
            label_str{i+buffer} = label_names{k};
        end
        buffer = buffer + color_cnt;
    end
end

NewPositions=[];
NewCenter_arr=[];
NewColor_arr=[];
NewLabel_str={};
for i=1:size(positions,1)
    if ~positions(i,:)==[0 0 0 0]
        NewPositions=[NewPositions;positions(i,:)];
        NewCenter_arr=[NewCenter_arr;center_arr(i,:)];
        NewColor_arr=[NewColor_arr,color_arr(1,i)];
        NewLabel_str=cat(2,NewLabel_str,label_str{1,i});
    end
end
positions=NewPositions;
center_arr=NewCenter_arr;
color_arr=NewColor_arr;
label_str=NewLabel_str;

if(positions ~= 0)
figure; imshow(insertObjectAnnotation(img,'rectangle',positions,label_str));
hold on;
plot(center_arr(:,1),center_arr(:,2),'ro');
hold off;
end

color_arr=color_arr';

% finds checkerboard corner extremes to be used for masking
% NOT PERFECT, just an estimation, corners will probably be cut off
% will be more accurate the less skewed the checkerboard is on camera
function [xL, xR, yB, yT, imagePoints] = findCornerExtremes(img)
    [imagePoints, boardSize] = detectCheckerboardPoints(img);
    x1 = imagePoints(1,1) - (imagePoints(5,1)-imagePoints(1,1)); % top left of cb
    x2 = imagePoints(4,1) - (imagePoints(8,1)-imagePoints(4,1)); % bottom left of cb
    x3 = imagePoints(37,1) + (imagePoints(37,1)-imagePoints(33,1)); % top right of cb
    x4 = imagePoints(40,1) + (imagePoints(40,1)-imagePoints(36,1)); % bottom right of cb
    y1 = imagePoints(1,2) - (imagePoints(2,2)-imagePoints(1,2)); % top left of cb
    y2 = imagePoints(4,2) + (imagePoints(4,2)-imagePoints(3,2)); % bottom left of cb
    y3 = imagePoints(37,2) - (imagePoints(38,2)-imagePoints(37,2)); % top right of cb
    y4 = imagePoints(40,2) + (imagePoints(40,2)-imagePoints(39,2)); % bottom right of cb
    xL = min([x1 x2]); % left most value
    xR = max([x3 x4]); % right most value
    yB = min([y1 y3]); % bottom most value
    yT = max([y2 y4]); % top most value
end
end % end script