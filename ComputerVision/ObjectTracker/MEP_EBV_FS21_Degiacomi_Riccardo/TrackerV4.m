%% Clean Up !
clc;
clear all;
close all;
%% Get Data
data = VideoReader('bildsequenzebv.mp4');
% data = VideoReader('mouse.mp4');
% data = VideoReader('charger.mp4');
frame = read(data,1);
%% Prepare Data
%save every 10th image in a struct, convert  the images to grayimages
% % alreadyRead=0;
% if alreadyRead==0
    num_frames=data.NumFrames;
    stepsize=10;
    frames=struct;
    idx2=1;
    
        for idx=1:stepsize:num_frames
            frames(idx2).frame=read(data,idx); 
            frames(idx2).frame = rgb2gray(frames(idx2).frame)
%             imshow(frames(idx2).frame);
            idx2=idx2+1;
            
        end
%     frames2=frames;
    
% end
%% Do Segmentation
backGround=imread('background.png');
% morphObj_rectangle = strel('rectangle', [3 3]);
morphObj_disk = strel('disk', 8);
offset=14;
start=2;
differenceImage=struct;

for N=start:(length(frames(:))-3)
    %cast to double
    actualImage=double(frames(N).frame);
    %get thresholdvalue
    threshold=255*graythresh(actualImage)+offset;
    %cast backGround image to double
    backGround=double(backGround); 
    %difference Image actual frame, frame before
    differenceImage(N-1).frame=uint8((255+backGround-actualImage)/2); 
    %do backGround estimation
    backGround=0.95*backGround+(1-0.95)*actualImage;
    %create Binary Image
    thresholdImage(N).frame=abs(differenceImage(N-1).frame-threshold)>11; 
    %do morphological operations
    ThreshImageOpen = imopen(thresholdImage(N).frame, morphObj_rectangle);    
    ThreshImageClose(N).frame = imclose(ThreshImageOpen, morphObj_disk);
    % Fill holes
    ThreshImageClose(N).frame = imfill(ThreshImageClose(N).frame, 'holes');
    imshow(ThreshImageClose(N).frame);
%     pause(0.1);
end
imwrite(uint8(backGround), 'background.png','png');
%% get the biggest blobs
%Note!! For some reason the Program sometimes can't finde 5 blobs.
%in this case, change the nr of blobs to 4 or less or restart the programm
%(matlab, not the script) if you get an error in this section.
% This happened twice and could not be reproduced. 


idx=2;
for idx=2:length(ThreshImageClose(:))
    %get the number of blobs
    [labeledImage, numberOfBlobs] = bwlabel((ThreshImageClose(idx).frame));
    %choose the 5 blobs with the biggest area in order to minimize noise
    numberToExtract = 5;
    %save all the different areas in an array
    blobMeasurements = regionprops(labeledImage, 'area');
    allAreas = [blobMeasurements.Area];
    %sort the areas in descending order
    [sortedAreas, sortIndexes] = sort(allAreas, 'descend');
    %get the 5 biggest blobs
    biggestBlob = ismember(labeledImage, sortIndexes(1:numberToExtract));
    ThreshImageClose(idx).frame = biggestBlob > 0;
    imshow(biggestBlob, []);
end
%% get the center + boundingbox
Prop=struct;
for idx=2:length(ThreshImageClose(:))
    %get the center coordinates and the boundingox values for each blob for
    %each frame
    Prop(idx).prop = regionprops(ThreshImageClose(idx).frame,'Centroid','BoundingBox');
    imshow(ThreshImageClose(idx).frame, []);
    for Ind=1:size(Prop(idx).prop(:),1) 
        % save the center coordinates in a variable
        Cent=Prop(idx).prop(Ind).Centroid;   
        %assign x and y coordinate
        X=Cent(1);Y=Cent(2);
        %draw center points
        line([X-0.5 X+0.5], [Y Y],'LineWidth',1,'Color',[1 0 0]);
        line([X X], [Y-0.5 Y+0.5],'LineWidth',1,'Color',[1 0 0]);
        % save the bounding box values
        BBox = Prop(idx).prop(Ind).BoundingBox;
        %print the bounding box
        rectangle('Position', BBox, 'EdgeColor',[0 1 0]);
    end
pause(0.1);
end

%% Fill the Boundinbox area with ones
for idx=2:length(ThreshImageClose(:))
    for Ind=1:size(Prop(idx).prop(:),1) 
        bbox = Prop(idx).prop(Ind).BoundingBox;
        x1 = ceil(bbox(1));
        x2 = (x1 + bbox(3));
        y1 = ceil(bbox(2));
        y2 = (y1 + bbox(4));
        BW2(y1:y2, x1:x2) = 1; % NOTE : y comes first, NOT x!!!
        ThreshImageClose(idx).frame(y1:y2,x1:x2)=1;
    end
imshow(ThreshImageClose(idx).frame);
end

%% Adjust the length

for idx=2:length(ThreshImageClose(:))
    % Find out the length of the shorter matrix
    minLengthx = min(length(frames(idx).frame(:,1)),length(ThreshImageClose(idx).frame(:,1)));
    minLengthy = min(length(frames(idx).frame(1,:)),length(ThreshImageClose(idx).frame(1,:)));
    % Removes any extra elements from the longer matrix
    ThreshImageClose(idx).frame= ThreshImageClose(idx).frame(1:minLengthx,:);
    ThreshImageClose(idx).frame= ThreshImageClose(idx).frame(:,1:minLengthy);
    
     frames(idx).frame=  frames(idx).frame(1:minLengthx,:);
     frames(idx).frame=  frames(idx).frame(:,1:minLengthy);
 

%     imshow(ThreshImageClose(idx).frame);
end
%% prepare images for feature extraction
DummyForBreakpoint=0;
for idx=2:length(ThreshImageClose(:))
  % multiply the blob image with the original image, in order to use 
  % the resulting image for the feature extraction
  ImageOrig(idx).frame = frames(idx).frame .* uint8(ThreshImageClose(idx).frame);
  % show the image
  imshow(ImageOrig(idx).frame);
end
%% Tracking custom 
close all;
maxNumPoints=15;
for idx=2:length(ThreshImageClose(:))   
    if idx==2 ||  (numel(indexPairs)<20)%(sum(isFoundSurf>0)<25) || (sum(isFoundEigen>0)<25) || (sum(isFoundBrisk>0)<25) || (sum(isFoundFast>0)<25)
        
        %comment this section, and uncomment the other section to use other
        %features. Note, you can not use two different features at once in
        %this section.
        startingPointsEigen=detectMinEigenFeatures(ImageOrig(idx).frame);
        [featuresEigen_1, valid_pointsEigen_1] = extractFeatures(ImageOrig(idx).frame, startingPointsEigen);
        [featuresEigen_2, valid_pointsEigen_2] = extractFeatures(ImageOrig(idx).frame, startingPointsEigen);
        
%          startingPointsFast=detectFASTFeatures(ImageOrig(idx).frame);
%          [featuresFast_1, valid_pointsFast_1]= extractFeatures(ImageOrig(idx).frame, startingPointsFast);
%          [featuresFast_2, valid_pointsFast_2] = extractFeatures(ImageOrig(idx).frame, startingPointsFast);
%         
%          startingPointsBrisk=detectBRISKFeatures(ImageOrig(idx).frame);
%          [featuresBrisk_1, valid_pointsBrisk_1]  = extractFeatures(ImageOrig(idx).frame, startingPointsBrisk);
%          [featuresBrisk_2, valid_pointsBrisk_2] = extractFeatures(ImageOrig(idx).frame, startingPointsBrisk);
%         
%          startingPointsSurf=detectSURFFeatures(ImageOrig(idx).frame);
%         [featuresSurf_1, valid_pointsSurf_1] = extractFeatures(ImageOrig(idx).frame, startingPointsSurf);
%         [featuresSurf_2, valid_pointsSurf_2] = extractFeatures(ImageOrig(idx).frame, startingPointsSurf);
%         
    else

          startingPointsEigen=detectMinEigenFeatures(ImageOrig(idx).frame);
          [featuresEigen_2, valid_pointsEigen_2] = extractFeatures(ImageOrig(idx).frame, startingPointsEigen);
          
%           startingPointsFast=detectFASTFeatures(ImageOrig(idx).frame);
%           [featuresFast_2, valid_pointsFast_2] = extractFeatures(ImageOrig(idx).frame, startingPointsFast);
%           
%           startingPointsBrisk=detectBRISKFeatures(ImageOrig(idx).frame);
%           [featuresBrisk_2, valid_pointsBrisk_2] = extractFeatures(ImageOrig(idx).frame, startingPointsBrisk);
%           
%           startingPointsSurf=detectSURFFeatures(ImageOrig(idx).frame);
%           [featuresSurf_2, valid_pointsSurf_2] = extractFeatures(ImageOrig(idx).frame, startingPointsSurf);

    end
    %match features
    indexPairs = matchFeatures(featuresEigen_1,featuresEigen_2);
    matchedPoints1 = valid_pointsEigen_1(indexPairs(:,1));
    matchedPoints2 = valid_pointsEigen_2(indexPairs(:,2));
    a=showMatchedFeatures(frames(idx).frame,frames(idx).frame,matchedPoints1,matchedPoints2);hold on;
    
%     indexPairs = matchFeatures(featuresFast_1,featuresFast_2);
%     matchedPoints1 = valid_pointsFast_1(indexPairs(:,1));
%     matchedPoints2 = valid_pointsFast_2(indexPairs(:,2));
%     showMatchedFeatures(frames(idx).frame,frames(idx).frame,matchedPoints1,matchedPoints2);hold on;
%     
%     indexPairs = matchFeatures(featuresBrisk_1,featuresBrisk_2);
%     matchedPoints1 = valid_pointsBrisk_1(indexPairs(:,1));
%     matchedPoints2 = valid_pointsBrisk_2(indexPairs(:,2));
%     showMatchedFeatures(frames(idx).frame,frames(idx).frame,matchedPoints1,matchedPoints2);hold on;
%     
%     indexPairs = matchFeatures(featuresSurf_1,featuresSurf_2);
%     matchedPoints1 = valid_pointsSurf_1(indexPairs(:,1));
%     matchedPoints2 = valid_pointsSurf_2(indexPairs(:,2));
%     showMatchedFeatures(frames(idx).frame,frames(idx).frame,matchedPoints1,matchedPoints2);hold on;
    
    hold off
    pause(0.2);
end
close all;
%% Tracking with Tracker Obj
close all;
maxNumPoints=15;
for idx=2:length(ThreshImageClose(:))   
    if idx==2 ||  (sum(isFoundSurf>0)<25) || (sum(isFoundEigen>0)<25) || (sum(isFoundBrisk>0)<25) || (sum(isFoundFast>0)<25)
        if idx==3    
            trackerSurf.release();
            trackerEigen.release();
            trackerBrisk.release();
            trackerFast.release();
        end
        % vision.PointTracker returns a point tracker object that tracks a set of points in a video.
        trackerSurf = vision.PointTracker('MaxBidirectionalError',1);
        trackerEigen = vision.PointTracker('MaxBidirectionalError',1);
        trackerBrisk = vision.PointTracker('MaxBidirectionalError',1);
        trackerFast = vision.PointTracker('MaxBidirectionalError',1);
        
        % Initialize Points    
        %points = detectMinEigenFeatures(I) returns a cornerPoints object, points
        %The object contains information about the feature points detected in a 2-D grayscale input image, I.
        startingPointsEigen=detectMinEigenFeatures(ImageOrig(idx).frame);
        
        %points = detectSURFFeatures(I) returns a SURFPoints object, points
        %containing information about SURF features detected in the 2-D grayscale input image I
        startingPointsSurf=detectSURFFeatures(ImageOrig(idx).frame);
        
        %points = detectBRISKFeatures(I) returns a BRISKPoints object, points
        %The object contains information about BRISK features detected in a 2-D grayscale input image, I.
        startingPointsBrisk=detectBRISKFeatures(ImageOrig(idx).frame);
        
        %points = detectFASTFeatures(I) returns a cornerPoints object, points.
        %The object contains information about the feature points detected in a 2-D grayscale input image, I
        startingPointsFast=detectFASTFeatures(ImageOrig(idx).frame);
        
        % save Location of Points
        startingPointsSurf=startingPointsSurf.Location;
        startingPointsEigen=startingPointsEigen.Location;
        startingPointsBrisk=startingPointsBrisk.Location;
        startingPointsFast=startingPointsFast.Location;
        
        %asign value 1 for each found feature in column vector
        isFoundSurf = true(length(startingPointsSurf),1);
        isFoundEigen= true(length(startingPointsEigen),1);
        isFoundBrisk= true(length(startingPointsBrisk),1);
        isFoundFast= true(length(startingPointsFast),1);
        
        %initialize(pointTracker,points,I) initializes points to track and sets the initial video frame.
        initialize(trackerSurf,startingPointsSurf,ImageOrig(idx).frame);
        initialize(trackerEigen,startingPointsEigen,ImageOrig(idx).frame);
        initialize(trackerBrisk,startingPointsBrisk,ImageOrig(idx).frame);
        initialize(trackerFast,startingPointsFast,ImageOrig(idx).frame);
        
        points_matchEigen = startingPointsEigen;
        points_matchSurf = startingPointsSurf;
        points_matchBrisk = startingPointsBrisk;
        points_matchFast = startingPointsFast;
    else
        %Run System object algorithm, the algorithm is defined by the
        %system object. In this case it does the Featuretracking, without
        %reseting the initial points
        [points_matchSurf, isFoundSurf] = step(trackerSurf, ImageOrig(idx).frame);
        [points_matchEigen, isFoundEigen] = step(trackerEigen, ImageOrig(idx).frame);
        [points_matchBrisk, isFoundBrisk] = step(trackerBrisk, ImageOrig(idx).frame);
        [points_matchFast, isFoundFast] = step(trackerFast, ImageOrig(idx).frame);
    end

    imshow(frames(idx).frame);hold on;
    %plot only points that were found
    indSurf = find(isFoundSurf);
    indEigen=find(isFoundEigen);
    indBrisk=find(isFoundBrisk);
    indFast=find(isFoundFast);
    %reference point
    plot(startingPointsEigen(isFoundEigen,1), startingPointsEigen(isFoundEigen,2), 'ro')
    plot(startingPointsSurf(isFoundSurf,1), startingPointsSurf(isFoundSurf,2), 'bo');
    plot(startingPointsBrisk(isFoundBrisk,1), startingPointsBrisk(isFoundBrisk,2), 'co');
    plot(startingPointsFast(isFoundFast,1), startingPointsFast(isFoundFast,2), 'mo');
    
    %matching point
    plot(points_matchEigen(isFoundEigen,1), points_matchEigen(isFoundEigen,2),'rx')
    plot(points_matchSurf(isFoundSurf,1), points_matchSurf(isFoundSurf,2), 'bx');
    plot(points_matchBrisk(isFoundBrisk,1), points_matchBrisk(isFoundBrisk,2), 'cx');
    plot(points_matchFast(isFoundFast,1), points_matchFast(isFoundFast,2), 'cx');
    
    deltaEigen = ceil(size(indEigen,1)/maxNumPoints);
    deltaSurf = ceil(size(indSurf,1)/maxNumPoints);
    deltaBrisk = ceil(size(indBrisk,1)/maxNumPoints);
    deltaFast = ceil(size(indBrisk,1)/maxNumPoints);
    
    for i0 = 1:deltaEigen:size(indEigen,1)
        pR = startingPointsEigen(indEigen(i0),:);
        pM = points_matchEigen(indEigen(i0),:);
        line([pR(1) pM(1)], [pR(2) pM(2)], 'Color','green','LineWidth',1);
        plot(pM(1), pM(2), 'rx');
    end
    
    for i0 = 1:deltaSurf:size(indSurf,1)
        pR = startingPointsSurf(indSurf(i0),:);
        pM = points_matchSurf(indSurf(i0),:);
        line([pR(1) pM(1)], [pR(2) pM(2)], 'Color','green','LineWidth',1);
        plot(pM(1), pM(2), 'rx');
    end
    
    for i0 = 1:deltaBrisk:size(indBrisk,1)
        pR = startingPointsBrisk(indBrisk(i0),:);
        pM = points_matchBrisk(indBrisk(i0),:);
        line([pR(1) pM(1)], [pR(2) pM(2)], 'Color','green','LineWidth',1);
        plot(pM(1), pM(2), 'rx');
    end
    
    for i0 = 1:deltaFast:size(indFast,1)
        pR = startingPointsFast(indFast(i0),:);
        pM = points_matchFast(indFast(i0),:);
        line([pR(1) pM(1)], [pR(2) pM(2)], 'Color','green','LineWidth',1);
        plot(pM(1), pM(2), 'rx');
    end

   for Ind=1:size(Prop(idx).prop(:),1) 
        Cent=Prop(idx).prop(Ind).Centroid;   
        X=Cent(1);Y=Cent(2);
        line([X-0.5 X+0.5], [Y Y],'LineWidth',1,'Color',[1 0 0]);
        line([X X], [Y-0.5 Y+0.5],'LineWidth',1,'Color',[1 0 0]);
        BBox = Prop(idx).prop(Ind).BoundingBox;
        rectangle('Position', BBox, 'EdgeColor',[0 1 0]);
    end
    
    hold off
    pause(0.2);
end
close all;

%% Track only Center 
trackerSurf.release();
trackerEigen.release();
trackerBrisk.release();
trackerFast.release();

idx=2;

for idx=2:length(ThreshImageClose(:))

    imshow(frames(idx).frame);hold on;
if idx==2 || (mod(idx,2)==0)
%     if (idx==2) || (x1Start-x1)>30 || (x2Start-x2)>30 || (x3Start-x3)>30 
%      if (idx==2) || (idx>3&&((x1Start == x1) || (x2Start==x2) || (x3Start==x3) ))
        a=Prop(idx).prop(1).Centroid;
        x1Start = a(1);
        y1Start = a(2);

        b=Prop(idx).prop(2).Centroid;
        x2Start = b(1);
        y2Start = b(2);

        c=Prop(idx).prop(3).Centroid;
        x3Start = c(1);
        y3Start = c(2);
        
        d=Prop(idx).prop(4).Centroid;
        x4Start = d(1);
        y4Start = d(2);
        
        e=Prop(idx).prop(5).Centroid;
        x5Start = e(1);
        y5Start = e(2);
    end
    a=Prop(idx).prop(1).Centroid;
    x1 = a(1);
    y1 = a(2);

    b=Prop(idx).prop(2).Centroid;
    x2 = b(1);
    y2 = b(2);

    c=Prop(idx).prop(3).Centroid;
    x3 = c(1);
    y3 = c(2);   
    
    d=Prop(idx).prop(4).Centroid;
    x4 = d(1);
    y4 = d(2);  
    
    d=Prop(idx).prop(5).Centroid;
    x5 = e(1);
    y5 = e(2);  
   
%     line([x1(1).start y1(1).start], [x2(1).actual y2(1).actual], 'Color','green','LineWidth',1);hold on;
    plot(x1Start, y1Start, 'go');hold on;
    plot(x1, y1, 'gx');hold on;
    
    plot(x2Start, y2Start, 'ro');hold on;
    plot(x2, y2, 'rx');hold on;
    
    plot(x3Start, y3Start, 'bo');hold on;
    plot(x3, y3, 'bx');hold on;
    
    plot(x4Start, y4Start, 'co');hold on;
    plot(x4, y4, 'cx');hold on;
    
    plot(x5Start, y5Start, 'mo');hold on;
    plot(x5, y5, 'mx');hold on;
   

    line([x1Start x1], [y1Start y1], 'Color','g','LineWidth',1);
    line([x2Start x2], [y2Start y2], 'Color','red','LineWidth',1);
    line([x3Start x3], [y3Start y3], 'Color','blue','LineWidth',1);%     plot(x1(2).actual, y1(2).actual, 'rx');
    line([x4Start x4], [y4Start y4], 'Color','cyan','LineWidth',1);
    line([x5Start x5], [y5Start y5], 'Color','magenta','LineWidth',1);
    
    for Ind=1:size(Prop(idx).prop(:),1) 
        Cent=Prop(idx).prop(Ind).Centroid;   
        X=Cent(1);Y=Cent(2);
        line([X-0.5 X+0.5], [Y Y],'LineWidth',1,'Color',[1 0 0]);
        line([X X], [Y-0.5 Y+0.5],'LineWidth',1,'Color',[1 0 0]);
        BBox = Prop(idx).prop(Ind).BoundingBox;
        rectangle('Position', BBox, 'EdgeColor',[0 1 0]);
    end

    pause(1)
end

%% Plot Center

idx=2;

for idx=2:length(ThreshImageClose(:))

    imshow(frames(idx).frame);hold on;
    if mod(idx,7)==0 
       hold off;
       start=idx;
       imshow(frames(idx).frame);hold on;
    end
    a=Prop(idx).prop(1).Centroid;
    x1(idx) = a(1);
    y1(idx) = a(2);

    b=Prop(idx).prop(2).Centroid;
    x2(idx) = b(1);
    y2(idx) = b(2);

    c=Prop(idx).prop(3).Centroid;
    x3(idx) = c(1);
    y3(idx) = c(2);   
    
    d=Prop(idx).prop(4).Centroid;
    x4(idx) = d(1);
    y4(idx) = d(2);  
    
    d=Prop(idx).prop(5).Centroid;
    x5(idx) = e(1);
    y5(idx) = e(2);  
   

    plot(x1(start:idx), y1(start:idx), 'rx');

    plot(x2(start:idx), y2(start:idx), 'rx');

    plot(x3(start:idx), y3(start:idx), 'rx');

    plot(x4(start:idx), y4(start:idx), 'rx');

    plot(x5(start:idx), y5(start:idx), 'rx');
   
    
    for Ind=1:size(Prop(idx).prop(:),1) 
        Cent=Prop(idx).prop(Ind).Centroid;   
        X=Cent(1);Y=Cent(2);
        line([X-0.5 X+0.5], [Y Y],'LineWidth',1,'Color',[1 0 0]);
        line([X X], [Y-0.5 Y+0.5],'LineWidth',1,'Color',[1 0 0]);
        BBox = Prop(idx).prop(Ind).BoundingBox;
        rectangle('Position', BBox, 'EdgeColor',[0 1 0]);
    end

    pause(0.2)
end
close all;


