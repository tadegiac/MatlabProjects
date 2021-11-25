%% Context
%HSLU T&A
%Degiacomi Riccardo
%EBV Testat 01
%FS 2021
%% Startup
clear all;
close all;
%% Add Path
addpath(".\Images\");
files=dir(".\Images\*.jpg");
%% Parameter (tweak performance)
Params = struct();
Params.AvgFactor = 0.95; %the average factor determines the speed of adaptation 
Params.Threshold = 10;   %this is the threshold value (chosen manually)
Params.Filtertype='Sobel'; %choose derivative
Params.Sigma=2; %std deviation
Params.k=0.04;  %scalar
Params.Border=1;%how much to cut off (picture corners)
Params.N_best=100;% nr. of edges to detect
%% Testbilder einlesen
NumOfFiles=length(files(:,1));
for idx = 1: NumOfFiles
    imageName=strcat(files(1).folder,'\',files(idx).name);
    Image=imread(imageName);
    Image = double(Image);
    %% Function Call
    BinImage=EdgeDetector(Image, Params);
    %% Mark Detected Edges
    figure(idx)
    subplot(1,2,1)
        imshow(uint8(Image));
        [Rows, Cols] = find(BinImage);
        for i1 = 1:length(Rows)
           BBox = [Cols(i1)-5 Rows(i1)-5 10 10];
           rectangle('Position', BBox, 'EdgeColor',[1 0 0], 'Curvature',[1,1]);     
        end
        title('Custom Edge Detector')
    %% Harris Corner Detector 
    subplot(1,2,2)
        I = Image;
        imshow(uint8(I))
        C = corner(I);    
        hold on
        plot(C(:,1),C(:,2),'r*');
        title('Harris Edge Detector')
end
