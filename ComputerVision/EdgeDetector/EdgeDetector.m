function BinImage = EdgeDetector(Image,Params)
%% 2.) Partielle Ableitung für jedes Pixel bestimmen
%filtertype
    if Params.Filtertype=='Sobel'
        DX = [-1 0 1; -2 0 2; -1 0 1];
        DY = DX';
    else
        DX = [-1 0 1; -1 0 1; -1 0 1];
        DY = DX';	
    end
    %apply filter -> Do derivation per pixel
    ImageDx = imfilter(Image, DX);
    ImageDy = imfilter(Image, DY);

%% 3.)Ableitungen Quadrieren, Gradientberechnen
    %Ix^2
    Ix_2=ImageDx.^2;
    %Iy^2
    Iy_2=ImageDy.^2;
    %Ix*IY
    Ixy=ImageDx.*ImageDy;
%% 4.) Ix^2, Iy^2, Ix*IY Mittel mit Gauss Filter
    %Ix^2
    Ix_2_G=imgaussfilt(Ix_2,Params.Sigma);
    %Iy^2
    Iy_2_G=imgaussfilt(Iy_2,Params.Sigma);
    %Ix*IY
    Ixy_G=imgaussfilt(Ixy,Params.Sigma);
%% 5.) Mc berechnen 
    Mc=(((Ix_2_G.*Iy_2_G)-(Ixy_G.^2))-Params.k*(Ix_2_G+Iy_2_G).^2);
%% 6.) Pixel am Rand auf Null setzen
    %get image size
    X=length(Image(1,1:end));
    Y=length(Image(1:end,1));
    
    Mc((Y-Params.Border)+1:Y,:)=0;
    Mc(1:Params.Border,:)=0;
    Mc(:,1:Params.Border)=0;
    Mc(:,(X-Params.Border)+1:X)=0;
%% 7.) Lokale Maxima bestimmen
    SizeRegion = 2*Params.Border+1;
    McNorm = uint8(255*Mc/max(Mc(:)));
    MaxVal = uint8(ordfilt2(McNorm, SizeRegion^2, ones(SizeRegion)));    
%% 8.) Grösste Werte Auslesen    
       OneDArray = reshape(MaxVal.',1,[]);
       OneDArray = unique(OneDArray);
       OneDArray =maxk(OneDArray,Params.N_best);
       borderValue=OneDArray(end);
       LocMax = (MaxVal >= borderValue);
       
       BinImage=LocMax;
    end