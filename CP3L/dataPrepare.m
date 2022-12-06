filename ='DataPrepare.xlsx';
tagNum = 7;
[num,txt,raw] = xlsread(filename,tagNum);
load('ground.mat');
[numTest,~] = size(raw);
[~,numImage] = size(data);

if tagNum == 1
    for j  = 1:numTest
        for i = 1:numImage
            if data(i).image == raw{j,1}
                groundTruth(j).image = data(i).image;
                tempPoint1 = data(i).point;
                finalPoint1(1,:) = tempPoint1(7,1:2);
                finalDepth1(1) = tempPoint1(7,3);
                finalPoint1(2,:) = tempPoint1(5,1:2);
                finalDepth1(2) = tempPoint1(5,3);
                finalPoint1(3,:) = tempPoint1(6,1:2);
                finalDepth1(3) = tempPoint1(6,3);
                finalPoint1(4,:) = tempPoint1(8,1:2);
                finalDepth1(4) = tempPoint1(8,3);
                finalPoint1(5,:) = tempPoint1(3,1:2);
                finalDepth1(5) = tempPoint1(3,3);
                finalPoint1(6,:) = tempPoint1(1,1:2);
                finalDepth1(6) = tempPoint1(1,3);
                finalPoint1(7,:) = tempPoint1(2,1:2);
                finalDepth1(7) = tempPoint1(2,3);
                finalPoint1(8,:) = tempPoint1(4,1:2);
                finalDepth1(8) = tempPoint1(4,3);
                groundTruth(j).point =finalPoint1;
                groundTruth(j).depth =finalDepth1;
                groundTruth(j).intrinsics_matrix = data(i).intrinsics_matrix;
                groundTruth(j).Lheight = raw{j,2};
                groundTruth(j).Lwidth = raw{j,3};
                intrinsics = data(i).intrinsics_matrix;
                M = str2num(raw{j,4});
                Rotation = [M(1:3);M(5:7);M(9:11)];
                Translation = [M(4);M(8);M(12)];
                groundTruth(j).Rotation =  Rotation;
                groundTruth(j).Translation =  Translation;
                for n = 1:length(finalDepth1) 
                    p = finalPoint1(n,:);
                    d = finalDepth1(n);
                    f1 = intrinsics(1,1);
                    f2 = intrinsics(2,2);
                    u0 = intrinsics(1,3);
                    v0 = intrinsics(2,3);
                    MT(1,1) = f1 * Rotation(1,1) + u0 * Rotation(3,1);
                    MT(1,2) = f1 * Rotation(1,2) + u0 * Rotation(3,2);
                    MT(1,3) = f1 * Rotation(1,3) + u0 * Rotation(3,3);
                    MT(1,4) = f1 * Translation(1) + u0 * Translation(3) - p(1)*d;
                    MT(2,1) = f2 * Rotation(2,1) + v0 * Rotation(3,1);
                    MT(2,2) = f2 * Rotation(2,2) + v0 * Rotation(3,2);
                    MT(2,3) = f2 * Rotation(2,3) + v0 * Rotation(3,3);
                    MT(2,4) = f2 * Translation(2) + v0 * Translation(3) - p(2)*d;
                    MT(3,1) = Rotation(3,1);
                    MT(3,2) = Rotation(3,2);
                    MT(3,3) = Rotation(3,3);
                    MT(3,4) = Translation(3)-d;
                    [UMatN, SMatN, VMatN] = svd(MT);
                    vecN = VMatN(:,4);
                    vecN = vecN/vecN(4);
                    P(n,:) = vecN(1:3);
                end
                groundTruth(j).wpoint = P;
                break;
            end
        end
    end
    save('type0GroundTruth1.mat','groundTruth');
end
if tagNum == 2
    for j  = 1:numTest
        for i = 1:numImage
            if data(i).image == raw{j,1}
                groundTruth(j).image = data(i).image;
                tempPoint2 = data(i).point;
                finalPoint2(1,:) = tempPoint2(5,1:2);
                finalDepth2(1) = tempPoint2(5,3);
                finalPoint2(2,:) = tempPoint2(6,1:2);
                finalDepth2(2) = tempPoint2(6,3);
                finalPoint2(3,:) = tempPoint2(1,1:2);
                finalDepth2(3) = tempPoint2(1,3);
                finalPoint2(4,:) = tempPoint2(3,1:2);
                finalDepth2(4) = tempPoint2(3,3);
                finalPoint2(5,:) = tempPoint2(2,1:2);
                finalDepth2(5) = tempPoint2(2,3);
                finalPoint2(6,:) = tempPoint2(4,1:2);
                finalDepth2(6) = tempPoint2(4,3);
                groundTruth(j).point =finalPoint2;
                groundTruth(j).depth =finalDepth2;
                groundTruth(j).intrinsics_matrix = data(i).intrinsics_matrix;
                intrinsics = data(i).intrinsics_matrix;
                M = str2num(raw{j,2});
                Rotation = [M(1:3);M(5:7);M(9:11)];
                Translation = [M(4);M(8);M(12)];
                groundTruth(j).Rotation =  Rotation;
                groundTruth(j).Translation =  Translation;
                for n = 1:length(finalDepth2) 
                    p = finalPoint2(n,:);
                    d = finalDepth2(n);
                    f1 = intrinsics(1,1);
                    f2 = intrinsics(2,2);
                    u0 = intrinsics(1,3);
                    v0 = intrinsics(2,3);
                    MT(1,1) = f1 * Rotation(1,1) + u0 * Rotation(3,1);
                    MT(1,2) = f1 * Rotation(1,2) + u0 * Rotation(3,2);
                    MT(1,3) = f1 * Rotation(1,3) + u0 * Rotation(3,3);
                    MT(1,4) = f1 * Translation(1) + u0 * Translation(3) - p(1)*d;
                    MT(2,1) = f2 * Rotation(2,1) + v0 * Rotation(3,1);
                    MT(2,2) = f2 * Rotation(2,2) + v0 * Rotation(3,2);
                    MT(2,3) = f2 * Rotation(2,3) + v0 * Rotation(3,3);
                    MT(2,4) = f2 * Translation(2) + v0 * Translation(3) - p(2)*d;
                    MT(3,1) = Rotation(3,1);
                    MT(3,2) = Rotation(3,2);
                    MT(3,3) = Rotation(3,3);
                    MT(3,4) = Translation(3)-d;
                    [UMatN, SMatN, VMatN] = svd(MT);
                    vecN = VMatN(:,4);
                    vecN = vecN/vecN(4);
                    P(n,:) = vecN(1:3);
                end
                groundTruth(j).wpoint = P;
                break;
            end
        end
    end
    save('type1GroundTruth1.mat','groundTruth');
end
if tagNum == 3
    for j  = 1:numTest
        for i = 1:numImage
            if data(i).image == raw{j,1}
                groundTruth(j).image = data(i).image;
                tempPoint3 = data(i).point;
                finalPoint3(1,:) = tempPoint3(5,1:2);
                finalDepth3(1) = tempPoint3(5,3);
                finalPoint3(2,:) = tempPoint3(6,1:2);
                finalDepth3(2) = tempPoint3(6,3);
                finalPoint3(3,:) = tempPoint3(3,1:2);
                finalDepth3(3) = tempPoint3(3,3);
                finalPoint3(4,:) = tempPoint3(1,1:2);
                finalDepth3(4) = tempPoint3(1,3);
                finalPoint3(5,:) = tempPoint3(4,1:2);
                finalDepth3(5) = tempPoint3(4,3);
                finalPoint3(6,:) = tempPoint3(2,1:2);
                finalDepth3(6) = tempPoint3(2,3);
                groundTruth(j).point =finalPoint3;
                groundTruth(j).depth =finalDepth3;
                groundTruth(j).intrinsics_matrix = data(i).intrinsics_matrix;
                intrinsics = data(i).intrinsics_matrix;
                M = str2num(raw{j,2});
                Rotation = [M(1:3);M(5:7);M(9:11)];
                Translation = [M(4);M(8);M(12)];
                groundTruth(j).Rotation =  Rotation;
                groundTruth(j).Translation =  Translation;
                for n = 1:length(finalDepth3) 
                    p = finalPoint3(n,:);
                    d = finalDepth3(n);
                    f1 = intrinsics(1,1);
                    f2 = intrinsics(2,2);
                    u0 = intrinsics(1,3);
                    v0 = intrinsics(2,3);
                    MT(1,1) = f1 * Rotation(1,1) + u0 * Rotation(3,1);
                    MT(1,2) = f1 * Rotation(1,2) + u0 * Rotation(3,2);
                    MT(1,3) = f1 * Rotation(1,3) + u0 * Rotation(3,3);
                    MT(1,4) = f1 * Translation(1) + u0 * Translation(3) - p(1)*d;
                    MT(2,1) = f2 * Rotation(2,1) + v0 * Rotation(3,1);
                    MT(2,2) = f2 * Rotation(2,2) + v0 * Rotation(3,2);
                    MT(2,3) = f2 * Rotation(2,3) + v0 * Rotation(3,3);
                    MT(2,4) = f2 * Translation(2) + v0 * Translation(3) - p(2)*d;
                    MT(3,1) = Rotation(3,1);
                    MT(3,2) = Rotation(3,2);
                    MT(3,3) = Rotation(3,3);
                    MT(3,4) = Translation(3)-d;
                    [UMatN, SMatN, VMatN] = svd(MT);
                    vecN = VMatN(:,4);
                    vecN = vecN/vecN(4);
                    P(n,:) = vecN(1:3);
                end
                groundTruth(j).wpoint = P;
                break;
            end
        end
    end
    save('type2GroundTruth1.mat','groundTruth');
end
if tagNum == 4
    for j  = 1:numTest
        for i = 1:numImage
            if data(i).image == raw{j,1}
                groundTruth(j).image = data(i).image;
                tempPoint4 = data(i).point;
                finalPoint4(1,:) = tempPoint4(4,1:2);
                finalDepth4(1) = tempPoint4(4,3);
                finalPoint4(2,:) = tempPoint4(2,1:2);
                finalDepth4(2) = tempPoint4(2,3);
                finalPoint4(3,:) = tempPoint4(3,1:2);
                finalDepth4(3) = tempPoint4(3,3);
                finalPoint4(4,:) = tempPoint4(1,1:2);
                finalDepth4(4) = tempPoint4(1,3);
                groundTruth(j).point =finalPoint4;
                groundTruth(j).depth =finalDepth4;
                groundTruth(j).intrinsics_matrix = data(i).intrinsics_matrix;
                intrinsics = data(i).intrinsics_matrix;
                M = str2num(raw{j,2});
                Rotation = [M(1:3);M(5:7);M(9:11)];
                Translation = [M(4);M(8);M(12)];
                groundTruth(j).Rotation =  Rotation;
                groundTruth(j).Translation =  Translation;
                for n = 1:length(finalDepth4) 
                    p = finalPoint4(n,:);
                    d = finalDepth4(n);
                    f1 = intrinsics(1,1);
                    f2 = intrinsics(2,2);
                    u0 = intrinsics(1,3);
                    v0 = intrinsics(2,3);
                    MT(1,1) = f1 * Rotation(1,1) + u0 * Rotation(3,1);
                    MT(1,2) = f1 * Rotation(1,2) + u0 * Rotation(3,2);
                    MT(1,3) = f1 * Rotation(1,3) + u0 * Rotation(3,3);
                    MT(1,4) = f1 * Translation(1) + u0 * Translation(3) - p(1)*d;
                    MT(2,1) = f2 * Rotation(2,1) + v0 * Rotation(3,1);
                    MT(2,2) = f2 * Rotation(2,2) + v0 * Rotation(3,2);
                    MT(2,3) = f2 * Rotation(2,3) + v0 * Rotation(3,3);
                    MT(2,4) = f2 * Translation(2) + v0 * Translation(3) - p(2)*d;
                    MT(3,1) = Rotation(3,1);
                    MT(3,2) = Rotation(3,2);
                    MT(3,3) = Rotation(3,3);
                    MT(3,4) = Translation(3)-d;
                    [UMatN, SMatN, VMatN] = svd(MT);
                    vecN = VMatN(:,4);
                    vecN = vecN/vecN(4);
                    P(n,:) = vecN(1:3);
                end
                groundTruth(j).wpoint = P;
                break;
            end
        end
    end
    save('type3GroundTruth1.mat','groundTruth');
end
if tagNum == 5
    for j  = 1:numTest
        for i = 1:numImage
            if data(i).image == raw{j,1}
                groundTruth(j).image = data(i).image;
                tempPoint5 = data(i).point;
                finalPoint5(1,:) = tempPoint5(4,1:2);
                finalDepth5(1) = tempPoint5(4,3);
                finalPoint5(2,:) = tempPoint5(2,1:2);
                finalDepth5(2) = tempPoint5(2,3);
                finalPoint5(3,:) = tempPoint5(3,1:2);
                finalDepth5(3) = tempPoint5(3,3);
                finalPoint5(4,:) = tempPoint5(1,1:2);
                finalDepth5(4) = tempPoint5(1,3);
                groundTruth(j).point =finalPoint5;
                groundTruth(j).depth =finalDepth5;
                groundTruth(j).intrinsics_matrix = data(i).intrinsics_matrix;
                intrinsics = data(i).intrinsics_matrix;
                M = str2num(raw{j,2});
                Rotation = [M(1:3);M(5:7);M(9:11)];
                Translation = [M(4);M(8);M(12)];
                groundTruth(j).Rotation =  Rotation;
                groundTruth(j).Translation =  Translation;
                for n = 1:length(finalDepth5) 
                    p = finalPoint5(n,:);
                    d = finalDepth5(n);
                    f1 = intrinsics(1,1);
                    f2 = intrinsics(2,2);
                    u0 = intrinsics(1,3);
                    v0 = intrinsics(2,3);
                    MT(1,1) = f1 * Rotation(1,1) + u0 * Rotation(3,1);
                    MT(1,2) = f1 * Rotation(1,2) + u0 * Rotation(3,2);
                    MT(1,3) = f1 * Rotation(1,3) + u0 * Rotation(3,3);
                    MT(1,4) = f1 * Translation(1) + u0 * Translation(3) - p(1)*d;
                    MT(2,1) = f2 * Rotation(2,1) + v0 * Rotation(3,1);
                    MT(2,2) = f2 * Rotation(2,2) + v0 * Rotation(3,2);
                    MT(2,3) = f2 * Rotation(2,3) + v0 * Rotation(3,3);
                    MT(2,4) = f2 * Translation(2) + v0 * Translation(3) - p(2)*d;
                    MT(3,1) = Rotation(3,1);
                    MT(3,2) = Rotation(3,2);
                    MT(3,3) = Rotation(3,3);
                    MT(3,4) = Translation(3)-d;
                    [UMatN, SMatN, VMatN] = svd(MT);
                    vecN = VMatN(:,4);
                    vecN = vecN/vecN(4);
                    P(n,:) = vecN(1:3);
                end
                groundTruth(j).wpoint = P;
                break;
            end
        end
    end
    save('type4GroundTruth1.mat','groundTruth');
end
if tagNum == 6
    for j  = 1:numTest
        for i = 1:numImage
            if data(i).image == raw{j,1}
                groundTruth(j).image = data(i).image;
                tempPoint6 = data(i).point;
                finalPoint6(1,:) = tempPoint6(6,1:2);
                finalDepth6(1) = tempPoint6(6,3);
                finalPoint6(2,:) = tempPoint6(5,1:2);
                finalDepth6(2) = tempPoint6(5,3);
                finalPoint6(3,:) = tempPoint6(3,1:2);
                finalDepth6(3) = tempPoint6(3,3);
                finalPoint6(4,:) = tempPoint6(4,1:2);
                finalDepth6(4) = tempPoint6(4,3);
                finalPoint6(5,:) = tempPoint6(1,1:2);
                finalDepth6(5) = tempPoint6(1,3);
                finalPoint6(6,:) = tempPoint6(2,1:2);
                finalDepth6(6) = tempPoint6(2,3);
                groundTruth(j).point =finalPoint6;
                groundTruth(j).depth =finalDepth6;
                groundTruth(j).intrinsics_matrix = data(i).intrinsics_matrix;
                intrinsics = data(i).intrinsics_matrix;
                M = str2num(raw{j,2});
                Rotation = [M(1:3);M(5:7);M(9:11)];
                Translation = [M(4);M(8);M(12)];
                groundTruth(j).Rotation =  Rotation;
                groundTruth(j).Translation =  Translation;
                for n = 1:length(finalDepth6) 
                    p = finalPoint6(n,:);
                    d = finalDepth6(n);
                    f1 = intrinsics(1,1);
                    f2 = intrinsics(2,2);
                    u0 = intrinsics(1,3);
                    v0 = intrinsics(2,3);
                    MT(1,1) = f1 * Rotation(1,1) + u0 * Rotation(3,1);
                    MT(1,2) = f1 * Rotation(1,2) + u0 * Rotation(3,2);
                    MT(1,3) = f1 * Rotation(1,3) + u0 * Rotation(3,3);
                    MT(1,4) = f1 * Translation(1) + u0 * Translation(3) - p(1)*d;
                    MT(2,1) = f2 * Rotation(2,1) + v0 * Rotation(3,1);
                    MT(2,2) = f2 * Rotation(2,2) + v0 * Rotation(3,2);
                    MT(2,3) = f2 * Rotation(2,3) + v0 * Rotation(3,3);
                    MT(2,4) = f2 * Translation(2) + v0 * Translation(3) - p(2)*d;
                    MT(3,1) = Rotation(3,1);
                    MT(3,2) = Rotation(3,2);
                    MT(3,3) = Rotation(3,3);
                    MT(3,4) = Translation(3)-d;
                    [UMatN, SMatN, VMatN] = svd(MT);
                    vecN = VMatN(:,4);
                    vecN = vecN/vecN(4);
                    P(n,:) = vecN(1:3);
                end
                groundTruth(j).wpoint = P;
                break;
            end
        end
    end
    save('type5GroundTruth1.mat','groundTruth');
end
if tagNum == 7
    for j  = 1:numTest
        for i = 1:numImage
            if data(i).image == raw{j,1}
                groundTruth(j).image = data(i).image;
                tempPoint7 = data(i).point;
                finalPoint7(1,:) = tempPoint7(3,1:2);
                finalDepth7(1) = tempPoint7(3,3);
                finalPoint7(2,:) = tempPoint7(4,1:2);
                finalDepth7(2) = tempPoint7(4,3);
                finalPoint7(3,:) = tempPoint7(1,1:2);
                finalDepth7(3) = tempPoint7(1,3);
                finalPoint7(4,:) = tempPoint7(2,1:2);
                finalDepth7(4) = tempPoint7(2,3);
                groundTruth(j).point =finalPoint7;
                groundTruth(j).depth =finalDepth7;
                groundTruth(j).intrinsics_matrix = data(i).intrinsics_matrix;
                groundTruth(j).CameraHeight =  raw{j,2};
                groundTruth(j).Lheight =  raw{j,3};
                groundTruth(j).Lwidth =  raw{j,4};
                intrinsics = data(i).intrinsics_matrix;
                M = str2num(raw{j,5});
                Rotation = [M(1:3);M(5:7);M(9:11)];
                Translation = [M(4);M(8);M(12)];
                groundTruth(j).Rotation =  Rotation;
                groundTruth(j).Translation =  Translation;
                for n = 1:length(finalDepth7) 
                    p = finalPoint7(n,:);
                    d = finalDepth7(n);
                    f1 = intrinsics(1,1);
                    f2 = intrinsics(2,2);
                    u0 = intrinsics(1,3);
                    v0 = intrinsics(2,3);
                    MT(1,1) = f1 * Rotation(1,1) + u0 * Rotation(3,1);
                    MT(1,2) = f1 * Rotation(1,2) + u0 * Rotation(3,2);
                    MT(1,3) = f1 * Rotation(1,3) + u0 * Rotation(3,3);
                    MT(1,4) = f1 * Translation(1) + u0 * Translation(3) - p(1)*d;
                    MT(2,1) = f2 * Rotation(2,1) + v0 * Rotation(3,1);
                    MT(2,2) = f2 * Rotation(2,2) + v0 * Rotation(3,2);
                    MT(2,3) = f2 * Rotation(2,3) + v0 * Rotation(3,3);
                    MT(2,4) = f2 * Translation(2) + v0 * Translation(3) - p(2)*d;
                    MT(3,1) = Rotation(3,1);
                    MT(3,2) = Rotation(3,2);
                    MT(3,3) = Rotation(3,3);
                    MT(3,4) = Translation(3)-d;
                    [UMatN, SMatN, VMatN] = svd(MT);
                    vecN = VMatN(:,4);
                    vecN = vecN/vecN(4);
                    P(n,:) = vecN(1:3);
                end
                groundTruth(j).wpoint = P;
                break;
            end
        end
    end
    save('type6GroundTruth.mat','groundTruth');
end
if tagNum == 8
    for j  = 1:numTest
        for i = 1:numImage
            if data(i).image == raw{j,1}
                groundTruth(j).image = data(i).image;
                tempPoint8 = data(i).point;
                finalPoint8(1,:) = tempPoint8(2,1:2);
                finalDepth8(1) = tempPoint8(2,3);
                finalPoint8(2,:) = tempPoint8(1,1:2);
                finalDepth8(2) = tempPoint8(1,3);
                finalPoint8(3,:) = tempPoint8(3,1:2);
                finalDepth8(3) = tempPoint8(3,3);
                finalPoint8(4,:) = tempPoint8(4,1:2);
                finalDepth8(4) = tempPoint8(4,3);
                groundTruth(j).point =finalPoint8;
                groundTruth(j).depth =finalDepth8;
                groundTruth(j).intrinsics_matrix = data(i).intrinsics_matrix;
                groundTruth(j).Lheight =  raw{j,2};
                groundTruth(j).Lwidth =  raw{j,3};
                groundTruth(j).CameraHeight =  raw{j,4};
                intrinsics = data(i).intrinsics_matrix;
                M = str2num(raw{j,5});
                Rotation = [M(1:3);M(5:7);M(9:11)];
                Translation = [M(4);M(8);M(12)];
                groundTruth(j).Rotation =  Rotation;
                groundTruth(j).Translation =  Translation;
                for n = 1:length(finalDepth8) 
                    p = finalPoint8(n,:);
                    d = finalDepth8(n);
                    f1 = intrinsics(1,1);
                    f2 = intrinsics(2,2);
                    u0 = intrinsics(1,3);
                    v0 = intrinsics(2,3);
                    MT(1,1) = f1 * Rotation(1,1) + u0 * Rotation(3,1);
                    MT(1,2) = f1 * Rotation(1,2) + u0 * Rotation(3,2);
                    MT(1,3) = f1 * Rotation(1,3) + u0 * Rotation(3,3);
                    MT(1,4) = f1 * Translation(1) + u0 * Translation(3) - p(1)*d;
                    MT(2,1) = f2 * Rotation(2,1) + v0 * Rotation(3,1);
                    MT(2,2) = f2 * Rotation(2,2) + v0 * Rotation(3,2);
                    MT(2,3) = f2 * Rotation(2,3) + v0 * Rotation(3,3);
                    MT(2,4) = f2 * Translation(2) + v0 * Translation(3) - p(2)*d;
                    MT(3,1) = Rotation(3,1);
                    MT(3,2) = Rotation(3,2);
                    MT(3,3) = Rotation(3,3);
                    MT(3,4) = Translation(3)-d;
                    [UMatN, SMatN, VMatN] = svd(MT);
                    vecN = VMatN(:,4);
                    vecN = vecN/vecN(4);
                    P(n,:) = vecN(1:3);
                end
                groundTruth(j).wpoint = P;
                break;
            end
        end
    end
    save('type7GroundTruth.mat','groundTruth');
end
    