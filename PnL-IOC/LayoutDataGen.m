function Data = LayoutDataGen(type, noiseStd, rateOutlier, pgNum)
        
%     type = 4;
%     layoutH = 256;
%     layoutW = 256;
%     layoutL = 256;
    width = 640;
    height = 640;
    focal = 180;
    
%     noiseStd = noiseStd;

    
    
%     pgNum = 10;
    Num = pgNum;
    
    for i =  1:Num    
        if type == 0
            A = [[focal   0 width/2]
                 [  0 focal height/2]
                  [  0   0   1]];

            
            while true
                cam_x = 128;
                cam_y = 128;
                cam_z = 128;
                ang_x = 0;
                ang_y = 0;
                ang_z = 0;
                randpx = 10*rand(1)-5;
                randpy = 10*rand(1)-5;
                randpz = 10*rand(1)-5;
                randax = 6*rand(1)-3;
                randay = 6*rand(1)-3;
                randaz = 6*rand(1)-3;
                p = [randpx, randpy, randpz];
                ang_x = ang_x + randax;
                ang_y = ang_y + randay;
                ang_z = ang_z + randaz;

                r = rotate(p, ang_x, ang_y, ang_z);
                cam_x = cam_x + r(1);
                cam_y = cam_y + r(2);
                cam_z = cam_z + r(3);


                [pointS,pointE,pointSt,pointEt] = getPoints(type,cam_x, cam_y, cam_z, ang_x, ang_y, ang_z, A,noiseStd);
                
                
                ps = pointS(:,4:5);
                pe = pointE(:,4:5);
                if (isempty(find(ps<=0))) && (isempty(find(pe<=0)))
                    break;
                end
                
            end
            
         
             o = [cam_x, cam_y, cam_z];
             a = [ang_x, ang_y, ang_z];
            
            [R, T] = get_extrinsic(o,a);
            
            Cc = -inv(R)*T;
            
            Data{i}.pointS = pointS;
            Data{i}.pointE = pointE;
            Data{i}.pointSt = pointSt;
            Data{i}.pointEt = pointEt;
            Data{i}.R = R;
            Data{i}.T = T;
            Data{i}.Cc = Cc;
            
            
            a = atan2(R(2,1),R(1,1));
            b = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
            c = atan2(R(3,2),R(3,3));
            Data{i}.S = [a,b,c];
             
            
             
        end 
        if type == 1
            A = [[focal   0 width/2]
                 [  0 focal height/2]
                  [  0   0   1]];
            
            while true
                cam_x = 120;
                cam_y = 100;
                cam_z = 172;
                ang_x = -24;
                ang_y = 0;
                ang_z = 0;
                randpx = 10*rand(1)-5;
                randpy = 10*rand(1)-5;
                randpz = 10*rand(1)-5;
                randax = 6*rand(1)-3;
                randay = 6*rand(1)-3;
                randaz = 6*rand(1)-3;
                
                
                p = [randpx, randpy, randpz];
                ang_x = ang_x + randax;
                ang_y = ang_y + randay;
                ang_z = ang_z + randaz;

                r = rotate(p, ang_x, ang_y, ang_z);
                cam_x = cam_x + r(1);
                cam_y = cam_y + r(2);
                cam_z = cam_z + r(3);


                [pointS,pointE,pointSt,pointEt] = getPoints(type,cam_x, cam_y, cam_z, ang_x, ang_y, ang_z, A,noiseStd);
                
                
                ps = pointS(:,4:5);
                pe = pointE(:,4:5);
                if (isempty(find(ps<=0))) && (isempty(find(pe<=0)))
                    break;
                end
                
            end
            
         
             o = [cam_x, cam_y, cam_z];
             a = [ang_x, ang_y, ang_z];
            
            [R, T] = get_extrinsic(o,a);
            
            Cc = -inv(R)*T;
            
            Data{i}.pointS = pointS;
            Data{i}.pointE = pointE;
            Data{i}.pointSt = pointSt;
            Data{i}.pointEt = pointEt;
            Data{i}.R = R;
            Data{i}.T = T;
            Data{i}.Cc = Cc;
            
            a = atan2(R(2,1),R(1,1));
            b = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
            c = atan2(R(3,2),R(3,3));
            Data{i}.S = [a,b,c];
             
        end
        if type == 2
            A = [[focal   0 width/2]
                 [  0 focal height/2]
                  [  0   0   1]];
            
            while true
                cam_x = 120;
                cam_y = 165;
                cam_z = 99;
                ang_x = 18;
                ang_y = 0;
                ang_z = 0;
                randpx = 10*rand(1)-5;
                randpy = 10*rand(1)-5;
                randpz = 10*rand(1)-5;
                randax = 6*rand(1)-3;
                randay = 6*rand(1)-3;
                randaz = 6*rand(1)-3;
                
                
                p = [randpx, randpy, randpz];
                ang_x = ang_x + randax;
                ang_y = ang_y + randay;
                ang_z = ang_z + randaz;

                r = rotate(p, ang_x, ang_y, ang_z);
                cam_x = cam_x + r(1);
                cam_y = cam_y + r(2);
                cam_z = cam_z + r(3);


                [pointS,pointE,pointSt,pointEt] = getPoints(type,cam_x, cam_y, cam_z, ang_x, ang_y, ang_z, A,noiseStd);
                
                
                ps = pointS(:,4:5);
                pe = pointE(:,4:5);
                if (isempty(find(ps<=0))) && (isempty(find(pe<=0)))
                    break;
                end
                
            end
            
         
             o = [cam_x, cam_y, cam_z];
             a = [ang_x, ang_y, ang_z];
            
            [R, T] = get_extrinsic(o,a);
            
            Cc = -inv(R)*T;
            
            Data{i}.pointS = pointS;
            Data{i}.pointE = pointE;
            Data{i}.pointSt = pointSt;
            Data{i}.pointEt = pointEt;
            Data{i}.R = R;
            Data{i}.T = T;
            Data{i}.Cc = Cc;
            
            a = atan2(R(2,1),R(1,1));
            b = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
            c = atan2(R(3,2),R(3,3));
            Data{i}.S = [a,b,c];
             
        end 
        if type == 3
            A = [[focal   0 width/2]
                 [  0 focal height/2]
                  [  0   0   1]];
   
            while true
                cam_x = 68;
                cam_y = 162;
                cam_z = 82;
                ang_x = 45;
                ang_y = 45;
                ang_z = 0;
                
                randpx = 10*rand(1)-5;
                randpy = 10*rand(1)-5;
                randpz = 10*rand(1)-5;
                randax = 6*rand(1)-3;
                randay = 6*rand(1)-3;
                randaz = 6*rand(1)-3;
                

                p = [randpx, randpy, randpz];

                ang_x = ang_x + randax;
                ang_y = ang_y + randay;
                ang_z = ang_z + randaz;

                r = rotate(p, ang_x, ang_y, ang_z);
                cam_x = cam_x + r(1);
                cam_y = cam_y + r(2);
                cam_z = cam_z + r(3);


                [pointS,pointE,pointSt,pointEt] = getPoints(type,cam_x, cam_y, cam_z, ang_x, ang_y, ang_z, A,noiseStd);
                
                 
                 ps = pointS(:,4:5);
                 pe = pointE(:,4:5);
                 if (isempty(find(ps<=0))) && (isempty(find(pe<=0)))
                     break;
                 end
                
            end
            
         
             o = [cam_x, cam_y, cam_z];
             a = [ang_x, ang_y, ang_z];
            
            [R, T] = get_extrinsic(o,a);
            
            Cc = -inv(R)*T;
            
            
            Data{i}.pointS = pointS;
            Data{i}.pointE = pointE;
            Data{i}.pointSt = pointSt;
            Data{i}.pointEt = pointEt;
            Data{i}.R = R;
            Data{i}.T = T;
            Data{i}.Cc = Cc;
            
            a = atan2(R(2,1),R(1,1));
            b = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
            c = atan2(R(3,2),R(3,3));
            Data{i}.S = [a,b,c];
        end
        if type == 4
            A = [[focal   0 width/2]
                 [  0 focal height/2]
                  [  0   0   1]];

            
            while true
                cam_x = 92;
                cam_y = 104;
                cam_z = 104;
                ang_x = -45;
                ang_y = 45;
                ang_z = 0;
                randpx = 10*rand(1)-5;
                randpy = 10*rand(1)-5;
                randpz = 10*rand(1)-5;
                randax = 6*rand(1)-3;
                randay = 6*rand(1)-3;
                randaz = 6*rand(1)-3;

                p = [randpx, randpy, randpz];

                ang_x = ang_x + randax;
                ang_y = ang_y + randay;
                ang_z = ang_z + randaz;

                r = rotate(p, ang_x, ang_y, ang_z);
                cam_x = cam_x + r(1);
                cam_y = cam_y + r(2);
                cam_z = cam_z + r(3);


                [pointS,pointE,pointSt,pointEt] = getPoints(type,cam_x, cam_y, cam_z, ang_x, ang_y, ang_z, A,noiseStd);
                
                 ps = pointS(:,4:5);
                 pe = pointE(:,4:5);
                 if (isempty(find(ps<=0))) && (isempty(find(pe<=0)))
                     break;
                 end
                
            end
            
         
             o = [cam_x, cam_y, cam_z];
             a = [ang_x, ang_y, ang_z];
            
            [R, T] = get_extrinsic(o,a);
            
            
            Data{i}.pointS = pointS;
            Data{i}.pointE = pointE;
            Data{i}.pointSt = pointSt;
            Data{i}.pointEt = pointEt;
            Data{i}.R = R;
            Data{i}.T = T;
            
            Cc = -inv(R)*T;
            Data{i}.Cc = Cc;
            
            S = Cayley(R);
            Data{i}.S = S;
        end
        if type == 5
            A = [[focal   0 width/2]
                 [  0 focal height/2]
                  [  0   0   1]];
            
            while true
                cam_x = 108;
                cam_y = 141;
                cam_z = 142;
                ang_x = 0;
                ang_y = 42;
                ang_z = 0;
                randpx = 10*rand(1)-5;
                randpy = 10*rand(1)-5;
                randpz = 10*rand(1)-5;
                randax = 6*rand(1)-3;
                randay = 6*rand(1)-3;
                randaz = 6*rand(1)-3;
                
                
                p = [randpx, randpy, randpz];
                ang_x = ang_x + randax;
                ang_y = ang_y + randay;
                ang_z = ang_z + randaz;

                r = rotate(p, ang_x, ang_y, ang_z);
                cam_x = cam_x + r(1);
                cam_y = cam_y + r(2);
                cam_z = cam_z + r(3);


                [pointS,pointE,pointSt,pointEt] = getPoints(type,cam_x, cam_y, cam_z, ang_x, ang_y, ang_z, A,noiseStd);
                
                
                ps = pointS(:,4:5);
                pe = pointE(:,4:5);
                if (isempty(find(ps<=0))) && (isempty(find(pe<=0)))
                    break;
                end
                
            end
            
         
            o = [cam_x, cam_y, cam_z];
            a = [ang_x, ang_y, ang_z];
            
            [R, T] = get_extrinsic(o,a);
            
            Cc = -inv(R)*T;
            
            Data{i}.pointS = pointS;
            Data{i}.pointE = pointE;
            Data{i}.pointSt = pointSt;
            Data{i}.pointEt = pointEt;
            Data{i}.R = R;
            Data{i}.T = T;
            Data{i}.Cc = Cc;
            
            S = Cayley(R);
            Data{i}.S = S;
             
        end 
        if type == 6
            width = 320;
            focal = 200;
            A = [[focal   0 width/2]
                 [  0 focal height/2]
                  [  0   0   1]];
            while true
                cam_x = 30;
                cam_y = 40;
                cam_z = 116;
                ang_x = 0;
                ang_y = 0;
                ang_z = 0;
                randpx = 8*rand(1)-4;
                randpy = 8*rand(1)-4;
                randpz = 8*rand(1)-4;
                randax = 6*rand(1)-3;
                randay = 6*rand(1)-3;
                randaz = 6*rand(1)-3;
                
                p = [randpx, randpy, randpz];
                ang_x = ang_x + randax;
                ang_y = ang_y + randay;
                ang_z = ang_z + randaz;

                r = rotate(p, ang_x, ang_y, ang_z);
                cam_x = cam_x + r(1);
                cam_y = cam_y + r(2);
                cam_z = cam_z + r(3);


                [pointS,pointE,pointSt,pointEt] = getPoints(type,cam_x, cam_y, cam_z, ang_x, ang_y, ang_z, A,noiseStd);
                
                
                ps = pointS(:,4:5);
                pe = pointE(:,4:5);
                if (isempty(find(ps<=0))) && (isempty(find(pe<=0)))
                    break;
                end
                
            end
                    
            o = [cam_x, cam_y, cam_z];
            a = [ang_x, ang_y, ang_z];
            
            [R, T] = get_extrinsic(o,a);
            
            Cc = -inv(R)*T;
            
            Data{i}.pointS = pointS;
            Data{i}.pointE = pointE;
            Data{i}.pointSt = pointSt;
            Data{i}.pointEt = pointEt;
            Data{i}.R = R;
            Data{i}.T = T;
            Data{i}.Cc = Cc;
             
        end 
        if type == 7
            height = 320;
            focal = 100;
            A = [[focal   0 width/2]
                 [  0 focal height/2]
                  [  0   0   1]];
            
            while true
                cam_x = 220;
                cam_y = 208;
                cam_z = 66;
                ang_x = 0;
                ang_y = 0;
                ang_z = 0;
                randpx = 8*rand(1)-4;
                randpy = 8*rand(1)-4;
                randpz = 8*rand(1)-4;
                randax = 6*rand(1)-3;
                randay = 6*rand(1)-3;
                randaz = 6*rand(1)-3;
                
                p = [randpx, randpy, randpz];
                ang_x = ang_x + randax;
                ang_y = ang_y + randay;
                ang_z = ang_z + randaz;

                r = rotate(p, ang_x, ang_y, ang_z);
                cam_x = cam_x + r(1);
                cam_y = cam_y + r(2);
                cam_z = cam_z + r(3);


                [pointS,pointE,pointSt,pointEt] = getPoints(type,cam_x, cam_y, cam_z, ang_x, ang_y, ang_z, A,noiseStd);
                
                
                ps = pointS(:,4:5);
                pe = pointE(:,4:5);
                if (isempty(find(ps<=0))) && (isempty(find(pe<=0)))
                    break;
                end
                
            end
                    
            o = [cam_x, cam_y, cam_z];
            a = [ang_x, ang_y, ang_z];
            
            [R, T] = get_extrinsic(o,a);
            
            Cc = -inv(R)*T;
            
            Data{i}.pointS = pointS;
            Data{i}.pointE = pointE;
            Data{i}.pointSt = pointSt;
            Data{i}.pointEt = pointEt;
            Data{i}.R = R;
            Data{i}.T = T;
            Data{i}.Cc = Cc;
            
        end 
    end
end

    
function r = rotate(p, a_x, a_y, a_z)

    sin_x = sin(a_x * (pi / 180));
    cos_x = cos(a_x * (pi / 180));

    sin_y = sin(a_y * (pi / 180));
    cos_y = cos(a_y * (pi / 180));

    sin_z = sin(a_z * (pi / 180));
    cos_z = cos(a_z * (pi / 180));

    rx = [
        [1.0, 0.0, 0.0]
        [0.0, cos_x, -sin_x]
        [0.0, sin_x, cos_x]
    ];

    ry =[
        [cos_y, 0.0, sin_y]
        [0.0, 1.0, 0.0]
        [-sin_y, 0.0, cos_y]
    ];

    rz = [
        [cos_z, -sin_z, 0.0]
        [sin_z, cos_z, 0.0]
        [0.0, 0.0, 1.0]
    ];
  

    r = rx*ry*rz*p';

end
         

function [pt,PT] = randPts(n,w,h,f,noiseStd,Uc)


	if Uc==0 % uncentred
		u = rand(1,n) * w * 0.25;
		v = rand(1,n) * h * 0.25;
    elseif Uc==1 % centred
		u = rand(1,n) * w;
		v = rand(1,n) * h;
	end


	% depth range [4,8]
	depth = 4*rand(1,n)+4;
	
	% normalize
	u = (u - (w*0.5)) / f;
	v = (v - (h*0.5)) / f;

	% 2d
	pt = [u; v];

	% 3d
	PT = [u; v; ones(1,n)] .* kron(ones(3,1),depth);

	%add some noise to the endpoints
	if noiseStd > 0
		pt = pt + noiseStd*(rand(2,n)-0.5);
    end
end
		
function [i1,i2] = randOutlier(n,rateOutlier)

	m = round(n * rateOutlier);
	i0 = randperm(n);
	i1 = i0(1:m);
	i2 = i1(randperm(m));
end


function p2d = project(point, observer, angles, focal_length, center)

    p_x = point(1);
    p_y = point(2);
    p_z = point(3);
    o_x = observer(1);
    o_y = observer(2);
    o_z = observer(3);
    a_x = angles(1);
    a_y = angles(2);
    a_z = angles(3);


    x_center = center(1);
    y_center = center(2);

    x = p_x - o_x;
    y = p_y - o_y;
    z = o_z - p_z;

    p = [x, y, z];
    r = rotate(p, a_x, a_y, a_z);

    x = r(1);
    y = r(2);
    z = r(3);

    of = focal_length / z;

    x = x * of;
    y = y * of;

    x = x_center + x;
    y = y_center - y;
    
    p2d = [x,y];
end


function [pointS,pointE,pointSt,pointEt] = getPoints(type, cam_x, cam_y, cam_z, ang_x, ang_y, ang_z, A, noiseStd)


    x_center = A(1,3);        
    y_center = A(2,3);

    f = A(1,1);

    c = [x_center, y_center];
    o = [cam_x, cam_y, cam_z];
    a = [ang_x, ang_y, ang_z];
     
    if type == 0        
        pointsFix = [0, 0, 0; 0, 256, 0; 256, 256, 0; 256, 0, 0];
        for i = 1:4
            pointS(i,1:3) = pointsFix(i,:);
            pointS(i,4:5) = project(pointS(i,1:3), o, a, f, c);
            pointS(i+4,1:3) = pointsFix(i,:);
            pointS(i+4,4:5) = pointS(i,4:5);
        end

        for i = 1:8
            if i < 4
                pointE(i,:) = pointS(i+1,:); 
            elseif i == 4
                pointE(i,:) = pointS(1,:);
            elseif i >4
                pointE(i,1:2) = pointS(i,1:2);
                j = pointS(i,3) + 1;
                while j < 256
                    temp = [pointS(i,1:2) j];
                    temp2d = project(temp, o, a, f, c);
                    if temp2d(1) >= 0 && temp2d(1) <= 640 && temp2d(2) >= 0 && temp2d(2) <= 640
                        pointE(i,3) = j;
                        pointE(i,4:5) = temp2d;
                        j = j+1;
                    else
                        break;   
                    end
                end
            end 
        end
        pointSt = pointS;
        pointEt = pointE;
        if noiseStd > 0
            tempNoiseS = noiseStd*(rand(4,2)-0.5);
            pointS(1:4,4:5) = pointS(1:4,4:5) + tempNoiseS;
            pointS(5:8,4:5) =  pointS(1:4,4:5);
            pointE(4,4:5) = pointS(1,4:5);
            pointE(1:3,4:5) = pointS(2:4,4:5);
            tempNoiseE = noiseStd*(rand(1,1)-0.5);
            if pointE(5,4)>5 && pointE(5,4)<635
                pointE(5,4) = pointE(5,4)  + tempNoiseE; 
            end
            tempNoiseE = noiseStd*(rand(1,1)-0.5);
            if pointE(5,5)>5 && pointE(5,5)<635
                pointE(5,5) = pointE(5,5)  + tempNoiseE; 
            end
            tempNoiseE = noiseStd*(rand(1,1)-0.5);
            if pointE(6,4)>5 && pointE(6,4)<635
                pointE(6,4) = pointE(6,4)  + tempNoiseE; 
            end
            tempNoiseE = noiseStd*(rand(1,1)-0.5);
            if pointE(6,5)>5 && pointE(6,5)<635
                pointE(6,5) = pointE(6,5)  + tempNoiseE; 
            end
            tempNoiseE = noiseStd*(rand(1,1)-0.5);
            if pointE(7,4)>5 && pointE(7,4)<635
                pointE(7,4) = pointE(7,4)  + tempNoiseE; 
            end
            tempNoiseE = noiseStd*(rand(1,1)-0.5);
            if pointE(7,5)>5 && pointE(7,5)<635
                pointE(7,5) = pointE(7,5)  + tempNoiseE; 
            end
            tempNoiseE = noiseStd*(rand(1,1)-0.5);
            if pointE(8,4)>5 && pointE(8,4)<635
                pointE(8,4) = pointE(8,4)  + tempNoiseE; 
            end
            tempNoiseE = noiseStd*(rand(1,1)-0.5);
            if pointE(8,5)>5 && pointE(8,5)<635
                pointE(8,5) = pointE(8,5)  + tempNoiseE; 
            end
        end
    elseif type == 1 
        pointsFix = [0, 0, 0; 256, 0, 0];
        for i = 1:5
            if i < 4
                pointS(i,1:3) = pointsFix(1,:);
                pointS(i,4:5) = project(pointS(i,1:3), o, a, f, c); 
            else
                pointS(i,1:3) = pointsFix(2,:);
                pointS(i,4:5) = project(pointS(i,1:3), o, a, f, c); 
            end
        end
        for i = 1:5
            if i == 1
                pointE(i,:) = pointS(4,:); 
            else
                if rem(i,2) == 0 
                    pointE(i,1:2) = pointS(i,1:2);
                    j = pointS(i,3) + 1;
                    while j < 256
                        temp = [pointS(i,1:2) j];
                        temp2d = project(temp, o, a, f, c);
                        if temp2d(1) >= 0 && temp2d(1) <= 640 && temp2d(2) >= 0 && temp2d(2) <= 640
                            pointE(i,3) = j;
                            pointE(i,4:5) = temp2d;
                            j = j+1;
                        else
                            break;   
                        end
                    end
                else
                    pointE(i,1) = pointS(i,1);
                    pointE(i,3) = pointS(i,3);
                    j = pointS(i,2) + 1;
                    while j < 256
                        temp = [pointS(i,1) j pointS(i,3)];
                        temp2d = project(temp, o, a, f, c);
                        if temp2d(1) >= 0 && temp2d(1) <= 640 && temp2d(2) >= 0 && temp2d(2) <= 640
                            pointE(i,2) = j;
                            pointE(i,4:5) = temp2d;
                            j = j+1;
                        else
                            break;   
                        end
                    end
                end
            end
        end
        pointSt = pointS;
        pointEt = pointE;
        if noiseStd > 0
            tempNoiseS = noiseStd*(rand(2,2)-0.5);
%             tempNoiseE = 0;
            pointS(1:3,4:5) = pointS(1:3,4:5) + ones(3,1)*tempNoiseS(1,:);
            pointS(4:5,4:5) = pointS(4:5,4:5) + ones(2,1)*tempNoiseS(2,:);
            pointE(1,4:5) = pointS(4,4:5);
            if pointE(2,4)>5 && pointE(2,4)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(2,4) = pointE(2,4)  + tempNoiseE; 
            end
            if pointE(2,5)>5 && pointE(2,5)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(2,5) = pointE(2,5)  + tempNoiseE; 
            end
            if pointE(3,4)>5 && pointE(3,4)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(3,4) = pointE(3,4)  + tempNoiseE; 
            end
            if pointE(3,5)>5 && pointE(3,5)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(3,5) = pointE(3,5)  + tempNoiseE; 
            end
            if pointE(4,4)>5 && pointE(4,4)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(4,4) = pointE(4,4)  + tempNoiseE; 
            end
            if pointE(4,5)>5 && pointE(4,5)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(4,5) = pointE(4,5)  + tempNoiseE; 
            end
            if pointE(5,4)>5 && pointE(5,4)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(5,4) = pointE(5,4)  + tempNoiseE; 
            end 
            if pointE(5,5)>5 && pointE(5,5)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(5,5) = pointE(5,5)  + tempNoiseE; 
            end
        end
    elseif type == 2 
        pointsFix = [0, 256, 0; 256, 256, 0];
        for i = 1:5
            if i < 4
                pointS(i,1:3) = pointsFix(1,:);
                pointS(i,4:5) = project(pointS(i,1:3), o, a, f, c); 
            else
                pointS(i,1:3) = pointsFix(2,:);
                pointS(i,4:5) = project(pointS(i,1:3), o, a, f, c); 
            end
        end
        for i = 1:5
            if i == 1
                pointE(i,:) = pointS(4,:); 
            else
                if rem(i,2) == 0 
                    pointE(i,1:2) = pointS(i,1:2);
                    j = pointS(i,3) + 1;
                    while j < 256
                        temp = [pointS(i,1:2) j];
                        temp2d = project(temp, o, a, f, c);
                        if temp2d(1) >= 0 && temp2d(1) <= 640 && temp2d(2) >= 0 && temp2d(2) <= 640
                            pointE(i,3) = j;
                            pointE(i,4:5) = temp2d;
                            j = j+1;
                        else
                            break;   
                        end
                    end
                else
                    pointE(i,1) = pointS(i,1);
                    pointE(i,3) = pointS(i,3);
                    j = pointS(i,2) - 1;
                    while j > 0
                        temp = [pointS(i,1) j pointS(i,3)];
                        temp2d = project(temp, o, a, f, c);
                        if temp2d(1) >= 0 && temp2d(1) <= 640 && temp2d(2) >= 0 && temp2d(2) <= 640
                            pointE(i,2) = j;
                            pointE(i,4:5) = temp2d;
                            j = j-1;
                        else
                            break;   
                        end
                    end
                end
            end
        end
        pointSt = pointS;
        pointEt = pointE;
        if noiseStd > 0
            tempNoiseS = noiseStd*(rand(2,2)-0.5);
%             tempNoiseE = 0;
            pointS(1:3,4:5) = pointS(1:3,4:5) + ones(3,1)*tempNoiseS(1,:);
            pointS(4:5,4:5) = pointS(4:5,4:5) + ones(2,1)*tempNoiseS(2,:);
            pointE(1,4:5) = pointS(4,4:5);
            if pointE(2,4)>5 && pointE(2,4)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(2,4) = pointE(2,4)  + tempNoiseE; 
            end
            if pointE(2,5)>5 && pointE(2,5)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(2,5) = pointE(2,5)  + tempNoiseE; 
            end
            if pointE(3,4)>5 && pointE(3,4)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(3,4) = pointE(3,4)  + tempNoiseE; 
            end
            if pointE(3,5)>5 && pointE(3,5)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(3,5) = pointE(3,5)  + tempNoiseE; 
            end
            if pointE(4,4)>5 && pointE(4,4)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(4,4) = pointE(4,4)  + tempNoiseE; 
            end
            if pointE(4,5)>5 && pointE(4,5)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(4,5) = pointE(4,5)  + tempNoiseE; 
            end
            if pointE(5,4)>5 && pointE(5,4)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(5,4) = pointE(5,4)  + tempNoiseE; 
            end 
            if pointE(5,5)>5 && pointE(5,5)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(5,5) = pointE(5,5)  + tempNoiseE; 
            end
        end    
    elseif type == 3
        pointsFix = [0, 256, 0];
        pointS(1,1:3) = pointsFix;
        pointS(1,4:5) =  project(pointS(1,1:3), o, a, f, c);
        pointS(2,:) = pointS(1,:);
        pointS(3,:) = pointS(1,:);
        pointE = zeros(3,5);
        for i = 1:3
            if i == 1
                j = 1;
                while j < 256
                    temp = [j 256 0];
                    temp2d = project(temp, o, a, f, c);
                    if temp2d(1) >= 0 && temp2d(1) <= 640 && temp2d(2) >= 0 && temp2d(2) <= 640
                        pointE(i,1:3) = temp;
                        pointE(i,4:5) = temp2d;
                        j = j+1;
                    else
                        break;   
                    end
                end
            elseif i == 2
                j = 255;
                while j > 0
                    temp = [0 j 0];
                    temp2d = project(temp, o, a, f, c);
                    if temp2d(1) >= 0 && temp2d(1) <= 640 && temp2d(2) >= 0 && temp2d(2) <= 640
                        pointE(i,1:3) = temp;
                        pointE(i,4:5) = temp2d;
                        j = j-1;
                    else
                        break;   
                    end
                end
            elseif i == 3
                j = 1;
                while j < 256
                    temp = [0 256 j];
                    temp2d = project(temp, o, a, f, c);
                    if temp2d(1) >= 0 && temp2d(1) <= 640 && temp2d(2) >= 0 && temp2d(2) <= 640
                        pointE(i,1:3) = temp;
                        pointE(i,4:5) = temp2d;
                        j = j+1;
                    else
                        break;   
                    end
                end
            end
        end
        pointSt = pointS;
        pointEt = pointE;
        if noiseStd > 0
            tempNoiseS = noiseStd*(rand(1,2)-0.5);
            tempNoiseE = noiseStd*(rand(1,1)-0.5);
            pointS(:,4:5) = pointS(:,4:5) + ones(3,1)*tempNoiseS;            
            pointE(1,5) = pointE(1,5)  + tempNoiseE; 
            tempNoiseE = noiseStd*(rand(1,1)-0.5);
            pointE(2,4) = pointE(2,4)  + tempNoiseE;
            tempNoiseE = noiseStd*(rand(1,1)-0.5);
            pointE(3,5) = pointE(3,5)  + tempNoiseE; 
        end
    elseif type == 4
        pointsFix = [0, 0, 0];
        pointS(1,1:3) = pointsFix;
        pointS(1,4:5) =  project(pointS(1,1:3), o, a, f, c);
        pointS(2,:) = pointS(1,:);
        pointS(3,:) = pointS(1,:);
        pointE = zeros(3,5);
        for i = 1:3
            if i == 1
                j = 1;
                while j < 256
                    temp = [j 0 0];
                    temp2d = project(temp, o, a, f, c);
                    if temp2d(1) >= 0 && temp2d(1) <= 640 && temp2d(2) >= 0 && temp2d(2) <= 640
                        pointE(i,1:3) = temp;
                        pointE(i,4:5) = temp2d;
                        j = j+1;
                    else
                        break;   
                    end
                end
            elseif i == 2
                j = 1;
                while j < 256
                    temp = [0 j 0];
                    temp2d = project(temp, o, a, f, c);
                    if temp2d(1) >= 0 && temp2d(1) <= 640 && temp2d(2) >= 0 && temp2d(2) <= 640
                        pointE(i,1:3) = temp;
                        pointE(i,4:5) = temp2d;
                        j = j+1;
                    else
                        break;   
                    end
                end
            elseif i == 3
                j = 1;
                while j < 256
                    temp = [0 0 j];
                    temp2d = project(temp, o, a, f, c);
                    if temp2d(1) >= 0 && temp2d(1) <= 640 && temp2d(2) >= 0 && temp2d(2) <= 640
                        pointE(i,1:3) = temp;
                        pointE(i,4:5) = temp2d;
                        j = j+1;
                    else
                        break;   
                    end
                end
            end
        end
        pointSt = pointS;
        pointEt = pointE;
        if noiseStd > 0
            tempNoiseS = noiseStd*(rand(1,2)-0.5);
            tempNoiseE = noiseStd*(rand(1,1)-0.5);
%             tempNoiseE = 0;
            pointS(:,4:5) = pointS(:,4:5) +  ones(3,1)*tempNoiseS;
            pointE(1,5) = pointE(1,5)  + tempNoiseE; 
            tempNoiseE = noiseStd*(rand(1,1)-0.5);
            pointE(2,4) = pointE(2,4)  + tempNoiseE; 
            tempNoiseE = noiseStd*(rand(1,1)-0.5);
            pointE(3,5) = pointE(3,5)  + tempNoiseE; 
        end
    elseif type == 5 
        pointsFix = [0, 0, 0; 0, 256, 0];
        for i = 1:5
            if i < 4
                pointS(i,1:3) = pointsFix(1,:);
                pointS(i,4:5) = project(pointS(i,1:3), o, a, f, c); 
            else
                pointS(i,1:3) = pointsFix(2,:);
                pointS(i,4:5) = project(pointS(i,1:3), o, a, f, c); 
            end
        end
        for i = 1:5
            if i == 1
                pointE(i,:) = pointS(4,:); 
            else
                if rem(i,2) == 0 
                    pointE(i,1:2) = pointS(i,1:2);
                    j = pointS(i,3) + 1;
                    while j < 256
                        temp = [pointS(i,1:2) j];
                        temp2d = project(temp, o, a, f, c);
                        if temp2d(1) >= 0 && temp2d(1) <= 640 && temp2d(2) >= 0 && temp2d(2) <= 640
                            pointE(i,3) = j;
                            pointE(i,4:5) = temp2d;
                            j = j+1;
                        else
                            break;   
                        end
                    end
                else
                    pointE(i,2:3) = pointS(i,2:3);
                    j = pointS(i,1) + 1;
                    while j < 256
                        temp = [j pointS(i,2:3)];
                        temp2d = project(temp, o, a, f, c);
                        if temp2d(1) >= 0 && temp2d(1) <= 640 && temp2d(2) >= 0 && temp2d(2) <= 640
                            pointE(i,1) = j;
                            pointE(i,4:5) = temp2d;
                            j = j+1;
                        else
                            break;   
                        end
                    end
                end
            end
        end
        pointSt = pointS;
        pointEt = pointE;
        if noiseStd > 0
            tempNoiseS = noiseStd*(rand(2,2)-0.5);
%             tempNoiseE = 0;
            pointS(1:3,4:5) = pointS(1:3,4:5) + ones(3,1)*tempNoiseS(1,:);
            pointS(4:5,4:5) = pointS(4:5,4:5) + ones(2,1)*tempNoiseS(2,:);
            pointE(1,4:5) = pointS(4,4:5);
            if pointE(2,4)>5 && pointE(2,4)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(2,4) = pointE(2,4)  + tempNoiseE; 
            end
            if pointE(2,5)>5 && pointE(2,5)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(2,5) = pointE(2,5)  + tempNoiseE; 
            end
            if pointE(3,4)>5 && pointE(3,4)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(3,4) = pointE(3,4)  + tempNoiseE; 
            end
            if pointE(3,5)>5 && pointE(3,5)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(3,5) = pointE(3,5)  + tempNoiseE; 
            end
            if pointE(4,4)>5 && pointE(4,4)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(4,4) = pointE(4,4)  + tempNoiseE; 
            end
            if pointE(4,5)>5 && pointE(4,5)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(4,5) = pointE(4,5)  + tempNoiseE; 
            end
            if pointE(5,4)>5 && pointE(5,4)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(5,4) = pointE(5,4)  + tempNoiseE; 
            end 
            if pointE(5,5)>5 && pointE(5,5)<635
                tempNoiseE = noiseStd*(rand(1,1)-0.5);
                pointE(5,5) = pointE(5,5)  + tempNoiseE; 
            end
        end 
    elseif type == 6
        j = 0;
        begin1 = 0;
        begin2 = 0;
        while j < 256
            temp = [0 j 0];
            temp2d = project(temp, o, a, f, c);
            if temp2d(1) >= 0 && temp2d(1) <= 640 && temp2d(2) >= 0 && temp2d(2) <= 320
                begin1 = begin1 + 1;
                if begin1 == 1
                   pointS(1,1:3) = temp;
                   pointS(1,4:5) = temp2d;
                end
                pointS(2,1:3) = temp;
                pointS(2,4:5) = temp2d;   
            end
            temp2 = [256 j 0];
            temp22d = project(temp2, o, a, f, c);
            if temp22d(1) >= 0 && temp22d(1) <= 640 && temp22d(2) >= 0 && temp22d(2) <= 320
                begin2 = begin2 + 1;
                if begin2 == 1
                   pointE(1,1:3) = temp2;
                   pointE(1,4:5) = temp22d;
                end
                pointE(2,1:3) = temp2;
                pointE(2,4:5) = temp22d;   
            end
            j = j +1;
        end
        pointSt = pointS;
        pointEt = pointE;
        if noiseStd > 0
            tempNoise = noiseStd*(rand(1,2)-0.5);
            pointS(1,4:5) = pointS(1,4:5)+ tempNoise(1:2);
            tempNoise = noiseStd*(rand(1,2)-0.5);
            pointS(2,4:5) = pointS(2,4:5)+ tempNoise(1:2);
            tempNoise = noiseStd*(rand(1,2)-0.5);
            pointE(1,4:5) = pointE(1,4:5)+ tempNoise(1:2);
            tempNoise = noiseStd*(rand(1,2)-0.5);
            pointE(2,4:5) = pointE(2,4:5)+ tempNoise(1:2);
        end
    elseif type == 7
        i = 0;
        begin1 = 0;
        begin2 = 0;
        while i < 256
            temp = [i 0 0];
            temp2d = project(temp, o, a, f, c);
            if temp2d(1) >= 0 && temp2d(1) <= 320 && temp2d(2) >= 0 && temp2d(2) <= 640
                begin1 = begin1 + 1;
                if begin1 == 1
                   pointS(1,1:3) = temp;
                   pointS(1,4:5) = temp2d;
                end
                pointS(2,1:3) = temp;
                pointS(2,4:5) = temp2d;   
            end
            temp2 = [i 256 0];
            temp22d = project(temp2, o, a, f, c);
            if temp22d(1) >= 0 && temp22d(1) <= 320 && temp22d(2) >= 0 && temp22d(2) <= 640
                begin2 = begin2 + 1;
                if begin2 == 1
                   pointE(1,1:3) = temp2;
                   pointE(1,4:5) = temp22d;
                end
                pointE(2,1:3) = temp2;
                pointE(2,4:5) = temp22d;   
            end
            i = i +1;
        end
        pointSt = pointS;
        pointEt = pointE;
        if noiseStd > 0
            tempNoise = noiseStd*(rand(1,2)-0.5);
            pointS(1,4:5) = pointS(1,4:5)+ tempNoise(1:2);
            tempNoise = noiseStd*(rand(1,2)-0.5);
            pointS(2,4:5) = pointS(2,4:5)+ tempNoise(1:2);
            tempNoise = noiseStd*(rand(1,2)-0.5);
            pointE(1,4:5) = pointE(1,4:5)+ tempNoise(1:2);
            tempNoise = noiseStd*(rand(1,2)-0.5);
            pointE(2,4:5) = pointE(2,4:5)+ tempNoise(1:2);
        end
    end             
end



function [Ro, T] = get_extrinsic(observer,angles)

    a_x = angles(1);
    a_y = angles(2); 
    a_z = angles(3);
    o_x = observer(1);
    o_y = observer(2);
    o_z = observer(3);


    sin_x = sin(a_x * (pi / 180));
    cos_x = cos(a_x * (pi / 180));

    sin_y = sin(a_y * (pi / 180));
    cos_y = cos(a_y * (pi / 180));

    sin_z = sin(a_z * (pi / 180));
    cos_z = cos(a_z * (pi / 180));

    rx1 = [
        [1.0, 0.0, 0.0]
        [0.0, cos_x, -sin_x]
        [0.0, sin_x, cos_x]
    ];

    ry1 =[
        [cos_y, 0.0, sin_y]
        [0.0, 1.0, 0.0]
        [-sin_y, 0.0, cos_y]
    ];

    rz1 = [
        [cos_z, -sin_z, 0.0]
        [sin_z, cos_z, 0.0]
        [0.0, 0.0, 1.0]
    ];
  
    

    r = rx1*ry1*rz1;

    z = atan2(r(2,1),r(1,1));
    y = atan2(-r(3,1),sqrt(r(3,2)^2 + r(3,3)^2));
    x = atan2(r(3,2),r(3,3));
    

    if x>=0
        x = pi-x;
    else
        x = -x-pi;
    end
    sin_x = sin(x);
    cos_x = cos(x);

    sin_y = sin(y);
    cos_y = cos(y);

    z = -z;
    sin_z = sin(z);
    cos_z = cos(z);

    rx = [
        [1.0, 0.0, 0.0]
        [0.0, cos_x, -sin_x]
        [0.0, sin_x, cos_x]
    ];

    ry = [
        [cos_y, 0.0, sin_y]
        [0.0, 1.0, 0.0]
        [-sin_y, 0.0, cos_y]
    ];

    rz = [
        [cos_z, -sin_z, 0.0]
        [sin_z, cos_z, 0.0]
        [0.0, 0.0, 1.0]
    ];



    Ro = rz*ry*rx;
    tempT = [o_x,o_y,o_z];
%     angleMC =  array([x,y,z])

    T = -Ro*tempT';

    
end
                        

        
     