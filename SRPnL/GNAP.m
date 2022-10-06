function [s, err] = GNAP(GG, s0)

%first compute the weight of each line and the normal of the interpretation plane passing through to camera center and the line 

s= GaussNewton(GG,s0);

sr = [1 s(1) s(2) s(3)]';

err = sr' * GG * sr;

return


function sf= GaussNewton(m,s0)

st = [1 s0(1) s0(2) s0(3)]';

obj_pre = st.'*m*st;

A1= [...
    2*m(6);
    m(7)+m(10);
    m(8)+m(14);
    m(2)+m(5)];



A2= [...
    m(7)+m(10);
    2*m(11);
    m(12)+m(15);
    m(3)+m(9)];


A3= [...
    m(8)+m(14);
    m(12)+m(15);
    2*m(16);
    m(4)+m(13)];

s= s0;
sf= s0;

pref= 10000;

for i= 1:1000
    
    s1= s(1);
    s2= s(2);
    s3= s(3);

    vecs= [s1 , s2 , s3 , 1];

    vecs1= [ 1, 0, 0, 0];
    vecs2= [ 0, 1, 0, 0];
    vecs3= [ 0, 0, 1, 0];
   
    f1= vecs*A1;
    f2= vecs*A2;
    f3= vecs*A3;


    curf= f1*f1+f2*f2+f3*f3;
    if curf > pref
	break;
    end

    pref= curf;

    f11= vecs1*A1;
    f12= vecs2*A1;
    f13= vecs3*A1;

    f21= vecs1*A2;
    f22= vecs2*A2;
    f23= vecs3*A2;

    f31= vecs1*A3;
    f32= vecs2*A3;
    f33= vecs3*A3;

    H= [f11 f12 f13; f21 f22 f23; f31 f32 f33];
    
    if det(H) < 2e-10
        s= s- pinv(H)*[f1; f2; f3];
    else
        s= s- H\[f1; f2; f3];
    end
    st = [1 s(1) s(2) s(3)]';
    obj_cur = st.'*m*st;
    if abs(obj_cur)<=abs(obj_pre) 
        sf=s;
        obj_pre = obj_cur;
    end
end


