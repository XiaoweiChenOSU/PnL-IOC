function R= CGR2(s)

s1= s(1);
s2= s(2);
s3= s(3);

R= [ s1^2 - s2^2 - s3^2 + 1,   2*s3 + 2*s1*s2,           2*s1*s3 - 2*s2;
     2*s1*s2 - 2*s3, - s1^2 + s2^2 - s3^2 + 1,           2*s1 + 2*s2*s3;
     2*s2 + 2*s1*s3,           2*s2*s3 - 2*s1, - s1^2 - s2^2 + s3^2 + 1];
 
R= R/(1+s1*s1+s2*s2+s3*s3);

return