x1=deg2rad([106.95 116.64 131 154 186.33 215.85 235.07 247.026 255.87]); %theta s (azimuth)
x4=[28.59 41.33 52.77 61.22 63.62 58.54 48.67 36.61 23.58]; %beta (elevation)
x2=deg2rad(180); %theta c
x3=deg2rad([1:1:90]); %summation angle
d=x4;
x4=deg2rad(x4);
I=zeros(90,360);
final=zeros(1,12);
m=zeros(1,20);
x=zeros(1,20);
n=272; %day number 29/11-272
k=1;
A=1160+75*sin((360/365)*(n-275));
K=0.174+.035*sin((360/365)*(n-100));
C=.095+.04*sin((360/365)*(n-100));
s=zeros(359,89);
sum=0;
flag=1;
%m=1/cos(degtorad(90-d(i))); %air mass ratio
p=.03; %reflectance

for i=1:360
    for j=1:90
        sum=0;
        for k=1:8
            m=1/cos(deg2rad(90-d(k)));
            x2=deg2rad(i);
    I=A*exp(-1*K*m)*(cos(x4(k))*cos(x1(k)-x2)*sin(x3(j))+sin(x4(k))*cos(x3(j))+C/2*(1+cos(x3(j)))+p/2*(sin(x4(k))+C)*(1+cos(x3(j))));
            sum=sum+I;
        end
        s(i,j)=sum;
        
    end
end
    
for i=1:20
[maxValue, linearIndexesOfMaxes] = max(s(:))
[rowsOfMaxes, colsOfMaxes] = find(s == maxValue);
final(i)=rowsOfMaxes;
x(i)=colsOfMaxes;
s(rowsOfMaxes, colsOfMaxes)=0;
end
plot(s)