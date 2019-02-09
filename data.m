fileID = fopen('1.txt','r');
A = fscanf(fileID,'%s');
length(A);
con=0;
str='';
k=1;
p=1;
for i=1:length(A)
    if(A(i)==';')
        con=con+1;
    end
end

d=struct('data','');

for i=1:con
    for j=k:k+18
        if(k<con*20-1)
        str=[str,A(j)];
        if(j==p+18)
            k=j+2;
        end
        end  
    end
    p=k;
d(i).data=str;
str='';
end

