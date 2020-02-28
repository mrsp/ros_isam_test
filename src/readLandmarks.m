corr00=load('corr00.txt');
corr01=load('corr01.txt');


corr11=load('corr11.txt');
corr12=load('corr12.txt');


corr22=load('corr22.txt');
corr23=load('corr23.txt');


cov(corr00-corr01)
cov(corr11-corr12)
cov(corr22-corr23)



figure
plot3(corr00(:,1)-corr01(:,1),corr00(:,2)-corr01(:,2),corr00(:,3)-corr01(:,3),'*')

figure
plot(corr00(:,1)-corr01(:,1))
rms(corr00(:,1)-corr01(:,1))

figure
plot(corr00(:,3)-corr01(:,3))
rms(corr00(:,3)-corr01(:,3))

figure
plot(corr00(:,2)-corr01(:,2))
rms(corr00(:,2)-corr01(:,2))


%%

figure
plot3(corr11(:,1)-corr12(:,1),corr11(:,2)-corr12(:,2),corr11(:,3)-corr12(:,3),'*')

figure
plot(corr11(:,1)-corr12(:,1))
rms(corr11(:,1)-corr12(:,1))

figure
plot(corr11(:,3)-corr12(:,3))
rms(corr11(:,3)-corr12(:,3))

figure
plot(corr11(:,2)-corr12(:,2))
rms(corr11(:,2)-corr12(:,2))

%%
figure
plot3(corr22(:,1)-corr23(:,1),corr22(:,2)-corr23(:,2),corr22(:,3)-corr23(:,3),'*')

figure
plot(corr22(:,1)-corr23(:,1))
rms(corr22(:,1)-corr23(:,1))

figure
plot(corr22(:,3)-corr23(:,3))
rms(corr22(:,3)-corr23(:,3))

figure
plot(corr22(:,2)-corr23(:,2))
rms(corr22(:,2)-corr23(:,2))