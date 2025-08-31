numpaths = 5;
channel = phased.IsoSpeedUnderwaterPaths('ChannelDepth',200,'BottomLoss',10, ...
    'NumPathsSource','Property','NumPaths',numpaths);
tstep = 1;
srcpos = [0;0;-160];
rcvpos = [100;0;-50];
speed = -20*1000/3600;
srcvel = [0;0;0];
rcvvel = [speed;0;0];
[pathmat,dop,absloss] = channel(srcpos,rcvpos,srcvel,rcvvel,tstep);
fs = 1e3;
nsamp = 500;
propagator = phased.MultipathChannel('OperatingFrequency',10e3,'SampleRate',fs);
t = [0:(nsamp-1)]'/fs;
sig0 = sin(2*pi*100*t);
sig = repmat(sig0,1,numpaths);
propsig = propagator(sig,pathmat,dop,absloss);
plot(t*1000,real(sum(propsig,2)))
xlabel('Time (millisec)')
