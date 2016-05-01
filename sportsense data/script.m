bbf2 = csvread('~/Desktop/sportsense data/bbf2.csv');
bbB2 = csvread('~/Desktop/sportsense data/bbB2.csv');

close all

%TODO: units ;
x = linspace(0,2*pi,500);

plot(x, bbB2(1:500,5),x,bbB2(1:500,6),x,bbB2(1:500,7),'DisplayName','bbB2')
axis([0,10,-0.05,0.05])
title('ball')

figure

plot(x, bbf2(1:500,5),x,bbf2(1:500,6),x,bbf2(1:500,7),'DisplayName','bbf2')
axis([0,10,-0.05,0.05])
title('swing')

bbf2FFT = [fft(bbf2(:,5)) fft(bbf2(:,6)) fft(bbf2(:,7))];
bbB2FFT = [fft(bbB2(:,5)) fft(bbB2(:,6)) fft(bbB2(:,7))];

figure
plot(x, bbB2FFT(1:500,1),x,bbB2FFT(1:500,2),x,bbB2FFT(1:500,3),'DisplayName','bbf2')
axis([0,10,-0.5,0.5])
title('ball')

figure
plot(x, bbf2FFT(1:500,1),x,bbf2FFT(1:500,2),x,bbf2FFT(1:500,3),'DisplayName','bbf2')
axis([0,10,-0.5,0.5])
title('swing')

x = linspace(0,2*pi,129);
bbf2SG = [spectrogram(bbf2(:,5),x) spectrogram(bbf2(:,6),x) spectrogram(bbf2(:,7),x)];
bbB2SG = [spectrogram(bbB2(:,5),x) spectrogram(bbB2(:,6),x) spectrogram(bbB2(:,7),x)];


figure
plot(x, bbB2SG(1:129,1),x,bbB2SG(1:129,2),x,bbB2SG(1:129,3),'DisplayName','bbf2')
axis([0,10,-0.05,0.05])
title('ball sg')

figure
plot(x, bbf2SG(1:129,1),x,bbf2SG(1:129,2),x,bbf2SG(1:129,3),'DisplayName','bbf2')
axis([0,10,-0.05,0.05])
title('swing sg')

