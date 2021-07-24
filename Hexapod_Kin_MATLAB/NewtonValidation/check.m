

datax = load('MaxErrorX.txt');
datay = load('MaxErrorY.txt');
dataz = load('MaxErrorZ.txt');

datarx = load('MaxErrorRX.txt');
datary = load('MaxErrorRY.txt');
datarz = load('MaxErrorRZ.txt');

dataiterations = load('Iterations.txt');

si = size(datax);
% disp(si)

% disp(max(datax));
% disp(max(datay));
% disp(max(dataz));
% 
% disp(max(datarx));
% disp(max(datary));
% disp(max(datarz));

disp(mean(dataiterations));


% disp(max(dataiterations));
% 
% disp(min(datax));
% disp(min(datay));
% disp(min(dataz));
% 
% disp(min(datarx));
% disp(min(datary));
% disp(min(datarz));
% 
% disp(min(dataiterations));
