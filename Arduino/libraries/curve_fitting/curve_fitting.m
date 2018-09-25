function curve_fitting(part_code, filename)
% This script performs a curve fitting for a Sharp IR range finder
% Arguments:
% * part_code - the name of the sensor { GP2D120XJ00F, GP2Y0A21YK, GP2Y0A02YK0F }
% * filename  - the file containing the data
% Example:
% % Navigate first to the folder containing the data and the code
% curve_fitting('GP2D120XJ00F','datapoints.csv')

% Specifies the models of the sensors that are derived from the datasheets
part_ds_model = containers.Map;
part_ds_model( 'GP2D120XJ00F' ) = @(v) 75.59 * exp( -3.877 * v ) + 21.31 * exp( -0.6346 * v );
part_ds_model( 'GP2Y0A21YK' ) = @(v) 223.9 * exp( -4.307 * v ) + 49.65 * exp( -0.7061 * v );
part_ds_model( 'GP2Y0A02YK0F' ) = @(v) v * ( v * ( v * ( v * ( -15.01 * v + 136.9 ) - 495 ) + 896.9 ) - 846.8 ) + 385.7;
fd = part_ds_model(part_code);

% Loads and parses the data
data = csvread(filename,1,0);
cm = data(:,1); v = data(:,2);

% Fits a model to the data
% It needs the Curve Fitting Toolbox. If you don't have this toolbox,
% alternatively you can use the EzyFit tool (http://bit.ly/1cjecIJ)
% The model that's being used is f(x) = a * exp(b * x) + c * exp(d * x)
% You could change this to whatever fits you
params = fit(v,cm,'exp2') %#ok<NOPRT>
% If you are using EzyFit tool, comment the line above and uncomment the next 2 lines
% plot(v,cm,'o','MarkerSize',7)
% params = showfit('a * exp(b * x) + c * exp(d * x) + e') %#ok<NOPRT>

% Plots the data, the model from the datasheet and the estimated model
t = 0.3:0.01:3.2;
yd = fd(t);  % Model from the datasheet
y = params.a * exp(params.b * t) + params.c * exp(params.d * t);  % Estimated model
% If you are using the EzyFit tool, comment the line above and uncomment the next one
% y = params.m(1) * exp(params.m(2) * t) + params.m(3) * exp(params.m(4) * t) + params.m(5);
figure; hold;
plot(v,cm,'o','MarkerSize',7)
plot(t,yd,'k','LineWidth',2)
plot(t,y,'r','LineWidth',2)
set(gcf,'Color',[1 1 1]);
title(sprintf('%s Transfer Function', part_code));
xlabel('V [V]'); ylabel('x [cm]');
legend('data points','model from datasheet','estimated model');
disp('Now go back to IRRanger.h and update the model')

end