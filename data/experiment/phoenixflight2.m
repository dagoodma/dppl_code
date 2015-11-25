close all;
clear all;

%% Parameters
localWaypointFilename='2015.11.07_east4wp_local.csv';
dataLogFilename='2015.11.18_phoenixflight2_gps.csv';

origin = [36.988505,-122.0509133,0]; % from raw telemetry data
initialPosition = [0, 0];
initialHeading = 0.0; % rad
[dataLocal] = csvread(localWaypointFilename, 1);
V= [...
		-57.8852 ,140.524;...
		-13.965 ,-13.3296;...
		29.9552 ,-167.183;...
		63.4918 ,-284.663;...
		-109.926 , -260.05;...
		-153.846 , -106.196;...
		-183.953, -0.730202];

%V = [dataLocal(:,2), dataLocal(:,3)];


%% Add dependencies
addpath('../../matlab', '../../matlab/lib');

%% Open and the convert the data
[dataGeo, ~] = readCsvGps(dataLogFilename);
[dataNed] = convertGpsData(dataGeo(:,2:4), origin);

% Split tour by position indices (found experimentally)
% For raw data
% indTour1=[1957:2217];
% indTour2=[2218:2475];
% indTour3=[2476:2708];
indTour1=[1957:2217];
indTour2=[2218:2475];
indTour3=[2476:2708];

ind = [5000:6700];

%% Plotting
figure();
plot(dataNed(ind,2), dataNed(ind,1), 'k-');
ylabel('North [m]');
xlabel('East [m]');
hold on;

%plot(V(:,1), V(:,2), 'go', 'MarkerFaceColor', 'g');
plot(V(:,1), V(:,2), '*', 'MarkerSize', 10, 'Color', [1 0 0]);
for i=1:length(V)
	text(V(i,1)+2.5, V(i,2)+2.0, sprintf('%d',i),'FontSize',11);
end

plot(initialPosition(1),initialPosition(2),'ro', 'MarkerFaceColor', 'r');
axis square;

%legend('Tour 1', 'Tour 2', 'Tour 3', 'Waypoint','Origin')
%title('Three Tours with 20 [m] Turn Radius')
hold off;
