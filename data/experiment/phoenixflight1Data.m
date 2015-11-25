close all;
clear all;

%% Parameters
localWaypointFilename='2015.11.07_east4wp_local.csv';
dataRawLogFilename='2015.11.07_phoenixflight1_gpsraw.csv';
dataLogFilename='2015.11.07_phoenixflight1_gps.csv';

origin = [36.988505,-122.0509133,0]; % from raw telemetry data
initialPosition = [0, 0];
initialHeading = 0.0; % rad
[dataLocal] = csvread(localWaypointFilename, 1);
%V = [14.1924, 91.5624;...
%    -23.7207, 171.914;...
%    -216.686, 189.736;...
%    -236.224, 47.4646];
V = [dataLocal(:,2), dataLocal(:,3)];


%% Add dependencies
addpath('../../matlab', '../../matlab/lib');

%% Open and the convert the data
[dataGeo, ~] = readCsvGps(dataLogFilename);
[dataGeoRaw, ~] = readCsvGps(dataRawLogFilename,1);
[dataNed] = convertGpsData(dataGeo(:,2:4), origin);
[dataNedRaw] = convertGpsData(dataGeoRaw(:,2:4), origin);

% Split tour by position indices (found experimentally)
% For raw data
% indTour1=[1957:2217];
% indTour2=[2218:2475];
% indTour3=[2476:2708];
indTour1=[1957:2217];
indTour2=[2218:2475];
indTour3=[2476:2708];

%% Plotting
figure();
%plot(dataNed(indTour1,2), dataNed(indTour1,1), 'k+');
plot(dataNed(:,2), dataNed(:,1), 'k+');
ylabel('North [m]');
xlabel('East [m]');
hold on;
% plot(dataNed(indTour2,2), dataNed(indTour2,1), 'bx');
% plot(dataNed(indTour3,2), dataNed(indTour3,1), 'mo');

%plot(V(:,1), V(:,2), 'go', 'MarkerFaceColor', 'g');
plot(V(:,1), V(:,2), '*', 'MarkerSize', 10, 'Color', [1 0 0]);
for i=1:length(V)
	text(V(i,1)+2.5, V(i,2)+2.0, sprintf('%d',i),'FontSize',11);
end

plot(initialPosition(1),initialPosition(2),'ro', 'MarkerFaceColor', 'r');

legend('Tour 1', 'Tour 2', 'Tour 3', 'Waypoint','Origin')
title('Three Tours with 20 [m] Turn Radius')
hold off;

%% Compare raw and estimated position
figure();
subplot(2,1,1);
plot(dataGeoRaw(:,1), dataNedRaw(:,1), 'k');
	hold on;
plot(dataGeo(:,1), dataNed(:,1), '--g');
xlabel('Time [s]')
ylabel('North Position [m]')
legend('Measured Position', 'Estimated Position')
	hold off;

subplot(2,1,2);
plot(dataGeoRaw(:,1), dataNedRaw(:,2), 'k');
	hold on;
plot(dataGeo(:,1), dataNed(:,2), '--g');
xlabel('Time [s]')
ylabel('East Position [m]')
legend('Measured Position', 'Estimated Position')
	hold off;
