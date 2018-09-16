% Matlab script to analyze the error in the localino data
% There are 6 csv files labeled localinoA_localinoB.csv
% Each has 3 columns corresponding to
% Actual Distance, localinoA's measurement of B, localinoB's measurement of A

clc; close all;

num_meas = 50 ; % num measurements at each distance

% Here's all of the data...

case_dkt = [case_drone(:,2), kipp_case(:,3), tars_case(:,3)] ;
drone_ckt = [case_drone(:,3), drone_kipp(:,2), tars_drone(:,3)] ;
tars_cdk = [tars_case(:,2), tars_drone(:,2), tars_kipp(:,2)] ;
kipp_cdt = [kipp_case(:,2), drone_kipp(:,3), tars_kipp(:,3)] ;

dists = [ case_drone(:,1), case_drone(:,1), case_drone(:,1)] ; 

%%%%%%%%%%%%%%%%%%% Plot the error %%%%%%%%%%%%%%%%%%%%%%%
figure('Name', 'Error Plot')
hold on;
% Case
scatter(case_dkt(:,1), dists(:,1), [], 'r')
scatter(case_dkt(:,2), dists(:,1), [], 'g')
scatter(case_dkt(:,3), dists(:,1), [], 'b')

% Drone
scatter(drone_ckt(:,1), dists(:,1), [], 'y')
scatter(drone_ckt(:,2), dists(:,1), [], 'm')
scatter(drone_ckt(:,3), dists(:,1), [], 'c')

% Tars
scatter(tars_cdk(:,1), dists(:,1), [], 'w')
scatter(tars_cdk(:,2), dists(:,1), [], 'k')
scatter(tars_cdk(:,3), dists(:,1), [], 'b', mkr='*')

% Kipp
scatter(kipp_cdt(:,1), dists(:,1), [], 'r', mkr='.')
scatter(kipp_cdt(:,2), dists(:,1), [], 'g', mkr='*')
scatter(kipp_cdt(:,3), dists(:,1), [], 'b', mkr='s')

xlabel("Actual Distance [m]")
ylabel('Measured Distance [m]')
title('Error Plot')

%%%%%%%%%%%%%%%%%%%% Visualize average error for each localino %%%%%%%%%%%%%%%%%%%%%%
% Calculate the error

case_dkt_err = case_dkt - dists ;
drone_ckt_err = drone_ckt - dists ;
tars_cdk_err = tars_cdk - dists ;
kipp_cdt_err = kipp_cdt - dists ;

case_avg_err = mean( case_dkt_err(:) )
drone_avg_err = mean( drone_ckt_err(:) )
tars_avg_err = mean( tars_cdk_err(:) )
kipp_avg_err = mean( kipp_cdt_err(:) )

mean_error_vector = [ mean(case_dkt_err) mean(drone_ckt_err) mean(tars_cdk_err) mean(kipp_cdt_err) ] ;
std_error_vector = [ std(case_dkt_err) std(drone_ckt_err) std(tars_cdk_err) std(kipp_cdt_err) ] ;

figure('Name', 'Mean & Variance of Error Graphs');
bar(mean_error_vector, color='b')
hold on;
bar(std_error_vector, color='m')

xlabel("Measurement #")
ylabel('Value [m]')
legend('Mean Error', 'Standard Deviation of Error')
title('Mean & Variance of Error by Test #')

return

%%%%%%%%%%%%%%%%%%% Looking at the error %%%%%%%%%%%%%%%%%%%%%%

case_dkt_err = case_dkt - dists ;
drone_ckt_err = drone_ckt - dists ;
tars_cdk_err = tars_cdk - dists ;
kipp_cdt_err = kipp_cdt - dists ;

ttest2(case_dkt_err(:,1), case_dkt_err(:,2))