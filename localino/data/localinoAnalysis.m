% Matlab script to analyze the error in the localino data
% There are 6 csv files labeled localinoA_localinoB.csv
% Each has 3 columns corresponding to
% Actual Distance, localinoA's measurement of B, localinoB's measurement of A
clc; close all;

num_meas = 50 * 6 % num measurements at each distance

# load the data
M = sort( [case_drone ; drone_kipp ; kipp_case ; tars_case ; tars_drone ; tars_kipp ] );

m_single = sort([M(:,1), M(:,2); M(:,1), M(:,3)]);
size(m_single)
figure
scatter(m_single(:,1), m_single(:,2))
xlabel('Distance [m]')
ylabel('Measurement [m]')
title('Total Measurements vs Distance')
% Plot the data


%plot( M(:,1), M(:,3))
%plot( M(:,1), M(:,2))
%plot( M(:,1), M(:,3))

%                   Let's plot the error
% Calculate the error at every given point

size(M, 2) ; % this is num columns

u_error_record = [] ;
%v_error_record = [] ;
std_error_record = [] ;
error = [] ;
index = -1
for i = 0:2:20 % we measured at all even numbers 0 to 20
  index += 1 ;
  for j = 1:num_meas
    % Record the error as a single vector
    error = [error ; M(index * num_meas + j, 2) - M(index * num_meas + j, 1); M(index * num_meas + j, 3) - M(index * num_meas + j, 1)] ;
  endfor
  u_error = mean(error) ;
%  v_error = var(error) ;
  std_error = std(error) ;
  % left most column is the distance, locA, locB\
  u_error_record = [u_error_record ; i, u_error] ;
%  v_error_record = [v_error_record ; i, v_error(1,1), v_error(1,2) ] ;
  std_error_record = [std_error_record ; i, std_error ] ;
endfor

index * num_meas + j

figure 
hold on;

subplot(2,1,1);
plot(u_error_record(:,1), u_error_record(:,2), '-bo')
title('Total Average Error vs Distance')
ylabel('Average Error [m]')
xlabel('Distance [m]')
subplot(2,1,2);
plot(std_error_record(:,1), std_error_record(:,2), '-ro')
title('Total Standard Deviation of Error vs Distance')
ylabel('Standard Deviation of Error [m]')
xlabel('Distance [m]')