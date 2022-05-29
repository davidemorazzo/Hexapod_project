function [theta_a_new] = angular_control_loop(a,Tsim,Tstep,theta_a)

% SIMPLE CONTROL LOOP OF THE ANGULAR POSITION W.R.T. Z AXIS

%   Control loop based on mpu6050 gyroscope, theta is the initial vector of
%   angular positions in time with theta_a first term. It perform a loop of
%   n_iter computing in real time the new theta as the previous + the
%   product of angular speed measured in real time by the sensor (in rad/s)
%   and the step time. We want to keep our trajectory for theta_a, so if
%   the new theta is quite different from it the new theta_a will become
%   exactly their difference otherwise the new theta will keep equal to the
%   initial one.

% Time parameters
% Tsim=10;
% Tstep=0.02;
n_iter=Tsim/Tstep;

% Define the sensor MPU6050
gyro=mpu6050(a,'OutputFormat','matrix','SampleRate',50,'SamplesPerRead',1);


% Initialization of position theta
theta=zeros(n_iter,1);       % set to zeros in order to have theta_a=0 as ideal condition for 'walk_forward'
theta_dot=zeros(n_iter,1);   % preallocated vector in order to save memory for the next for statement
theta(1)=theta_a;

% Control loop for n iterations
for n=1:n_iter
    [gyroReadings]=read(gyro);                  % you need sensor fusion and tracking toolbox or navigation toolbox to use read
    theta_dot(n,1)=gyroReadings(3);             % multiply n coefficient if you want to pick greater samples og angular speed
    theta(n+1)=theta(n)+theta_dot(n)*Tstep;     % new angular position
    if abs(theta_a-theta(n+1))>0.1              % set the best fit tolerance
        theta_a_new=theta_a-theta(n+1);         % theta_a_new is the new value with which computing the trajectory
    else 
        theta_a_new=theta_a;
    end
end

end