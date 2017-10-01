function T=getTopics()
T.labels={'nV';         % number of connected vehicles
          'IMU';        % IMU data
          'ATT';        % Attitude data
          'POS';        % NED position from Pixhawk
          'MOCAPOS';    % NED position from mocap
          'GPS';        % GPS from Pixhawk
          'BATT';       % Battery voltage
          'VMODE'};     % vehicle mode

T.nT=length(T.labels);    % total number of topics

T.Tnumber=(1:T.nT)';      % topic numbers (IDs)
T.nFields=[1,7,7,7,3,4,1,2]';   % number of data fields in each topic
end