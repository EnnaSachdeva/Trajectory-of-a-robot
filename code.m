% This code represents the trajectory of a robot as seen from a VO frame 
% Assuming 0,0,0 be the initial coordinates of robot in IMU frame at t=0

clc;
clear all;
close all;

%%%%%%%%%%%%%%%%%% trajectory of a robot in IMU frame %%%%%%%%%%%%%%%%%%%
t = 0:0.05:2*pi;

x = linspace (0, 1, numel (t));
z = linspace (0, 1, numel (t));
y = linspace (0, 1, numel (t));

% axes1 = axes('Parent',Parent1,'OuterPosition',[-10 10 -10 10]);
% view(axes1,[-37.5 30]);
% grid(axes1,'on');

%fig=figure('Position',[1 scrsz(4)/2 scrsz(3)/2 scrsz(4)/2]);

az=160;
el=24;
%view(az,el);
rotation_spd=0.5;

scrsz = get(0,'ScreenSize');
%ScreenSize is a four-element vector: [left, bottom, width, height]:

axis([0 8 0 8 0 8])
hold on
grid on



%%%%%%%% Assuming the VO frame translates by (0.001,0.001,0) after every instant t 
tr_vo= [0 ; 0 ; 0];

%%%%%%%%%%%%%%%% initially IMU frame coincides with global frame
yaw_imu = 0;  
pitch_imu = 0; 
roll_imu = 0;

%%%% IMU to be rotated by 90 degree clockwise w.r.t VO frame about y axis
yaw_vo = 0;  
pitch_vo = -pi/2; 
roll_vo = 0;

angle = 0.2617;   % assuming IMU rotates by 15 degrees w.r.t frame G after every instant t

rotimu = angle2dcm( yaw_imu, pitch_imu, roll_imu ); %rotation of IMU w.r.t global frame G 

rotvo = angle2dcm( yaw_vo, pitch_vo, roll_vo ) % rotation matrix for rotating the IMU frame 

rotimu= rotimu* rotvo; % vo is rotated by -90 about y axis , w.r.t imu frame

T = [];

for i=1:length(t),
  
   %%%%%% Assuming evert time t, imu rotates by 15 degree clockwise %%%%%%% 
   yaw_imu = yaw_imu-angle;  
   pitch_imu = pitch_imu-angle; 
   roll_imu = roll_imu-angle;

   rotimu = angle2dcm( yaw_imu, pitch_imu, roll_imu ); % rotation matrix for rotating the IMU frame 
   
   traj_imu = [t(i); 0; 0]; %%%% imu trajectory along X axis  
   traj_vo =  [0; 0 ; t(i)]; %%%% vo trajectory along Z axis  
   Pimu = rotimu * traj_imu; 
   
   display(rotimu);
   %imu = plot3 (Pimu(1,1), Pimu(2,1),Pimu(3,1),'*');
   %Pimu = rotimu * Pimu;
   
   %%%%%%%%%%%%%%%%%%% trajectory as seen in VO frame %%%%%%%%%%%%%%%%%%%%
   
   tr_vo = tr_vo + [0.001 ; 0 ; 0] ;
   display(tr_vo);
   Pvo = Pimu + tr_vo; 
   
   %%%%%%%%%%%%%% transformation matrix calculation %%%%%%%%%%%%%%%%%%%%%% 
   T = [rotimu,               tr_vo;...
        zeros(1,3) ,          1  ];
  
        display(T);   
        
        
   %%%%%%%%%%%%%%%%%%%% trajectory in imu plane %%%%%%%%%%%%%%%%%%%%
   imu = plot3(traj_imu(1,1),traj_imu(2,1),traj_imu(3,1),'marker', '.', ...
                    'color', 'Red', ...
                    'MarkerSize', 10, ...
                    'LineStyle', '-');
   
 xlabel('------------X axis') % x-axis label
 ylabel('------------Y axis') % y-axis label
 zlabel('------------Z axis') % z-axis label
   
legend('trajectory in IMU','trajectory in IMU',...
       'trajectory in IMU as seen from VO','Location','BestOutside');
  
  %%%%%%%%%%%%%%%%%% trajectory in rotated imu plane %%%%%%%%%%%%%%%%%%%%
   vo = plot3 (traj_vo(1,1), traj_vo(2,1),traj_vo(3,1),'marker', '.', ...
                    'color', 'green', ...
                    'MarkerSize', 10, ...
                    'LineStyle', '-');
                
     %%%%%%%%%%%%%%%%%%% trajectory in final vo plane %%%%%%%%%%%%%%%%%%%%
%   

% (1,1), Pvo(2,1),Pvo(3,1));

  vo_imu = plot3(Pvo(1,1), Pvo(2,1),Pvo(3,1), 'marker', '*', ...
                   'color', 'blue', ...
                    'MarkerSize', 10, ...
                    'LineStyle', '-');
                
%  plotStreamLines(Pvo(1,1), Pvo(2,1),Pvo(3,1));
      % vo = mesh(Pvo(1,1), Pvo(2,1),Pvo(3,1));
    %az=az+rotation_spd;
   view(az,el);
   drawnow;   

 pause (0.001);

end


   
             