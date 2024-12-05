clear all;

% aa = rosbag('chungra_0712_rear_v0_realtime_pose.bag');   %% With Lidar
% % aa = rosbag('2024-07-12-15-01-11.bag');
% aa = rosbag('chungra_0712_rear_v3_pose_2024-07-12-16-03-44.bag');
aa = rosbag('chungra_0712_mapv4_Slampose.bag');
aa = rosbag('map_v1_pose_2024-07-17-15-32-29.bag');
aa = rosbag('2024-07-23-19-01-04.bag');

IMGpose = select(aa,'Topic','/orb_slam3/camera_pose');
IMGMsg = readMessages(IMGpose,'DataFormat','struct');

IMGHeadingInit = 0;
for kkg = 2 : length(IMGMsg)
    IMGpos(kkg,:) = [IMGMsg{kkg}.Pose.Position.X IMGMsg{kkg}.Pose.Position.Y IMGMsg{kkg}.Pose.Position.Z];
    IMGHeading(kkg,:) = [IMGMsg{kkg}.Pose.Orientation.X IMGMsg{kkg}.Pose.Orientation.Y IMGMsg{kkg}.Pose.Orientation.Z IMGMsg{kkg}.Pose.Orientation.W];
    Temp = IMGHeading(kkg,:);
    EulTemp = qua2eul(Temp);
    
    % DCMTemp = qua2dcm(Temp);
    % Xaixs = DCMTemp*[0;0;1];
    % HeadingEul(kkg,1) = atan2(Xaixs(2),Xaixs(3));
    % Yaixs = DCMTemp*[1;0;0];
    % HeadingEul(kkg,2) = atan2(Yaixs(3),Yaixs(1));
    % Zaixs = DCMTemp*[0;1;0];
    % HeadingEul(kkg,3) = atan2(Yaixs(1),Yaixs(2));
    % 
    % RotMat_diff = (qua2dcm(IMGHeading(kkg,:)))/(qua2dcm(IMGHeading(kkg-1,:)));
    % HeadingEul2(kkg,:) = dcm2eulr(RotMat_diff);
    % 
    % if Temp(1) ~= 0
    %     if (any(any(IMGHeadingInit == 0))) || (any(abs(IMGHeading(kkg,:)-IMGHeading(kkg-1,:)) > 1))
    %         DAinit = qua2dcm(Temp);
    %         IMGHeadingInit = DAinit';
    %     end
    % end
    % DCMH(kkg,:) = quat2eul(Temp);
end
% HeadingEulDiff = HeadingEul(2:end,:)-HeadingEul(1:end-1,:);
% 
% figure(121312);hold off;plot3(IMGpos(:,1),IMGpos(:,3),-IMGpos(:,2),'.');axis equal
% figure(121313);hold off;plot(IMGpos(:,1),'r.');hold on;plot(IMGpos(:,3),'b.');
% figure(12134);hold off;plot(DCMH(:,1),'r.');hold on;plot(DCMH(:,2),'g.');plot(DCMH(:,3),'b.');
% figure(12135);hold off;plot(HeadingEul2(:,1),'r.');hold on;plot(HeadingEul2(:,2),'g.');plot(HeadingEul2(:,3),'b.');
% figure(12136);hold off;plot(HeadingEul(:,1),'r.');hold on;plot(HeadingEul(:,2),'g.');plot(HeadingEul(:,3),'b.');
% figure(12137);hold off;plot(HeadingEulDiff(:,1),'r.');hold on;plot(HeadingEulDiff(:,2),'g.');plot(HeadingEulDiff(:,3),'b.');
% 
% figure(123145);plot(mod(cumsum(HeadingEul2),2*pi),'.')

