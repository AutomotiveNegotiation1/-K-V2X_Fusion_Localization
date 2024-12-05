
firstV = 0;
load('MapParams2.mat');
for dfkg = 1 : length(SLAMtime)
    if (SLAMtime(dfkg)~=0) && (SLAMpos(dfkg,1)~=0)

        if firstV == 0
            prev_pos = transpose(SLAMpos(dfkg,:));
            % prev_pos = SLAMpos(dfkg,[1 3 2]);
            % prev_pos(3) = -prev_pos(3);
            prev_time = SLAMtime(dfkg);
            prev_quat = SLAMorient(dfkg,:);

            
            % SLAMposn(:,dfkg) = prev_pos;

            RotMatn = eye(3);
            SLAMposn(:,dfkg) = zeros(3,1);

            RotMatn = qua2dcm(prev_quat);

            % TE = quat2eul(prev_quat);
            % TE(3) = 0;
            % TE(2) = 0;
            % 
            % RotMatnInit = inv(eulr2dcm(TE));
            % RotMatnInit = inv(qua2dcm(prev_quat));
            RotMatnInit = eye(3);
            % SLAMposn(:,dfkg) = prev_pos;

            
            SLAMposPrev = SLAMposn(:,dfkg);
            firstV = 1;
        else
            curr_pos = transpose(SLAMpos(dfkg,:));
            % curr_pos = SLAMpos(dfkg,[1 3 2]);
            % curr_pos(3) = -curr_pos(3);
            curr_time = SLAMtime(dfkg);
            curr_quat = SLAMorient(dfkg,:);

            RotMat = RotMatnInit*qua2dcm(prev_quat);
            prev_pos_org = RotMat'*(prev_pos);
            curr_pos_org = RotMat'*(curr_pos);
            pos_diff_org(:,dfkg) = curr_pos_org - prev_pos_org;
            
            if sqrt(mean(abs(pos_diff_org(:,dfkg)).^2))<0.5
                RotMat_diff = (qua2dcm(curr_quat))/(qua2dcm(prev_quat));
            
                SLAMposn(:,dfkg) = SLAMposPrev + RotMatn*pos_diff_org(:,dfkg);
                RotMatn = RotMat_diff*RotMatn;
            else
                % firstV = 0;
                SLAMposn(:,dfkg) = SLAMposPrev;
                RotMatn = qua2dcm(curr_quat);
                
                % TE = quat2eul(curr_quat);
                % TD = dcm2eulr(RotMatn);
                % TE(2) = TD(2);
                % RotMatn = eulr2dcm(TE);
                % TE = quat2eul(curr_quat);
                % TE(2) = 0;
                % TE(3) = 0;
                % 
                % RotMatnInit = inv(eulr2dcm(TE));

                % RotMatnInit = inv(qua2dcm(curr_quat));
                RotMatnInit = eye(3);
                % RotMatn = eye(3);
            end

            SLAMposPrev = SLAMposn(:,dfkg) ;

            prev_pos = transpose(SLAMpos(dfkg,:));
            % prev_pos = SLAMpos(dfkg,[1 3 2]);
            % prev_pos(3) = -prev_pos(3);
            prev_time = SLAMtime(dfkg);
            prev_quat = SLAMorient(dfkg,:);
        end
    end
end

figure(7777);hold off;plot3(SLAMposn(1,:),SLAMposn(3,:),SLAMposn(2,:),'b.');
figure(7778);hold off;plot(SLAMposn(1,:),'b.');

figure(8889);hold off;plot(angular_rate(:,1),'r.');
hold on;plot(angular_rate(:,2),'g.');
hold on;plot(angular_rate(:,3),'b.');

figure(8890);hold off;plot(IMU_tot(:,1),'r.');
hold on;plot(IMU_tot(:,2),'g.');
hold on;plot(IMU_tot(:,3),'b.');

figure(8891);hold off;plot(IMU_tot(:,4),'r.');
hold on;plot(IMU_tot(:,5),'g.');
hold on;plot(IMU_tot(:,6),'b.');

figure(8892);hold off;plot(pos_diff_org(1,:),'r.');
hold on;plot(pos_diff_org(2,:),'g.');
hold on;plot(pos_diff_org(3,:),'b.');
