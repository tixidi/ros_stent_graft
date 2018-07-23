% This script do the hand-eye calibration 

% Read data from file
% File format:
% Camera x	y	z(mm)	Rz 	Ry	Rx(degree)	Robot x	y z(mm)	 A	B C(degree)
% .20160509.toolLs
clear variables;
data_cam = load('./data/marker2camera.txt.20160509.toolLs');
data_rob = load('./data/eePose.txt.20160509.toolLs');
nbPose_c = size(data_cam, 1)/4;
nbPose_r = size(data_rob, 1)/4;
if nbPose_r > nbPose_c
	data_rob(end-3:end,:) = [];	
end
nbPose = size(data_rob,1)

%% ---------------------------------------------------------------------
ind = 0;
step = 3 * 4;
for i=1:step:nbPose
    if (norm(data_cam(i:i+2, 4)) > 1E-5)
        ind = ind + 1;
        marker2camera(1:4,1:4,ind) = data_cam(i:i+3,1:4);
        ee2robot(1:4,1:4,ind) = data_rob(i:i+3,1:4);
        
        tmp = trans2posrodr(marker2camera(:,:,ind));
        m2c(ind,1:3) = tmp(1:3);
        m2c_rot(ind, 1:3) = tmp(4:6);
        
        tmp = trans2posrodr(ee2robot(:,:,ind));
        ee2r(ind,1:3) = tmp(1:3);
        ee2r_rot(ind,1:3) = tmp(4:6);
        
    end
end
display ([int2str(ind) ' points!']);
%clear data_cam data_rob;

%%
marker2ee = handEye(ee2robot(:,:,1:end), marker2camera(:,:,1:end))


%% Plot
cc = hsv(3);
figure;hold on;grid on;
for i=1:3
    plot(ee2r_rot(:,i),'color', cc(i,:));
end
figure;hold on; grid on;
for i=1:3
    plot(m2c_rot(:,i),'color', cc(i,:));
end


