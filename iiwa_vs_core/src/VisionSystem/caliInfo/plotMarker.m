clear all;
%m2r = load('./olddata/marker2robot_straight.txt');
%m2c = load('./olddata/marker2camera_straight.txt');
m2r = load('..\data\marker2robot.txt');
m2c = load('..\data\marker2camera.txt');
ee2r = load('..\data\eePose.txt');
markerInEE_handeye = load('../data/ToolLInEE_new.txt')

nbm2r = size(m2r,1)/4;
nbm2c = size(m2c,1)/4;
if nbm2r > nbm2c
	m2r(end-3:end,:) = [];
    ee2r(end-3:end,:) = [];
	nbm2r = size(m2r,1)/4;
end

% Reduce number of points ---------------
step = 8 * 4;
% ---------------------------------------

m2r = [];
for i=1:nbm2c
    m2r(4*(i-1)+1:4*(i-1)+4,1:4) = ee2r(4*(i-1)+1:4*(i-1)+4,1:4) * markerInEE_handeye;
end

left_m2r = [m2r(1:step:end,4) m2r(2:step:end,4) m2r(3:step:end,4)]';
right_m2c = [m2c(1:step:end,4) m2c(2:step:end,4) m2c(3:step:end,4)]';
nbm2c = size(right_m2c,2);

for i=nbm2c:-1:1
	if or(norm(right_m2c(:,i))<1E-5, norm(left_m2r(:,i))<1E-5)
		right_m2c(:,i) = [];
		left_m2r(:,i) = [];
		m2r(i*4-3:i*4,:) =[];
		m2c(i*4-3:i*4,:) =[];
	end
end
nbm2c = size(right_m2c,2);
display ([int2str(nbm2c) ' points!'])


trans = absoluteOrientation(left_m2r, right_m2c)


err = 0;
for i=1:nbm2c
	tmp = trans * m2r((i-1)*step+1:(i-1)*step+4,:) ;
	m2r_trans((i-1)*4+1:i*4,:) = tmp;
 	err = err + norm([m2c((i-1)*step+1:(i-1)*step+3,4)] - m2r_trans((i-1)*4+1:i*4-1,4));
end


%% 
display ([int2str(nbm2r) ' points!']);
display (['error per point is ' num2str(err/size(right_m2c,2)) ' !']);
figure; hold on;
grid on; xlabel('x'); ylabel('y'); zlabel('z');
plot3(m2r_trans(1:4:end,4), m2r_trans(2:4:end,4), m2r_trans(3:4:end,4), 'r-*');
plot3(right_m2c(1,:), right_m2c(2,:), right_m2c(3,:),'g-*');
%plot3(m2r(1:4:end,4), m2r(2:4:end,4), m2r(3:4:end,4),'b*');
legend('trans', 'm2c');


% figure; hold on;
% grid on; xlabel('x'); ylabel('y'); zlabel('z');
% plot3(left_m2r(1,:), left_m2r(2,:), left_m2r(3,:), 'r*-');
% title('m2r_trans');
% 
% figure; hold on;
% grid on; xlabel('x'); ylabel('y'); zlabel('z');
% plot3(right_m2c(1,:), right_m2c(2,:), right_m2c(3,:),'g*-');
% title('m2c');


