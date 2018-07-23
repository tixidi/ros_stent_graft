clear all;
%m2r = load('./olddata/marker2robot_straight.txt');
%m2c = load('./olddata/marker2camera_straight.txt');
m2r = load('marker2robot.txt');
m2c = load('marker2camera.txt');

nbm2r = size(m2r,1)/4;
nbm2c = size(m2c,1)/4;
if nbm2r > nbm2c
	m2r(end-3:end,:) = [];
	nbm2r = size(m2r,1)/4;
end

left_m2r = [m2r(1:4:end,4) m2r(2:4:end,4) m2r(3:4:end,4)]';
right_m2c = [m2c(1:4:end,4) m2c(2:4:end,4) m2c(3:4:end,4)]';
nbm2r = size(m2r,1)/4;

for i=nbm2r:-1:1
	if (right_m2c(:,i) == [0; 0; 0])
		right_m2c(:,i) = [];
		left_m2r(:,i) = [];
		m2r(i*4-3:i*4,:) =[];
		m2c(i*4-3:i*4,:) =[];
	end
end
nbm2r = size(right_m2c,2);
display ([int2str(nbm2r) ' points!'])


trans = absoluteOrientation(left_m2r, right_m2c)


err = 0;
for i=1:nbm2r
	tmp = trans * m2r((i-1)*4+1:i*4,:) ;
	m2r_trans((i-1)*4+1:i*4,:) = tmp;
 	err = err + norm([m2c((i-1)*4+1:i*4-1,4)] - m2r_trans((i-1)*4+1:i*4-1,4));
    
end


display (['error per point is ' num2str(err/size(right_m2c,2)) ' !']);
figure; hold on;
grid on; xlabel('x'); ylabel('y'); zlabel('z');
plot3(m2r_trans(1:4:end,4), m2r_trans(2:4:end,4), m2r_trans(3:4:end,4), 'r-*');
plot3(m2c(1:4:end,4), m2c(2:4:end,4), m2c(3:4:end,4),'g-*');
%plot3(m2r(1:4:end,4), m2r(2:4:end,4), m2r(3:4:end,4),'b*');
legend('trans', 'm2c');


figure; hold on;
grid on; xlabel('x'); ylabel('y'); zlabel('z');
plot3(left_m2r(1,:), left_m2r(2,:), left_m2r(3,:), 'r*-');
title('m2r_trans');

figure; hold on;
grid on; xlabel('x'); ylabel('y'); zlabel('z');
plot3(right_m2c(1,:), right_m2c(2,:), right_m2c(3,:),'g*-');
title('m2c');


