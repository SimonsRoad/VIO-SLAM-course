clear 
close all




dt = dlmread('./data_imu_sim1_gyr_t.txt');         
data_x = dlmread('./data_imu_sim1_gyr_x.txt'); 
data_y= dlmread('./data_imu_sim1_gyr_y.txt'); 
data_z = dlmread('./data_imu_sim1_gyr_z.txt'); 
data_draw=[data_x data_y data_z] ;
data_draw = data_draw/3600.0;

data_sim_x= dlmread('./data_imu_sim1_sim_gyr_x.txt'); 
data_sim_y= dlmread('./data_imu_sim1_sim_gyr_y.txt'); 
data_sim_z= dlmread('./data_imu_sim1_sim_gyr_z.txt'); 
data_sim_draw= (data_sim_x + data_sim_y+ data_sim_z)/3;
data_sim_draw = data_sim_draw/3600.0;


figure
loglog(dt, data_draw , 'o');
% loglog(dt, data_sim_draw , '-');
xlabel('time:sec');                
ylabel('Sigma:red/s');             
% legend('x','y','z');      
grid on;                           
hold on;                           
loglog(dt, data_sim_draw , '-');





dt = dlmread('./data_imu_sim2_gyr_t.txt');         
data_x = dlmread('./data_imu_sim2_gyr_x.txt'); 
data_y= dlmread('./data_imu_sim2_gyr_y.txt'); 
data_z = dlmread('./data_imu_sim2_gyr_z.txt'); 
data_draw=[data_x data_y data_z] ;
data_draw = data_draw/3600.0;

data_sim_x= dlmread('./data_imu_sim2_sim_gyr_x.txt'); 
data_sim_y= dlmread('./data_imu_sim2_sim_gyr_y.txt'); 
data_sim_z= dlmread('./data_imu_sim2_sim_gyr_z.txt'); 
data_sim_draw= (data_sim_x + data_sim_y+ data_sim_z)/3;
data_sim_draw = data_sim_draw/3600.0;


figure
loglog(dt, data_draw , 'b+');
% loglog(dt, data_sim_draw , 'o');
xlabel('time:sec');                
ylabel('Sigma:rad/s');             
% legend('x','y','z');      
grid on;                           
hold on;                           
loglog(dt, data_sim_draw , 'r-');
