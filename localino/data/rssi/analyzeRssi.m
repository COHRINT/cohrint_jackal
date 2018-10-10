
clf; close all;

col = 6;
 
u = mean( [ hor_hor_rssi(:,col); vert_hor_rssi(:,col); vert_vert_rssi(:,col) ] ) ;
sigma = std( [ hor_hor_rssi(:,col); vert_hor_rssi(:,col); vert_vert_rssi(:,col) ] ) ;

z_hor_hor = ( hor_hor_rssi(:,col) );
scatter3( hor_hor_rssi(:,1), hor_hor_rssi(:,2), hor_hor_rssi(:,3), [], z_hor_hor) ;
title('Horizontal, Horizontal Signal Quality') ;
xlabel('x') ;
ylabel('y') ;
zlabel('z') ;
xlim([-6, 6]);
ylim([-3,3]);
zlim([0, 2]);
colormap('autumn')
colorbar()
caxis([0 550])

figure;
z_vert_hor = ( vert_hor_rssi(:,col)  );
scatter3( vert_hor_rssi(:,1), vert_hor_rssi(:,2), vert_hor_rssi(:,3), [], z_vert_hor) ;
title('Vertical, Horizontal Signal Quality')
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-6, 6]);
ylim([-3,3]);
zlim([0, 2]);
colormap('autumn')
colorbar
caxis([0 550])

figure;
z_vert_vert = ( vert_vert_rssi(:,col) );
scatter3( vert_vert_rssi(:,1), vert_vert_rssi(:,2), vert_vert_rssi(:,3), [], z_vert_vert) ;
title('Vertical, Vertical Signal Quality')
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-6, 6]);
ylim([-3,3]);
zlim([0, 2]);
colormap('autumn')
colorbar
caxis([0 550])


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% RX POWER %%%%%%%%%%%%%%%%%%%%%%%%%%%

figure;
col = 5;

u = mean( [ hor_hor_rssi(:,col) ; vert_hor_rssi(:,col) ; vert_vert_rssi(:,col) ] ) ;

z_hor_hor = ( hor_hor_rssi(:,col)  );
scatter3( hor_hor_rssi(:,1), hor_hor_rssi(:,2), hor_hor_rssi(:,3), [], z_hor_hor) ;
title('Horizontal, Horizontal Receive Power')
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-6, 6]);
ylim([-3,3]);
zlim([0, 2]);
colormap('autumn')
colorbar
caxis([-100 -60])

figure;
z_vert_hor = ( vert_hor_rssi(:,col)  );
scatter3( vert_hor_rssi(:,1), vert_hor_rssi(:,2), vert_hor_rssi(:,3), [], z_vert_hor) ;
title('Vertical, Horizontal Receive Power')
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-6, 6]);
ylim([-3,3]);
zlim([0, 2]);
colormap('autumn')
colorbar
caxis([-100 -60])

figure;
z_vert_vert = ( vert_vert_rssi(:,col) );
scatter3( vert_vert_rssi(:,1), vert_vert_rssi(:,2), vert_vert_rssi(:,3), [], z_vert_vert) ;
title('Vertical, Vertical Receive Power')
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-6, 6]);
ylim([-3,3]);
zlim([0, 2]);
colormap('autumn')
colorbar
caxis([-100 -60])