% plotting sensor hemisphere, base frame, and contact frame

% sensor radius
r = 10; % in mm

T_W = eye(4);
T_contact_init = [eye(3),[0;0;r];0,0,0,1];

theta_deg = 80;
phi_deg = 35;

T_contact_angles = [rotx(180,'deg')*roty(phi_deg,'deg')*rotx(theta_deg,'deg'), zeros(3,1); 0,0,0,1];

T_contact = T_contact_angles*T_contact_init;

[Xs,Ys,Zs] = sphere;
Xs = r*Xs; Ys = r*Ys; Zs = r*Zs;

figure(2); clf; hold on;
surf(Xs,Zs,Ys);
colormap(gray(1));
alpha 0.15

% base frame
sc = 10;
quiver3(0,0,0,1,0,0,sc,'r','LineWidth',2.5);
quiver3(0,0,0,0,1,0,sc,'g','LineWidth',2.5);
quiver3(0,0,0,0,0,1,sc,'b','LineWidth',2.5);
% contact frame
quiver3(T_contact(1,4),T_contact(2,4),T_contact(3,4),T_contact(1,1),T_contact(2,1),T_contact(3,1),sc,'r','LineWidth',2.5);
quiver3(T_contact(1,4),T_contact(2,4),T_contact(3,4),T_contact(1,2),T_contact(2,2),T_contact(3,2),sc,'g','LineWidth',2.5);
quiver3(T_contact(1,4),T_contact(2,4),T_contact(3,4),T_contact(1,3),T_contact(2,3),T_contact(3,3),sc,'b','LineWidth',2.5);

title(['Contact frame at \theta = ',num2str(theta_deg), '\phi = ',num2str(phi_deg)]);


grid on
axis equal
xlim([-15,15]); ylim([-15,15]); zlim([-15,15]);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
view(30,30);