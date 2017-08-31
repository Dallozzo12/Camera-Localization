function eul = quat2eul(qwxyz)

qw=qwxyz(1);
qx=qwxyz(2);
qy=qwxyz(3);
qz=qwxyz(4);

qx2=qx*qx;
qy2=qy*qy;
qz2=qz*qz;

m = [1-2*qy2-2*qz2      2*qx*qy-2*qz*qw     2*qx*qz+2*qy*qw;
     2*qx*qy+2*qz*qw    1-2*qx2-2*qz2       2*qy*qz-2*qx*qw;
     2*qx*qz-2*qy*qw    2*qy*qz+2*qx*qw     1-2*qx2-2*qy2];
%m = quat2rotm(q);

roll = atan2(m(3,2), m(3,3));
pitch = atan2(-m(3,1), sqrt( m(3,2)^2 + m(3,3)^2) );
yaw = atan2(m(2,1), m(1,1));

eul = [roll pitch yaw];