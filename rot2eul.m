function eul = rot2eul(m)
%#codegen

roll = atan2(m(3,2), m(3,3));
pitch = atan2(-m(3,1), sqrt( m(3,2)^2 + m(3,3)^2) );
yaw = atan2(m(2,1), m(1,1));

eul = [roll pitch yaw];
end