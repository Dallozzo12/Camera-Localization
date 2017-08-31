function res=eul2rot(angle)

R_x=[1 0 0; 0 cos(angle(1)) -sin(angle(1)); 0 sin(angle(1)) cos(angle(1))];
R_y=[cos(angle(2)) 0 sin(angle(2)); 0 1 0; -sin(angle(2)) 0 cos(angle(2))];
R_z=[cos(angle(3)) -sin(angle(3)) 0; sin(angle(3)) cos(angle(3)) 0; 0 0 1];

res=R_z*R_y*R_x;

end