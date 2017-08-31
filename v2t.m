%from 6d vector to homogeneous matrix
function T=v2t(v)
    T=eye(4);
    T(1:3,1:3)=rx(v(4))*ry(v(5))*rz(v(6));
    T(1:3,4)=v(1:3);
end