function pq = Pq_from_T(Tmat)
% return tx ty tz qx qy qz qw
pq = zeros(1, 7);
pq(1:3) = Tmat(1:3, 4)';
qwxyz = rotm2quat(Tmat(1:3, 1:3));
pq(4:6) = qwxyz(2:4);
pq(7) = qwxyz(1);
end
