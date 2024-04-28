function T = T_from_Pq(pq)
%pq: tx ty tz qx qy qz qw
T = eye(4);
T(1:3, 1:3) = quat2rotm([pq(7), pq(4:6)]);
T(1:3, 4) = pq(1:3)';
end
