function T = OculiiToBody(comment)
% The transform of the oculii point cartesian frame relative to the body frame.
if nargin < 1
    comment = 'refined';
end
switch comment
case 'manual'
    R_body_oculii = eye(3);
    p_body_oculii = [0.07; 0; -0.115];
    T = [R_body_oculii p_body_oculii; 0, 0, 0, 1];

otherwise % refined
    R_body_oculii = eye(3);
    p_body_oculii = [0.07; 0; -0.115];
    T = [R_body_oculii p_body_oculii; 0, 0, 0, 1];
end
end
