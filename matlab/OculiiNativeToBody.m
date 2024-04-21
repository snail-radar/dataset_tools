function T = OculiiNativeToBody(comment)
% The transform of the oculii's native frame relative to the body frame.
if nargin < 1
    comment = 'refined';
end
T_pointcart_native = eye(4);
T_pointcart_native(1:3, 1:3) = [0, 0, 1; -1, 0, 0; 0, -1, 0];
T = OculiiToBody(comment) * T_pointcart_native;
end
