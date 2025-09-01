function out = quat_utils(cmd, varargin)

switch cmd
    case 'mul'  % q = mul(q1,q2)
        q1 = varargin{1}(:)'; q2 = varargin{2}(:)';
        w1=q1(1); x1=q1(2); y1=q1(3); z1=q1(4);
        w2=q2(1); x2=q2(2); y2=q2(3); z2=q2(4);
        out = [w1*w2 - x1*x2 - y1*y2 - z1*z2, ...
               w1*x2 + x1*w2 + y1*z2 - z1*y2, ...
               w1*y2 - x1*z2 + y1*w2 + z1*x2, ...
               w1*z2 + x1*y2 - y1*x2 + z1*w2]';
    case 'norm' % q_unit = norm(q)
        q = varargin{1}; out = q./norm(q);
    case 'fromSmallAngle' % dq from angular velocity*dt (rad)
        dth = varargin{1}(:);
        ang = norm(dth);
        if ang < 1e-12
            out = [1; 0; 0; 0];
        else
            axis = dth/ang;
            out = [cos(ang/2); axis*sin(ang/2)];
        end
    case 'rotm' % R = rotm(q)
        q = quat_utils('norm', varargin{1});
        w=q(1); x=q(2); y=q(3); z=q(4);
        out = [1-2*(y^2+z^2), 2*(x*y - z*w), 2*(x*z + y*w);
               2*(x*y + z*w), 1-2*(x^2+z^2), 2*(y*z - x*w);
               2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x^2+y^2)];
    case 'rotate' % v_world = rotate(q, v_body)
        q = quat_utils('norm', varargin{1});
        v = varargin{2}(:);
        R = quat_utils('rotm', q); out = R*v;
    case 'yaw' % heading from R
        q = quat_utils('norm', varargin{1});
        R = quat_utils('rotm', q);
        out = atan2(R(2,1), R(1,1));
    otherwise
        error('Unknown cmd');
end
