%% Track Kalman function

function [xh, yh, zh, count] = TrackKalman3d(xm, ym, zm, i, count)
%
%
persistent A H Q R
persistent x P
persistent firstRun

num = 33;

if count == 1
    A = zeros(6,6,num);
    H = zeros(3,6,num);
    Q = zeros(6,6,num);
    R = zeros(3,3,num);
    x = zeros(6,1,num);
    P = zeros(6,6,num);
    firstRun = zeros(1,num);
end

if firstRun(1,i) == 0
    dt = 1/20;
    
    A(:,:,i) = [1 dt 0 0 0 0; 0 1 0 0 0 0; 0 0 1 dt 0 0; 0 0 0 1 0 0; 0 0 0 0 1 dt; 0 0 0 0 0 1];
    H(:,:,i) = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];
    
    Q(:,:,i) = 5*eye(6); % 1.0*eye(6): original value, 5
    R(:,:,i) = 80*eye(3); % 50*eye(3): original value, 80
    
    x(:,:,i) = [0 0 0 0 0 0]';
    P(:,:,i) = 100*eye(6);
    
    firstRun(1,i) = 1;
end

xp = A(:,:,i)*x(:,:,i);
Pp = A(:,:,i)*P(:,:,i)*A(:,:,i)' + Q(:,:,i);

K = Pp*H(:,:,i)'*inv(H(:,:,i)*Pp*H(:,:,i)' + R(:,:,i));

z = [xm ym zm]';
x(:,:,i) = xp + K*(z - H(:,:,i)*xp);
P(:,:,i) = Pp - K*H(:,:,i)*Pp;

xh = x(1,1,i);
yh = x(3,1,i);
zh = x(5,1,i);
count = count + 1;

end