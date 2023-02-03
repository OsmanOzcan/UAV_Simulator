
function output = Quaternion2Euler(quaternion)
    % converts a quaternion attitude to an euler angle attitude
    ee(1) = quaternion(1);
    ee(4) = quaternion(2);
    ee(3) = quaternion(3);
    ee(2) = quaternion(4);

    output = quat2eul(ee);
end
