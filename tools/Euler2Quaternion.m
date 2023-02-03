
function quaternion = Euler2Quaternion(phi, theta, psi)
    % converts euler angles to a quaternion
    ee = eul2quat([phi, theta, psi]);
    e0 = ee(1);
    e1 = ee(4);
    e2 = ee(3);
    e3 = ee(2);

    quaternion = [e0; e1; e2; e3];
end
