function rotated_vector = rotate(vector, angle)
    rotated_vector = zeros(size(vector));
    % angle in radians
    for i = 1:size(angle,1)
        M = [cos(angle(i)), sin(angle(i)); -sin(angle(i)), cos(angle(i))];
        rotated_vector(i,:) = (M * vector(i,:)')';
    end
end