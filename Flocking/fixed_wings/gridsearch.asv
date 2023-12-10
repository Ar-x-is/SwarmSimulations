% Grid-searching tuning params for the proposed flocking law
% 1 - gradient
% 2 - consensus
% 3 - turning

nn = 51;

gg = ones(nn, 1);
cc = ones(nn, 1);
tt0 = ones(nn, 1);
tt5 = 5*ones(nn, 1);
tt10 = 10*ones(nn, 1);

numberOfCollisions = zeros(nn, 3);

xx = linspace(1, nn, nn);

for ii = 1:nn
    gradient_coeff = gg(ii);
    consensus_coeff = cc(ii);
    turning_coeff = tt0(ii);

    fixed_wing_testing;

    numberOfCollisions(ii, 1) = num_collisions;
end

for ii = 1:nn
    gradient_coeff = gg(ii);
    consensus_coeff = cc(ii);
    turning_coeff = tt5(ii);

    fixed_wing_testing;

    numberOfCollisions(ii, 2) = num_collisions;
end

for ii = 1:nn
    gradient_coeff = gg(ii);
    consensus_coeff = cc(ii);
    turning_coeff = tt10(ii);

    fixed_wing_testing;

    numberOfCollisions(ii, 3) = num_collisions;
end

disp(max(numberOfCollisions))

lower_bin = min(min(numberOfCollisions));
upper_bin = max(max(numberOfCollisions));
bin_array = linspace(lower_bin, upper_bin, 11);

clf;
hold on;
histogram(numberOfCollisions(:,1), bin_array, 'FaceAlpha', 0.4);
histogram(numberOfCollisions(:,2), bin_array, 'FaceAlpha', 0.4);
histogram(numberOfCollisions(:,3), bin_array, 'FaceAlpha', 0.4);
legend('turning = 0', 'turning = 5', 'turning = 10'); 