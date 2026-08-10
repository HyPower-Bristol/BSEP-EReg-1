function valve_area = fcn(valve_angle)
%     % BUG RECREATION: hardware has angle convention inverted vs sim
%     % Sim convention was: 0°=closed, 90°=open
%     % Hardware reality:   0°=open,   90°=closed
%     % Equivalent: feed (90 - valve_angle) through the same polynomial
%     inverted_angle = 90 - valve_angle;
%     inverted_angle = inverted_angle - 5;  % keep the same deadband
%     if inverted_angle < 0
%         valve_area = 0;
%     else
%         valve_area = 1.1*((5*10^-6)*inverted_angle^3 - 0.0006*inverted_angle^2 + 0.0278*inverted_angle);
%     end
% end

valve_angle = valve_angle - 5; %Account for first part of rotation where valve doesn't open at all
if valve_angle < 0
    valve_area = 0;
else
    valve_area = 1.1*((5*10^-6)*valve_angle^3 - 0.0006*valve_angle^2 + 0.0278*valve_angle); %calculate normalised valve area
end

