%Created by Kriston Rickman
%Date created 07/06/26
%V1.00
%Energy based constraint analysis
%Note:
%/* Rough idea placed on matlab with one constraints using 
% custom design aircraft that is built similar to the 
% Cirrus SR 20 which is why we assumed coefficentis and other
% variables based on its own data*\

clearvars
Gravity = 9.81;
P_SL = 1.225;
P_cruise = 0.905;
S_G = 502.92;
K_TO = 1.2;
Velocity_TO = 29.323;
Cruise_velocity = 74.594;
C_Dmin = 0.027; % Assumption from Cirrus sr 20
C_Lmin = 0.3; % Assumption from Cirrus sr 20
Sweep_angle = 3; % Made from assumption
Mass_without_wing_skin = 780; % assume based on aircraft type
Wing_mass_per_volume = 2700; % assumption of mass of GA aluminum
Wing_skin_thickness = 0.002; % assumption of mass of GA aluminum
Rolling_friction_coefficient = 0.03;
Propeller_efficiency_TO = 0.75; % assume based on propeller
HP_to_watts = 745.7;
% NOTE Measuremnt in HP will be convert to Watts
Engine_power_HP = [190, 200, 210]; % Assumption of Eninge need data
Span = [11.8, 11.4, 11]; % assume based on aircraft type
Wing_area = [15.5, 14.4, 13.3]; % assume based on aircraft type

% Engine Characteristics
Engine_power_watts = Engine_power_HP .* HP_to_watts;
Fuel_spent_ground_to_TO = 0.11;
Fuel_mass_per_gallon = 4.5359; % ARD
Weight_loss_on_TO = Fuel_spent_ground_to_TO .* Fuel_mass_per_gallon;
Thrust_TO = (Propeller_efficiency_TO .* Engine_power_watts) ./ Velocity_TO;

% Wing calculation
AR_wing = Span.^2 ./ Wing_area;
Wing_skin_area_total = 2 .* Wing_area;
Wing_skin_volume = Wing_skin_area_total .* Wing_skin_thickness;
Wing_skin_mass = Wing_skin_volume .* Wing_mass_per_volume;
Mass_aircraft = Mass_without_wing_skin + Wing_skin_mass; % assume based on aircraft type
Mass_TO = Mass_aircraft - Weight_loss_on_TO;
Weight_TO = Mass_TO * Gravity;
Wing_loading = Weight_TO ./ Wing_area;
C_L_required_TO = 2 .* Weight_TO ./(P_SL .* Velocity_TO.^2 .* Wing_area);
C_Lmax_TO = 1.7; % Assumption including flaps, elevator, and wing
Lift_TO = 0.5 .* C_Lmax_TO .* P_SL .* Wing_area .* Velocity_TO.^2;
C_Lcruise = 2 .*Weight_TO./(P_cruise .* Cruise_velocity.^2 .* Wing_area);
B = Mass_TO./Mass_aircraft;

% Sweep Angle Calculation Decision
if Sweep_angle == 0
    e = 1.78.*(1-0.045.*(AR_wing).^0.68) - 0.64;
elseif Sweep_angle >= 30
    e = 4.61.*(1-0.045.*(AR_wing).^0.68).*(cosd(Sweep_angle));
elseif (Sweep_angle > 0) && (Sweep_angle < 30)
    e0 = 1.78.*(1-0.045.*(AR_wing).^0.68) - 0.64;
    e30 = 4.61.*(1-0.045.*(AR_wing).^0.68).*(cosd(Sweep_angle));
    e = e0 + (Sweep_angle./30).*(e30 - e0);
else
    error("Sweep angle is out of range")
end

% Full Drag Polar Buildup
K_1 = 1./(pi.*AR_wing.*e);

C_D0 = C_Dmin + K_1.*C_Lmin.^2;

K_2 = -2.*K_1.*C_Lmin;

C_DTO = C_D0 + (K_1.*C_Lmax_TO.^2) + K_2.*C_Lmax_TO;

C_Dcruise = C_D0 + (K_1.*C_Lcruise.^2) + K_2.*C_Lcruise;

Drag_TO = 0.5 .* C_DTO .* P_SL .* Wing_area .* Velocity_TO.^2;

% Equation for Constraints
Acceleration_TO = Velocity_TO.^2./(2 .* S_G);


Thrust_to_Weight_TO = (B.^2./Acceleration_TO) .* (K_TO.^2./(S_G.*P_SL.*Gravity.*C_Lmax_TO)).*Wing_loading; % Takeoff Constraint


Acceleration_horizontal = (Thrust_TO-Drag_TO-Rolling_friction_coefficient .* (Weight_TO - Lift_TO))./Mass_TO; % Horizontal acceleration

figure;
scatter(Wing_loading, Thrust_to_Weight_TO, 'filled');
xlabel('Wing Loading (N./m.^2)');
ylabel('Thrust to Weight Ratio (N./W)');
title('Takeoff Constraint: T/W vs Wing Loading');
grid on;