%Created by Kriston Rickman
%Date created 07/06/26
%V1.12
%Energy based constraint analysis
%Note:

%/*Goals*\

%/*simple concept of the takeoff constraint. formulas have been double
% checked and have changed base on the developing knowledge on the
% matter.*\

clearvars

%What the user wants to do

%user_path = user_state()
user_path = 1; % Delete when furhter advancing user inputs

switch user_path

    case 0
        state = "Find T/W and Wing laoding from past value"; % You plug in the value you were looking for then after looking at the grtaph and ifnding the numbers, you enter this state and type in the number and it rounds to the nearest values that it was computing of its related data.

    case 1
        state = "Find T/W";

    case 2
        state = "find_hp";

    case 3
        state = "find_wing_area";

    case 4
        state = "find_weight";


    case 5
        state = "find_span";

    case 6
        state = "explore_aircraft";


    otherwise
        disp("Invalid choice");
end



%Initial assumptions of aircraft design

Gravity = 9.81;

rho_SL = 1.225;

Cruise_velocity = 74.594;

C_Dmin = 0.027; % Assumption from Cirrus sr 20
C_Lmin = 0.3; % Assumption from Cirrus sr 20

Sweep_angle = 3; % Made from assumption

HP_to_watts = 745.7;

%The Design of the aircraft inputs
%NOTE Measuremnt in HP will be convert to Watts

Engine_power_HP = 230; % Assumption of Eninge need data
Span = 9; % assume based on aircraft type
Wing_area = 8:0.1:16; % assume based on aircraft type

Mass_without_wing_skin = 780; % assume based on aircraft type
Wing_mass_per_volume = 2700; % assumption of mass of GA aluminum
Wing_skin_thickness = 0.002; % assumption of mass of GA aluminum

% Engine Characteristics calculations

Engine_power_watts = Engine_power_HP .* HP_to_watts;

%Mass of the aircraft and related Geometry calculations

Fuel_spent_ground_to_TO = 0.11;
Fuel_mass_per_gallon = 2.8; % ARD
mass_loss_on_TO = Fuel_spent_ground_to_TO .* Fuel_mass_per_gallon;

Wing_skin_area_total = 2 .* Wing_area;
Wing_skin_volume = Wing_skin_area_total .* Wing_skin_thickness;
Wing_skin_mass = Wing_skin_volume .* Wing_mass_per_volume;

Mass_aircraft = Mass_without_wing_skin + Wing_skin_mass; % assume based on aircraft type
Mass_TO = Mass_aircraft - mass_loss_on_TO;

Weight_TO = Mass_TO * Gravity;
Wing_loading = Weight_TO ./ Wing_area;
AR_wing = Span.^2 ./ Wing_area;

%Aerodynamic calculations

    %Oswald efficiency calculations - Sweep angle calculation decision
if Sweep_angle == 0
    e = 1.78 .* (1 - 0.045 .* AR_wing.^0.68) - 0.64;
elseif Sweep_angle >= 30
    e = 4.61 .* (1 - 0.045 .* AR_wing.^0.68) .* (cosd(Sweep_angle)).^0.15 - 3.1;
elseif (Sweep_angle > 0) && (Sweep_angle < 30)
    e0 = 1.78 .* (1 - 0.045 .* AR_wing.^0.68) - 0.64;
    e30 = 4.61 .* (1 - 0.045 .* AR_wing.^0.68) .* (cosd(30)).^0.15 - 3.1;
    e = e0 + (Sweep_angle ./ 30) .* (e30 - e0);
else
    error("Sweep angle is out of range")
end
    
    %Full Drag Polar Buildup
K_1 = 1./(pi.*AR_wing.*e);

C_D0 = C_Dmin + K_1.*C_Lmin.^2;

K_2 = -2.*K_1.*C_Lmin;

%Takeoff constraint assumptions, variables and formulas
    %variables

    %Desired inputs
S_G = 502.92; % Ground roll takeoff distance
K_TO = 1.2;
C_Lmax_TO = 1.7; % Assumption including flaps, elevator, and wing
Rolling_friction_coefficient = 0.03;
Propeller_efficiency_TO = 0.75; % assume based on propeller

    %Velocity formulas
Velocity_stall = sqrt((2 .* Weight_TO) ./ (rho_SL .* Wing_area .* C_Lmax_TO));

Velocity_TO = K_TO * Velocity_stall;

Velocity_avg_TO = Velocity_TO ./ sqrt(2);

    %constraints sub-formulas
q_avg_TO = 0.5 .* rho_SL .* Velocity_avg_TO.^2;

C_L_required_TO = 2 .* Weight_TO ./(rho_SL .* Velocity_TO.^2 .* Wing_area);

%/*C_L_ground_TO For a more accurate constraint create a lift
%coefficient that is specific for ground because the ground 
%and takeoff coefficents are not the same*\

C_DTO = C_D0 + (K_1.*C_L_required_TO.^2) + K_2.*C_L_required_TO;

Lift_TO = 0.5 .* rho_SL .* Velocity_TO.^2 .* Wing_area .* C_L_required_TO;

Lift_avg_TO = 0.5 .* rho_SL .* Velocity_avg_TO.^2 .* Wing_area .* C_L_required_TO;

%/*The lift average take off will change based on the average dynamic
% pressure, lift coefficenets for the ground Take off and etc*\

Drag_TO = 0.5 .* C_DTO .* rho_SL .* Wing_area .* Velocity_TO.^2;

Drag_avg_TO = 0.5 .* rho_SL .* Velocity_avg_TO.^2 .* Wing_area .* C_DTO;

%/*Will also chagned when created ground drag coeffiecients for TO*\

Thrust_TO = (Propeller_efficiency_TO .* Engine_power_watts) ./ Velocity_TO;

Acceleration_TO = Velocity_TO.^2 ./ (2 .* S_G);

Thrust_to_Weight_TO = (Acceleration_TO ./ Gravity) + ... 
    (q_avg_TO .* C_DTO) ./ Wing_loading + Rolling_friction_coefficient ...
    .* (1 - (q_avg_TO .* C_L_required_TO) ./ Wing_loading); % Takeoff Constraint
%Creating scatter plot for takeoff constraint
figure;
plot(Wing_loading, Thrust_to_Weight_TO, '-');

hold on; %Stops from other plots from overriding the first



%Climb constraint

%/*This assumes no turns and constant climbing velocity, thus
% keeping the load factor (n) roughly around 1. Additionaly, the
% claculation assumes that there are no resistance such as landing
% gears or flaps that can have an inlfluence to drag are all not
% accounted for.*\

%Variables
alpha = 1; % assuming Simple sea-level climb

    %Aircraft desired Inputs
n = 1;
rate_of_climb = 3.5; %(meters per sec)
velocity_climb = 48;
rho_climb = 1.225;

%sub-formulas for the climb constraint

q_climb = 0.5 .* rho_climb .* velocity_climb.^2;

Beta = Mass_TO ./ Mass_aircraft; % Weight fraction

%A more specific version of Beta could have bee the mass while 
% climbing ./ Mass_TO

Thrust_to_Weight_climb = Beta/alpha .* (((K_1 .* n.^2 .* Beta) ./ ...
    q_climb) .* (Weight_TO ./ Wing_area) + K_2 .* n + C_D0 ./ ...
    ((Beta ./ q_climb) .* Wing_loading) + rate_of_climb ./ velocity_climb);


plot(Wing_loading, Thrust_to_Weight_climb, '-');



% Cruise constraint

%variable
knot_to_ms = 0.51444444;

%aircraft user inputs
rho_cruise = 0.905; % from the Aircraft Requirement Document (ARD)
velocity_cruise = 155 .* knot_to_ms;
%/*Will change in the future for user input when wanting to know
% the T/W for the desired cruise knots they want. Additionaly, will
% add the calculation of the cruise velocity when user does not have
% a desire velcoity and will be based on the parameter they place for
% the aircraft.*\

% sub-formulas for constraint

q_cruise = 0.5 .* rho_cruise .* velocity_cruise.^2;

C_Lcruise = Wing_loading ./ q_cruise;

C_D_cruise = C_D0 + (K_1 .* C_Lcruise.^2) + K_2 .* C_Lcruise;

% Calculate the thrust-to-weight ratio for cruise
Thrust_to_Weight_cruise = (q_cruise * C_D_cruise) ./ Wing_loading;

plot(Wing_loading, Thrust_to_Weight_cruise, '-');



%Turn constraint

%Variables

    %User desire input
rho_turn = 0.905;

%in meters of user idea of their aircraft turning
radius_turn = 300;


%Sub-formulas
velocity_stall_straight = sqrt((2 .* Wing_loading) ...
    ./ (rho_turn .* C_Lmax_TO)); %A straight line of aircraft stall velocity

%The guess of the safe turn velocity
velocity_guess_turn = K_TO .* velocity_stall_straight;

%Bank angle calculation from the guessed velocity and radius
bank_angle = atand(velocity_guess_turn.^2 ./ (radius_turn * Gravity));

%/*The start of the loop. We make it run
%through the loop to be accurate*\
n = 1;

for refine_loop = 1:1000

    n_old = n;
    
    velocity_stall_turn = sqrt((2 * Wing_loading .* n) ...
        ./ (rho_turn .* C_Lmax_TO));
    
    %The safe turn
    velocity_safe_turn = K_TO .* velocity_stall_turn;
    
    %Usign the new safe turn velocity we update the load factor
    update_load_factor = 1 ./ (cosd(atand(velocity_safe_turn.^2 ./  ...
        (radius_turn .* Gravity))));
    
    n = update_load_factor;

    if max(abs(n-n_old)) < 0.0001
        break
    end
    
end

%the dynamic pressure using the safe turn velcoity
q_turn = 0.5 .* rho_turn .* velocity_safe_turn.^2;

%coefficents
C_Lturn = update_load_factor .* Wing_loading ./ q_turn;
C_D_turn = C_D0 + K_1 .* C_Lturn.^2 + K_2 .* C_Lturn;

%Thrust to Weight calculation
Thrust_to_Weight_turn = (q_turn .* C_D_turn) ./ Wing_loading;
plot(Wing_loading, Thrust_to_Weight_turn, '-');

%/*Acceleration_horizontal_TO = (Thrust_TO - Drag_TO - ...
% Rolling_friction_coefficient .* (Weight_TO - Lift_TO)) ...
%./ Mass_TO; % Horizontal acceleration*\


xlabel('Wing Loading (N/m^2)');
ylabel('Thrust to Weight Ratio, T/W');
title('Constraints: T/W vs Wing Loading');
grid on;
legend('Takeoff', 'Climb', 'Cruise', 'Turn')

%/*stops the plots from being in the hold 
%mode thus future plots can ovveride*\

hold off; 


%/*What the user specifically wants from the program. Do they want
%specifically explore wing area based on fixed data they already know
% or does the user wants to explore multiple aircrafts and which would
% fit their goals.*\

function user_path = user_state()
disp("1 - Explore multiple aircraft designs")
disp("2 - Find required engine power")
disp("3 - Find wing area")
disp("4 - Find aircraft weight")
disp("5 - Find wingspan")

user_path = input("Choose an option: ");
end