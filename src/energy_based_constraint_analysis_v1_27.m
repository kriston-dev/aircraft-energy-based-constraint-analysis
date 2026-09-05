%Created by Kriston Rickman
%Date created 07/06/26
%V1.26
%Energy based constraint analysis
%Note:

%/*Goals*\

%/*simple concept of the takeoff constraint. formulas have been double
% checked and have changed base on the developing knowledge on the
% matter.*\


clearvars -except user_aircraft_input
clear functions
clear classes

clc

%What the user wants to do

currState = Aircraft_constraint_states.MAIN_MENU;

while true
    switch currState
    
        case Aircraft_constraint_states.MAIN_MENU
            currState = user_state();

        case Aircraft_constraint_states.SAVED_TW_AIRCRAFT_DATA
            doc_options();

            %/*Currently here debating whether to create another page so
            %we can access multiople saveddocs when entering into saved 
            % aircraft data we have options and when clicked into those
            %, we enter into another state for that specific saved doc
            % that give us the option to run, edit, or exit from there.*\

        case Aircraft_constraint_states.EDIT_SAVED_TW_AIRCRAFT
            edit_TW_aircraft(user_aircraft_input);
            currState = Aircraft_constraint_states.SAVED_CUSTOM_AIRCRAFT_DATA;

        
        case Aircraft_constraint_states.GET_TW_AIRCRAFT_DATA
            user_aircraft_input = get_user_aircraft_design_inputs();
            currState = Aircraft_constraint_states.CALCULATE_TW;
        
        case Aircraft_constraint_states.CALCULATE_TW
            TW = calculate_TW_constraints(user_aircraft_input);

            currState = Aircraft_constraint_states.MAIN_MENU;

        case Aircraft_constraint_states.GRAPH_TW

            figure;

            plot(TW.Wing_loading, TW.Takeoff, '-');
            hold on;
            
            plot(TW.Wing_loading, TW.Climb, '-');
            
            plot(TW.Wing_loading, TW.Cruise, '-');
            
            plot(TW.Wing_loading, TW.Turn, '-');
            
            plot(TW.Wing_loading, TW.Horizontal_acceleration, '-');
            
            % Approach is a vertical wing-loading constraint
            xline(TW.Approach_wing_loading, '-');

            xlabel('Wing Loading (N/m^2)');
            ylabel('Thrust to Weight Ratio, T/W');
            title('Constraints: T/W vs Wing Loading');
            grid on;
            legend('Takeoff', 'Climb', 'Cruise', 'Turn', 'Horizontal Acceleration', 'Approach');

            %/*stops the plots from being in the hold 
            %mode thus future plots can ovveride*\

            hold off; 

    end
end



function TW = calculate_TW_constraints(user_aircraft_input)

%Initial assumptions of aircraft design

Gravity = 9.81;

% rho_SL = 1.225;

% Cruise_velocity = 74.594;

% C_Dmin = 0.027; % Assumption from Cirrus sr 20
% C_Lmin = 0.3; % Assumption from Cirrus sr 20

% Sweep_angle = 3; % Made from assumption

% HP_to_watts = 745.7;

%The Design of the aircraft inputs
%NOTE Measuremnt in HP will be convert to Watts

% Engine_power_HP = 230; % Assumption of Eninge need data
% Span = 9; % assume based on aircraft type
% Wing_area = 8:0.1:16; % assume based on aircraft type

% Mass_without_wing_skin = 780; % assume based on aircraft type
% Wing_material_density = 2700; % assumption of kg/m^3 density of GA aluminum
% Wing_skin_thickness = 0.002; % assumption of mass of GA aluminum

% Engine Characteristics calculations

% Engine_power_watts = user_aircraft_input.Engine_power_HP .* HP_to_watts;

%Mass of the aircraft and related Geometry calculations

% Fuel_spent_ground_to_TO = 0.11;
% Fuel_mass_per_gallon = 2.8; % ARD
mass_loss_on_TO = user_aircraft_input.Fuel_spent_ground_to_TO .* user_aircraft_input.Fuel_mass_per_gallon;

Wing_skin_area_total = 2 .* user_aircraft_input.Wing_area;
Wing_skin_volume = Wing_skin_area_total .* user_aircraft_input.Wing_skin_thickness;
Wing_skin_mass = Wing_skin_volume .* user_aircraft_input.Wing_material_density;

Mass_aircraft = user_aircraft_input.Mass_without_wing_skin + Wing_skin_mass; % assume based on aircraft type
Mass_TO = Mass_aircraft - mass_loss_on_TO;

Weight_TO = Mass_TO * Gravity;
Wing_loading = Weight_TO ./ user_aircraft_input.Wing_area;
AR_wing = user_aircraft_input.Span.^2 ./ user_aircraft_input.Wing_area;

%Aerodynamic calculations

    %Oswald efficiency calculations - Sweep angle calculation decision
if user_aircraft_input.Sweep_angle == 0
    e = 1.78 .* (1 - 0.045 .* AR_wing.^0.68) - 0.64;
elseif user_aircraft_input.Sweep_angle >= 30
    e = 4.61 .* (1 - 0.045 .* AR_wing.^0.68) .* (cosd(user_aircraft_input.Sweep_angle)).^0.15 - 3.1;
elseif (user_aircraft_input.Sweep_angle > 0) && (user_aircraft_input.Sweep_angle < 30)
    e0 = 1.78 .* (1 - 0.045 .* AR_wing.^0.68) - 0.64;
    e30 = 4.61 .* (1 - 0.045 .* AR_wing.^0.68) .* (cosd(30)).^0.15 - 3.1;
    e = e0 + (user_aircraft_input.Sweep_angle ./ 30) .* (e30 - e0);
else
    error("Sweep angle is out of range")
end
    
    %Full Drag Polar Buildup
K_1 = 1./(pi.*AR_wing.*e);

C_D0 = user_aircraft_input.C_Dmin + K_1.*user_aircraft_input.C_Lmin.^2;

K_2 = -2.*K_1.*user_aircraft_input.C_Lmin;

%Takeoff constraint assumptions, variables and formulas
    %variables

    %Desired inputs
% S_G = 502.92; % Ground roll takeoff distance
% K_TO = 1.2;
% C_Lmax_TO = 1.7; % Assumption including flaps, elevator, and wing
% Rolling_friction_coefficient = 0.03;
% Propeller_efficiency_TO = 0.75; % assume based on propeller

user_aircraft_input.rho_TO = ...
    altitude_find_rho(user_aircraft_input.altitude_TO);

    %Velocity formulas
Velocity_stall = sqrt((2 .* Weight_TO) ./ (user_aircraft_input.rho_TO .* user_aircraft_input.Wing_area .* user_aircraft_input.C_Lmax_TO));

Velocity_TO = user_aircraft_input.K_TO * Velocity_stall;

Velocity_avg_TO = Velocity_TO ./ sqrt(2);

    %constraints sub-formulas
q_avg_TO = 0.5 .* user_aircraft_input.rho_TO .* Velocity_avg_TO.^2;

C_L_required_TO = 2 .* Weight_TO ./(user_aircraft_input.rho_TO .* Velocity_TO.^2 .* user_aircraft_input.Wing_area);

%/*C_L_ground_TO For a more accurate constraint create a lift
%coefficient that is specific for ground because the ground 
%and takeoff coefficents are not the same*\

C_DTO = C_D0 + (K_1.*C_L_required_TO.^2) + K_2.*C_L_required_TO;

% Lift_TO = 0.5 .* user_aircraft_input.rho_TO .* Velocity_TO.^2 .* user_aircraft_input.Wing_area .* C_L_required_TO;

% Lift_avg_TO = 0.5 .* user_aircraft_input.rho_TO .* Velocity_avg_TO.^2 .* user_aircraft_input.Wing_area .* C_L_required_TO;

%/*The lift average take off will change based on the average dynamic
% pressure, lift coefficenets for the ground Take off and etc*\

% Drag_TO = 0.5 .* C_DTO .* user_aircraft_input.rho_TO .* user_aircraft_input.Wing_area .* Velocity_TO.^2;

% Drag_avg_TO = 0.5 .* user_aircraft_input.rho_TO .* Velocity_avg_TO.^2 .* user_aircraft_input.Wing_area .* C_DTO;

%/*Will also chagned when created ground drag coeffiecients for TO*\

% Thrust_TO = (user_aircraft_input.Propeller_efficiency_TO .* Engine_power_watts) ./ Velocity_TO;

Acceleration_TO = Velocity_TO.^2 ./ (2 .* user_aircraft_input.S_G);

Thrust_to_Weight_TO = (Acceleration_TO ./ Gravity) + ... 
    (q_avg_TO .* C_DTO) ./ Wing_loading + user_aircraft_input.Rolling_friction_coefficient ...
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
% rate_of_climb = 3.5; %(meters per sec)
% velocity_climb = 48;

user_aircraft_input.rho_climb = ...
    altitude_find_rho(user_aircraft_input.altitude_climb);

%sub-formulas for the climb constraint

q_climb = 0.5 .* user_aircraft_input.rho_climb .* user_aircraft_input.velocity_climb.^2;

Beta = Mass_TO ./ Mass_aircraft; % Weight fraction

%A more specific version of Beta could have bee the mass while 
% climbing ./ Mass_TO

Thrust_to_Weight_climb = Beta/alpha .* (((K_1 .* n.^2 .* Beta) ./ ...
    q_climb) .* (Weight_TO ./ user_aircraft_input.Wing_area) + K_2 .* n + C_D0 ./ ...
    ((Beta ./ q_climb) .* Wing_loading) + user_aircraft_input.rate_of_climb ./ user_aircraft_input.velocity_climb);


plot(Wing_loading, Thrust_to_Weight_climb, '-');



% Cruise constraint

%variable
% knot_to_ms = 0.51444444;

%aircraft user inputs
% velocity_cruise = 155 .* knot_to_ms;
%/*Will change in the future for user input when wanting to know
% the T/W for the desired cruise knots they want. Additionaly, will
% add the calculation of the cruise velocity when user does not have
% a desire velcoity and will be based on the parameter they place for
% the aircraft.*\

user_aircraft_input.rho_cruise = ...
    altitude_find_rho(user_aircraft_input.altitude_cruise);

% sub-formulas for constraint

% velocity_cruise_knts = user_aircraft_input.velocity_cruise .* knot_to_ms;

q_cruise = 0.5 .* user_aircraft_input.rho_cruise .* user_aircraft_input.velocity_cruise.^2;

C_Lcruise = Wing_loading ./ q_cruise;

C_D_cruise = C_D0 + (K_1 .* C_Lcruise.^2) + K_2 .* C_Lcruise;

% Calculate the thrust-to-weight ratio for cruise
Thrust_to_Weight_cruise = (q_cruise * C_D_cruise) ./ Wing_loading;

plot(Wing_loading, Thrust_to_Weight_cruise, '-');



%Turn constraint

%Variables

    %User desire input

%in meters of user idea of their aircraft turning
% radius_turn = 300;

user_aircraft_input.rho_turn = ...
    altitude_find_rho(user_aircraft_input.altitude_turn);

C_Lmax_turn = user_aircraft_input.C_Lmax_turn;


%Sub-formulas
% velocity_stall_straight = sqrt((2 .* Wing_loading) ...
    % ./ (user_aircraft_input.rho_turn .* C_Lmax_turn)); %A straight line of aircraft stall velocity

%The guess of the safe turn velocity
% velocity_guess_turn = user_aircraft_input.K_turn .* velocity_stall_straight;

%Bank angle calculation from the guessed velocity and radius
%bank_angle = atand(velocity_guess_turn.^2 ./ (user_aircraft_input.radius_turn * Gravity));

%/*The start of the loop. We make it run
%through the loop to be accurate*\
n = 1;

for refine_loop = 1:1000

    n_old = n;
    
    velocity_stall_turn = sqrt((2 * Wing_loading .* n) ...
        ./ (user_aircraft_input.rho_turn .* C_Lmax_turn));
    
    %The safe turn
    velocity_safe_turn = user_aircraft_input.K_turn .* velocity_stall_turn;
    
    %Usign the new safe turn velocity we update the load factor
    update_load_factor = 1 ./ (cosd(atand(velocity_safe_turn.^2 ./  ...
        (user_aircraft_input.radius_turn .* Gravity))));
    
    n = update_load_factor;

    if max(abs(n-n_old)) < 0.0001
        break
    end
    
end

%the dynamic pressure using the safe turn velcoity
q_turn = 0.5 .* user_aircraft_input.rho_turn .* velocity_safe_turn.^2;

%coefficents
C_L_turn = update_load_factor .* Wing_loading ./ q_turn;
C_D_turn = C_D0 + K_1 .* C_L_turn.^2 + K_2 .* C_L_turn;

%Thrust to Weight calculation
Thrust_to_Weight_turn = (q_turn .* C_D_turn) ./ Wing_loading;
plot(Wing_loading, Thrust_to_Weight_turn, '-');

if any(C_L_turn > C_Lmax_turn)
    % aircraft would stall / turn condition is infeasible
    error("The aircraft will stall due to the turn needing to be higher than" + ...
        " the lift coefficent. This aircraft I would not recommend to use" + ...
        " this any design from this graph unless you found out which aircraft" + ...
        "was causing the problem.");
end



% Horizontal Acceleration constraint

%/*The constraitn assumes that the aircraft is accelerating in a horizontal
%position, no banking nor pitching. The constraitn is asuming that the
% aircraft is in a cruise phase.*\

%variables

    %user inputs
% velocity_accel = 50;
% accel_horiz = 1.1;

user_aircraft_input.rho_horiz_accel = ...
    altitude_find_rho(user_aircraft_input.altitude_horiz_accel);

%sub-formulas

q_accel = 0.5 .* user_aircraft_input.rho_horiz_accel .* user_aircraft_input.velocity_accel.^2;

C_L_accel = Wing_loading ./ q_accel;

C_D_accel = C_D0 + K_1 .* C_L_accel.^2 + K_2 .* C_L_accel;

%horizontal accel. constraint
Thrust_to_Weight_horiz_accel = (q_accel .* C_D_accel) ./ Wing_loading ...
    + user_aircraft_input.accel_horiz ./ Gravity; % Horizontal acceleration
plot(Wing_loading, Thrust_to_Weight_horiz_accel, '-');



%Approach Constraint

%variables

    %User input
% velocity_approach = user_aircraft_input.velocity_approach .* knot_to_ms;
% K_approach = 1.3;

user_aircraft_input.rho_approach = ...
    altitude_find_rho(user_aircraft_input.altitude_approach);

%sub-formulas

velocity_stall_approach = user_aircraft_input.velocity_approach ./ user_aircraft_input.K_approach;

q_approach = 0.5 .* user_aircraft_input.rho_approach .* velocity_stall_approach.^2;

% C_L_approach = Wing_loading ./ q_approach;

%approach formula constraint
Wing_loading_approach = q_approach .* user_aircraft_input.C_Lmax_approach;

%This creates a veritcal line for the apporach constraint
xline(Wing_loading_approach, '-');



% xlabel('Wing Loading (N/m^2)');
% ylabel('Thrust to Weight Ratio, T/W');
% title('Constraints: T/W vs Wing Loading');
% grid on;
% legend('Takeoff', 'Climb', 'Cruise', 'Turn', 'Horizontal Acceleration', 'Approach');
% 
% %/*stops the plots from being in the hold 
% %mode thus future plots can ovveride*\
% 
% hold off; 

TW.Takeoff = Thrust_to_Weight_TO;
TW.Climb = Thrust_to_Weight_climb;
TW.Cruise = Thrust_to_Weight_cruise;
TW.Turn = Thrust_to_Weight_turn;
TW.Horizontal_acceleration = Thrust_to_Weight_horiz_accel;
TW.Approach_wing_loading = Wing_loading_approach;

TW.Wing_loading = Wing_loading;

end

function rho = altitude_find_rho(altitude_TO)
rho_SL = 1.225;
Gravity = 9.80665;
temp_SL = 288.15;
temp_lapse_rate = 0.0065;
air_gas = 287.05;

rho = rho_SL .* (1 - (temp_lapse_rate .* altitude_TO) ./ temp_SL) ...
    .^ ((Gravity ./ (air_gas .* temp_lapse_rate)) - 1);

end



%/*From the GET_TW_AIRCRAFT_DATA *\
function user_aircraft_input = get_user_aircraft_design_inputs()

%user_aircraft_input.
%Will add more once organized all the user inputs and can add

%Ask user about:

% Aircraft geometry / design

user_aircraft_input.Engine_power_HP = input("Engine power (HP): ");

user_aircraft_input.Span = input("Wing span (m): ");

user_aircraft_input.Wing_area_raw = input("Wing area (m^2): ");

user_aircraft_input.Sweep_angle = input("Wing sweep angle (deg): ");


% Aircraft mass / material assumptions

user_aircraft_input.Mass_without_wing_skin = ...
    input("Aircraft mass without wing skin (kg): ");

user_aircraft_input.Wing_material_density = ...
    input("Wing material density (kg/m^3): ");

user_aircraft_input.Wing_skin_thickness = ...
    input("Wing skin thickness (m): ");


% Aerodynamic assumptions

user_aircraft_input.C_Dmin = input("Minimum drag coefficient C_Dmin: ");

user_aircraft_input.C_Lmin = ...
    input("Lift coefficient at minimum drag C_Lmin: ");

user_aircraft_input.C_Lmax_TO = ...
    input("Maximum takeoff lift coefficient C_Lmax_TO: ");


% Takeoff requirements / assumptions

user_aircraft_input.S_G = input("Takeoff ground roll distance (m): ");

user_aircraft_input.K_TO = input("Takeoff stall-speed safety factor K_TO: ");

user_aircraft_input.Rolling_friction_coefficient = ...
    input("Rolling friction coefficient: ");

user_aircraft_input.Propeller_efficiency_TO = ...
    input("Takeoff propeller efficiency: ");

user_aircraft_input.altitude_TO = input("Takeoff altitude (m): ");


% Fuel assumptions

user_aircraft_input.Fuel_spent_ground_to_TO = ...
    input("Fuel used before/during takeoff (gal): ");

user_aircraft_input.Fuel_mass_per_gallon = ...
    input("Fuel mass per gallon (kg/gal): ");


% Climb requirements

user_aircraft_input.rate_of_climb = input("Required climb rate: ");

user_aircraft_input.velocity_climb = ...
    conv_knts_to_ms(input("Climb velocity (knots): "));

user_aircraft_input.altitude_climb = input("Climb constraint altitude (m): ");


% Cruise requirements

user_aircraft_input.velocity_cruise = ...
    conv_knts_to_ms(input("Cruise velocity (knots): "));

user_aircraft_input.altitude_cruise = input("Cruise altitude (m): ");


% Turn requirements

user_aircraft_input.radius_turn = input("Turn radius (m): ");

user_aircraft_input.altitude_turn = input("Turn altitude (m): ");

user_aircraft_input.C_Lmax_turn = input("Turn max lift coefficent: ");

user_aircraft_input.K_turn = input("Takeoff stall-speed safety factor K_turn: ");


% Horizontal acceleration requirements

user_aircraft_input.velocity_accel = ...
    conv_knts_to_ms(input("Horizontal acceleration's velocity (knots): "));

user_aircraft_input.accel_horiz = ...
    input("Required horizontal acceleration (m/s^2): ");

user_aircraft_input.altitude_horiz_accel = ...
    input("Horizontal acceleration altitude (m): ");


% Approach requirements

user_aircraft_input.velocity_approach = ...
    conv_knts_to_ms(input("Approach velocity (knots): "));

user_aircraft_input.K_approach = ...
    input("Approach stall-speed safety factor K_approach: ");

user_aircraft_input.C_Lmax_approach = ...
    input("Maximum approach lift coefficient C_Lmax_approach: ");

user_aircraft_input.altitude_approach = input("Approach altitude (m): ");

% Creating the wing_Area bounds

user_aircraft_input.Wing_area_final = user_aircraft_input.Wing_area_raw .* 2;
user_aircraft_input.Wing_area = user_aircraft_input.Wing_area_raw:0.1:user_aircraft_input.Wing_area_final;

end


%/*What the user specifically wants from the program. Do they want
%specifically explore wing area based on fixed data they already know
% or does the user wants to explore multiple aircrafts and which would
% fit their goals.*\

function currState = user_state() %From main menu

 disp('Choose from the following options');
 disp(' - Find T/W');
    
 user_path = input('My choice is: ', 's');
    
if strcmp(user_path, 'find T/W')
    currState = Aircraft_constraint_states.GET_TW_AIRCRAFT_DATA;


elseif strcmp(user_path, 'docs')
    currState = Aircraft_constraint_states.SAVED_TW_AIRCRAFT_DATA;

else
    disp("Invalid choice");
    currState = Aircraft_constraint_states.MAIN_MENU;
end

end



%/*Need to finish function that gathers data from the user 
% before starting the calculation of the aircraft T/W.*\

% function TW = found_TW()

% TW = 3;
% end

function edit_TW_aircraft(user_aircraft_input)

disp("Choose what aircraft input you want to change:");
disp("1  - Engine power (HP)");
disp("2  - Wing span");
disp("3  - Wing area");
disp("4  - Wing sweep angle");

disp("5  - Aircraft mass without wing skin");
disp("6  - Wing material density");
disp("7  - Wing skin thickness");

disp("8  - Minimum drag coefficient C_Dmin");
disp("9  - Lift coefficient at minimum drag C_Lmin");
disp("10 - Maximum takeoff lift coefficient C_Lmax_TO");

disp("11 - Takeoff ground roll distance");
disp("12 - Takeoff stall-speed factor K_TO");
disp("13 - Rolling friction coefficient");
disp("14 - Takeoff propeller efficiency");
disp("15 - Takeoff altitude");

disp("16 - Fuel used before/during takeoff");
disp("17 - Fuel mass per gallon");

disp("18 - Required climb rate");
disp("19 - Climb velocity");
disp("20 - Climb altitude");

disp("21 - Cruise velocity");
disp("22 - Cruise altitude");

disp("23 - Turn radius");
disp("24 - Turn altitude");
disp("25 - Turn max lift coefficient C_Lmax_turn");
disp("26 - Turn stall-speed factor K_turn");

disp("27 - Horizontal acceleration velocity");
disp("28 - Required horizontal acceleration");
disp("29 - Horizontal acceleration altitude");

disp("30 - Approach velocity");
disp("31 - Approach stall-speed factor K_approach");
disp("32 - Maximum approach lift coefficient C_Lmax_approach");
disp("33 - Approach altitude");

edit = input("Choose number: ");

switch edit

    case 1
        user_aircraft_input.Engine_power_HP = ...
            input("Engine power (HP): ");

    case 2
        user_aircraft_input.Span = ...
            input("Wing span (m): ");

    case 3
        user_aircraft_input.Wing_area_raw = ...
            input("Wing area (m^2): ");

    case 4
        user_aircraft_input.Sweep_angle = ...
            input("Wing sweep angle (deg): ");


    case 5
        user_aircraft_input.Mass_without_wing_skin = ...
            input("Aircraft mass without wing skin (kg): ");

    case 6
        user_aircraft_input.Wing_material_density = ...
            input("Wing material density (kg/m^3): ");

    case 7
        user_aircraft_input.Wing_skin_thickness = ...
            input("Wing skin thickness (m): ");


    case 8
        user_aircraft_input.C_Dmin = ...
            input("Minimum drag coefficient C_Dmin: ");

    case 9
        user_aircraft_input.C_Lmin = ...
            input("Lift coefficient at minimum drag C_Lmin: ");

    case 10
        user_aircraft_input.C_Lmax_TO = ...
            input("Maximum takeoff lift coefficient C_Lmax_TO: ");


    case 11
        user_aircraft_input.S_G = ...
            input("Takeoff ground roll distance (m): ");

    case 12
        user_aircraft_input.K_TO = ...
            input("Takeoff stall-speed safety factor K_TO: ");

    case 13
        user_aircraft_input.Rolling_friction_coefficient = ...
            input("Rolling friction coefficient: ");

    case 14
        user_aircraft_input.Propeller_efficiency_TO = ...
            input("Takeoff propeller efficiency: ");

    case 15
        user_aircraft_input.altitude_TO = ...
            input("Takeoff altitude (m): ");


    case 16
        user_aircraft_input.Fuel_spent_ground_to_TO = ...
            input("Fuel used before/during takeoff (gal): ");

    case 17
        user_aircraft_input.Fuel_mass_per_gallon = ...
            input("Fuel mass per gallon (kg/gal): ");


    case 18
        user_aircraft_input.rate_of_climb = ...
            input("Required climb rate: ");

    case 19
        user_aircraft_input.velocity_climb = ...
           conv_knts_to_ms(input("Climb velocity (knots): "));

    case 20
        user_aircraft_input.altitude_climb = ...
            input("Climb constraint altitude (m): ");


    case 21
        user_aircraft_input.velocity_cruise = ...
            conv_knts_to_ms(input("Cruise velocity (knots): "));

    case 22
        user_aircraft_input.altitude_cruise = ...
            input("Cruise altitude (m): ");


    case 23
        user_aircraft_input.radius_turn = ...
            input("Turn radius (m): ");

    case 24
        user_aircraft_input.altitude_turn = ...
            input("Turn altitude (m): ");

    case 25
        user_aircraft_input.C_Lmax_turn = ...
            input("Turn max lift coefficient: ");

    case 26
        user_aircraft_input.K_turn = ...
            input("Turn stall-speed safety factor K_turn: ");


    case 27
        user_aircraft_input.velocity_accel = ...
            conv_knts_to_ms(input("Horizontal acceleration velocity (knots): "));

    case 28
        user_aircraft_input.accel_horiz = ...
            input("Required horizontal acceleration (m/s^2): ");

    case 29
        user_aircraft_input.altitude_horiz_accel = ...
            input("Horizontal acceleration altitude (m): ");


    case 30
        user_aircraft_input.velocity_approach = ...
            conv_knts_to_ms(input("Approach velocity (knots): "));

    case 31
        user_aircraft_input.K_approach = ...
            input("Approach stall-speed safety factor K_approach: ");

    case 32
        user_aircraft_input.C_Lmax_approach = ...
            input("Maximum approach lift coefficient C_Lmax_approach: ");

    case 33
        user_aircraft_input.altitude_approach = ...
            input("Approach altitude (m): ");

    otherwise
        disp("Invalid choice");

end
end



function velocity_ms = conv_knts_to_ms(velocity_knots)

velocity_ms = velocity_knots .* 0.5144;

end