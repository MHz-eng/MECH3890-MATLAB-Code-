%% =========================================================================
%  ZMP BALANCE CONTROLLER - INTEGRATED WITH ASSISTIVE GAIT CONTROLLER
%  WITH IMPROVED ANKLE/HIP BALANCE STRATEGY FOR ROUGH TERRAIN
%  
%  This code simulates a lower-limb exoskeleton for stroke rehabilitation.
%  It combines:
%    1. Assistive PID controller - helps paretic leg track healthy pattern
%    2. Balance controller - maintains stability using ankle/hip strategies
%    3. ZMP analysis - monitors stability margin throughout gait
%    4. Terrain adaptation - auto-tunes gains based on terrain difficulty
%  
%
%  Color convention: RED = Paretic (assisted), BLUE = Non-Paretic (healthy)
% =========================================================================

clear; clc; close all;

%% ========== SETTINGS ==========
% Path to stroke patient gait data file
data_path = 'MAT_normalizedData_PostStrokeAdults_v27-02-23.mat';

sID = 4;  % Subject ID to simulate (1-50 available)

% Controller settings
assist_level = 0.80;  % Fraction of correction provided (0-1)

% PID gains for gait assistance
Kp = 0.8;    % Proportional gain - responds to current error
Ki = 0.1;   % Integral gain - eliminates steady-state error 
Kd = 0.15;   % Derivative gain - adds damping
I_max = 15;  % Anti-windup limit for integral term
corr_max = 20;  % Maximum correction in degrees

% ZMP/terrain settings
terrain_type = 'flat';  % Options: 'flat', 'slope', 'step', 'rough', 'sine'

% Balance controller settings (auto-tuned based on terrain)
paretic_strength = 0.6;  % Paretic leg strength as fraction of healthy (60%)

% Walkway parameters
walkway_length = 12.0;  % Total distance to walk (meters)
trail_length = 40;      % Number of frames to show in foot trail

%% ========== LOAD DATA ==========
% Load the MAT file containing normalized gait data from stroke patients
fprintf('Loading data...\n');
load(data_path);

fprintf('Selected subject: %d\n', sID);
fprintf('Assistance level: %.0f%%\n', assist_level * 100);
fprintf('Terrain: %s\n', terrain_type);
fprintf('Paretic strength: %.0f%%\n\n', paretic_strength * 100);

% Extract subject data structures
S = Sub(sID);                    % Full subject structure
P = S.PsideSegm_PsideData;       % Paretic (stroke-affected) side
N = S.NsideSegm_NsideData;       % Non-paretic (healthy) side

%% ========== PATIENT ANTHROPOMETRICS ==========
% Extract actual patient characteristics from the data file
% sub_char contains measured values for each subject from clinical assessment

sub_char = S.sub_char;  % Get subject characteristics structure

% ----- Extract patient data (convert mm to m where needed) -----
patient_mass = sub_char.Weight;           % kg (already in correct units)
patient_height = sub_char.Height / 1000;  % Convert mm to m
leg_length = sub_char.LegLength / 1000;   % Convert mm to m (measured)

% ----- Clinical assessment scores (for reference/display) -----
% These scores characterize the patient's functional status
patient_age = sub_char.Age;               % Patient age in years
is_male = sub_char.Male;                  % 1 = male, 0 = female
time_post_stroke = sub_char.TPS;          % Time post stroke (days or months)
lesion_left = sub_char.LesionLeft;        % 1 = left hemisphere stroke (right side affected)
FAC_score = sub_char.FAC;                 % Functional Ambulation Category (0-5)
                                          % 0=non-functional, 5=independent outdoors
POMA_score = sub_char.POMA;               % Tinetti Performance-Oriented Mobility Assessment (0-28)
                                          % Higher = better balance and gait
TIS_score = sub_char.TIS;                 % Trunk Impairment Scale
                                          % Measures sitting balance and trunk control

% ----- Calculate segment lengths using Winter's proportions -----
% Winter (1990) established standard ratios for body segment dimensions
L_thigh = 0.245 * patient_height;   % Thigh length (~24.5% of height)
L_shank = 0.246 * patient_height;   % Shank length (~24.6% of height)
L_foot = 0.152 * patient_height;    % Foot length (~15.2% of height)
L_pelvis = 0.191 * patient_height;  % Pelvis width (~19.1% of height)

% ----- Scale segments to match measured leg length -----
% This ensures kinematics use actual patient dimensions rather than estimates
% The scale factor adjusts Winter's proportions to match measured leg length
scale_factor = leg_length / (L_thigh + L_shank);
L_thigh = L_thigh * scale_factor;   % Adjusted thigh length
L_shank = L_shank * scale_factor;   % Adjusted shank length

% ----- Segment masses as fractions of total body mass (Winter 2009) -----
mass_thigh = 0.100 * patient_mass;   % Each thigh ~10% of body mass
mass_shank = 0.0465 * patient_mass;  % Each shank ~4.65% of body mass
mass_foot = 0.0145 * patient_mass;   % Each foot ~1.45% of body mass
mass_trunk = 0.497 * patient_mass;   % Trunk (head, arms, torso) ~50%

% ----- Physical constants -----
g = 9.81;                        % Gravitational acceleration (m/s²)
body_weight = patient_mass * g;  % Body weight in Newtons

% ----- Foot dimensions for support polygon calculations -----
foot_length = L_foot;
foot_width = 0.08;  % Typical foot width (meters)

% ----- Display patient information -----
fprintf('\n========== PATIENT CHARACTERISTICS ==========\n');
fprintf('Subject ID: %d\n', sID);
if is_male
    sex_str = 'Male';
else
    sex_str = 'Female';
end
fprintf('Age: %d years, Sex: %s\n', patient_age, sex_str);
fprintf('Height: %.2f m, Mass: %.1f kg\n', patient_height, patient_mass);
fprintf('Measured leg length: %.3f m\n', leg_length);
fprintf('Calculated: L_thigh=%.3f m, L_shank=%.3f m\n', L_thigh, L_shank);
fprintf('\nClinical Scores:\n');
fprintf('  Time post-stroke: %d\n', time_post_stroke);
if lesion_left
    lesion_str = 'Left';
else
    lesion_str = 'Right';
end
fprintf('  Lesion side: %s hemisphere\n', lesion_str);
fprintf('  FAC: %d/5 (Functional Ambulation Category)\n', FAC_score);
fprintf('  POMA: %d/28 (Tinetti Mobility)\n', POMA_score);
fprintf('  TIS: %d (Trunk Impairment Scale)\n', TIS_score);
fprintf('=============================================\n');

%% ========== EXTRACT GAIT EVENT TIMING ==========
% The data contains precise timing of gait events from force plate data:
%   IC = Initial Contact (heel strike) - start of stance phase
%   TO = Toe Off - end of stance phase, start of swing
%
% These events are more accurate than clearance-based detection because
% they come from actual force plate measurements during data collection.
% The force plates measure when the foot actually contacts the ground,
% eliminating estimation errors from kinematic data alone.

fprintf('\nExtracting gait event timing...\n');

% Initialize flag and default timing values
has_gait_events = false;

% Default values: TO at 60% of gait cycle (typical healthy gait)
% Stance phase: 0% to ~60%, Swing phase: ~60% to 100%
P_TO_pct = 60;  % Paretic leg Toe Off percentage
N_TO_pct = 60;  % Non-paretic leg Toe Off percentage

% ----- CHECK Sub(sID).events STRUCT FIRST -----
% Gait events are stored in Sub(sID).events for most subjects,
% not inside PsideSegm_PsideData — check here first.
if isfield(S, 'events')
    E = S.events;
    fprintf('  Found Sub(%d).events struct\n', sID);

    % Paretic TOnorm
    if isfield(E, 'P_TOnorm')
        P_TO_norm = E.P_TOnorm;
        if isnumeric(P_TO_norm) && ~isempty(P_TO_norm)
            % Values may be frame indices (0-1001) or percentages (0-100)
            % Convert frame indices to percentage if values exceed 100
            if max(P_TO_norm(:)) > 100
                P_TO_norm = P_TO_norm / 1001 * 100;
            end
            valid_vals = P_TO_norm(P_TO_norm > 0 & P_TO_norm < 100);
            if ~isempty(valid_vals)
                P_TO_pct = mean(valid_vals);
                has_gait_events = true;
                fprintf('  Paretic TO at %.1f%% (from events.P_TOnorm)\n', P_TO_pct);
            end
        end
    end

    % Non-paretic TOnorm
    if isfield(E, 'N_TOnorm')
        N_TO_norm = E.N_TOnorm;
        if isnumeric(N_TO_norm) && ~isempty(N_TO_norm)
            if max(N_TO_norm(:)) > 100
                N_TO_norm = N_TO_norm / 1001 * 100;
            end
            valid_vals = N_TO_norm(N_TO_norm > 0 & N_TO_norm < 100);
            if ~isempty(valid_vals)
                N_TO_pct = mean(valid_vals);
                has_gait_events = true;
                fprintf('  Non-paretic TO at %.1f%% (from events.N_TOnorm)\n', N_TO_pct);
            end
        end
    end

    % IC/TO frame counts if available
    if isfield(E, 'P_IC_cnt'), P_IC_frames = E.P_IC_cnt; end
    if isfield(E, 'P_TO_cnt'), P_TO_frames = E.P_TO_cnt; end
    if isfield(E, 'N_IC_cnt'), N_IC_frames = E.N_IC_cnt; end
    if isfield(E, 'N_TO_cnt'), N_TO_frames = E.N_TO_cnt; end
end

% ----- PARETIC SIDE GAIT EVENTS (fallback: inside PsideSegm_PsideData) -----
if isfield(P, 'P_IC_cnt') && isfield(P, 'P_TO_cnt')
    % P_IC_cnt: Frame indices where Initial Contact occurs
    % P_TO_cnt: Frame indices where Toe Off occurs
    P_IC_frames = P.P_IC_cnt;  % Initial Contact frames
    P_TO_frames = P.P_TO_cnt;  % Toe Off frames
    
    % Get TO as percentage of gait cycle from normalized data
    % P_TOnorm contains TO timing normalized to 0-100% of gait cycle
    if isfield(P, 'P_TOnorm')
        P_TO_norm = P.P_TOnorm;
        if isnumeric(P_TO_norm) && ~isempty(P_TO_norm)
            % Filter valid values (between 0 and 100%)
            valid_vals = P_TO_norm(P_TO_norm > 0 & P_TO_norm < 100);
            if ~isempty(valid_vals)
                P_TO_pct = mean(valid_vals);  % Average TO percentage
            end
        end
    end
    
    % Force plate validity flags indicate reliable measurements
    % P_GoodForcePlate: 1 = valid force plate data, 0 = invalid/missing
    if isfield(P, 'P_GoodForcePlate')
        P_force_plate_valid = P.P_GoodForcePlate;
    else
        P_force_plate_valid = ones(1, length(P_IC_frames));  % Assume all valid
    end
    
    has_gait_events = true;
    fprintf('  Paretic IC frames: %s\n', mat2str(P_IC_frames));
    fprintf('  Paretic TO frames: %s\n', mat2str(P_TO_frames));
    fprintf('  Paretic TO at %.1f%% of gait cycle\n', P_TO_pct);
end

% ----- NON-PARETIC SIDE GAIT EVENTS -----
if isfield(N, 'N_IC_cnt') && isfield(N, 'N_TO_cnt')
    % Same structure as paretic side
    N_IC_frames = N.N_IC_cnt;  % Initial Contact frames
    N_TO_frames = N.N_TO_cnt;  % Toe Off frames
    
    if isfield(N, 'N_TOnorm')
        N_TO_norm = N.N_TOnorm;
        if isnumeric(N_TO_norm) && ~isempty(N_TO_norm)
            valid_vals = N_TO_norm(N_TO_norm > 0 & N_TO_norm < 100);
            if ~isempty(valid_vals)
                N_TO_pct = mean(valid_vals);
            end
        end
    end
    
    if isfield(N, 'N_GoodForcePlate')
        N_force_plate_valid = N.N_GoodForcePlate;
    else
        N_force_plate_valid = ones(1, length(N_IC_frames));
    end
    
    has_gait_events = true;
    fprintf('  Non-paretic IC frames: %s\n', mat2str(N_IC_frames));
    fprintf('  Non-paretic TO frames: %s\n', mat2str(N_TO_frames));
    fprintf('  Non-paretic TO at %.1f%% of gait cycle\n', N_TO_pct);
end

if has_gait_events
    fprintf('  Gait events loaded from force plate data.\n');
else
    fprintf('  No gait event data found - will use clearance-based detection.\n');
end

%% ========== TERRAIN FUNCTION ==========
% Nested function returns ground height z at any position (x,y)
% Used throughout simulation to determine foot-ground interaction
    function z = terrain_height(x, y, type)
        switch type
            case 'flat'
                % Level ground - baseline condition for indoor use
                z = 0;
            case 'slope'
                % 5-degree uphill incline (common outdoor slope, ramps)
                z = x * tand(5);
            case 'step'
                % Single 5cm step at x = 5 meters (simulates curb/threshold)
                if x > 5.0
                    z = 0.05;
                else
                    z = 0;
                end
            case 'rough'
                % Irregular terrain using superimposed sinusoids
                % Creates realistic outdoor ground variations (grass, gravel)
                z = 0.02 * sin(2*x) .* cos(3*y) + ...
                    0.01 * sin(5*x + 1) + ...
                    0.015 * cos(4*y + 2);
            case 'sine'
                % Regular wave pattern with 2m wavelength
                % Tests periodic disturbance rejection
                z = 0.03 * sin(2*pi*x / 2.0);
            otherwise
                z = 0;
        end
    end

%% ========== AUTO-TUNE BALANCE GAINS BASED ON TERRAIN ==========
% Sample terrain to measure difficulty (standard deviation of height)
% Rougher terrain requires more aggressive balance corrections
% This adaptive gain selection mimics how humans adjust their balance
% strategy based on terrain conditions

terrain_samples = arrayfun(@(x) terrain_height(x, 0, terrain_type), linspace(0, walkway_length, 200));
terrain_roughness = std(terrain_samples) * 100;  % Convert to cm

fprintf('\nTerrain roughness: %.2f cm (std)\n', terrain_roughness);

% Select gain profile based on terrain difficulty
% Three profiles: ROUGH (>1.5cm std), MODERATE (0.5-1.5cm), FLAT (<0.5cm)
if terrain_roughness > 1.5
    % ROUGH TERRAIN - aggressive gains for fast response
    % High gains needed to react quickly to unpredictable perturbations
    Kp_ankle = 250;          % High proportional gain for quick response
    Kd_ankle = 30;           % High derivative for damping oscillations
    Kp_hip = 180;            % Hip also needs strong correction
    Kd_hip = 20;
    ankle_threshold = 0.10;  % ZMP error threshold for ankle strategy (m)
    hip_threshold = 0.20;    % ZMP error threshold for hip strategy (m)
    max_ankle_torque = 150;  % Maximum ankle torque (Nm) - motor limit
    max_hip_torque = 200;    % Maximum hip torque (Nm)
    ankle_stiffness = 250;   % Joint stiffness (Nm/rad) - lower = more correction
    hip_stiffness = 180;
    smoothing_factor = 10;   % Window size for signal smoothing
    fprintf('Using AGGRESSIVE balance gains for rough terrain\n');
elseif terrain_roughness > 0.5
    % MODERATE TERRAIN - balanced response
    % Moderate gains provide stability without excessive energy use
    Kp_ankle = 150;
    Kd_ankle = 18;
    Kp_hip = 100;
    Kd_hip = 12;
    ankle_threshold = 0.07;
    hip_threshold = 0.14;
    max_ankle_torque = 120;
    max_hip_torque = 160;
    ankle_stiffness = 300;
    hip_stiffness = 220;
    smoothing_factor = 7;
    fprintf('Using MODERATE balance gains\n');
else
    % FLAT/MILD TERRAIN - gentle corrections sufficient
    % Low gains save energy while maintaining stability on easy terrain
    Kp_ankle = 50;
    Kd_ankle = 5;
    Kp_hip = 30;
    Kd_hip = 3;
    ankle_threshold = 0.05;
    hip_threshold = 0.10;
    max_ankle_torque = 80;
    max_hip_torque = 120;
    ankle_stiffness = 400;   % Higher stiffness = smaller corrections
    hip_stiffness = 300;
    smoothing_factor = 5;
    fprintf('Using STANDARD balance gains\n');
end

%% ========== EXTRACT JOINT ANGLES ==========
% Extract hip, knee, ankle angles from the loaded data structure
% Data format varies between subjects, so helper function handles different field names
fprintf('\nExtracting joint angles...\n');

% Helper function to extract angle data from various struct formats
% The data may store angles in 'X', 'x', or the first available field
    function out = extract_angle_data(data)
        if isstruct(data)
            if isfield(data, 'X')
                out = data.X(:);      % Uppercase X field
            elseif isfield(data, 'x')
                out = data.x(:);      % Lowercase x field
            else
                fn = fieldnames(data);
                if ~isempty(fn)
                    out = data.(fn{1})(:);  % First available field
                else
                    out = [];
                end
            end
        elseif isnumeric(data)
            out = data(:);  % Already numeric array
        else
            out = [];
        end
    end

% Extract paretic (stroke-affected) side angles
% These show the impaired movement pattern
hip_flex_P_raw = extract_angle_data(P.HipAngles);
knee_flex_P_raw = extract_angle_data(P.KneeAngles);
ankle_flex_P_raw = extract_angle_data(P.AnkleAngles);

% Extract non-paretic (healthy) side angles
% These serve as the target pattern for rehabilitation
hip_flex_N_raw = extract_angle_data(N.HipAngles);
knee_flex_N_raw = extract_angle_data(N.KneeAngles);
ankle_flex_N_raw = extract_angle_data(N.AnkleAngles);

% Get data lengths (may differ between sides due to recording)
len_P = length(hip_flex_P_raw);
len_N = length(hip_flex_N_raw);

%% ========== NORMALIZE TO SAME LENGTH ==========
% Resample both sides to same number of points for synchronization
% Uses spline interpolation to preserve smooth curves
% This is necessary because paretic and non-paretic gait cycles
% may have different durations due to asymmetry

n_cycle = min(len_P, len_N);  % Use shorter length as reference

if len_P ~= len_N
    % Resample paretic side if needed
    if len_P ~= n_cycle
        x_old = linspace(0, 1, len_P);
        x_new = linspace(0, 1, n_cycle);
        hip_flex_P = interp1(x_old, hip_flex_P_raw, x_new, 'spline')';
        knee_flex_P = interp1(x_old, knee_flex_P_raw, x_new, 'spline')';
        ankle_flex_P = interp1(x_old, ankle_flex_P_raw, x_new, 'spline')';
    else
        hip_flex_P = hip_flex_P_raw;
        knee_flex_P = knee_flex_P_raw;
        ankle_flex_P = ankle_flex_P_raw;
    end
    
    % Resample non-paretic side if needed
    if len_N ~= n_cycle
        x_old = linspace(0, 1, len_N);
        x_new = linspace(0, 1, n_cycle);
        hip_flex_N = interp1(x_old, hip_flex_N_raw, x_new, 'spline')';
        knee_flex_N = interp1(x_old, knee_flex_N_raw, x_new, 'spline')';
        ankle_flex_N = interp1(x_old, ankle_flex_N_raw, x_new, 'spline')';
    else
        hip_flex_N = hip_flex_N_raw;
        knee_flex_N = knee_flex_N_raw;
        ankle_flex_N = ankle_flex_N_raw;
    end
else
    % Same length - use raw data directly
    hip_flex_P = hip_flex_P_raw;
    knee_flex_P = knee_flex_P_raw;
    ankle_flex_P = ankle_flex_P_raw;
    hip_flex_N = hip_flex_N_raw;
    knee_flex_N = knee_flex_N_raw;
    ankle_flex_N = ankle_flex_N_raw;
end

% Fill any NaN values using linear interpolation
% Missing data points can occur from marker occlusion during capture
hip_flex_P = fillmissing(hip_flex_P(:), 'linear');
hip_flex_N = fillmissing(hip_flex_N(:), 'linear');
knee_flex_P = fillmissing(knee_flex_P(:), 'linear');
knee_flex_N = fillmissing(knee_flex_N(:), 'linear');
ankle_flex_P = fillmissing(ankle_flex_P(:), 'linear');
ankle_flex_N = fillmissing(ankle_flex_N(:), 'linear');

% Normalise to 200 points/cycle to prevent dt blow-up in ZMP calculation.
% Raw sample counts (8008, 9009 etc.) make dt tiny, causing CoM acceleration
% to produce nonsensical ZMP margins (-4000 cm). sID 2 (9009) is borderline
% but sID 1 (8008) and sID 4 (1001) both need this fix.
N_NORM = 200;
if n_cycle ~= N_NORM
    x_old = linspace(0, 1, n_cycle);
    x_new = linspace(0, 1, N_NORM);
    hip_flex_P   = interp1(x_old, hip_flex_P,   x_new, 'pchip')';
    knee_flex_P  = interp1(x_old, knee_flex_P,  x_new, 'pchip')';
    ankle_flex_P = interp1(x_old, ankle_flex_P, x_new, 'pchip')';
    hip_flex_N   = interp1(x_old, hip_flex_N,   x_new, 'pchip')';
    knee_flex_N  = interp1(x_old, knee_flex_N,  x_new, 'pchip')';
    ankle_flex_N = interp1(x_old, ankle_flex_N, x_new, 'pchip')';
    fprintf('Resampled: %d -> %d points per cycle\n', n_cycle, N_NORM);
    n_cycle = N_NORM;
end

fprintf('Gait cycle: %d samples\n', n_cycle);

%% ========== ASSISTIVE PID CONTROLLER ==========
% Core rehabilitation algorithm: helps paretic leg track healthy pattern
% Error = healthy angle - paretic angle (what the paretic leg should do)
% Correction = PID output scaled by assist_level
%
% The assist_level parameter implements "assist-as-needed" paradigm:
% - High assist (0.8-1.0): Early rehabilitation, patient learning
% - Low assist (0.2-0.4): Late rehabilitation, patient doing more work
fprintf('\n========== ASSISTIVE PID CONTROLLER ==========\n');

% Initialize PID states
iHip = 0; iKnee = 0; iAnkle = 0;           % Integral terms (accumulated error)
eHip_prev = 0; eKnee_prev = 0; eAnkle_prev = 0;  % Previous errors for derivative

% Pre-allocate output arrays
hip_flex_A = zeros(n_cycle, 1);    % Assisted hip angles
knee_flex_A = zeros(n_cycle, 1);   % Assisted knee angles
ankle_flex_A = zeros(n_cycle, 1);  % Assisted ankle angles

% Diagnostic arrays for controller analysis
gap_hip = zeros(n_cycle, 1);         % Error before correction
gap_knee = zeros(n_cycle, 1);
gap_ankle = zeros(n_cycle, 1);
correction_hip = zeros(n_cycle, 1);   % Applied correction
correction_knee = zeros(n_cycle, 1);
correction_ankle = zeros(n_cycle, 1);

% Run PID controller for each sample in gait cycle
for k = 1:n_cycle
    % Calculate error: healthy - paretic (positive = paretic is lagging)
    % This represents the "gap" between desired and actual movement
    eHip = hip_flex_N(k) - hip_flex_P(k);
    eKnee = knee_flex_N(k) - knee_flex_P(k);
    eAnkle = ankle_flex_N(k) - ankle_flex_P(k);
    
    % Store gaps for analysis
    gap_hip(k) = eHip;
    gap_knee(k) = eKnee;
    gap_ankle(k) = eAnkle;
    
    % Update integral terms with anti-windup (clamp to prevent saturation)
    % Anti-windup prevents integral term from growing unbounded when
    % the system cannot achieve the setpoint (motor saturated, etc.)
    iHip = max(-I_max, min(I_max, iHip + eHip));
    iKnee = max(-I_max, min(I_max, iKnee + eKnee));
    iAnkle = max(-I_max, min(I_max, iAnkle + eAnkle));
    
    % Calculate derivative terms (rate of error change)
    % Derivative provides damping and anticipates future error
    dHip = eHip - eHip_prev;
    dKnee = eKnee - eKnee_prev;
    dAnkle = eAnkle - eAnkle_prev;
    
    % Full PID output: u = Kp*e + Ki*∫e + Kd*de/dt
    uHip_full = Kp * eHip + Ki * iHip + Kd * dHip;
    uKnee_full = Kp * eKnee + Ki * iKnee + Kd * dKnee;
    uAnkle_full = Kp * eAnkle + Ki * iAnkle + Kd * dAnkle;
    
    % Clamp to maximum correction (safety limit)
    % Prevents excessive corrections that could harm patient
    uHip_full = max(-corr_max, min(corr_max, uHip_full));
    uKnee_full = max(-corr_max, min(corr_max, uKnee_full));
    uAnkle_full = max(-corr_max, min(corr_max, uAnkle_full));
    
    % Scale by assistance level (partial assistance paradigm)
    % Patient provides (1 - assist_level) of effort themselves
    uHip = assist_level * uHip_full;
    uKnee = assist_level * uKnee_full;
    uAnkle = assist_level * uAnkle_full;
    
    % Store corrections
    correction_hip(k) = uHip;
    correction_knee(k) = uKnee;
    correction_ankle(k) = uAnkle;
    
    % Apply correction: assisted = paretic + correction
    hip_flex_A(k) = hip_flex_P(k) + uHip;
    knee_flex_A(k) = knee_flex_P(k) + uKnee;
    ankle_flex_A(k) = ankle_flex_P(k) + uAnkle;
    
    % Store previous errors for derivative calculation
    eHip_prev = eHip;
    eKnee_prev = eKnee;
    eAnkle_prev = eAnkle;
end

% Calculate performance metrics
mean_gap_hip = mean(abs(gap_hip));      % Average error before correction
mean_gap_knee = mean(abs(gap_knee));
mean_gap_ankle = mean(abs(gap_ankle));

residual_hip = hip_flex_N - hip_flex_A;  % Remaining error after correction
residual_knee = knee_flex_N - knee_flex_A;
residual_ankle = ankle_flex_N - ankle_flex_A;

mean_res_hip = mean(abs(residual_hip));
mean_res_knee = mean(abs(residual_knee));
mean_res_ankle = mean(abs(residual_ankle));

% Gap reduction = how much the controller improved tracking (%)
% 100% would mean perfect tracking of healthy pattern
reduction_hip = 100 * (1 - mean_res_hip / mean_gap_hip);
reduction_knee = 100 * (1 - mean_res_knee / mean_gap_knee);
reduction_ankle = 100 * (1 - mean_res_ankle / mean_gap_ankle);

fprintf('Gap reduction: Hip=%.0f%%, Knee=%.0f%%, Ankle=%.0f%%\n', ...
    reduction_hip, reduction_knee, reduction_ankle);

% -------------------------------------------------------------------------
% CIRCUMDUCTION / HYPERMETRIA DETECTION
% Fires when paretic hip ROM exceeds non-paretic by >80%.
% Only clinically relevant for sID 1 (left hemisphere, 98% excess).
% For sID 2 and sID 4 circumduction_detected stays false — no effect.
% -------------------------------------------------------------------------
hip_ROM_P  = max(hip_flex_P) - min(hip_flex_P);
hip_ROM_N  = max(hip_flex_N) - min(hip_flex_N);
knee_ROM_P = max(knee_flex_P) - min(knee_flex_P);
knee_ROM_N = max(knee_flex_N) - min(knee_flex_N);
ankle_ROM_P= max(ankle_flex_P) - min(ankle_flex_P);
ankle_ROM_N= max(ankle_flex_N) - min(ankle_flex_N);

circumduction_detected = false;

if hip_ROM_P > hip_ROM_N * 1.80
    circumduction_detected = true;
    fprintf('\n========== CIRCUMDUCTION / HYPERMETRIA DETECTED ==========\n');
    fprintf('  Hip ROM  — Paretic: %.1f deg  Non-paretic: %.1f deg\n', hip_ROM_P, hip_ROM_N);
    fprintf('  Knee ROM — Paretic: %.1f deg  Non-paretic: %.1f deg\n', knee_ROM_P, knee_ROM_N);
    fprintf('  Ankle ROM— Paretic: %.1f deg  Non-paretic: %.1f deg\n', ankle_ROM_P, ankle_ROM_N);
    fprintf('\n  CLINICAL NOTE:\n');
    fprintf('  Paretic hip ROM exceeds non-paretic by %.0f%%.\n', ...
            100*(hip_ROM_P - hip_ROM_N)/hip_ROM_N);
    fprintf('  This indicates spastic circumduction rather than reduced ROM.\n');
    fprintf('  The PID is CORRECTLY reducing hypermetric swing toward healthy ROM.\n');
    fprintf('  Primary outcomes: step asymmetry and GRF symmetry.\n');
    fprintf('==========================================================\n\n');
else
    fprintf('  Gait pattern: Classical hemiparesis (paretic ROM < non-paretic)\n');
    fprintf('  Hip ROM — Paretic: %.1f deg  Non-paretic: %.1f deg\n', hip_ROM_P, hip_ROM_N);
end

% Boost paretic_strength only for extreme circumduction (>90% excess ROM)
if circumduction_detected && (hip_ROM_P > hip_ROM_N * 1.90)
    paretic_strength_original = paretic_strength;
    paretic_strength = min(0.80, paretic_strength + 0.12);
    fprintf('Paretic strength adjusted: %.2f -> %.2f (extreme circumduction)\n', ...
            paretic_strength_original, paretic_strength);
else
    fprintf('Paretic strength unchanged: %.2f\n', paretic_strength);
end
% Estimate gait parameters from hip range of motion
% Uses pendulum model: step length ≈ leg_length * (sin(flex) + sin(ext))
% This assumes the leg swings like a pendulum from the hip
hip_max_flex = max(hip_flex_A);   % Maximum hip flexion (forward swing)
hip_max_ext = min(hip_flex_A);    % Maximum hip extension (push-off)

% Calculate step lengths for each leg
step_length_P = leg_length * (sind(hip_max_flex) + sind(abs(hip_max_ext)));
step_length_N = leg_length * (sind(max(hip_flex_N)) + sind(abs(min(hip_flex_N))));

% Stride = two steps (left + right)
stride_length = step_length_P + step_length_N;
stride_length = max(0.4, min(1.8, stride_length));  % Clamp to realistic range

% Calculate walking speed from stride length and assumed cadence
cadence = 90;  % Steps per minute (typical for post-stroke, healthy ~120)
walking_speed = stride_length * (cadence / 60) / 2;
walking_speed = max(0.2, min(1.5, walking_speed));  % Clamp to realistic range

% Gait asymmetry = difference in step lengths between legs
% High asymmetry indicates more severe gait impairment
asymmetry = abs(step_length_P - step_length_N) / ((step_length_P + step_length_N)/2) * 100;

fprintf('Stride: %.3f m, Speed: %.2f m/s, Asymmetry: %.1f%%\n', stride_length, walking_speed, asymmetry);

%% ========== CREATE CONTINUOUS WALKING ==========
% Extend single gait cycle to cover full walkway by repeating pattern
n_strides = ceil(walkway_length / stride_length);  % Number of strides needed
total_time = walkway_length / walking_speed;        % Total simulation time
dt = total_time / (n_strides * n_cycle);           % Time step per frame
n_total = n_strides * n_cycle;                     % Total number of frames

fprintf('\nSimulation: %d strides, %d frames, %.1f seconds\n', n_strides, n_total, total_time);

% Repeat gait cycle pattern for full walkway
hip_A_full = repmat(hip_flex_A, n_strides, 1);
knee_A_full = repmat(knee_flex_A, n_strides, 1);
ankle_A_full = repmat(ankle_flex_A, n_strides, 1);

hip_N_full = repmat(hip_flex_N, n_strides, 1);
knee_N_full = repmat(knee_flex_N, n_strides, 1);
ankle_N_full = repmat(ankle_flex_N, n_strides, 1);

% Apply 50% phase shift to non-paretic leg (alternating gait)
% In normal walking, legs are 180° out of phase
shift = round(n_cycle / 2);
hip_N_full = circshift(hip_N_full, shift);
knee_N_full = circshift(knee_N_full, shift);
ankle_N_full = circshift(ankle_N_full, shift);

% Create time vector
t = linspace(0, total_time, n_total)';

%% ========== COMPUTE INITIAL KINEMATICS (BEFORE BALANCE) ==========
% Forward kinematics: calculate 3D positions from joint angles
% Kinematic chain: pelvis → hip → knee → ankle → toe
% This creates the 3D skeleton representation for visualization
fprintf('\nComputing initial kinematics...\n');

pelvis_height = L_thigh + L_shank;  % Standing pelvis height (approximate)

% Pre-allocate position arrays [x, y, z] for each body point
pelvis_pos = zeros(n_total, 3);
hip_A_pos = zeros(n_total, 3);
hip_N_pos = zeros(n_total, 3);
knee_A_pos = zeros(n_total, 3);
knee_N_pos = zeros(n_total, 3);
ankle_A_pos = zeros(n_total, 3);
ankle_N_pos = zeros(n_total, 3);
toe_A_pos = zeros(n_total, 3);
toe_N_pos = zeros(n_total, 3);
CoM_pos = zeros(n_total, 3);        % Center of Mass
terrain_z = zeros(n_total, 1);       % Ground height at each frame

% Calculate positions for each frame
for i = 1:n_total
    % Forward position based on constant walking speed
    x = walking_speed * t(i);
    y = 0;  % Centered on walkway
    phase = mod(i-1, n_cycle) / n_cycle;  % Phase within gait cycle (0-1)
    
    % Get terrain height at current position
    terrain_z(i) = terrain_height(x, y, terrain_type);
    
    % Natural pelvis motion during walking
    % Vertical bobbing: pelvis rises and falls ~1.2cm each step
    % Lateral sway: pelvis shifts side-to-side ~1.5cm
    z_bob = 0.012 * sin(2 * pi * phase);   % ~1.2cm vertical oscillation
    y_sway = 0.015 * sin(2 * pi * phase);  % ~1.5cm lateral sway
    
    % Pelvis position (base of kinematic chain)
    pelvis_pos(i,:) = [x, y_sway, pelvis_height + z_bob + terrain_z(i)];
    
    % Hip joint positions (offset from pelvis center by half pelvis width)
    hip_A_pos(i,:) = pelvis_pos(i,:) + [0, L_pelvis/2, 0];   % Assisted side
    hip_N_pos(i,:) = pelvis_pos(i,:) + [0, -L_pelvis/2, 0];  % Non-paretic side
    
    % ----- ASSISTED PARETIC LEG KINEMATICS -----
    hf = deg2rad(hip_A_full(i));           % Hip flexion angle (radians)
    kf = deg2rad(abs(knee_A_full(i)));     % Knee flexion (always positive)
    af = deg2rad(ankle_A_full(i));         % Ankle angle
    
    % Thigh segment: hip to knee
    % Thigh hangs from hip, rotated by hip flexion angle
    thigh = L_thigh * [sin(hf), 0, -cos(hf)];
    knee_A_pos(i,:) = hip_A_pos(i,:) + thigh;
    
    % Shank segment: knee to ankle
    % Shank angle = hip angle - knee flexion (knee bends leg backward)
    shank_ang = hf - kf;  % Shank angle relative to vertical
    shank = L_shank * [sin(shank_ang), 0, -cos(shank_ang)];
    ankle_A_pos(i,:) = knee_A_pos(i,:) + shank;
    
    % Foot/toe position
    % Foot angle depends on ankle dorsi/plantarflexion
    foot_ang = shank_ang + af - pi/2;
    toe_A_pos(i,:) = ankle_A_pos(i,:) + L_foot * 0.7 * [cos(foot_ang), 0, sin(foot_ang)];
    
    % ----- NON-PARETIC LEG KINEMATICS -----
    % Same calculation for healthy leg
    hf = deg2rad(hip_N_full(i));
    kf = deg2rad(abs(knee_N_full(i)));
    af = deg2rad(ankle_N_full(i));
    
    thigh = L_thigh * [sin(hf), 0, -cos(hf)];
    knee_N_pos(i,:) = hip_N_pos(i,:) + thigh;
    
    shank_ang = hf - kf;
    shank = L_shank * [sin(shank_ang), 0, -cos(shank_ang)];
    ankle_N_pos(i,:) = knee_N_pos(i,:) + shank;
    
    foot_ang = shank_ang + af - pi/2;
    toe_N_pos(i,:) = ankle_N_pos(i,:) + L_foot * 0.7 * [cos(foot_ang), 0, sin(foot_ang)];
    
    % ----- CENTER OF MASS CALCULATION -----
    % Weighted average of segment CoM positions
    % Each segment's CoM is approximately at its midpoint
    trunk_CoM = pelvis_pos(i,:) + [0, 0, 0.3 * 0.3 * patient_height];  % Trunk above pelvis
    thigh_A_CoM = (hip_A_pos(i,:) + knee_A_pos(i,:)) / 2;
    shank_A_CoM = (knee_A_pos(i,:) + ankle_A_pos(i,:)) / 2;
    thigh_N_CoM = (hip_N_pos(i,:) + knee_N_pos(i,:)) / 2;
    shank_N_CoM = (knee_N_pos(i,:) + ankle_N_pos(i,:)) / 2;
    
    % Total body CoM = mass-weighted average
    CoM_pos(i,:) = (mass_trunk * trunk_CoM + ...
                    mass_thigh * thigh_A_CoM + mass_shank * shank_A_CoM + ...
                    mass_thigh * thigh_N_CoM + mass_shank * shank_N_CoM) / ...
                   (mass_trunk + 2*mass_thigh + 2*mass_shank);
end

% Ground correction: shift all positions so feet touch ground
% Find lowest point and adjust everything up
min_z = min([ankle_A_pos(:,3); ankle_N_pos(:,3); toe_A_pos(:,3); toe_N_pos(:,3)]);
offset = -min_z + 0.005;  % Small offset prevents ground penetration

% Apply vertical offset to all positions
pelvis_pos(:,3) = pelvis_pos(:,3) + offset;
hip_A_pos(:,3) = hip_A_pos(:,3) + offset;
hip_N_pos(:,3) = hip_N_pos(:,3) + offset;
knee_A_pos(:,3) = knee_A_pos(:,3) + offset;
knee_N_pos(:,3) = knee_N_pos(:,3) + offset;
ankle_A_pos(:,3) = ankle_A_pos(:,3) + offset;
ankle_N_pos(:,3) = ankle_N_pos(:,3) + offset;
toe_A_pos(:,3) = toe_A_pos(:,3) + offset;
toe_N_pos(:,3) = toe_N_pos(:,3) + offset;
CoM_pos(:,3) = CoM_pos(:,3) + offset;

%% ========== GAIT PHASE DETECTION ==========
% Determine when each foot is in stance (on ground) vs swing (in air).
%
% Two methods available:
%   1. FORCE PLATE DATA (preferred): Uses actual IC/TO timing from force plates
%   2. CLEARANCE-BASED (fallback): Estimates stance from foot height
%
% Force plate data is more accurate because it measures actual ground contact,
% while clearance-based detection can miss subtle timing differences.

fprintf('\n========== GAIT PHASE DETECTION ==========\n');

% Initialize stance arrays (now n_total is defined)
stance_A = false(n_total, 1);  % Assisted (paretic) leg stance
stance_N = false(n_total, 1);  % Non-paretic leg stance

if has_gait_events
    % =====================================================================
    % METHOD 1: USE FORCE PLATE GAIT EVENTS (More Accurate)
    % =====================================================================
    % IC (Initial Contact) = heel strike = START of stance phase
    % TO (Toe Off) = end of stance phase = START of swing phase
    %
    % Gait cycle breakdown:
    %   0% = Initial Contact (heel strike)
    %   0-60% = Stance phase (foot on ground)
    %   60% = Toe Off (foot leaves ground)
    %   60-100% = Swing phase (foot in air)
    %   100% = Next Initial Contact
    
    fprintf('Using force plate gait events for stance detection.\n');
    fprintf('  Paretic TO at %.1f%% of gait cycle\n', P_TO_pct);
    fprintf('  Non-paretic TO at %.1f%% of gait cycle\n', N_TO_pct);
    
    % Apply timing to each gait cycle in the simulation
    for stride = 1:n_strides
        cycle_start = (stride - 1) * n_cycle + 1;
        cycle_end = stride * n_cycle;
        
        for k = cycle_start:cycle_end
            % Phase within this cycle (0 to 100%)
            phase_in_cycle = (k - cycle_start) / n_cycle;
            phase_pct = phase_in_cycle * 100;
            
            % ----- ASSISTED (PARETIC) LEG -----
            % Stance from IC (0%) to TO
            if phase_pct <= P_TO_pct
                stance_A(k) = true;
            end
            
            % ----- NON-PARETIC LEG -----
            % Non-paretic is shifted by 50% (half a gait cycle)
            % When paretic heel strikes, non-paretic is at midstance
            shifted_phase = mod(phase_pct + 50, 100);
            
            % Stance from IC (0%) to TO
            if shifted_phase <= N_TO_pct
                stance_N(k) = true;
            end
        end
    end
    
    % Report stance percentages
    fprintf('\nStance duration (from force plate events):\n');
    fprintf('  Assisted (paretic):  %.1f%%\n', 100 * sum(stance_A) / n_total);
    fprintf('  Non-paretic:         %.1f%%\n', 100 * sum(stance_N) / n_total);
    
    % Verify double support phases exist
    double_support_pct = 100 * sum(stance_A & stance_N) / n_total;
    fprintf('  Double support:      %.1f%%\n', double_support_pct);
    
else
    % =====================================================================
    % METHOD 2: CLEARANCE-BASED DETECTION (Fallback)
    % =====================================================================
    % Use foot height above ground to estimate stance/swing
    % Less accurate but works when force plate data is unavailable
    
    fprintf('Using clearance-based stance detection (no force plate data).\n');
    
    % Calculate foot clearance (minimum of ankle and toe height above terrain)
    clearance_A = min(ankle_A_pos(:,3) - terrain_z - offset, ...
                      toe_A_pos(:,3) - terrain_z - offset);
    clearance_N = min(ankle_N_pos(:,3) - terrain_z - offset, ...
                      toe_N_pos(:,3) - terrain_z - offset);
    
    % Dynamic threshold based on minimum clearance in data
    % Foot is in stance if clearance is below threshold
    stance_threshold = min([clearance_A; clearance_N]) + 0.02;

    % For circumduction cases the min clearance is deeply negative due to
    % hypermetric swing, causing ~49% false flight phase. Try increasing
    % percentiles until threshold is positive (foot actually on ground).
    if circumduction_detected && stance_threshold < 0
        all_clearance = [clearance_A; clearance_N];
        for pct = [15, 25, 35, 45, 55, 65, 75]
            stance_threshold = prctile(all_clearance, pct);
            if stance_threshold >= 0.002
                break;
            end
        end

        % If all percentiles still negative, clearance-based detection
        % has completely failed for this circumduction patient.
        % Fall back to clinically-derived stance timing:
        % FAC=3 circumduction: paretic ~58%, non-paretic ~62%, DS ~20%
        if stance_threshold < 0
            fprintf('  WARNING: All clearances negative — using clinical stance timing\n');
            p_stance_pct = 58;  % % of gait cycle paretic in stance
            n_stance_pct = 62;  % % of gait cycle non-paretic in stance
            ds_offset    = 10;  % double support starts at this % offset
            for k = 1:n_total
                phase_pct = mod(k-1, n_cycle) / n_cycle * 100;
                stance_A(k) = phase_pct <= p_stance_pct;
                shifted = mod(phase_pct + 50, 100);
                stance_N(k) = shifted <= n_stance_pct;
            end
            fprintf('  Clinical stance — Paretic: %.0f%%  Non-paretic: %.0f%%\n', ...
                    p_stance_pct, n_stance_pct);
            % Skip the clearance threshold assignment
            stance_threshold = 0;
        end
        fprintf('  Stance threshold (percentile-based for circumduction): %.3f m\n', stance_threshold);
    else
        fprintf('  Stance threshold (auto): %.3f m\n', stance_threshold);
    end
    
    % Detect stance phases
    stance_A = clearance_A < stance_threshold;
    stance_N = clearance_N < stance_threshold;
    
    fprintf('\nStance duration (from clearance):\n');
    fprintf('  Assisted (paretic):  %.1f%%\n', 100*sum(stance_A)/n_total);
    fprintf('  Non-paretic:         %.1f%%\n', 100*sum(stance_N)/n_total);
end

% =========================================================================
% GAIT PHASE LABELS (for analysis and visualization)
% =========================================================================
% Create detailed gait phase labels for each frame
%   0 = Flight (both feet off ground - rare in normal walking)
%   1 = Single support paretic (only paretic foot on ground)
%   2 = Single support non-paretic (only non-paretic foot on ground)
%   3 = Double support (both feet on ground)

gait_phase = zeros(n_total, 1);

for k = 1:n_total
    if stance_A(k) && stance_N(k)
        gait_phase(k) = 3;  % Double support
    elseif stance_A(k)
        gait_phase(k) = 1;  % Single support paretic
    elseif stance_N(k)
        gait_phase(k) = 2;  % Single support non-paretic
    else
        gait_phase(k) = 0;  % Flight phase
    end
end

% Calculate phase statistics
pct_double = 100 * sum(gait_phase == 3) / n_total;
pct_single_P = 100 * sum(gait_phase == 1) / n_total;
pct_single_N = 100 * sum(gait_phase == 2) / n_total;
pct_flight = 100 * sum(gait_phase == 0) / n_total;

fprintf('\nGait phase distribution:\n');
fprintf('  Double support:         %5.1f%%\n', pct_double);
fprintf('  Single support (P):     %5.1f%%\n', pct_single_P);
fprintf('  Single support (N):     %5.1f%%\n', pct_single_N);
fprintf('  Flight:                 %5.1f%%\n', pct_flight);
fprintf('==========================================\n');

%% ========== CoM VELOCITY & ACCELERATION (IMPROVED SMOOTHING) ==========
% Calculate CoM dynamics using numerical differentiation
% Heavy smoothing is critical to reduce noise in balance calculations
% Noisy acceleration → noisy ZMP → unstable balance controller
CoM_vel = zeros(n_total, 3);
CoM_acc = zeros(n_total, 3);

% Central difference for velocity: v = (x[i+1] - x[i-1]) / (2*dt)
% More accurate than forward/backward difference
for i = 2:n_total-1
    CoM_vel(i,:) = (CoM_pos(i+1,:) - CoM_pos(i-1,:)) / (2*dt);
end
CoM_vel(1,:) = CoM_vel(2,:);       % Copy boundary values
CoM_vel(end,:) = CoM_vel(end-1,:);

% Central difference for acceleration: a = (x[i+1] - 2*x[i] + x[i-1]) / dt²
for i = 2:n_total-1
    CoM_acc(i,:) = (CoM_pos(i+1,:) - 2*CoM_pos(i,:) + CoM_pos(i-1,:)) / (dt^2);
end
CoM_acc(1,:) = CoM_acc(2,:);
CoM_acc(end,:) = CoM_acc(end-1,:);

% Apply heavy smoothing (window size based on terrain difficulty)
% Larger window = more smoothing = less noise but also less responsiveness
window = max(smoothing_factor, round(n_cycle / 5));
CoM_vel = movmean(CoM_vel, window);
CoM_acc = movmean(CoM_acc, window);

% Additional Gaussian smoothing if available (R2017a+)
if exist('smoothdata', 'file')
    CoM_acc = smoothdata(CoM_acc, 'gaussian', window);
end

% Clamp accelerations to physically realistic values
% Human walking typically stays within these bounds
max_acc = 5.0;  % m/s² - typical maximum for human walking
CoM_acc(:,1) = max(-max_acc, min(max_acc, CoM_acc(:,1)));     % Horizontal (AP)
CoM_acc(:,2) = max(-max_acc, min(max_acc, CoM_acc(:,2)));     % Lateral (ML)
CoM_acc(:,3) = max(-max_acc*2, min(max_acc*2, CoM_acc(:,3))); % Vertical (can be higher)

%% ========== ZMP CALCULATION (BEFORE BALANCE) ==========
% Zero Moment Point (ZMP) is where ground reaction force must act
% If ZMP leaves support polygon (area under feet), person will fall
%
% METHOD 1 (preferred): Direct from force plate GRF + Moment data
%   ZMP_x = -Moment_y / GRF_z
%   ZMP_y =  Moment_x / GRF_z
%   Exact formula — no kinematic estimation, no noise amplification.
%
% METHOD 2 (fallback): Inverted pendulum model from CoM kinematics
%   ZMP_x = CoM_x - h * (a_x / (g + a_z))
fprintf('\nCalculating ZMP...\n');

zmp_pos = zeros(n_total, 2);
zmp_margin = zeros(n_total, 1);
zmp_stable = zeros(n_total, 1);

% ---- Attempt measured ZMP from force plate data ----------------------
has_measured_zmp = false;
zmp_from_measurement = zeros(n_cycle, 2);  % one cycle, tiled later

if isfield(P, 'GroundReactionForce') && isfield(P, 'GroundReactionMoment')
    grf_Pz_raw = P.GroundReactionForce.z;
    grm_Py_raw = P.GroundReactionMoment.y;
    grm_Px_raw = P.GroundReactionMoment.x;
    grf_Nz_raw = N.GroundReactionForce.z;
    grm_Ny_raw = N.GroundReactionMoment.y;
    grm_Nx_raw = N.GroundReactionMoment.x;

    % Average valid columns only (exclude all-NaN columns)
    valid_P = any(grf_Pz_raw > 0, 1) & ~all(isnan(grf_Pz_raw), 1);
    valid_N = any(grf_Nz_raw > 0, 1) & ~all(isnan(grf_Nz_raw), 1);

    if any(valid_P)
        grf_Pz  = mean(grf_Pz_raw(:, valid_P),  2, 'omitnan');
        grm_Py  = mean(grm_Py_raw(:, valid_P),  2, 'omitnan');
        grm_Px  = mean(grm_Px_raw(:, valid_P),  2, 'omitnan');
    else
        grf_Pz = []; 
    end

    if any(valid_N)
        grf_Nz  = mean(grf_Nz_raw(:, valid_N),  2, 'omitnan');
        grm_Ny  = mean(grm_Ny_raw(:, valid_N),  2, 'omitnan');
        grm_Nx  = mean(grm_Nx_raw(:, valid_N),  2, 'omitnan');
    else
        grf_Nz = [];
    end

    % Convert units if N/kg (max < 50 implies normalised)
    if ~isempty(grf_Pz) && max(grf_Pz) < 50
        grf_Pz = grf_Pz * patient_mass;
        grm_Py = grm_Py * patient_mass;
        grm_Px = grm_Px * patient_mass;
        fprintf('  GRF/Moment units: N/kg -> converted to N\n');
    end
    if ~isempty(grf_Nz) && max(grf_Nz) < 50
        grf_Nz = grf_Nz * patient_mass;
        grm_Ny = grm_Ny * patient_mass;
        grm_Nx = grm_Nx * patient_mass;
    end

    % Resample from 1001 to n_cycle points
    x_src = linspace(0, 1, 1001);
    x_dst = linspace(0, 1, n_cycle);

    if ~isempty(grf_Pz)
        grf_Pz_c = interp1(x_src, grf_Pz, x_dst, 'pchip')';
        grm_Py_c = interp1(x_src, grm_Py, x_dst, 'pchip')';
        grm_Px_c = interp1(x_src, grm_Px, x_dst, 'pchip')';
    else
        grf_Pz_c = zeros(n_cycle,1);
        grm_Py_c = zeros(n_cycle,1);
        grm_Px_c = zeros(n_cycle,1);
    end

    if ~isempty(grf_Nz)
        grf_Nz_c = interp1(x_src, grf_Nz, x_dst, 'pchip')';
        grm_Ny_c = interp1(x_src, grm_Ny, x_dst, 'pchip')';
        grm_Nx_c = interp1(x_src, grm_Nx, x_dst, 'pchip')';
    else
        grf_Nz_c = zeros(n_cycle,1);
        grm_Ny_c = zeros(n_cycle,1);
        grm_Nx_c = zeros(n_cycle,1);
    end

    % Compute ZMP per cycle frame, weighted by GRF
    grf_threshold_N = 10;  % Newtons
    for ci = 1:n_cycle
        fz_P = grf_Pz_c(ci);
        fz_N = grf_Nz_c(ci);
        fz_tot = fz_P + fz_N;
        if fz_tot > grf_threshold_N
            if fz_P > grf_threshold_N && fz_N > grf_threshold_N
                zx_P = -grm_Py_c(ci)/fz_P;  zy_P = grm_Px_c(ci)/fz_P;
                zx_N = -grm_Ny_c(ci)/fz_N;  zy_N = grm_Nx_c(ci)/fz_N;
                zmp_from_measurement(ci,1) = (fz_P*zx_P + fz_N*zx_N)/fz_tot;
                zmp_from_measurement(ci,2) = (fz_P*zy_P + fz_N*zy_N)/fz_tot;
            elseif fz_P > grf_threshold_N
                zmp_from_measurement(ci,1) = -grm_Py_c(ci)/fz_P;
                zmp_from_measurement(ci,2) =  grm_Px_c(ci)/fz_P;
            elseif fz_N > grf_threshold_N
                zmp_from_measurement(ci,1) = -grm_Ny_c(ci)/fz_N;
                zmp_from_measurement(ci,2) =  grm_Nx_c(ci)/fz_N;
            end
        end
    end

    % Validate — ZMP should be within ±0.5m (foot-relative coordinates)
    zmp_range = max(abs(zmp_from_measurement(:,1)));
    if zmp_range > 0.001 && zmp_range < 0.5
        % Clamp to foot boundary — outlier frames at heel strike/toe-off
        % transitions can produce CoP values slightly outside foot outline
        % due to trial averaging noise. Clamp to foot dimensions.
        max_zmp_x = foot_length * 0.6;  % allow slight overshoot of foot length
        max_zmp_y = foot_width  * 0.6;
        zmp_from_measurement(:,1) = max(-max_zmp_x, min(max_zmp_x, zmp_from_measurement(:,1)));
        zmp_from_measurement(:,2) = max(-max_zmp_y, min(max_zmp_y, zmp_from_measurement(:,2)));
        has_measured_zmp = true;
        fprintf('  ZMP from force plate moments (exact). Range: %.3f m\n', zmp_range);
    else
        fprintf('  ZMP moment data invalid (range=%.3f m) — kinematic fallback\n', zmp_range);
    end
end

for i = 1:n_total
    ground_z = terrain_z(i);
    h_com = CoM_pos(i,3) - ground_z - offset;

    if has_measured_zmp
        % METHOD 1: measured CoP in foot-relative coords
        % Add current ankle position to get global frame ZMP
        ci = mod(i-1, n_cycle) + 1;
        if zmp_from_measurement(ci,1) ~= 0 || zmp_from_measurement(ci,2) ~= 0
            % Use stance foot ankle as reference origin
            if stance_A(i)
                ref_x = ankle_A_pos(i,1);
                ref_y = ankle_A_pos(i,2);
            elseif stance_N(i)
                ref_x = ankle_N_pos(i,1);
                ref_y = ankle_N_pos(i,2);
            else
                ref_x = CoM_pos(i,1);
                ref_y = CoM_pos(i,2);
            end
            zmp_x = ref_x + zmp_from_measurement(ci,1);  % CoP in local foot frame, no offset needed
            zmp_y = ref_y + zmp_from_measurement(ci,2);
        else
            % Flight phase — kinematic fallback
            denom = max(g + CoM_acc(i,3), 0.1);
            zmp_x = CoM_pos(i,1) - h_com * (CoM_acc(i,1) / denom);
            zmp_y = CoM_pos(i,2) - h_com * (CoM_acc(i,2) / denom);
        end
    else
        % METHOD 2: ZMP from inverted pendulum model
        denom = max(g + CoM_acc(i,3), 0.1);
        zmp_x = CoM_pos(i,1) - h_com * (CoM_acc(i,1) / denom);
        zmp_y = CoM_pos(i,2) - h_com * (CoM_acc(i,2) / denom);
    end
    
    zmp_pos(i,:) = [zmp_x, zmp_y];
    
    % Define support polygon based on stance phase
    % Support polygon = convex hull of all foot contact points
    if stance_A(i) && stance_N(i)
        % Double support: quadrilateral connecting both feet
        poly_x = [ankle_A_pos(i,1) - foot_length/2, ankle_A_pos(i,1) + foot_length/2, ...
                  ankle_N_pos(i,1) + foot_length/2, ankle_N_pos(i,1) - foot_length/2];
        poly_y = [ankle_A_pos(i,2) - foot_width/2, ankle_A_pos(i,2) + foot_width/2, ...
                  ankle_N_pos(i,2) + foot_width/2, ankle_N_pos(i,2) - foot_width/2];
    elseif stance_A(i)
        % Single support on assisted leg: rectangle under that foot
        poly_x = ankle_A_pos(i,1) + [-1, 1, 1, -1] * foot_length/2;
        poly_y = ankle_A_pos(i,2) + [-1, -1, 1, 1] * foot_width/2;
    elseif stance_N(i)
        % Single support on non-paretic leg
        poly_x = ankle_N_pos(i,1) + [-1, 1, 1, -1] * foot_length/2;
        poly_y = ankle_N_pos(i,2) + [-1, -1, 1, 1] * foot_width/2;
    else
        % Flight phase: small area at CoM projection (unstable by definition)
        poly_x = CoM_pos(i,1) + [-1, 1, 1, -1] * 0.1;
        poly_y = CoM_pos(i,2) + [-1, -1, 1, 1] * 0.1;
    end
    
    % Check if ZMP is inside support polygon using MATLAB's inpolygon
    zmp_stable(i) = inpolygon(zmp_x, zmp_y, poly_x, poly_y);
    
    % Calculate stability margin (distance from ZMP to polygon edge)
    % Positive margin = safe, negative = unstable
    poly_center_x = mean(poly_x);
    poly_center_y = mean(poly_y);
    
    dist_to_center = sqrt((zmp_x - poly_center_x)^2 + (zmp_y - poly_center_y)^2);
    poly_radius = min([max(poly_x) - min(poly_x), max(poly_y) - min(poly_y)]) / 2;
    
    if zmp_stable(i)
        zmp_margin(i) = poly_radius - dist_to_center;  % Positive = inside
    else
        zmp_margin(i) = -(dist_to_center - poly_radius);  % Negative = outside
    end
end

%% ========== IMPROVED BALANCE CONTROLLER (ANKLE/HIP STRATEGY) ==========
% Human-inspired balance control using three strategies:
%   1. ANKLE STRATEGY: Small perturbations - adjust ankle torque only
%      Used for gentle corrections, most energy-efficient
%   2. HIP STRATEGY: Medium perturbations - use both ankle and hip torque
%      Faster corrections for larger disturbances
%   3. STEPPING STRATEGY: Large perturbations - maximum effort
%      Last resort before falling, would require taking a step
%
% The controller mimics how humans naturally balance:
% - Small sway: ankle muscles adjust
% - Larger sway: hip flexors/extensors engage
% - Very large sway: take a step to recover
fprintf('\n========== BALANCE CONTROLLER ==========\n');

% Pre-allocate torque and strategy arrays
ankle_torque_A = zeros(n_total, 1);  % Ankle torque for assisted leg (Nm)
ankle_torque_N = zeros(n_total, 1);  % Ankle torque for non-paretic leg
hip_torque_A = zeros(n_total, 1);    % Hip torque for assisted leg
hip_torque_N = zeros(n_total, 1);    % Hip torque for non-paretic leg
balance_strategy = zeros(n_total, 1); % 1=ankle, 2=hip, 3=step

% Low-pass filter state for smooth ZMP error signal
% Filtering reduces jitter in motor commands
alpha_lpf = 0.3;  % Filter coefficient (0-1, lower = more smoothing)
filtered_zmp_error_x = 0;
filtered_zmp_error_y = 0;

prev_zmp_error = [0, 0];  % Previous error for derivative calculation

% Run balance controller for each frame
for i = 1:n_total
    % Calculate target ZMP position (center of support polygon)
    % We want ZMP to be at the center for maximum stability margin
    if stance_A(i) && stance_N(i)
        % Double support: midpoint between ankles
        support_center_x = (ankle_A_pos(i,1) + ankle_N_pos(i,1)) / 2;
        support_center_y = (ankle_A_pos(i,2) + ankle_N_pos(i,2)) / 2;
    elseif stance_A(i)
        % Single support: under stance foot
        support_center_x = ankle_A_pos(i,1);
        support_center_y = ankle_A_pos(i,2);
    elseif stance_N(i)
        support_center_x = ankle_N_pos(i,1);
        support_center_y = ankle_N_pos(i,2);
    else
        % Swing phase: no correction possible
        support_center_x = zmp_pos(i,1);
        support_center_y = zmp_pos(i,2);
    end
    
    % Raw ZMP error (difference between target and actual ZMP)
    raw_zmp_error_x = support_center_x - zmp_pos(i,1);
    raw_zmp_error_y = support_center_y - zmp_pos(i,2);
    
    % Apply low-pass filter to reduce jitter
    % filtered = alpha * raw + (1-alpha) * previous_filtered
    filtered_zmp_error_x = alpha_lpf * raw_zmp_error_x + (1 - alpha_lpf) * filtered_zmp_error_x;
    filtered_zmp_error_y = alpha_lpf * raw_zmp_error_y + (1 - alpha_lpf) * filtered_zmp_error_y;
    
    zmp_error_x = filtered_zmp_error_x;
    zmp_error_y = filtered_zmp_error_y;
    zmp_error_mag = sqrt(zmp_error_x^2 + zmp_error_y^2);  % Error magnitude
    
    % Calculate error derivative for damping term
    d_zmp_error_x = (zmp_error_x - prev_zmp_error(1)) / dt;
    d_zmp_error_y = (zmp_error_y - prev_zmp_error(2)) / dt;
    
    % Clamp derivative to prevent noise spikes
    max_d_error = 2.0;  % Maximum derivative (m/s)
    d_zmp_error_x = max(-max_d_error, min(max_d_error, d_zmp_error_x));
    d_zmp_error_y = max(-max_d_error, min(max_d_error, d_zmp_error_y));
    
    % === SELECT BALANCE STRATEGY BASED ON ERROR MAGNITUDE ===
    if zmp_error_mag < ankle_threshold
        % ----- ANKLE STRATEGY -----
        % Small error: ankle torque only (most efficient)
        balance_strategy(i) = 1;
        
        % PD control: torque = Kp*error + Kd*d_error
        tau_ankle_x = Kp_ankle * zmp_error_x + Kd_ankle * d_zmp_error_x;
        tau_ankle_y = Kp_ankle * zmp_error_y + Kd_ankle * d_zmp_error_y;
        tau_ankle = sqrt(tau_ankle_x^2 + tau_ankle_y^2) * sign(tau_ankle_x);
        
        % Distribute torque between legs based on stance
        if stance_A(i) && stance_N(i)
            weight_ratio_A = 0.5;  % Equal distribution in double support
            ankle_torque_A(i) = tau_ankle * weight_ratio_A * paretic_strength;
            ankle_torque_N(i) = tau_ankle * (1 - weight_ratio_A);
        elseif stance_A(i)
            ankle_torque_A(i) = tau_ankle * paretic_strength;
        elseif stance_N(i)
            ankle_torque_N(i) = tau_ankle;
        end
        
    elseif zmp_error_mag < hip_threshold
        % ----- HIP STRATEGY -----
        % Medium error: use both ankle and hip torque
        balance_strategy(i) = 2;
        
        tau_ankle_x = Kp_ankle * zmp_error_x + Kd_ankle * d_zmp_error_x;
        tau_hip_x = Kp_hip * zmp_error_x + Kd_hip * d_zmp_error_x;
        
        tau_ankle = tau_ankle_x;
        tau_hip = tau_hip_x;
        
        if stance_A(i) && stance_N(i)
            ankle_torque_A(i) = tau_ankle * 0.5 * paretic_strength;
            ankle_torque_N(i) = tau_ankle * 0.5;
            hip_torque_A(i) = tau_hip * 0.5 * paretic_strength;
            hip_torque_N(i) = tau_hip * 0.5;
        elseif stance_A(i)
            ankle_torque_A(i) = tau_ankle * paretic_strength;
            hip_torque_A(i) = tau_hip * paretic_strength;
        elseif stance_N(i)
            ankle_torque_N(i) = tau_ankle;
            hip_torque_N(i) = tau_hip;
        end
        
    else
        % ----- STEPPING STRATEGY -----
        % Proportionally scaled — ramps from 0 at hip_threshold to max at
        % 2x hip_threshold. Prevents full saturation torque for borderline
        % violations which was destabilising CoM for sID 1.
        balance_strategy(i) = 3;

        direction = sign(zmp_error_x);
        step_scale = min(1.0, zmp_error_mag / hip_threshold);
        ankle_torque_A(i) = direction * max_ankle_torque * paretic_strength * step_scale;
        ankle_torque_N(i) = direction * max_ankle_torque * step_scale;
        hip_torque_A(i)   = direction * max_hip_torque   * paretic_strength * step_scale;
        hip_torque_N(i)   = direction * max_hip_torque   * step_scale;
    end
    
    % Clamp all torques to motor/muscle limits
    ankle_torque_A(i) = max(-max_ankle_torque, min(max_ankle_torque, ankle_torque_A(i)));
    ankle_torque_N(i) = max(-max_ankle_torque, min(max_ankle_torque, ankle_torque_N(i)));
    hip_torque_A(i) = max(-max_hip_torque, min(max_hip_torque, hip_torque_A(i)));
    hip_torque_N(i) = max(-max_hip_torque, min(max_hip_torque, hip_torque_N(i)));
    
    prev_zmp_error = [zmp_error_x, zmp_error_y];
end

% Smooth torques to reduce motor command jitter
smooth_window = max(3, round(n_cycle / 30));
ankle_torque_A = movmean(ankle_torque_A, smooth_window);
ankle_torque_N = movmean(ankle_torque_N, smooth_window);
hip_torque_A = movmean(hip_torque_A, smooth_window);
hip_torque_N = movmean(hip_torque_N, smooth_window);

% === CONVERT TORQUES TO ANGLE CORRECTIONS ===
% Using simplified muscle model: angle = torque / stiffness
% Higher stiffness = smaller angle change for same torque
ankle_corr_A = ankle_torque_A / ankle_stiffness;
ankle_corr_N = ankle_torque_N / ankle_stiffness;
hip_corr_A = hip_torque_A / hip_stiffness;
hip_corr_N = hip_torque_N / hip_stiffness;

% Smooth angle corrections
ankle_corr_A = movmean(ankle_corr_A, smooth_window);
ankle_corr_N = movmean(ankle_corr_N, smooth_window);
hip_corr_A = movmean(hip_corr_A, smooth_window);
hip_corr_N = movmean(hip_corr_N, smooth_window);

% Clamp to biomechanically realistic limits
max_ankle_corr = deg2rad(15);  % ±15° maximum ankle correction
max_hip_corr = deg2rad(10);    % ±10° maximum hip correction

ankle_corr_A = max(-max_ankle_corr, min(max_ankle_corr, ankle_corr_A));
ankle_corr_N = max(-max_ankle_corr, min(max_ankle_corr, ankle_corr_N));
hip_corr_A = max(-max_hip_corr, min(max_hip_corr, hip_corr_A));
hip_corr_N = max(-max_hip_corr, min(max_hip_corr, hip_corr_N));

% Apply balance corrections to joint angles
ankle_A_balanced = ankle_A_full + rad2deg(ankle_corr_A);
ankle_N_balanced = ankle_N_full + rad2deg(ankle_corr_N);
hip_A_balanced = hip_A_full + rad2deg(hip_corr_A);
hip_N_balanced = hip_N_full + rad2deg(hip_corr_N);

% Calculate strategy usage statistics
n_ankle = sum(balance_strategy == 1);  % Frames using ankle strategy
n_hip = sum(balance_strategy == 2);    % Frames using hip strategy
n_step = sum(balance_strategy == 3);   % Frames needing stepping strategy

fprintf('\n=== BALANCE STRATEGY USAGE ===\n');
fprintf('Ankle strategy: %.1f%% of frames\n', 100 * n_ankle / n_total);
fprintf('Hip strategy:   %.1f%% of frames\n', 100 * n_hip / n_total);
fprintf('Step needed:    %.1f%% of frames\n', 100 * n_step / n_total);

fprintf('\nMean torques:\n');
fprintf('  Ankle (Paretic):     %.1f Nm\n', mean(abs(ankle_torque_A)));
fprintf('  Ankle (Non-Paretic): %.1f Nm\n', mean(abs(ankle_torque_N)));
fprintf('  Hip (Paretic):       %.1f Nm\n', mean(abs(hip_torque_A)));
fprintf('  Hip (Non-Paretic):   %.1f Nm\n', mean(abs(hip_torque_N)));

fprintf('\nAngle corrections:\n');
fprintf('  Ankle (Paretic):     %.1f°\n', mean(abs(rad2deg(ankle_corr_A))));
fprintf('  Ankle (Non-Paretic): %.1f°\n', mean(abs(rad2deg(ankle_corr_N))));
fprintf('  Hip (Paretic):       %.1f°\n', mean(abs(rad2deg(hip_corr_A))));
fprintf('  Hip (Non-Paretic):   %.1f°\n', mean(abs(rad2deg(hip_corr_N))));

%% ========== RECOMPUTE KINEMATICS WITH BALANCE ==========
% Recalculate all body positions using balance-corrected joint angles
% This gives us the "improved" gait with balance controller active
fprintf('\nRecomputing kinematics with balance corrections...\n');

% Pre-allocate balanced position arrays
pelvis_pos_bal = zeros(n_total, 3);
hip_A_pos_bal = zeros(n_total, 3);
hip_N_pos_bal = zeros(n_total, 3);
knee_A_pos_bal = zeros(n_total, 3);
knee_N_pos_bal = zeros(n_total, 3);
ankle_A_pos_bal = zeros(n_total, 3);
ankle_N_pos_bal = zeros(n_total, 3);
toe_A_pos_bal = zeros(n_total, 3);
toe_N_pos_bal = zeros(n_total, 3);
CoM_pos_bal = zeros(n_total, 3);

for i = 1:n_total
    x = walking_speed * t(i);
    y = 0;
    phase = mod(i-1, n_cycle) / n_cycle;
    
    z_bob = 0.012 * sin(2 * pi * phase);
    y_sway = 0.015 * sin(2 * pi * phase);
    
    pelvis_pos_bal(i,:) = [x, y_sway, pelvis_height + z_bob + terrain_z(i)];
    hip_A_pos_bal(i,:) = pelvis_pos_bal(i,:) + [0, L_pelvis/2, 0];
    hip_N_pos_bal(i,:) = pelvis_pos_bal(i,:) + [0, -L_pelvis/2, 0];
    
    % Assisted paretic leg with balance corrections applied
    hf = deg2rad(hip_A_balanced(i));     % Now includes balance correction
    kf = deg2rad(abs(knee_A_full(i)));   % Knee unchanged by balance
    af = deg2rad(ankle_A_balanced(i));   % Now includes balance correction
    
    thigh = L_thigh * [sin(hf), 0, -cos(hf)];
    knee_A_pos_bal(i,:) = hip_A_pos_bal(i,:) + thigh;
    
    shank_ang = hf - kf;
    shank = L_shank * [sin(shank_ang), 0, -cos(shank_ang)];
    ankle_A_pos_bal(i,:) = knee_A_pos_bal(i,:) + shank;
    
    foot_ang = shank_ang + af - pi/2;
    toe_A_pos_bal(i,:) = ankle_A_pos_bal(i,:) + L_foot * 0.7 * [cos(foot_ang), 0, sin(foot_ang)];
    
    % Non-paretic leg with balance corrections
    hf = deg2rad(hip_N_balanced(i));
    kf = deg2rad(abs(knee_N_full(i)));
    af = deg2rad(ankle_N_balanced(i));
    
    thigh = L_thigh * [sin(hf), 0, -cos(hf)];
    knee_N_pos_bal(i,:) = hip_N_pos_bal(i,:) + thigh;
    
    shank_ang = hf - kf;
    shank = L_shank * [sin(shank_ang), 0, -cos(shank_ang)];
    ankle_N_pos_bal(i,:) = knee_N_pos_bal(i,:) + shank;
    
    foot_ang = shank_ang + af - pi/2;
    toe_N_pos_bal(i,:) = ankle_N_pos_bal(i,:) + L_foot * 0.7 * [cos(foot_ang), 0, sin(foot_ang)];
    
    % Recalculate CoM with balanced positions
    trunk_CoM = pelvis_pos_bal(i,:) + [0, 0, 0.3 * 0.3 * patient_height];
    thigh_A_CoM = (hip_A_pos_bal(i,:) + knee_A_pos_bal(i,:)) / 2;
    shank_A_CoM = (knee_A_pos_bal(i,:) + ankle_A_pos_bal(i,:)) / 2;
    thigh_N_CoM = (hip_N_pos_bal(i,:) + knee_N_pos_bal(i,:)) / 2;
    shank_N_CoM = (knee_N_pos_bal(i,:) + ankle_N_pos_bal(i,:)) / 2;
    
    CoM_pos_bal(i,:) = (mass_trunk * trunk_CoM + ...
                        mass_thigh * thigh_A_CoM + mass_shank * shank_A_CoM + ...
                        mass_thigh * thigh_N_CoM + mass_shank * shank_N_CoM) / ...
                       (mass_trunk + 2*mass_thigh + 2*mass_shank);
end

% Ground correction for balanced positions
min_z_bal = min([ankle_A_pos_bal(:,3); ankle_N_pos_bal(:,3); toe_A_pos_bal(:,3); toe_N_pos_bal(:,3)]);
offset_bal = -min_z_bal + 0.005;

pelvis_pos_bal(:,3) = pelvis_pos_bal(:,3) + offset_bal;
hip_A_pos_bal(:,3) = hip_A_pos_bal(:,3) + offset_bal;
hip_N_pos_bal(:,3) = hip_N_pos_bal(:,3) + offset_bal;
knee_A_pos_bal(:,3) = knee_A_pos_bal(:,3) + offset_bal;
knee_N_pos_bal(:,3) = knee_N_pos_bal(:,3) + offset_bal;
ankle_A_pos_bal(:,3) = ankle_A_pos_bal(:,3) + offset_bal;
ankle_N_pos_bal(:,3) = ankle_N_pos_bal(:,3) + offset_bal;
toe_A_pos_bal(:,3) = toe_A_pos_bal(:,3) + offset_bal;
toe_N_pos_bal(:,3) = toe_N_pos_bal(:,3) + offset_bal;
CoM_pos_bal(:,3) = CoM_pos_bal(:,3) + offset_bal;

%% ========== GRF CALCULATION ==========
% Ground Reaction Force from Newton's 2nd law: F = m(g + a)
% The ground must push up on the body with force equal to body weight
% plus any additional force needed to accelerate the CoM
fprintf('Calculating GRF...\n');

GRF_A = zeros(n_total, 3);  % [Fx, Fy, Fz] for assisted leg
GRF_N = zeros(n_total, 3);  % [Fx, Fy, Fz] for non-paretic leg

% Calculate CoM acceleration with balanced positions
CoM_acc_bal = zeros(n_total, 3);
for i = 2:n_total-1
    CoM_acc_bal(i,:) = (CoM_pos_bal(i+1,:) - 2*CoM_pos_bal(i,:) + CoM_pos_bal(i-1,:)) / (dt^2);
end
CoM_acc_bal(1,:) = CoM_acc_bal(2,:);
CoM_acc_bal(end,:) = CoM_acc_bal(end-1,:);

% Heavy smoothing
CoM_acc_bal = movmean(CoM_acc_bal, window);
if exist('smoothdata', 'file')
    CoM_acc_bal = smoothdata(CoM_acc_bal, 'gaussian', window);
end

% Clamp to realistic values
CoM_acc_bal(:,1) = max(-max_acc, min(max_acc, CoM_acc_bal(:,1)));
CoM_acc_bal(:,2) = max(-max_acc, min(max_acc, CoM_acc_bal(:,2)));
CoM_acc_bal(:,3) = max(-max_acc*2, min(max_acc*2, CoM_acc_bal(:,3)));

% Total vertical GRF must support body weight plus vertical acceleration
GRF_total_z = patient_mass * (g + CoM_acc_bal(:,3));

% Distribute GRF between legs based on stance phase
for i = 1:n_total
    if stance_A(i) && stance_N(i)
        % Double support: distribute based on gait phase
        phase = mod(i-1, n_cycle) / n_cycle;
        
        % Weight transfer pattern during double support
        % At heel strike, weight shifts onto new stance leg
        if phase < 0.1
            ratio = 0.3 + 0.4 * (phase / 0.1);  % Loading onto paretic
        elseif phase > 0.5 && phase < 0.6
            ratio = 0.7 - 0.4 * ((phase - 0.5) / 0.1);  % Unloading paretic
        else
            ratio = 0.5;  % Equal distribution
        end
        
        GRF_A(i,3) = GRF_total_z(i) * ratio;
        GRF_N(i,3) = GRF_total_z(i) * (1 - ratio);
    elseif stance_A(i)
        % Single support on paretic leg - bears all weight
        GRF_A(i,3) = GRF_total_z(i);
    elseif stance_N(i)
        % Single support on non-paretic leg
        GRF_N(i,3) = GRF_total_z(i);
    end
    
    % Horizontal GRF components from horizontal acceleration
    % F = ma in each direction
    if stance_A(i)
        GRF_A(i,1) = patient_mass * CoM_acc_bal(i,1) * 0.5;  % AP force
        GRF_A(i,2) = patient_mass * CoM_acc_bal(i,2) * 0.3;  % ML force
    end
    if stance_N(i)
        GRF_N(i,1) = patient_mass * CoM_acc_bal(i,1) * 0.5;
        GRF_N(i,2) = patient_mass * CoM_acc_bal(i,2) * 0.3;
    end
end

% Smooth and ensure non-negative vertical GRF
% Ground can only push, not pull
GRF_A = movmean(GRF_A, window);
GRF_N = movmean(GRF_N, window);
GRF_A(:,3) = max(0, GRF_A(:,3));
GRF_N(:,3) = max(0, GRF_N(:,3));

% Normalize to body weight for clinical interpretation
% GRF is often reported as percentage of body weight
GRF_A_BW = GRF_A / body_weight;
GRF_N_BW = GRF_N / body_weight;

%% ========== ZMP WITH BALANCE ==========
% Recalculate ZMP using balanced kinematics to verify improvement
fprintf('Recalculating ZMP with balance...\n');

zmp_pos_bal = zeros(n_total, 2);
zmp_margin_bal = zeros(n_total, 1);
zmp_stable_bal = zeros(n_total, 1);

for i = 1:n_total
    ground_z = terrain_z(i);
    h_com = CoM_pos_bal(i,3) - ground_z - offset_bal;

    if has_measured_zmp
        ci = mod(i-1, n_cycle) + 1;
        if zmp_from_measurement(ci,1) ~= 0 || zmp_from_measurement(ci,2) ~= 0
            if stance_A(i)
                ref_x = ankle_A_pos_bal(i,1);
                ref_y = ankle_A_pos_bal(i,2);
            elseif stance_N(i)
                ref_x = ankle_N_pos_bal(i,1);
                ref_y = ankle_N_pos_bal(i,2);
            else
                ref_x = CoM_pos_bal(i,1);
                ref_y = CoM_pos_bal(i,2);
            end
            zmp_x = ref_x + zmp_from_measurement(ci,1);  % CoP in local foot frame, no offset needed
            zmp_y = ref_y + zmp_from_measurement(ci,2);
        else
            denom = max(g + CoM_acc_bal(i,3), 0.1);
            zmp_x = CoM_pos_bal(i,1) - h_com * (CoM_acc_bal(i,1) / denom);
            zmp_y = CoM_pos_bal(i,2) - h_com * (CoM_acc_bal(i,2) / denom);
        end
    else
        denom = max(g + CoM_acc_bal(i,3), 0.1);
        zmp_x = CoM_pos_bal(i,1) - h_com * (CoM_acc_bal(i,1) / denom);
        zmp_y = CoM_pos_bal(i,2) - h_com * (CoM_acc_bal(i,2) / denom);
    end
    
    zmp_pos_bal(i,:) = [zmp_x, zmp_y];
    
    % Support polygon with balanced positions
    if stance_A(i) && stance_N(i)
        poly_x = [ankle_A_pos_bal(i,1) - foot_length/2, ankle_A_pos_bal(i,1) + foot_length/2, ...
                  ankle_N_pos_bal(i,1) + foot_length/2, ankle_N_pos_bal(i,1) - foot_length/2];
        poly_y = [ankle_A_pos_bal(i,2) - foot_width/2, ankle_A_pos_bal(i,2) + foot_width/2, ...
                  ankle_N_pos_bal(i,2) + foot_width/2, ankle_N_pos_bal(i,2) - foot_width/2];
    elseif stance_A(i)
        poly_x = ankle_A_pos_bal(i,1) + [-1, 1, 1, -1] * foot_length/2;
        poly_y = ankle_A_pos_bal(i,2) + [-1, -1, 1, 1] * foot_width/2;
    elseif stance_N(i)
        poly_x = ankle_N_pos_bal(i,1) + [-1, 1, 1, -1] * foot_length/2;
        poly_y = ankle_N_pos_bal(i,2) + [-1, -1, 1, 1] * foot_width/2;
    else
        poly_x = CoM_pos_bal(i,1) + [-1, 1, 1, -1] * 0.1;
        poly_y = CoM_pos_bal(i,2) + [-1, -1, 1, 1] * 0.1;
    end
    
    zmp_stable_bal(i) = inpolygon(zmp_x, zmp_y, poly_x, poly_y);
    
    poly_center_x = mean(poly_x);
    poly_center_y = mean(poly_y);
    
    dist_to_center = sqrt((zmp_x - poly_center_x)^2 + (zmp_y - poly_center_y)^2);
    poly_radius = min([max(poly_x) - min(poly_x), max(poly_y) - min(poly_y)]) / 2;
    
    if zmp_stable_bal(i)
        zmp_margin_bal(i) = poly_radius - dist_to_center;
    else
        zmp_margin_bal(i) = -(dist_to_center - poly_radius);
    end
end

% Compare before/after stability
stability_before = 100 * sum(zmp_stable) / n_total;
stability_after = 100 * sum(zmp_stable_bal) / n_total;

fprintf('\n=== ZMP STABILITY IMPROVEMENT ===\n');
fprintf('Before balance control: %.1f%%\n', stability_before);
fprintf('After balance control:  %.1f%%\n', stability_after);
fprintf('Improvement: %.1f%%\n', stability_after - stability_before);

%% ========== TERRAIN ROBUSTNESS METRICS ==========
% Comprehensive metrics for outdoor deployment readiness assessment
% These metrics help determine if the exoskeleton + controller system
% is ready for use in challenging outdoor environments
fprintf('\n========== TERRAIN ROBUSTNESS ANALYSIS ==========\n');

% ===== 1. STABILITY METRICS =====
% Percentage of time ZMP stays inside support polygon
stability_pct_before = 100 * sum(zmp_stable) / n_total;
stability_pct_after = 100 * sum(zmp_stable_bal) / n_total;

% Safety margin: average distance from ZMP to support edge
mean_margin_before = mean(zmp_margin) * 100;  % Convert to cm
mean_margin_after = mean(zmp_margin_bal) * 100;

min_margin_before = min(zmp_margin) * 100;
min_margin_after = min(zmp_margin_bal) * 100;

% Critical zone: time with margin < 2cm (high fall risk)
critical_threshold = 0.02;  % 2cm
critical_pct_before = 100 * sum(zmp_margin < critical_threshold) / n_total;
critical_pct_after = 100 * sum(zmp_margin_bal < critical_threshold) / n_total;

fprintf('\n--- STABILITY METRICS ---\n');
fprintf('                           Before    After    Target\n');
fprintf('ZMP Stability (%%):         %6.1f   %6.1f    >95%%\n', stability_pct_before, stability_pct_after);
fprintf('Mean Margin (cm):          %6.2f   %6.2f    >3 cm\n', mean_margin_before, mean_margin_after);
fprintf('Min Margin (cm):           %6.2f   %6.2f    >0 cm\n', min_margin_before, min_margin_after);
fprintf('Critical Zone (%%):         %6.1f   %6.1f    <5%%\n', critical_pct_before, critical_pct_after);

% ===== 2. SMOOTHNESS METRICS (CoM Jerk) =====
% Jerk = rate of change of acceleration (lower = smoother)
% High jerk indicates jerky, uncomfortable movement
CoM_jerk = zeros(n_total, 3);
for i = 2:n_total-1
    CoM_jerk(i,:) = (CoM_acc_bal(i+1,:) - CoM_acc_bal(i-1,:)) / (2*dt);
end
CoM_jerk(1,:) = CoM_jerk(2,:);
CoM_jerk(end,:) = CoM_jerk(end-1,:);
CoM_jerk = movmean(CoM_jerk, window);

jerk_magnitude = sqrt(sum(CoM_jerk.^2, 2));
mean_jerk = mean(jerk_magnitude);
peak_jerk = max(jerk_magnitude);
rms_jerk = sqrt(mean(jerk_magnitude.^2));

% Original jerk for comparison
CoM_jerk_orig = zeros(n_total, 3);
for i = 2:n_total-1
    CoM_jerk_orig(i,:) = (CoM_acc(i+1,:) - CoM_acc(i-1,:)) / (2*dt);
end
CoM_jerk_orig(1,:) = CoM_jerk_orig(2,:);
CoM_jerk_orig(end,:) = CoM_jerk_orig(end-1,:);
CoM_jerk_orig = movmean(CoM_jerk_orig, window);

jerk_magnitude_orig = sqrt(sum(CoM_jerk_orig.^2, 2));
mean_jerk_orig = mean(jerk_magnitude_orig);

fprintf('\n--- SMOOTHNESS METRICS (CoM Jerk) ---\n');
fprintf('                           Before    After    Target\n');
fprintf('Mean Jerk (m/s³):          %6.1f   %6.1f    <5\n', mean_jerk_orig, mean_jerk);
fprintf('Peak Jerk (m/s³):          %6.1f   %6.1f    <20\n', max(jerk_magnitude_orig), peak_jerk);
fprintf('RMS Jerk (m/s³):           %6.1f   %6.1f    <8\n', sqrt(mean(jerk_magnitude_orig.^2)), rms_jerk);

% ===== 3. EFFORT METRICS =====
% Torque requirements for balance - indicates motor/energy needs
peak_ankle_torque_A = max(abs(ankle_torque_A));
peak_ankle_torque_N = max(abs(ankle_torque_N));
peak_hip_torque_A = max(abs(hip_torque_A));
peak_hip_torque_N = max(abs(hip_torque_N));

mean_ankle_torque_A = mean(abs(ankle_torque_A));
mean_ankle_torque_N = mean(abs(ankle_torque_N));
mean_hip_torque_A = mean(abs(hip_torque_A));
mean_hip_torque_N = mean(abs(hip_torque_N));

% Total effort = integral of torque over time (energy-like metric)
total_effort_ankle = sum(abs(ankle_torque_A) + abs(ankle_torque_N)) * dt;
total_effort_hip = sum(abs(hip_torque_A) + abs(hip_torque_N)) * dt;
total_effort = total_effort_ankle + total_effort_hip;

% Asymmetry in effort between legs
effort_asymmetry_ankle = abs(sum(abs(ankle_torque_A)) - sum(abs(ankle_torque_N))) / ...
                         (sum(abs(ankle_torque_A)) + sum(abs(ankle_torque_N)) + 0.001) * 100;
effort_asymmetry_hip = abs(sum(abs(hip_torque_A)) - sum(abs(hip_torque_N))) / ...
                       (sum(abs(hip_torque_A)) + sum(abs(hip_torque_N)) + 0.001) * 100;

fprintf('\n--- EFFORT METRICS (Torque) ---\n');
fprintf('                           Paretic  Non-Par   Limit\n');
fprintf('Peak Ankle Torque (Nm):    %6.1f   %6.1f    <%.0f\n', peak_ankle_torque_A, peak_ankle_torque_N, max_ankle_torque);
fprintf('Peak Hip Torque (Nm):      %6.1f   %6.1f    <%.0f\n', peak_hip_torque_A, peak_hip_torque_N, max_hip_torque);
fprintf('Mean Ankle Torque (Nm):    %6.1f   %6.1f\n', mean_ankle_torque_A, mean_ankle_torque_N);
fprintf('Mean Hip Torque (Nm):      %6.1f   %6.1f\n', mean_hip_torque_A, mean_hip_torque_N);
fprintf('\nTotal Effort (Nm·s):       %.1f\n', total_effort);
fprintf('Effort Asymmetry - Ankle:  %.1f%%\n', effort_asymmetry_ankle);
fprintf('Effort Asymmetry - Hip:    %.1f%%\n', effort_asymmetry_hip);

% ===== 4. ADAPTABILITY METRICS =====
% How often controller switches between strategies
strategy_switches = sum(abs(diff(balance_strategy)) > 0);
switch_rate = strategy_switches / total_time;  % Switches per second

% Time in each strategy
time_ankle = sum(balance_strategy == 1) / n_total * 100;
time_hip = sum(balance_strategy == 2) / n_total * 100;
time_step = sum(balance_strategy == 3) / n_total * 100;

% Terrain difficulty
terrain_variance = var(terrain_samples) * 10000;  % cm²
terrain_range = (max(terrain_samples) - min(terrain_samples)) * 100;  % cm

fprintf('\n--- ADAPTABILITY METRICS ---\n');
fprintf('Strategy Switches:         %d (%.2f /s)\n', strategy_switches, switch_rate);
fprintf('Time in Ankle Strategy:    %.1f%%\n', time_ankle);
fprintf('Time in Hip Strategy:      %.1f%%\n', time_hip);
fprintf('Time in Step Strategy:     %.1f%%\n', time_step);
fprintf('\nTerrain Difficulty:\n');
fprintf('  Variance:                %.2f cm²\n', terrain_variance);
fprintf('  Range:                   %.2f cm\n', terrain_range);
fprintf('  Roughness:               %.2f cm (std)\n', terrain_roughness);

% ===== 5. GRF SYMMETRY =====
peak_vGRF_A = max(GRF_A(:,3));
peak_vGRF_N = max(GRF_N(:,3));

% Calculate asymmetry during double support
stance_both = stance_A & stance_N;
if sum(stance_both) > 0
    grf_A_stance = mean(GRF_A(stance_both, 3));
    grf_N_stance = mean(GRF_N(stance_both, 3));
    grf_asymmetry = abs(grf_A_stance - grf_N_stance) / ((grf_A_stance + grf_N_stance)/2 + 0.001) * 100;
else
    grf_asymmetry = 0;
end

% Loading rate (affects joint stress)
dGRF_A = diff(GRF_A(:,3)) / dt;
dGRF_N = diff(GRF_N(:,3)) / dt;
max_loading_rate_A = max(abs(dGRF_A));
max_loading_rate_N = max(abs(dGRF_N));

fprintf('\n--- GRF SYMMETRY & LOADING ---\n');
fprintf('                           Paretic  Non-Par   Target\n');
fprintf('Peak vGRF (N):             %6.1f   %6.1f\n', peak_vGRF_A, peak_vGRF_N);
fprintf('Peak vGRF (%% BW):          %6.1f   %6.1f    <130%%\n', 100*peak_vGRF_A/body_weight, 100*peak_vGRF_N/body_weight);
fprintf('GRF Asymmetry (%%):         %6.1f            <15%%\n', grf_asymmetry);
fprintf('Max Loading Rate (N/s):    %6.0f   %6.0f\n', max_loading_rate_A, max_loading_rate_N);

% ===== 6. PERTURBATION RESPONSE =====
% Measure recovery time from instability episodes
recovery_times = [];
in_unstable = false;
unstable_start = 0;

for i = 1:n_total
    if ~zmp_stable_bal(i) && ~in_unstable
        % Entering unstable state
        in_unstable = true;
        unstable_start = i;
    elseif zmp_stable_bal(i) && in_unstable
        % Recovering to stable state
        in_unstable = false;
        recovery_time_ms = (i - unstable_start) * dt * 1000;
        recovery_times = [recovery_times; recovery_time_ms];
    end
end

if ~isempty(recovery_times)
    mean_recovery_time = mean(recovery_times);
    max_recovery_time = max(recovery_times);
    num_recoveries = length(recovery_times);
else
    mean_recovery_time = 0;
    max_recovery_time = 0;
    num_recoveries = 0;
end

fprintf('\n--- PERTURBATION RESPONSE ---\n');
fprintf('Number of Recoveries:      %d\n', num_recoveries);
fprintf('Mean Recovery Time (ms):   %.0f     Target: <300\n', mean_recovery_time);
fprintf('Max Recovery Time (ms):    %.0f     Target: <500\n', max_recovery_time);

% ===== 7. OVERALL TERRAIN READINESS SCORE =====
% Weighted composite score (0-100) for deployment decision
score_stability = min(100, stability_pct_after);
score_margin = min(100, max(0, (mean_margin_after + 5) / 8 * 100));
score_smoothness = min(100, max(0, 100 - mean_jerk * 2));
score_effort = min(100, max(0, 100 - (peak_ankle_torque_A / max_ankle_torque) * 30));
score_recovery = min(100, max(0, 100 - mean_recovery_time / 5));
score_symmetry = min(100, max(0, 100 - grf_asymmetry * 2));

% Weights for each component (sum to 1.0)
% Stability is most important for safety
w = [0.25, 0.20, 0.15, 0.15, 0.15, 0.10];

terrain_readiness_score = w(1)*score_stability + w(2)*score_margin + ...
                          w(3)*score_smoothness + w(4)*score_effort + ...
                          w(5)*score_recovery + w(6)*score_symmetry;

fprintf('\n========== TERRAIN READINESS SCORE ==========\n');
fprintf('Component Scores (0-100):\n');
fprintf('  Stability:        %5.1f (weight: %.0f%%)\n', score_stability, w(1)*100);
fprintf('  Safety Margin:    %5.1f (weight: %.0f%%)\n', score_margin, w(2)*100);
fprintf('  Smoothness:       %5.1f (weight: %.0f%%)\n', score_smoothness, w(3)*100);
fprintf('  Effort Efficiency:%5.1f (weight: %.0f%%)\n', score_effort, w(4)*100);
fprintf('  Recovery Speed:   %5.1f (weight: %.0f%%)\n', score_recovery, w(5)*100);
fprintf('  Symmetry:         %5.1f (weight: %.0f%%)\n', score_symmetry, w(6)*100);
fprintf('\n  OVERALL SCORE:    %5.1f / 100\n', terrain_readiness_score);

% Interpret score into readiness level
if terrain_readiness_score >= 85
    readiness_level = 'EXCELLENT - Ready for rough terrain';
elseif terrain_readiness_score >= 70
    readiness_level = 'GOOD - Suitable for moderate terrain';
elseif terrain_readiness_score >= 55
    readiness_level = 'FAIR - Limited to mild terrain';
else
    readiness_level = 'NEEDS IMPROVEMENT - Indoor/flat only';
end

fprintf('  Assessment:       %s\n', readiness_level);
fprintf('==============================================\n');

%% ========== SUMMARY ==========
% Print final summary of all results
fprintf('\n================== SUMMARY ==================\n');
fprintf('Subject: %d\n', sID);
fprintf('Patient: %d years, %s, %.2f m, %.1f kg\n', patient_age, sex_str, patient_height, patient_mass);
fprintf('Clinical: FAC=%d, POMA=%d, TIS=%d\n', FAC_score, POMA_score, TIS_score);
fprintf('Terrain: %s (roughness: %.2f cm)\n', terrain_type, terrain_roughness);
fprintf('Assistance: %.0f%%\n', assist_level * 100);
fprintf('Paretic strength: %.0f%%\n', paretic_strength * 100);

fprintf('\nGait Controller:\n');
fprintf('  Gap reduction: Hip=%.0f%%, Knee=%.0f%%, Ankle=%.0f%%\n', ...
    reduction_hip, reduction_knee, reduction_ankle);

fprintf('\nBalance Controller:\n');
fprintf('  Ankle strategy: %.1f%%\n', 100 * n_ankle / n_total);
fprintf('  Hip strategy:   %.1f%%\n', 100 * n_hip / n_total);
fprintf('  Step needed:    %.1f%%\n', 100 * n_step / n_total);
fprintf('  Switch rate:    %.1f /s\n', switch_rate);

fprintf('\nZMP Stability:\n');
fprintf('  Before balance: %.1f%%\n', stability_before);
fprintf('  After balance:  %.1f%%\n', stability_after);
fprintf('  Improvement:    %.1f%%\n', stability_after - stability_before);

fprintf('\nTerrain Readiness:\n');
fprintf('  Score: %.1f / 100\n', terrain_readiness_score);
fprintf('  Assessment: %s\n', readiness_level);

fprintf('\nGait Parameters:\n');
fprintf('  Stride: %.3f m\n', stride_length);
fprintf('  Speed: %.2f m/s\n', walking_speed);
fprintf('  Asymmetry: %.1f%%\n', asymmetry);
fprintf('=============================================\n');

%% ========== FIGURE 1: CONTROLLER PERFORMANCE ==========
% Visualize PID controller effectiveness in reducing gait asymmetry
fig1 = figure('Position', [50, 50, 1200, 600], 'Color', 'w', ...
              'Name', sprintf('Subject %d - Controller Performance', sID));

gait_pct = linspace(0, 100, n_cycle);

% Hip angles: healthy vs paretic vs assisted
subplot(2,3,1);
plot(gait_pct, hip_flex_N, 'b-', 'LineWidth', 2); hold on;
plot(gait_pct, hip_flex_P, 'r--', 'LineWidth', 1.5);
plot(gait_pct, hip_flex_A, 'g-', 'LineWidth', 2);
xlabel('Gait Cycle (%)'); ylabel('Angle (°)');
title('Hip Flexion');
legend('Healthy', 'Paretic', 'Assisted', 'Location', 'best');
grid on;

% Knee angles
subplot(2,3,2);
plot(gait_pct, knee_flex_N, 'b-', 'LineWidth', 2); hold on;
plot(gait_pct, knee_flex_P, 'r--', 'LineWidth', 1.5);
plot(gait_pct, knee_flex_A, 'g-', 'LineWidth', 2);
xlabel('Gait Cycle (%)'); ylabel('Angle (°)');
title('Knee Flexion');
legend('Healthy', 'Paretic', 'Assisted', 'Location', 'best');
grid on;

% Ankle angles
subplot(2,3,3);
plot(gait_pct, ankle_flex_N, 'b-', 'LineWidth', 2); hold on;
plot(gait_pct, ankle_flex_P, 'r--', 'LineWidth', 1.5);
plot(gait_pct, ankle_flex_A, 'g-', 'LineWidth', 2);
xlabel('Gait Cycle (%)'); ylabel('Angle (°)');
title('Ankle Angle');
legend('Healthy', 'Paretic', 'Assisted', 'Location', 'best');
grid on;

% Bar chart: Gap / Correction / Residual
subplot(2,3,4);
bar([mean_gap_hip, mean_gap_knee, mean_gap_ankle; ...
     mean(abs(correction_hip)), mean(abs(correction_knee)), mean(abs(correction_ankle)); ...
     mean_res_hip, mean_res_knee, mean_res_ankle]');
set(gca, 'XTickLabel', {'Hip', 'Knee', 'Ankle'});
ylabel('Degrees');
title('Gap / Correction / Residual');
legend('Gap', 'Correction', 'Residual', 'Location', 'best');
grid on;

% Gap reduction percentage
subplot(2,3,5);
bar([reduction_hip, reduction_knee, reduction_ankle], 'FaceColor', [0.3 0.7 0.4]);
set(gca, 'XTickLabel', {'Hip', 'Knee', 'Ankle'});
ylabel('Reduction (%)');
title(sprintf('Gap Reduction (%.0f%% Assist)', assist_level * 100));
ylim([0 100]); grid on;

% Applied corrections over gait cycle
subplot(2,3,6);
plot(gait_pct, correction_hip, 'r-', 'LineWidth', 1.5); hold on;
plot(gait_pct, correction_knee, 'b-', 'LineWidth', 1.5);
plot(gait_pct, correction_ankle, 'g-', 'LineWidth', 1.5);
xlabel('Gait Cycle (%)'); ylabel('Correction (°)');
title('Applied Corrections');
legend('Hip', 'Knee', 'Ankle', 'Location', 'best');
grid on;

sgtitle(sprintf('Subject %d - Assistive Controller (%.0f%% Assist)', sID, assist_level*100), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ========== FIGURE 2: BALANCE ANALYSIS ==========
% Visualize balance controller behavior and ZMP stability
fig2 = figure('Position', [100, 50, 1400, 600], 'Color', 'w', ...
              'Name', sprintf('Subject %d - Balance Analysis', sID));

n_show = min(3 * n_cycle, n_total);
t_show = t(1:n_show);

% Balance strategy over time
subplot(2,3,1);
area(t_show, balance_strategy(1:n_show), 'FaceColor', [0.3 0.6 0.9], 'EdgeColor', 'none');
yticks([1 2 3]);
yticklabels({'Ankle', 'Hip', 'Step'});
xlabel('Time (s)'); ylabel('Strategy');
title('Balance Strategy Used');
ylim([0.5 3.5]);
grid on;

% Ankle torques
subplot(2,3,2);
plot(t_show, ankle_torque_A(1:n_show), 'r-', 'LineWidth', 1.5); hold on;
plot(t_show, ankle_torque_N(1:n_show), 'b-', 'LineWidth', 1.5);
yline(0, 'k:', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Torque (Nm)');
title('Ankle Balance Torques');
legend('Paretic', 'Non-Paretic', 'Location', 'best');
grid on;

% Hip torques
subplot(2,3,3);
plot(t_show, hip_torque_A(1:n_show), 'r-', 'LineWidth', 1.5); hold on;
plot(t_show, hip_torque_N(1:n_show), 'b-', 'LineWidth', 1.5);
yline(0, 'k:', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Torque (Nm)');
title('Hip Balance Torques');
legend('Paretic', 'Non-Paretic', 'Location', 'best');
grid on;

% ZMP stability margin
subplot(2,3,4);
plot(t_show, zmp_margin(1:n_show)*100, 'r--', 'LineWidth', 1.5); hold on;
plot(t_show, zmp_margin_bal(1:n_show)*100, 'g-', 'LineWidth', 2);
yline(0, 'k:', 'LineWidth', 1.5);
yline(3, 'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Margin (cm)');
title('ZMP Stability Margin');
legend('Before', 'After', 'Edge', 'Target', 'Location', 'best');
grid on;

% Stability improvement bar chart
subplot(2,3,5);
bar([stability_before, stability_after], 'FaceColor', [0.3 0.7 0.5]);
hold on;
yline(95, 'r--', 'LineWidth', 2);
set(gca, 'XTickLabel', {'Before', 'After'});
ylabel('Stability (%)');
title('ZMP Stability Improvement');
ylim([0 105]); grid on;

% Strategy distribution pie chart
subplot(2,3,6);
pie([n_ankle, n_hip, n_step], {'Ankle', 'Hip', 'Step'});
title('Strategy Distribution');
colormap([0.3 0.8 0.3; 0.9 0.7 0.2; 0.9 0.3 0.3]);

sgtitle(sprintf('Subject %d - Balance Control on %s Terrain', sID, terrain_type), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ========== FIGURE 3: CoM, HIP, ANKLE, GRF (X, Y, Z) ==========
% Comprehensive position and force analysis in all three axes
fig3 = figure('Position', [50, 50, 1600, 900], 'Color', 'w', ...
              'Name', sprintf('Subject %d - Position & Force Analysis', sID));

% Row 1: CoM positions
subplot(4,3,1);
plot(t_show, CoM_pos(1:n_show,1), 'r--', 'LineWidth', 1.5); hold on;
plot(t_show, CoM_pos_bal(1:n_show,1), 'g-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('X Position (m)');
title('CoM - Forward (X)');
legend('Before', 'After', 'Location', 'best');
grid on;

subplot(4,3,2);
plot(t_show, CoM_pos(1:n_show,2), 'r--', 'LineWidth', 1.5); hold on;
plot(t_show, CoM_pos_bal(1:n_show,2), 'g-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Y Position (m)');
title('CoM - Lateral (Y)');
legend('Before', 'After', 'Location', 'best');
grid on;

subplot(4,3,3);
plot(t_show, CoM_pos(1:n_show,3), 'r--', 'LineWidth', 1.5); hold on;
plot(t_show, CoM_pos_bal(1:n_show,3), 'g-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Z Position (m)');
title('CoM - Vertical (Z)');
legend('Before', 'After', 'Location', 'best');
grid on;

% Row 2: Hip positions
subplot(4,3,4);
plot(t_show, hip_A_pos(1:n_show,1), 'r--', 'LineWidth', 1.5); hold on;
plot(t_show, hip_A_pos_bal(1:n_show,1), 'r-', 'LineWidth', 2);
plot(t_show, hip_N_pos(1:n_show,1), 'b--', 'LineWidth', 1.5);
plot(t_show, hip_N_pos_bal(1:n_show,1), 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('X Position (m)');
title('Hip - Forward (X)');
legend('P Before', 'P After', 'N Before', 'N After', 'Location', 'best');
grid on;

subplot(4,3,5);
plot(t_show, hip_A_pos(1:n_show,2), 'r--', 'LineWidth', 1.5); hold on;
plot(t_show, hip_A_pos_bal(1:n_show,2), 'r-', 'LineWidth', 2);
plot(t_show, hip_N_pos(1:n_show,2), 'b--', 'LineWidth', 1.5);
plot(t_show, hip_N_pos_bal(1:n_show,2), 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Y Position (m)');
title('Hip - Lateral (Y)');
legend('P Before', 'P After', 'N Before', 'N After', 'Location', 'best');
grid on;

subplot(4,3,6);
plot(t_show, hip_A_pos(1:n_show,3), 'r--', 'LineWidth', 1.5); hold on;
plot(t_show, hip_A_pos_bal(1:n_show,3), 'r-', 'LineWidth', 2);
plot(t_show, hip_N_pos(1:n_show,3), 'b--', 'LineWidth', 1.5);
plot(t_show, hip_N_pos_bal(1:n_show,3), 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Z Position (m)');
title('Hip - Vertical (Z)');
legend('P Before', 'P After', 'N Before', 'N After', 'Location', 'best');
grid on;

% Row 3: Ankle positions
subplot(4,3,7);
plot(t_show, ankle_A_pos(1:n_show,1), 'r--', 'LineWidth', 1.5); hold on;
plot(t_show, ankle_A_pos_bal(1:n_show,1), 'r-', 'LineWidth', 2);
plot(t_show, ankle_N_pos(1:n_show,1), 'b--', 'LineWidth', 1.5);
plot(t_show, ankle_N_pos_bal(1:n_show,1), 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('X Position (m)');
title('Ankle - Forward (X)');
legend('P Before', 'P After', 'N Before', 'N After', 'Location', 'best');
grid on;

subplot(4,3,8);
plot(t_show, ankle_A_pos(1:n_show,2), 'r--', 'LineWidth', 1.5); hold on;
plot(t_show, ankle_A_pos_bal(1:n_show,2), 'r-', 'LineWidth', 2);
plot(t_show, ankle_N_pos(1:n_show,2), 'b--', 'LineWidth', 1.5);
plot(t_show, ankle_N_pos_bal(1:n_show,2), 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Y Position (m)');
title('Ankle - Lateral (Y)');
legend('P Before', 'P After', 'N Before', 'N After', 'Location', 'best');
grid on;

subplot(4,3,9);
plot(t_show, ankle_A_pos(1:n_show,3), 'r--', 'LineWidth', 1.5); hold on;
plot(t_show, ankle_A_pos_bal(1:n_show,3), 'r-', 'LineWidth', 2);
plot(t_show, ankle_N_pos(1:n_show,3), 'b--', 'LineWidth', 1.5);
plot(t_show, ankle_N_pos_bal(1:n_show,3), 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Z Position (m)');
title('Ankle - Vertical (Z)');
legend('P Before', 'P After', 'N Before', 'N After', 'Location', 'best');
grid on;

% Row 4: Ground Reaction Forces
subplot(4,3,10);
plot(t_show, GRF_A(1:n_show,1), 'r-', 'LineWidth', 2); hold on;
plot(t_show, GRF_N(1:n_show,1), 'b-', 'LineWidth', 2);
yline(0, 'k:', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Force (N)');
title('GRF - Anterior-Posterior (X)');
legend('Paretic', 'Non-Paretic', 'Location', 'best');
grid on;

subplot(4,3,11);
plot(t_show, GRF_A(1:n_show,2), 'r-', 'LineWidth', 2); hold on;
plot(t_show, GRF_N(1:n_show,2), 'b-', 'LineWidth', 2);
yline(0, 'k:', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Force (N)');
title('GRF - Medial-Lateral (Y)');
legend('Paretic', 'Non-Paretic', 'Location', 'best');
grid on;

subplot(4,3,12);
plot(t_show, GRF_A(1:n_show,3), 'r-', 'LineWidth', 2); hold on;
plot(t_show, GRF_N(1:n_show,3), 'b-', 'LineWidth', 2);
yline(body_weight, 'k:', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Force (N)');
title('GRF - Vertical (Z)');
legend('Paretic', 'Non-Paretic', 'Body Weight', 'Location', 'best');
grid on;

sgtitle(sprintf('Subject %d - CoM, Hip, Ankle Positions & GRF (X, Y, Z) | %s Terrain', sID, terrain_type), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ========== FIGURE 4: TERRAIN ROBUSTNESS DASHBOARD ==========
% Comprehensive metrics for outdoor deployment readiness
fig4 = figure('Position', [50, 50, 1600, 900], 'Color', 'w', ...
              'Name', sprintf('Subject %d - Terrain Robustness Analysis', sID));

% Stability bar chart
subplot(3,4,1);
bar([stability_pct_before, stability_pct_after], 'FaceColor', [0.3 0.7 0.5]);
hold on;
yline(95, 'r--', 'LineWidth', 2);
set(gca, 'XTickLabel', {'Before', 'After'});
ylabel('ZMP Stability (%)');
title('Stability (Target: >95%)');
ylim([0 105]); grid on;

% Stability margin timeline
subplot(3,4,2);
plot(t_show, zmp_margin(1:n_show)*100, 'r--', 'LineWidth', 1); hold on;
plot(t_show, zmp_margin_bal(1:n_show)*100, 'g-', 'LineWidth', 2);
yline(0, 'k-', 'LineWidth', 1.5);
yline(3, 'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Margin (cm)');
title('Stability Margin (Target: >3cm)');
legend('Before', 'After', 'Edge', 'Target', 'Location', 'best');
grid on;

% Margin histogram
subplot(3,4,3);
histogram(zmp_margin_bal*100, 30, 'FaceColor', [0.3 0.6 0.9], 'EdgeColor', 'none');
hold on;
xline(0, 'r-', 'LineWidth', 2);
xline(3, 'g--', 'LineWidth', 2);
xlabel('Stability Margin (cm)'); ylabel('Count');
title('Margin Distribution');
legend('', 'Edge', 'Target', 'Location', 'best');
grid on;

% Critical zone pie chart
subplot(3,4,4);
pie([100-critical_pct_after, critical_pct_after], {'Safe', 'Critical'});
title(sprintf('Critical Zone: %.1f%%', critical_pct_after));
colormap([0.3 0.8 0.3; 0.9 0.3 0.3]);

% CoM jerk timeline
subplot(3,4,5);
plot(t_show, jerk_magnitude_orig(1:n_show), 'r--', 'LineWidth', 1); hold on;
plot(t_show, jerk_magnitude(1:n_show), 'g-', 'LineWidth', 2);
yline(5, 'b--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Jerk (m/s³)');
title('CoM Jerk (Target: <5)');
legend('Before', 'After', 'Target', 'Location', 'best');
grid on;

% Jerk comparison bars
subplot(3,4,6);
bar([mean_jerk_orig, mean_jerk; max(jerk_magnitude_orig), peak_jerk]', 'grouped');
set(gca, 'XTickLabel', {'Mean', 'Peak'});
ylabel('Jerk (m/s³)');
title('Jerk Comparison');
legend('Before', 'After', 'Location', 'best');
grid on;

% Mean torque bars
subplot(3,4,7);
bar_data = [mean_ankle_torque_A, mean_ankle_torque_N; mean_hip_torque_A, mean_hip_torque_N];
bar(bar_data, 'grouped');
set(gca, 'XTickLabel', {'Ankle', 'Hip'});
ylabel('Mean Torque (Nm)');
title('Balance Effort');
legend('Paretic', 'Non-Paretic', 'Location', 'best');
grid on;

% Peak torque vs limits
subplot(3,4,8);
bar_data = [peak_ankle_torque_A, peak_ankle_torque_N; peak_hip_torque_A, peak_hip_torque_N];
bar(bar_data, 'grouped'); hold on;
plot([0.5 1.5], [max_ankle_torque, max_ankle_torque], 'r--', 'LineWidth', 2);
plot([1.5 2.5], [max_hip_torque, max_hip_torque], 'r--', 'LineWidth', 2);
set(gca, 'XTickLabel', {'Ankle', 'Hip'});
ylabel('Peak Torque (Nm)');
title('Peak Torque vs Limits');
legend('Paretic', 'Non-Paretic', 'Limit', 'Location', 'best');
grid on;

% Strategy switching
subplot(3,4,9);
area(t_show, balance_strategy(1:n_show), 'FaceColor', [0.3 0.6 0.9], 'EdgeColor', 'none');
yticks([1 2 3]);
yticklabels({'Ankle', 'Hip', 'Step'});
xlabel('Time (s)'); ylabel('Strategy');
title(sprintf('Strategy Switching (%.1f/s)', switch_rate));
ylim([0.5 3.5]); grid on;

% Terrain profile
subplot(3,4,10);
x_terrain_plot = linspace(0, walkway_length, 200);
z_terrain_plot = arrayfun(@(x) terrain_height(x, 0, terrain_type), x_terrain_plot) * 100;
plot(x_terrain_plot, z_terrain_plot, 'k-', 'LineWidth', 2);
xlabel('X (m)'); ylabel('Height (cm)');
title(sprintf('Terrain: %s (Range: %.1fcm)', terrain_type, terrain_range));
grid on;

% Recovery times histogram
subplot(3,4,11);
if ~isempty(recovery_times)
    histogram(recovery_times, 10, 'FaceColor', [0.9 0.6 0.2], 'EdgeColor', 'none');
    hold on;
    xline(300, 'r--', 'LineWidth', 2);
    xlabel('Recovery Time (ms)'); ylabel('Count');
    title(sprintf('Recovery Times (Mean: %.0fms)', mean_recovery_time));
    legend('', 'Target', 'Location', 'best');
else
    text(0.5, 0.5, 'No recoveries needed', 'HorizontalAlignment', 'center', ...
         'FontSize', 14, 'Units', 'normalized');
    title('Recovery Times');
end
grid on;

% Component scores and readiness
subplot(3,4,12);
scores = [score_stability, score_margin, score_smoothness, score_effort, score_recovery, score_symmetry];
bar(scores, 'FaceColor', [0.2 0.6 0.8]);
hold on;
yline(terrain_readiness_score, 'r-', 'LineWidth', 3);
set(gca, 'XTickLabel', {'Stab', 'Marg', 'Smooth', 'Effort', 'Recov', 'Symm'});
ylabel('Score (0-100)');
title(sprintf('READINESS: %.0f/100', terrain_readiness_score));
ylim([0 105]); grid on;

sgtitle(sprintf('Subject %d - Terrain Robustness | %s | Score: %.0f/100 - %s', ...
        sID, terrain_type, terrain_readiness_score, readiness_level), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ========== FIGURE 5: 3D ANIMATION WITH TERRAIN ==========
% Interactive 3D visualization of walking on terrain
fprintf('\nStarting 3D animation with terrain...\n');

fig5 = figure('Position', [50, 50, 1200, 700], 'Color', 'w', ...
              'Name', sprintf('Subject %d - Walking on %s Terrain', sID, terrain_type));

% Define colors for legs
col_A = [0.85, 0.2, 0.2];     % Red for paretic
col_N = [0.2, 0.4, 0.85];     % Blue for non-paretic
col_pelvis = [0.3, 0.3, 0.3]; % Gray for pelvis

% Frame selection for animation speed
frame_skip = max(1, floor(n_total / 400));
frames = 1:frame_skip:n_total;

% Create terrain mesh
[X_terrain_mesh, Y_terrain_mesh] = meshgrid(linspace(0, walkway_length, 120), linspace(-0.8, 0.8, 20));
Z_terrain_mesh = arrayfun(@(x,y) terrain_height(x, y, terrain_type), X_terrain_mesh, Y_terrain_mesh);

% Animation loop
for idx = 1:length(frames)
    i = frames(idx);
    
    if ~ishandle(fig5), break; end  % Stop if figure closed
    
    clf;
    hold on;
    
    % Draw terrain surface
    surf(X_terrain_mesh, Y_terrain_mesh, Z_terrain_mesh, 'FaceColor', [0.88 0.88 0.82], ...
         'EdgeColor', [0.75 0.75 0.7], 'FaceAlpha', 0.9, 'EdgeAlpha', 0.3, ...
         'HandleVisibility', 'off');
    
    % Draw grid lines on terrain
    for gx = 0:1:walkway_length
        z_line = arrayfun(@(y) terrain_height(gx, y, terrain_type), linspace(-0.6, 0.6, 20));
        plot3(gx*ones(20,1), linspace(-0.6, 0.6, 20), z_line + 0.002, ...
              'Color', [0.7 0.7 0.65], 'LineWidth', 0.5, 'HandleVisibility', 'off');
    end
    
    % Calculate foot contact points on terrain
    ground_A = terrain_height(toe_A_pos_bal(i,1), toe_A_pos_bal(i,2), terrain_type) + offset_bal;
    ground_N = terrain_height(toe_N_pos_bal(i,1), toe_N_pos_bal(i,2), terrain_type) + offset_bal;
    
    foot_A_contact = [toe_A_pos_bal(i,1), toe_A_pos_bal(i,2), ground_A];
    foot_N_contact = [toe_N_pos_bal(i,1), toe_N_pos_bal(i,2), ground_N];
    
    % Draw foot trails
    trail_start = max(1, i - trail_length * frame_skip);
    trail_idx = trail_start:frame_skip:i;
    
    if length(trail_idx) > 1
        trail_z_A = arrayfun(@(j) terrain_height(toe_A_pos_bal(j,1), toe_A_pos_bal(j,2), terrain_type), trail_idx) + offset_bal + 0.005;
        trail_z_N = arrayfun(@(j) terrain_height(toe_N_pos_bal(j,1), toe_N_pos_bal(j,2), terrain_type), trail_idx) + offset_bal + 0.005;
        
        plot3(toe_A_pos_bal(trail_idx,1), toe_A_pos_bal(trail_idx,2), trail_z_A, ...
              '-', 'Color', [col_A, 0.5], 'LineWidth', 2, 'DisplayName', 'Trail Paretic');
        plot3(toe_N_pos_bal(trail_idx,1), toe_N_pos_bal(trail_idx,2), trail_z_N, ...
              '-', 'Color', [col_N, 0.5], 'LineWidth', 2, 'DisplayName', 'Trail Non-Paretic');
    end
    
    % Draw pelvis
    plot3([hip_A_pos_bal(i,1), hip_N_pos_bal(i,1)], ...
          [hip_A_pos_bal(i,2), hip_N_pos_bal(i,2)], ...
          [hip_A_pos_bal(i,3), hip_N_pos_bal(i,3)], ...
          '-', 'Color', col_pelvis, 'LineWidth', 8, 'DisplayName', 'Pelvis');
    
    % Draw paretic leg (red)
    plot3([hip_A_pos_bal(i,1), knee_A_pos_bal(i,1)], ...
          [hip_A_pos_bal(i,2), knee_A_pos_bal(i,2)], ...
          [hip_A_pos_bal(i,3), knee_A_pos_bal(i,3)], ...
          '-', 'Color', col_A, 'LineWidth', 5, 'DisplayName', 'Paretic (Red)');
    plot3([knee_A_pos_bal(i,1), ankle_A_pos_bal(i,1)], ...
          [knee_A_pos_bal(i,2), ankle_A_pos_bal(i,2)], ...
          [knee_A_pos_bal(i,3), ankle_A_pos_bal(i,3)], ...
          '-', 'Color', col_A, 'LineWidth', 5, 'HandleVisibility', 'off');
    plot3([ankle_A_pos_bal(i,1), foot_A_contact(1)], ...
          [ankle_A_pos_bal(i,2), foot_A_contact(2)], ...
          [ankle_A_pos_bal(i,3), foot_A_contact(3)], ...
          '-', 'Color', col_A, 'LineWidth', 4, 'HandleVisibility', 'off');
    
    % Draw non-paretic leg (blue)
    plot3([hip_N_pos_bal(i,1), knee_N_pos_bal(i,1)], ...
          [hip_N_pos_bal(i,2), knee_N_pos_bal(i,2)], ...
          [hip_N_pos_bal(i,3), knee_N_pos_bal(i,3)], ...
          '-', 'Color', col_N, 'LineWidth', 5, 'DisplayName', 'Non-Paretic (Blue)');
    plot3([knee_N_pos_bal(i,1), ankle_N_pos_bal(i,1)], ...
          [knee_N_pos_bal(i,2), ankle_N_pos_bal(i,2)], ...
          [knee_N_pos_bal(i,3), ankle_N_pos_bal(i,3)], ...
          '-', 'Color', col_N, 'LineWidth', 5, 'HandleVisibility', 'off');
    plot3([ankle_N_pos_bal(i,1), foot_N_contact(1)], ...
          [ankle_N_pos_bal(i,2), foot_N_contact(2)], ...
          [ankle_N_pos_bal(i,3), foot_N_contact(3)], ...
          '-', 'Color', col_N, 'LineWidth', 4, 'HandleVisibility', 'off');
    
    % Draw joint markers
    plot3([hip_A_pos_bal(i,1), knee_A_pos_bal(i,1), ankle_A_pos_bal(i,1)], ...
          [hip_A_pos_bal(i,2), knee_A_pos_bal(i,2), ankle_A_pos_bal(i,2)], ...
          [hip_A_pos_bal(i,3), knee_A_pos_bal(i,3), ankle_A_pos_bal(i,3)], ...
          'o', 'MarkerSize', 8, 'MarkerFaceColor', col_A, 'MarkerEdgeColor', 'k', ...
          'HandleVisibility', 'off');
    plot3([hip_N_pos_bal(i,1), knee_N_pos_bal(i,1), ankle_N_pos_bal(i,1)], ...
          [hip_N_pos_bal(i,2), knee_N_pos_bal(i,2), ankle_N_pos_bal(i,2)], ...
          [hip_N_pos_bal(i,3), knee_N_pos_bal(i,3), ankle_N_pos_bal(i,3)], ...
          'o', 'MarkerSize', 8, 'MarkerFaceColor', col_N, 'MarkerEdgeColor', 'k', ...
          'HandleVisibility', 'off');
    
    % Draw foot markers
    plot3(foot_A_contact(1), foot_A_contact(2), foot_A_contact(3) + 0.005, ...
          's', 'MarkerSize', 10, 'MarkerFaceColor', col_A, 'MarkerEdgeColor', 'k', ...
          'HandleVisibility', 'off');
    plot3(foot_N_contact(1), foot_N_contact(2), foot_N_contact(3) + 0.005, ...
          's', 'MarkerSize', 10, 'MarkerFaceColor', col_N, 'MarkerEdgeColor', 'k', ...
          'HandleVisibility', 'off');
    
    % Draw support polygon
    if stance_A(i) || stance_N(i)
        if stance_A(i) && stance_N(i)
            poly_x = [foot_A_contact(1) - foot_length/2, foot_A_contact(1) + foot_length/2, ...
                      foot_N_contact(1) + foot_length/2, foot_N_contact(1) - foot_length/2];
            poly_y = [foot_A_contact(2) + foot_width/2, foot_A_contact(2) + foot_width/2, ...
                      foot_N_contact(2) - foot_width/2, foot_N_contact(2) - foot_width/2];
        elseif stance_A(i)
            poly_x = foot_A_contact(1) + [-1, 1, 1, -1] * foot_length/2;
            poly_y = foot_A_contact(2) + [1, 1, -1, -1] * foot_width/2;
        else
            poly_x = foot_N_contact(1) + [-1, 1, 1, -1] * foot_length/2;
            poly_y = foot_N_contact(2) + [1, 1, -1, -1] * foot_width/2;
        end
        poly_z = arrayfun(@(x,y) terrain_height(x, y, terrain_type), poly_x, poly_y) + offset_bal + 0.003;
        
        fill3(poly_x, poly_y, poly_z, [0.6 0.6 0.6], 'FaceAlpha', 0.3, ...
              'EdgeColor', [0.3 0.3 0.3], 'LineWidth', 1.5, 'HandleVisibility', 'off');
    end
    
    % Draw ZMP marker
    zmp_z_ground = terrain_height(zmp_pos_bal(i,1), zmp_pos_bal(i,2), terrain_type) + offset_bal;
    if zmp_stable_bal(i)
        zmp_color = [0, 0.8, 0];
        stability_text = 'STABLE';
    else
        zmp_color = [1, 0, 0];
        stability_text = 'UNSTABLE';
    end
    
    plot3(zmp_pos_bal(i,1), zmp_pos_bal(i,2), zmp_z_ground + 0.01, ...
          'p', 'MarkerSize', 15, 'MarkerFaceColor', zmp_color, ...
          'MarkerEdgeColor', 'k', 'LineWidth', 1, 'DisplayName', sprintf('ZMP (%s)', stability_text));
    
    % Draw CoM marker
    plot3(CoM_pos_bal(i,1), CoM_pos_bal(i,2), CoM_pos_bal(i,3), ...
          'o', 'MarkerSize', 14, 'MarkerFaceColor', [1 0.9 0], ...
          'MarkerEdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'CoM');
    
    % Draw CoM projection line
    com_ground_z = terrain_height(CoM_pos_bal(i,1), CoM_pos_bal(i,2), terrain_type) + offset_bal;
    plot3([CoM_pos_bal(i,1), CoM_pos_bal(i,1)], [CoM_pos_bal(i,2), CoM_pos_bal(i,2)], ...
          [CoM_pos_bal(i,3), com_ground_z + 0.005], '--', 'Color', [0.8 0.8 0], ...
          'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % Draw GRF vectors
    grf_scale = 0.001;
    if GRF_A(i,3) > 10
        quiver3(foot_A_contact(1), foot_A_contact(2), foot_A_contact(3), ...
                GRF_A(i,1)*grf_scale, GRF_A(i,2)*grf_scale, GRF_A(i,3)*grf_scale, ...
                0, 'Color', [0.9 0.3 0.3], 'LineWidth', 3, 'MaxHeadSize', 0.5, ...
                'DisplayName', 'GRF Paretic');
    end
    if GRF_N(i,3) > 10
        quiver3(foot_N_contact(1), foot_N_contact(2), foot_N_contact(3), ...
                GRF_N(i,1)*grf_scale, GRF_N(i,2)*grf_scale, GRF_N(i,3)*grf_scale, ...
                0, 'Color', [0.3 0.4 0.9], 'LineWidth', 3, 'MaxHeadSize', 0.5, ...
                'DisplayName', 'GRF Non-Paretic');
    end
    
    % Set view
    view([-30, 20]);
    
    axis equal;
    xlim([0 walkway_length]);
    ylim([-0.8 0.8]);
    zlim([-0.1 1.4]);
    
    xlabel('X - Forward (m)', 'FontSize', 10);
    ylabel('Y - Lateral (m)', 'FontSize', 10);
    zlabel('Z - Height (m)', 'FontSize', 10);
    
    % Get strategy text for title
    switch balance_strategy(i)
        case 1
            strat_text = 'Ankle';
        case 2
            strat_text = 'Hip';
        case 3
            strat_text = 'Step';
        otherwise
            strat_text = '-';
    end
    
    title(sprintf('Subject %d | %s Terrain | Time: %.1fs | ZMP: %s | Strategy: %s | Score: %.0f', ...
          sID, terrain_type, t(i), stability_text, strat_text, terrain_readiness_score), ...
          'FontSize', 12, 'FontWeight', 'bold');
    
    grid on;
    box on;
    
    legend('Location', 'eastoutside', 'FontSize', 9);
    
    drawnow;
    pause(0.01);
end

rotate3d on;
fprintf('Animation complete.\n');

%% ========== SUMMARY ==========
% Print final summary of all results
fprintf('\n================== SUMMARY ==================\n');
fprintf('Subject: %d\n', sID);
fprintf('Terrain: %s (roughness: %.2f cm)\n', terrain_type, terrain_roughness);
fprintf('Assistance: %.0f%%\n', assist_level * 100);
fprintf('Paretic strength: %.0f%%\n', paretic_strength * 100);

fprintf('\nGait Controller:\n');
fprintf('  Gap reduction: Hip=%.0f%%, Knee=%.0f%%, Ankle=%.0f%%\n', ...
    reduction_hip, reduction_knee, reduction_ankle);

fprintf('\nBalance Controller:\n');
fprintf('  Ankle strategy: %.1f%%\n', 100 * n_ankle / n_total);
fprintf('  Hip strategy:   %.1f%%\n', 100 * n_hip / n_total);
fprintf('  Step needed:    %.1f%%\n', 100 * n_step / n_total);
fprintf('  Switch rate:    %.1f /s\n', switch_rate);

fprintf('\nZMP Stability:\n');
fprintf('  Before balance: %.1f%%\n', stability_before);
fprintf('  After balance:  %.1f%%\n', stability_after);
fprintf('  Improvement:    %.1f%%\n', stability_after - stability_before);

fprintf('\nTerrain Readiness:\n');
fprintf('  Score: %.1f / 100\n', terrain_readiness_score);
fprintf('  Assessment: %s\n', readiness_level);

fprintf('\nGait Parameters:\n');
fprintf('  Stride: %.3f m\n', stride_length);
fprintf('  Speed: %.2f m/s\n', walking_speed);
fprintf('  Asymmetry: %.1f%%\n', asymmetry);
fprintf('=============================================\n');
