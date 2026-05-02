%% =========================================================================
%  POST-STROKE GAIT ANALYSIS WITH ASSISTIVE CONTROLLER
%  =========================================================================
%
%  PURPOSE:
%  This script simulates an assistive exoskeleton or robotic gait orthosis
%  for post-stroke patients. It uses a PID controller to correct the
%  paretic (affected) leg's joint angles toward the non-paretic (healthy)
%  reference pattern, then visualizes the improvement in gait kinematics
%  and ground reaction forces.
%
%  CLINICAL CONTEXT:
%  Post-stroke patients often have asymmetric gait due to weakness,
%  spasticity, or impaired motor control on one side. Assistive devices
%  (exoskeletons, functional electrical stimulation, robotic orthoses)
%  can help by:
%  - Providing powered assistance at hip, knee, and ankle joints
%  - Guiding the paretic limb through a more normal movement pattern
%  - Reducing compensatory overloading of the non-paretic leg
%
%  CONTROL STRATEGY:
%  The controller uses the non-paretic (unaffected) leg as the "healthy
%  reference" and applies PID control to reduce the error between paretic
%  and non-paretic joint angles. This is a common approach in rehabilitation
%  robotics called "contralateral limb mirroring" or "assist-as-needed."
%
%  OUTPUTS:
%  - Figure 1: Controller performance (joint angles, corrections, gap reduction)
%  - Figure 2: Kinematics comparison (positions and angles vs time)
%  - Figure 3: Ground reaction force comparison (unassisted vs assisted)
%  - Figure 4: Average GRF patterns over normalized gait cycle
%  - Figure 5: Interactive 3D walking animation
%
%  COLOR CODING:
%  - RED = Paretic side (unassisted) - the impaired leg without help
%  - GREEN = Paretic side (assisted) - the impaired leg WITH controller
%  - BLUE = Non-paretic side - the "good" leg (reference)
%
%  =========================================================================

clear; clc; close all;

%% ========== USER SETTINGS ==========
% These parameters control the simulation and can be adjusted.

% Path to the normalized gait data file
data_path = 'MAT_normalizedData_PostStrokeAdults_v27-02-23.mat';

% Subject ID to analyze (1-50 available in the dataset)
sID = 1;

% =========================================================================
% CONTROLLER SETTINGS
% =========================================================================
% assist_level: How much of the calculated correction to apply
%   0.0 = No assistance (paretic leg moves on its own)
%   0.5 = 50% assistance (controller provides half the needed correction)
%   1.0 = Full assistance (controller tries to fully match healthy pattern)
%
% In real exoskeletons, this is often called "assistance level" or "power level"
% Lower values are used during training to encourage patient effort
% Higher values are used when patient needs more support

assist_level = 0.80;  % 80% assistance level

% =========================================================================
% PID CONTROLLER GAINS
% =========================================================================
% PID = Proportional-Integral-Derivative control
% This is the most common control algorithm in robotics and engineering.
%
% The controller calculates: u = Kp*e + Ki*∫e + Kd*de/dt
% where e = error (reference - actual)
%
% Kp (Proportional gain): Reacts to current error
%   - Higher Kp = faster response but may overshoot
%   - Lower Kp = slower, more conservative response
%
% Ki (Integral gain): Accumulates past errors to eliminate steady-state offset
%   - Higher Ki = faster elimination of persistent errors
%   - Too high Ki = oscillation and instability ("integral windup")
%
% Kd (Derivative gain): Anticipates future error based on rate of change
%   - Higher Kd = better damping, reduces overshoot
%   - Too high Kd = amplifies noise, causes jitter
%
% These gains are tuned for smooth gait assistance (not too aggressive)

Kp = 0.8;    % Proportional gain
Ki = 0.10;   % Integral gain
Kd = 0.15;   % Derivative gain

% Safety limits to prevent excessive corrections
I_max = 15;       % Integral windup limit (degrees) - prevents runaway accumulation
corr_max = 20;    % Maximum correction per sample (degrees) - safety clamp

% =========================================================================
% MATSUOKA CPG SETTINGS
% =========================================================================
% The Matsuoka oscillator is a biologically-inspired central pattern generator
% (CPG) that produces rhythmic, alternating outputs resembling the neural
% signals that drive locomotion in biological systems.
%
% Each neuron pair (flexor/extensor) obeys:
%   τ1 * du_i/dt + u_i = -w_fe * y_j - β * v_i + s_i
%   τ2 * dv_i/dt + v_i = y_i
%   y_i = max(0, u_i)
%
% where:
%   u_i = internal state (membrane potential)
%   v_i = adaptation/fatigue variable
%   y_i = output (firing rate) = max(0, u_i)
%   τ1  = rise time constant (controls oscillation frequency)
%   τ2  = adaptation time constant (controls duty cycle)
%   w_fe = mutual inhibition weight (flexor inhibits extensor and vice versa)
%   β    = self-inhibition (fatigue) weight
%   s_i  = tonic excitatory drive input
%
% CONTROLLER INTEGRATION STRATEGY:
%   The Matsuoka CPG replaces the raw non-paretic data as the timing/rhythm
%   source. Its outputs are BLENDED with the non-paretic kinematics to give
%   a bio-inspired, smooth reference trajectory for each joint.
%   This lets the controller use a continuous, adaptable oscillator reference
%   rather than a fixed replay of the healthy leg data — more robust and
%   clinically meaningful for exoskeleton rhythm generation.

use_matsuoka = true;    % Set false to revert to pure non-paretic reference

% Matsuoka neuron time constants
% τ1 controls oscillation period: T ≈ 2 * τ2 * ln(1 + 2*β/w_fe) approximately
% For gait at ~0.9 Hz cadence (typical post-stroke), tune accordingly.
tau1 = 0.18;   % Rise time constant (s)  — smaller = faster dynamics
tau2 = 0.72;   % Adaptation time constant (s) — larger = more fatigue/inhibition

% Mutual inhibition and fatigue weights
w_fe   = 2.5;  % Flexor-extensor mutual inhibition weight
beta   = 2.5;  % Self-inhibition (adaptation/fatigue) weight

% Tonic drive input to each neuron (symmetric = symmetric oscillation)
s_drive = 1.0;

% NOTE: cpg_blend is no longer used.  In online mode the CPG generates the
% full reference trajectory from its own rhythm; the non-paretic data only
% contributes its amplitude envelope (mean ± range/2), not its timing.

% =========================================================================
% PHASE-AWARE SAFETY CLAMP SETTINGS 
% =========================================================================
% The Matsuoka adaptation variable v_i is high when a neuron has been
% firing for a while, and low immediately after a switch.
% The SUM (v_flexor + v_extensor) therefore PEAKS at phase transitions
% (heel-strike, toe-off) — precisely the moments when a sudden large
% correction would be most dangerous (joint instability, stumble risk).
%
% We exploit this to make corr_max phase-dependent:
%   corr_max_k = corr_min  +  (corr_max - corr_min) * (1 - transition_k)
%
% where transition_k is the normalised transition signal [0,1].
% Result:
%   - Mid-swing / mid-stance  → full corr_max (device can push hard)
%   - Heel-strike / toe-off   → tightened toward corr_min (safe zone)

use_cpg_clamp = true;   % Set false to revert to fixed corr_max

corr_min = 5;    % Tightest clamp allowed at phase transitions (degrees)
                 % corr_max (20°) is already defined above — that remains
                 % the mid-phase ceiling

% =========================================================================
% SIMULATION SETTINGS
% =========================================================================
walkway_length = 15.0;  % Length of virtual walkway (meters)
trail_length = 40;      % Number of past frames to show in foot trail

%% ========== LOAD DATA ==========
% Load the MAT file containing normalized gait data for all 50 subjects.
% Each subject has joint angle data for one complete gait cycle.

fprintf('Loading data...\n');
load(data_path);

fprintf('Selected subject: %d\n', sID);
fprintf('Assistance level: %.0f%%\n\n', assist_level * 100);

% Extract data structures for selected subject
% S = complete subject data
% P = Paretic side data (affected by stroke)
% N = Non-paretic side data (unaffected, used as reference)
S = Sub(sID);
P = S.PsideSegm_PsideData;
N = S.NsideSegm_NsideData;

%% ========== PATIENT ANTHROPOMETRICS ==========
% Extract actual patient characteristics from the data file
% sub_char contains measured values for each subject

sub_char = S.sub_char;  % Get subject characteristics structure

% Extract patient data (convert mm to m where needed)
patient_mass = sub_char.Weight;           % kg (already in correct units)
patient_height = sub_char.Height / 1000;  % Convert mm to m
leg_length = sub_char.LegLength / 1000;   % Convert mm to m

% Clinical assessment scores (for reference/display)
patient_age = sub_char.Age;
is_male = sub_char.Male;
time_post_stroke = sub_char.TPS;          % Time post stroke
lesion_left = sub_char.LesionLeft;        % 1 = left hemisphere stroke
FAC_score = sub_char.FAC;                 % Functional Ambulation Category (0-5)
POMA_score = sub_char.POMA;               % Tinetti mobility score (0-28)
TIS_score = sub_char.TIS;                 % Trunk Impairment Scale

% Calculate segment lengths using Winter's proportions
L_thigh = 0.245 * patient_height;   % Thigh length (~24.5% of height)
L_shank = 0.246 * patient_height;   % Shank length (~24.6% of height)
L_foot = 0.152 * patient_height;    % Foot length (~15.2% of height)
L_pelvis = 0.191 * patient_height;  % Pelvis width (~19.1% of height)

% Scale thigh/shank to match measured leg length
% This ensures kinematics use actual patient dimensions
scale_factor = leg_length / (L_thigh + L_shank);
L_thigh = L_thigh * scale_factor;
L_shank = L_shank * scale_factor;

% Segment masses as fractions of total body mass (Winter 2009)
mass_thigh = 0.100 * patient_mass;   % Each thigh ~10%
mass_shank = 0.0465 * patient_mass;  % Each shank ~4.65%
mass_foot = 0.0145 * patient_mass;   % Each foot ~1.45%
mass_trunk = 0.497 * patient_mass;   % Trunk (head, arms, torso) ~50%

% Physical constants
g = 9.81;                        % Gravitational acceleration (m/s²)
body_weight = patient_mass * g;  % Body weight in Newtons

% Foot dimensions for support polygon calculations
foot_length = L_foot;
foot_width = 0.08;  % Typical foot width (meters)

% Display patient information
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

fprintf('\nExtracting gait event timing...\n');

% Check if gait event data exists in the structure
has_gait_events = false;

% Initialize timing variables
P_TO_pct = 60;  % Default: TO at 60% of gait cycle
N_TO_pct = 60;

if isfield(P, 'P_IC_cnt') && isfield(P, 'P_TO_cnt')
    % Paretic side gait events (frame indices)
    P_IC_frames = P.P_IC_cnt;  % Initial Contact frames
    P_TO_frames = P.P_TO_cnt;  % Toe Off frames
    
    % Get TO as percentage of gait cycle (from normalized data if available)
    if isfield(P, 'P_TOnorm')
        P_TO_norm = P.P_TOnorm;
        if isnumeric(P_TO_norm) && ~isempty(P_TO_norm)
            valid_vals = P_TO_norm(P_TO_norm > 0 & P_TO_norm < 100);
            if ~isempty(valid_vals)
                P_TO_pct = mean(valid_vals);
            end
        end
    end
    
    % Force plate validity flags
    if isfield(P, 'P_GoodForcePlate')
        P_force_plate_valid = P.P_GoodForcePlate;
    else
        P_force_plate_valid = ones(1, length(P_IC_frames));
    end
    
    has_gait_events = true;
    fprintf('  Paretic IC frames: %s\n', mat2str(P_IC_frames));
    fprintf('  Paretic TO frames: %s\n', mat2str(P_TO_frames));
    fprintf('  Paretic TO at %.1f%% of gait cycle\n', P_TO_pct);
end

if isfield(N, 'N_IC_cnt') && isfield(N, 'N_TO_cnt')
    % Non-paretic side gait events
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

%% ========== EXTRACT JOINT ANGLES ==========
% Extract sagittal plane joint angles from the data structures.
% The data may be stored in different formats, so we use a helper function.

fprintf('\nExtracting joint angles...\n');

% Anonymous function handle for the extraction helper
extract_angle = @(data) extract_angle_data(data);

% =========================================================================
% HELPER FUNCTION: extract_angle_data
% =========================================================================
% Handles different data storage formats in the MAT file.
% The joint angle data might be stored as:
%   - A struct with X, Y, Z fields (most common)
%   - A struct with other field names
%   - A direct numeric array
%
% We primarily need the X component (sagittal plane = flexion/extension)
% as this is the dominant motion plane during walking.

    function out = extract_angle_data(data)
        if isstruct(data)
            if isfield(data, 'X')
                out = data.X(:);          % Standard format
            elseif isfield(data, 'x')
                out = data.x(:);          % Lowercase variant
            else
                fn = fieldnames(data);
                if ~isempty(fn)
                    out = data.(fn{1})(:); % Use first available field
                else
                    out = [];
                end
            end
        elseif isnumeric(data)
            out = data(:);                 % Direct numeric array
        else
            out = [];
        end
    end

% =========================================================================
% EXTRACT PARETIC SIDE JOINT ANGLES
% =========================================================================
% These are the "impaired" angles that the controller will try to correct.
% Post-stroke patients typically show:
%   - Reduced hip flexion during swing phase
%   - Insufficient knee flexion (stiff-legged gait)
%   - Foot drop (inadequate ankle dorsiflexion)

hip_flex_P_raw = extract_angle_data(P.HipAngles);    % Hip flexion/extension
knee_flex_P_raw = extract_angle_data(P.KneeAngles);  % Knee flexion
ankle_flex_P_raw = extract_angle_data(P.AnkleAngles); % Ankle dorsi/plantarflexion

% =========================================================================
% EXTRACT NON-PARETIC SIDE JOINT ANGLES (HEALTHY REFERENCE)
% =========================================================================
% The non-paretic leg serves as the "target" pattern for the controller.
% We assume the unaffected leg has relatively normal kinematics.
%
% Note: In real clinical applications, you might use:
%   - Age-matched healthy normative data
%   - The patient's pre-stroke gait (if available)
%   - Optimized reference trajectories

hip_flex_N_raw = extract_angle_data(N.HipAngles);
knee_flex_N_raw = extract_angle_data(N.KneeAngles);
ankle_flex_N_raw = extract_angle_data(N.AnkleAngles);

len_P = length(hip_flex_P_raw);
len_N = length(hip_flex_N_raw);

fprintf('Paretic samples: %d, Non-paretic samples: %d\n', len_P, len_N);

%% ========== NORMALIZE TO SAME LENGTH ==========
% Both legs must have the same number of samples for the controller to work.
% We resample to the shorter length using spline interpolation.
%
% This is necessary because:
%   1. Gait cycle duration may differ between legs
%   2. Data collection timing may vary
%   3. Controller needs synchronized reference and actual signals

n_cycle = min(len_P, len_N);  % Use shorter length as reference

if len_P ~= len_N
    fprintf('Resampling to %d samples...\n', n_cycle);
    
    % Resample paretic side if needed
    if len_P ~= n_cycle
        x_old = linspace(0, 1, len_P);   % Original normalized time
        x_new = linspace(0, 1, n_cycle); % Target normalized time
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
    % No resampling needed - lengths already match
    hip_flex_P = hip_flex_P_raw;
    knee_flex_P = knee_flex_P_raw;
    ankle_flex_P = ankle_flex_P_raw;
    hip_flex_N = hip_flex_N_raw;
    knee_flex_N = knee_flex_N_raw;
    ankle_flex_N = ankle_flex_N_raw;
end

% Handle missing data (NaN values) using linear interpolation
% This fills any gaps that might occur from marker dropout during capture
hip_flex_P = fillmissing(hip_flex_P(:), 'linear');
hip_flex_N = fillmissing(hip_flex_N(:), 'linear');
knee_flex_P = fillmissing(knee_flex_P(:), 'linear');
knee_flex_N = fillmissing(knee_flex_N(:), 'linear');
ankle_flex_P = fillmissing(ankle_flex_P(:), 'linear');
ankle_flex_N = fillmissing(ankle_flex_N(:), 'linear');

fprintf('Gait cycle: %d samples\n', n_cycle);

%% ========== ONLINE MATSUOKA CPG + PID CONTROLLER ==========
%
% ARCHITECTURE — what "online" means here:
% -------------------------------------------------------------------------
% The Matsuoka oscillator no longer runs in a separate offline pass.
% Instead it lives INSIDE the sample-by-sample control loop and receives
% sensory feedback at every step.  At each sample k:
%
%   1. Read the current paretic joint angles (plant output)
%   2. Detect gait phase from foot clearance (simulated ground contact)
%   3. Feed phase signal back into the CPG as s_flexor / s_extensor drives
%      → swing onset  → boost flexor drive  → CPG speeds up into swing
%      → stance onset → boost extensor drive → CPG locks into stance
%   4. Step the CPG equations forward by dt (Euler)
%   5. Map the live CPG firing rates → joint angle reference via a
%      shape template derived from the non-paretic data (amplitude only,
%      no timing borrowed from it)
%   6. Compute PID error against that reference
%   7. Apply phase-aware safety clamp using the LIVE v_i (no pre-storage)
%   8. Output the corrected angle
%
% SENSORY FEEDBACK — foot clearance entrainment:
% -------------------------------------------------------------------------
% A simulated ground contact signal is computed from the paretic ankle
% angle.  Ankle plantarflexion below a threshold → foot on ground (stance).
% Ankle dorsiflexion above threshold → foot lifting (swing onset).
%
% At swing onset  → s_flexor  gets a pulse (s_drive + s_feedback)
%                   s_extensor stays at s_drive
% At stance onset → s_extensor gets a pulse
%                   s_flexor  stays at s_drive
%
% This is the standard Matsuoka entrainment mechanism from Mori et al. and
% Righetti & Ijspeert (2006) — the tonic drive stays on, the phasic pulse
% tips the oscillator into the correct phase.
%
% SHAPE TEMPLATE — how CPG output becomes a joint angle:
% -------------------------------------------------------------------------
% We extract the peak-to-peak range and mean of each non-paretic signal
% ONCE before the loop.  Inside the loop, the CPG net output (y_f - y_e)
% ∈ [-1, +1] is mapped to degrees:
%
%   ref_k = mean_nonp + (y_f - y_e) * (range_nonp / 2)
%
% The non-paretic data now only contributes its AMPLITUDE envelope —
% all timing comes from the live oscillator.
% =========================================================================

fprintf('\n========== ONLINE MATSUOKA-CPG + PID CONTROLLER ==========\n');

% -------------------------------------------------------------------------
% Shape template — computed once from non-paretic data
% -------------------------------------------------------------------------
hip_mean   = mean(hip_flex_N);    hip_amp   = range(hip_flex_N)   / 2;
knee_mean  = mean(knee_flex_N);   knee_amp  = range(knee_flex_N)  / 2;
ankle_mean = mean(ankle_flex_N);  ankle_amp = range(ankle_flex_N) / 2;

% Stance/swing threshold — ankle angle below this = foot on ground
% (typical: ankle is plantarflexed ~5–15° in stance, dorsiflexed in swing)
ankle_stance_thresh = mean(ankle_flex_P);  % adaptive per-subject threshold

% Sensory feedback gain — how much a ground contact event perturbs s_drive
s_feedback = 0.6 * s_drive;  % 60% boost on the winning neuron at transitions

fprintf('Shape template:  hip %.1f±%.1f°,  knee %.1f±%.1f°,  ankle %.1f±%.1f°\n', ...
        hip_mean, hip_amp, knee_mean, knee_amp, ankle_mean, ankle_amp);
fprintf('Stance threshold (ankle): %.2f°\n', ankle_stance_thresh);
fprintf('Feedback gain: s_base=%.2f  s_pulse=%.2f\n', s_drive, s_drive + s_feedback);

% -------------------------------------------------------------------------
% Gait period for integration timestep
% -------------------------------------------------------------------------
% Use cadence = 90 steps/min as initial estimate; the oscillator will
% self-entrain to the actual data rhythm through sensory feedback.
T_stride_est = 2 * 60 / 90;
dt_cpg = T_stride_est / n_cycle;
fprintf('CPG dt = %.4f s\n', dt_cpg);

% -------------------------------------------------------------------------
% Initialize CPG neuron states
% -------------------------------------------------------------------------
% Neuron index 1 = flexor,  2 = extensor
% Start with flexor slightly dominant (= beginning of swing phase)
u_hip  = [0.5; -0.5];   v_hip  = zeros(2,1);
u_knee = [0.5; -0.5];   v_knee = zeros(2,1);
u_ank  = [0.5; -0.5];   v_ank  = zeros(2,1);

prev_in_stance = false;   % Previous-sample ground contact state (for edge detection)

% -------------------------------------------------------------------------
% Initialize PID state
% -------------------------------------------------------------------------
iHip = 0; iKnee = 0; iAnkle = 0;
eHip_prev = 0; eKnee_prev = 0; eAnkle_prev = 0;

% -------------------------------------------------------------------------
% Preallocate output and diagnostic arrays
% -------------------------------------------------------------------------
hip_flex_A    = zeros(n_cycle, 1);
knee_flex_A   = zeros(n_cycle, 1);
ankle_flex_A  = zeros(n_cycle, 1);

hip_ref_online   = zeros(n_cycle, 1);   % Live CPG reference (for plotting)
knee_ref_online  = zeros(n_cycle, 1);
ankle_ref_online = zeros(n_cycle, 1);

cpg_hip_out  = zeros(n_cycle, 1);       % Raw CPG net output y_f - y_e
cpg_knee_out = zeros(n_cycle, 1);
cpg_ank_out  = zeros(n_cycle, 1);

cpg_v_hip    = zeros(n_cycle, 2);       % Adaptation variables (for clamp + plotting)
cpg_v_knee   = zeros(n_cycle, 2);
cpg_v_ank    = zeros(n_cycle, 2);

clamp_hip_rec   = zeros(n_cycle, 1);    % Recorded per-sample clamp ceiling
clamp_knee_rec  = zeros(n_cycle, 1);
clamp_ankle_rec = zeros(n_cycle, 1);

gap_hip       = zeros(n_cycle, 1);
gap_knee      = zeros(n_cycle, 1);
gap_ankle     = zeros(n_cycle, 1);
correction_hip   = zeros(n_cycle, 1);
correction_knee  = zeros(n_cycle, 1);
correction_ankle = zeros(n_cycle, 1);

% Helper: running normaliser for v_sum (needed for live clamp scaling)
% We use a sliding window max to avoid needing future samples.
% Initialise conservatively — it will converge within a few samples.
v_sum_max_hip   = 1.0;
v_sum_max_knee  = 1.0;
v_sum_max_ank   = 1.0;

% =========================================================================
% ONLINE CPG + PID LOOP — one sample at a time
% =========================================================================
fprintf('Running online CPG+PID loop (%d samples)...\n', n_cycle);

for k = 1:n_cycle

    % =====================================================================
    % STAGE A: SENSORY FEEDBACK — detect gait phase transition
    % =====================================================================
    % Estimate ground contact from paretic ankle angle.
    % Plantarflexion (angle < threshold) = foot loaded = stance.
    % Dorsiflexion  (angle > threshold) = foot clearing = swing.

    in_stance = (ankle_flex_P(k) < ankle_stance_thresh);

    % Detect edges (transitions)
    swing_onset  = (~in_stance) && prev_in_stance;   % stance → swing
    stance_onset = (in_stance)  && (~prev_in_stance); % swing  → stance

    % Build per-neuron tonic drives with phasic feedback pulses
    %   Swing onset  → flexor  gets the boost (drives hip/knee flexion)
    %   Stance onset → extensor gets the boost (drives extension / loading)
    if swing_onset
        s_hip  = [s_drive + s_feedback;  s_drive];   % [flexor; extensor]
        s_knee = [s_drive + s_feedback;  s_drive];
        s_ank  = [s_drive + s_feedback;  s_drive];
    elseif stance_onset
        s_hip  = [s_drive;  s_drive + s_feedback];
        s_knee = [s_drive;  s_drive + s_feedback];
        s_ank  = [s_drive;  s_drive + s_feedback];
    else
        s_hip  = [s_drive;  s_drive];
        s_knee = [s_drive;  s_drive];
        s_ank  = [s_drive;  s_drive];
    end

    prev_in_stance = in_stance;

    % =====================================================================
    % STAGE B: CPG STEP — advance oscillator by dt with sensory drive
    % =====================================================================

    % Current firing rates (half-wave rectified membrane potential)
    y_hip  = max(0, u_hip);
    y_knee = max(0, u_knee);
    y_ank  = max(0, u_ank);

    % Net CPG output: flexor minus extensor  ∈ [-1, +1] approximately
    net_hip  = y_hip(1)  - y_hip(2);
    net_knee = y_knee(1) - y_knee(2);
    net_ank  = y_ank(1)  - y_ank(2);

    cpg_hip_out(k)  = net_hip;
    cpg_knee_out(k) = net_knee;
    cpg_ank_out(k)  = net_ank;

    % Store adaptation variable for clamp calculation below
    cpg_v_hip(k,:)  = v_hip(:)';
    cpg_v_knee(k,:) = v_knee(:)';
    cpg_v_ank(k,:)  = v_ank(:)';

    % Euler integration of Matsuoka equations
    %   τ1 * du/dt = -u - w_fe*y_other - β*v + s
    %   τ2 * dv/dt = y - v

    % Hip
    du_hip(1) = (1/tau1)*(-u_hip(1)  - w_fe*y_hip(2)  - beta*v_hip(1)  + s_hip(1));
    du_hip(2) = (1/tau1)*(-u_hip(2)  - w_fe*y_hip(1)  - beta*v_hip(2)  + s_hip(2));
    dv_hip(1) = (1/tau2)*(y_hip(1)   - v_hip(1));
    dv_hip(2) = (1/tau2)*(y_hip(2)   - v_hip(2));

    % Knee
    du_knee(1) = (1/tau1)*(-u_knee(1) - w_fe*y_knee(2) - beta*v_knee(1) + s_knee(1));
    du_knee(2) = (1/tau1)*(-u_knee(2) - w_fe*y_knee(1) - beta*v_knee(2) + s_knee(2));
    dv_knee(1) = (1/tau2)*(y_knee(1)  - v_knee(1));
    dv_knee(2) = (1/tau2)*(y_knee(2)  - v_knee(2));

    % Ankle
    du_ank(1) = (1/tau1)*(-u_ank(1)  - w_fe*y_ank(2)  - beta*v_ank(1)  + s_ank(1));
    du_ank(2) = (1/tau1)*(-u_ank(2)  - w_fe*y_ank(1)  - beta*v_ank(2)  + s_ank(2));
    dv_ank(1) = (1/tau2)*(y_ank(1)   - v_ank(1));
    dv_ank(2) = (1/tau2)*(y_ank(2)   - v_ank(2));

    u_hip  = u_hip  + dt_cpg * du_hip(:);
    v_hip  = v_hip  + dt_cpg * dv_hip(:);
    u_knee = u_knee + dt_cpg * du_knee(:);
    v_knee = v_knee + dt_cpg * dv_knee(:);
    u_ank  = u_ank  + dt_cpg * du_ank(:);
    v_ank  = v_ank  + dt_cpg * dv_ank(:);

    % =====================================================================
    % STAGE C: MAP CPG OUTPUT → JOINT ANGLE REFERENCE
    % =====================================================================
    % Apply the non-paretic shape template (amplitude + offset only).
    % Timing is entirely from the live oscillator — not from stored data.

    hip_ref_k   = hip_mean   + net_hip  * hip_amp;
    knee_ref_k  = knee_mean  + net_knee * knee_amp;
    ankle_ref_k = ankle_mean + net_ank  * ankle_amp;

    hip_ref_online(k)   = hip_ref_k;
    knee_ref_online(k)  = knee_ref_k;
    ankle_ref_online(k) = ankle_ref_k;

    % =====================================================================
    % STAGE D: PHASE-AWARE SAFETY CLAMP (live, from current v_i)
    % =====================================================================
    % v_sum = v_flexor + v_extensor.  It is high during a sustained burst
    % and peaks at phase transitions — use it to tighten the clamp live.
    % Running max ensures the normalisation doesn't need future samples.

    v_sum_hip_k  = sum(v_hip);
    v_sum_knee_k = sum(v_knee);
    v_sum_ank_k  = sum(v_ank);

    v_sum_max_hip  = max(v_sum_max_hip,  v_sum_hip_k);
    v_sum_max_knee = max(v_sum_max_knee, v_sum_knee_k);
    v_sum_max_ank  = max(v_sum_max_ank,  v_sum_ank_k);

    if use_matsuoka && use_cpg_clamp
        trans_hip_k  = v_sum_hip_k  / v_sum_max_hip;
        trans_knee_k = v_sum_knee_k / v_sum_max_knee;
        trans_ank_k  = v_sum_ank_k  / v_sum_max_ank;

        clamp_k_hip   = corr_min + (corr_max - corr_min) * (1 - trans_hip_k);
        clamp_k_knee  = corr_min + (corr_max - corr_min) * (1 - trans_knee_k);
        clamp_k_ankle = corr_min + (corr_max - corr_min) * (1 - trans_ank_k);
    else
        clamp_k_hip   = corr_max;
        clamp_k_knee  = corr_max;
        clamp_k_ankle = corr_max;
    end

    clamp_hip_rec(k)   = clamp_k_hip;
    clamp_knee_rec(k)  = clamp_k_knee;
    clamp_ankle_rec(k) = clamp_k_ankle;

    % =====================================================================
    % STAGE E: PID CONTROLLER — track CPG reference
    % =====================================================================

    % E1. Error: CPG reference minus current paretic angle
    eHip   = hip_ref_k   - hip_flex_P(k);
    eKnee  = knee_ref_k  - knee_flex_P(k);
    eAnkle = ankle_ref_k - ankle_flex_P(k);

    gap_hip(k)   = eHip;
    gap_knee(k)  = eKnee;
    gap_ankle(k) = eAnkle;

    % E2. Integral (with anti-windup)
    iHip   = max(-I_max, min(I_max, iHip   + eHip));
    iKnee  = max(-I_max, min(I_max, iKnee  + eKnee));
    iAnkle = max(-I_max, min(I_max, iAnkle + eAnkle));

    % E3. Derivative
    dHip   = eHip   - eHip_prev;
    dKnee  = eKnee  - eKnee_prev;
    dAnkle = eAnkle - eAnkle_prev;

    % E4. Full PID output
    uHip_full   = Kp*eHip   + Ki*iHip   + Kd*dHip;
    uKnee_full  = Kp*eKnee  + Ki*iKnee  + Kd*dKnee;
    uAnkle_full = Kp*eAnkle + Ki*iAnkle + Kd*dAnkle;

    % E5. Phase-aware safety clamp
    uHip_full   = max(-clamp_k_hip,   min(clamp_k_hip,   uHip_full));
    uKnee_full  = max(-clamp_k_knee,  min(clamp_k_knee,  uKnee_full));
    uAnkle_full = max(-clamp_k_ankle, min(clamp_k_ankle, uAnkle_full));

    % E6. Scale by assistance level
    uHip   = assist_level * uHip_full;
    uKnee  = assist_level * uKnee_full;
    uAnkle = assist_level * uAnkle_full;

    correction_hip(k)   = uHip;
    correction_knee(k)  = uKnee;
    correction_ankle(k) = uAnkle;

    % E7. Apply correction to paretic angles
    hip_flex_A(k)   = hip_flex_P(k) + uHip;
    knee_flex_A(k)  = knee_flex_P(k) + uKnee;
    ankle_flex_A(k) = ankle_flex_P(k) + uAnkle;

    % E8. Update PID state
    eHip_prev   = eHip;
    eKnee_prev  = eKnee;
    eAnkle_prev = eAnkle;
end

% Expose online references under the names used by the rest of the script
hip_ref   = hip_ref_online;
knee_ref  = knee_ref_online;
ankle_ref = ankle_ref_online;

% Also expose scaled CPG signals for Figure 0 (using same shape template)
scale_cpg     = @(raw, amp, mn) raw * amp + mn;
cpg_hip_scaled  = scale_cpg(cpg_hip_out,  hip_amp,   hip_mean);
cpg_knee_scaled = scale_cpg(cpg_knee_out, knee_amp,  knee_mean);
cpg_ank_scaled  = scale_cpg(cpg_ank_out,  ankle_amp, ankle_mean);

fprintf('Online CPG+PID complete.\n');
if use_matsuoka && use_cpg_clamp
    fprintf('Phase-aware clamp: %.1f°–%.1f°  |  mean (hip/knee/ankle): %.1f° / %.1f° / %.1f°\n', ...
            corr_min, corr_max, mean(clamp_hip_rec), mean(clamp_knee_rec), mean(clamp_ankle_rec));
end

% =========================================================================
% CONTROLLER PERFORMANCE METRICS
% =========================================================================
% Calculate how well the controller reduced the gap between paretic and healthy.

% Mean absolute gap (how different paretic is from healthy, on average)
mean_gap_hip = mean(abs(gap_hip));
mean_gap_knee = mean(abs(gap_knee));
mean_gap_ankle = mean(abs(gap_ankle));

% Residual = remaining error after assistance
% residual = healthy - assisted
residual_hip = hip_flex_N - hip_flex_A;
residual_knee = knee_flex_N - knee_flex_A;
residual_ankle = ankle_flex_N - ankle_flex_A;

% Mean absolute residual (how different assisted is from healthy)
mean_res_hip = mean(abs(residual_hip));
mean_res_knee = mean(abs(residual_knee));
mean_res_ankle = mean(abs(residual_ankle));

% Gap reduction percentage
% reduction = 100 * (1 - residual/gap)
% 100% = perfect correction (residual = 0)
% 0% = no improvement (residual = gap)
reduction_hip = 100 * (1 - mean_res_hip / mean_gap_hip);
reduction_knee = 100 * (1 - mean_res_knee / mean_gap_knee);
reduction_ankle = 100 * (1 - mean_res_ankle / mean_gap_ankle);

% Print controller summary
fprintf('PID gains: Kp=%.2f, Ki=%.2f, Kd=%.2f\n', Kp, Ki, Kd);
fprintf('Assistance: %.0f%%\n\n', assist_level * 100);

fprintf('Gap reduction (how much closer to healthy pattern):\n');
fprintf('  Hip:   %.0f%%\n', reduction_hip);
fprintf('  Knee:  %.0f%%\n', reduction_knee);
fprintf('  Ankle: %.0f%%\n', reduction_ankle);
fprintf('  Mean:  %.0f%%\n', mean([reduction_hip, reduction_knee, reduction_ankle]));
fprintf('================================================\n');

%% ========== CALCULATE STRIDE LENGTH ==========
% Calculate gait parameters based on the ASSISTED joint angles.
% These represent the improved gait with the controller active.

fprintf('\n========== GAIT PARAMETERS ==========\n');

% Hip angle extremes (using assisted angles)
hip_max_flex = max(hip_flex_A);   % Maximum hip flexion (swing phase)
hip_max_ext = min(hip_flex_A);    % Maximum hip extension (stance phase)

% =========================================================================
% STEP LENGTH ESTIMATION
% =========================================================================
% Step length can be estimated from hip angle excursion using pendulum geometry.
% When the leg swings like a pendulum from the hip:
%   step_length ≈ leg_length × (sin(max_flexion) + sin(max_extension))
%
% With assistance, the paretic side should have better hip ROM and thus
% longer step length.

step_length_P = leg_length * (sind(hip_max_flex) + sind(abs(hip_max_ext)));
step_length_N = leg_length * (sind(max(hip_flex_N)) + sind(abs(min(hip_flex_N))));

% Stride length = one complete gait cycle = paretic step + non-paretic step
stride_length = step_length_P + step_length_N;
stride_length = max(0.4, min(1.8, stride_length));  % Clamp to realistic range

% Walking speed estimation
% speed = stride_length × stride_frequency
% stride_frequency = cadence / 2 (since cadence is steps/min)
cadence = 90;  % steps/min (typical for post-stroke, reduced from healthy ~110)
walking_speed = stride_length * (cadence / 60) / 2;
walking_speed = max(0.2, min(1.5, walking_speed));  % Clamp to realistic range

% Gait asymmetry
% Measures the difference in step lengths between legs
% Healthy: <10%, Post-stroke: typically 15-40%
asymmetry = abs(step_length_P - step_length_N) / ((step_length_P + step_length_N)/2) * 100;

fprintf('Stride length: %.3f m\n', stride_length);
fprintf('Walking speed: %.2f m/s\n', walking_speed);
fprintf('Gait asymmetry: %.1f%%\n', asymmetry);
fprintf('======================================\n');

%% ========== CREATE CONTINUOUS WALKING ==========
% Extend the single gait cycle data to simulate walking the full walkway.
% This involves repeating the cycle multiple times.

% Calculate how many strides are needed to cover the walkway
n_strides = ceil(walkway_length / stride_length);
total_time = walkway_length / walking_speed;
dt = total_time / (n_strides * n_cycle);  % Time step between samples

n_total = n_strides * n_cycle;  % Total number of animation frames

fprintf('\nSimulation: %d strides, %d frames, %.1f seconds\n', n_strides, n_total, total_time);

% =========================================================================
% REPEAT GAIT CYCLE DATA FOR CONTINUOUS WALKING
% =========================================================================

% Paretic leg - UNASSISTED (what it would do without the controller)
hip_P_full = repmat(hip_flex_P, n_strides, 1);
knee_P_full = repmat(knee_flex_P, n_strides, 1);
ankle_P_full = repmat(ankle_flex_P, n_strides, 1);

% Paretic leg - ASSISTED (what it does with the controller)
hip_A_full = repmat(hip_flex_A, n_strides, 1);
knee_A_full = repmat(knee_flex_A, n_strides, 1);
ankle_A_full = repmat(ankle_flex_A, n_strides, 1);

% Non-paretic leg (50% phase shifted for alternating gait)
hip_N_full = repmat(hip_flex_N, n_strides, 1);
knee_N_full = repmat(knee_flex_N, n_strides, 1);
ankle_N_full = repmat(ankle_flex_N, n_strides, 1);

% Apply 50% phase shift to non-paretic leg
% In walking, the two legs are 180° out of phase
shift = round(n_cycle / 2);
hip_N_full = circshift(hip_N_full, shift);
knee_N_full = circshift(knee_N_full, shift);
ankle_N_full = circshift(ankle_N_full, shift);

% Time vector
t = linspace(0, total_time, n_total)';

%% ========== COMPUTE KINEMATICS (BOTH PARETIC AND ASSISTED) ==========
% Use forward kinematics to calculate 3D joint positions from angles.
% We compute positions for BOTH unassisted and assisted paretic leg
% to enable direct comparison.

fprintf('Computing kinematics...\n');

pelvis_height = L_thigh + L_shank;  % Standing pelvis height

% =========================================================================
% PREALLOCATE POSITION ARRAYS
% =========================================================================

% Positions for UNASSISTED paretic leg (what patient would do naturally)
pelvis_pos_P = zeros(n_total, 3);
hip_P_pos = zeros(n_total, 3);
knee_P_pos = zeros(n_total, 3);
ankle_P_pos = zeros(n_total, 3);
toe_P_pos = zeros(n_total, 3);
CoM_pos_P = zeros(n_total, 3);

% Positions for ASSISTED paretic leg (what patient does with controller)
pelvis_pos_A = zeros(n_total, 3);
hip_A_pos = zeros(n_total, 3);
knee_A_pos = zeros(n_total, 3);
ankle_A_pos = zeros(n_total, 3);
toe_A_pos = zeros(n_total, 3);
CoM_pos_A = zeros(n_total, 3);

% Non-paretic positions (same for both scenarios)
hip_N_pos = zeros(n_total, 3);
knee_N_pos = zeros(n_total, 3);
ankle_N_pos = zeros(n_total, 3);
toe_N_pos = zeros(n_total, 3);

% =========================================================================
% FORWARD KINEMATICS LOOP
% =========================================================================
for i = 1:n_total
    
    % Forward position based on walking speed
    x = walking_speed * t(i);
    
    % Phase within current gait cycle (0 to 1)
    phase = mod(i-1, n_cycle) / n_cycle;
    
    % Pelvis motion (vertical bob and lateral sway)
    z_bob = 0.012 * sin(2 * pi * phase);   % Vertical oscillation (~12mm)
    y_sway = 0.015 * sin(2 * pi * phase);  % Lateral sway (~15mm)
    
    % =====================================================================
    % UNASSISTED PARETIC LEG KINEMATICS
    % =====================================================================
    % This shows what the patient's leg would do WITHOUT the controller
    
    pelvis_pos_P(i,:) = [x, y_sway, pelvis_height + z_bob];
    hip_P_pos(i,:) = pelvis_pos_P(i,:) + [0, L_pelvis/2, 0];
    
    % Convert angles from degrees to radians
    hf = deg2rad(hip_P_full(i));
    kf = deg2rad(abs(knee_P_full(i)));
    af = deg2rad(ankle_P_full(i));
    
    % Thigh segment (hip to knee)
    thigh = L_thigh * [sin(hf), 0, -cos(hf)];
    knee_P_pos(i,:) = hip_P_pos(i,:) + thigh;
    
    % Shank segment (knee to ankle)
    shank_ang = hf - kf;
    shank = L_shank * [sin(shank_ang), 0, -cos(shank_ang)];
    ankle_P_pos(i,:) = knee_P_pos(i,:) + shank;
    
    % Foot segment (ankle to toe)
    foot_ang = shank_ang + af - pi/2;
    toe_P_pos(i,:) = ankle_P_pos(i,:) + L_foot * 0.7 * [cos(foot_ang), 0, sin(foot_ang)];
    
    % =====================================================================
    % ASSISTED PARETIC LEG KINEMATICS
    % =====================================================================
    % This shows what the patient's leg does WITH the controller active
    
    pelvis_pos_A(i,:) = [x, y_sway, pelvis_height + z_bob];
    hip_A_pos(i,:) = pelvis_pos_A(i,:) + [0, L_pelvis/2, 0];
    
    % Use ASSISTED angles (corrected by controller)
    hf = deg2rad(hip_A_full(i));
    kf = deg2rad(abs(knee_A_full(i)));
    af = deg2rad(ankle_A_full(i));
    
    thigh = L_thigh * [sin(hf), 0, -cos(hf)];
    knee_A_pos(i,:) = hip_A_pos(i,:) + thigh;
    
    shank_ang = hf - kf;
    shank = L_shank * [sin(shank_ang), 0, -cos(shank_ang)];
    ankle_A_pos(i,:) = knee_A_pos(i,:) + shank;
    
    foot_ang = shank_ang + af - pi/2;
    toe_A_pos(i,:) = ankle_A_pos(i,:) + L_foot * 0.7 * [cos(foot_ang), 0, sin(foot_ang)];
    
    % =====================================================================
    % NON-PARETIC LEG KINEMATICS
    % =====================================================================
    % The non-paretic leg is the same in both scenarios
    
    hip_N_pos(i,:) = pelvis_pos_A(i,:) + [0, -L_pelvis/2, 0];
    
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
    
    % =====================================================================
    % CENTER OF MASS CALCULATIONS
    % =====================================================================
    % Calculate CoM for both unassisted and assisted scenarios
    
    % Segment CoM positions (using segment midpoints as approximation)
    trunk_CoM_P = pelvis_pos_P(i,:) + [0, 0, 0.3 * 0.3 * patient_height];
    thigh_P_CoM = (hip_P_pos(i,:) + knee_P_pos(i,:)) / 2;
    shank_P_CoM = (knee_P_pos(i,:) + ankle_P_pos(i,:)) / 2;
    thigh_N_CoM = (hip_N_pos(i,:) + knee_N_pos(i,:)) / 2;
    shank_N_CoM = (knee_N_pos(i,:) + ankle_N_pos(i,:)) / 2;
    
    % Whole-body CoM for unassisted scenario
    CoM_pos_P(i,:) = (mass_trunk * trunk_CoM_P + ...
                      mass_thigh * thigh_P_CoM + mass_shank * shank_P_CoM + ...
                      mass_thigh * thigh_N_CoM + mass_shank * shank_N_CoM) / ...
                     (mass_trunk + 2*mass_thigh + 2*mass_shank);
    
    % Segment CoM positions for assisted scenario
    trunk_CoM_A = pelvis_pos_A(i,:) + [0, 0, 0.3 * 0.3 * patient_height];
    thigh_A_CoM = (hip_A_pos(i,:) + knee_A_pos(i,:)) / 2;
    shank_A_CoM = (knee_A_pos(i,:) + ankle_A_pos(i,:)) / 2;
    
    % Whole-body CoM for assisted scenario
    CoM_pos_A(i,:) = (mass_trunk * trunk_CoM_A + ...
                      mass_thigh * thigh_A_CoM + mass_shank * shank_A_CoM + ...
                      mass_thigh * thigh_N_CoM + mass_shank * shank_N_CoM) / ...
                     (mass_trunk + 2*mass_thigh + 2*mass_shank);
end

% =========================================================================
% GROUND CORRECTION
% =========================================================================
% Ensure no body part goes below the ground plane (z = 0)
% Find the lowest point and shift everything up

min_z = min([ankle_P_pos(:,3); ankle_A_pos(:,3); ankle_N_pos(:,3); ...
             toe_P_pos(:,3); toe_A_pos(:,3); toe_N_pos(:,3)]);
offset = -min_z + 0.005;  % Add 5mm clearance

% Apply offset to all positions
pelvis_pos_P(:,3) = pelvis_pos_P(:,3) + offset;
pelvis_pos_A(:,3) = pelvis_pos_A(:,3) + offset;
hip_P_pos(:,3) = hip_P_pos(:,3) + offset;
hip_A_pos(:,3) = hip_A_pos(:,3) + offset;
hip_N_pos(:,3) = hip_N_pos(:,3) + offset;
knee_P_pos(:,3) = knee_P_pos(:,3) + offset;
knee_A_pos(:,3) = knee_A_pos(:,3) + offset;
knee_N_pos(:,3) = knee_N_pos(:,3) + offset;
ankle_P_pos(:,3) = ankle_P_pos(:,3) + offset;
ankle_A_pos(:,3) = ankle_A_pos(:,3) + offset;
ankle_N_pos(:,3) = ankle_N_pos(:,3) + offset;
toe_P_pos(:,3) = toe_P_pos(:,3) + offset;
toe_A_pos(:,3) = toe_A_pos(:,3) + offset;
toe_N_pos(:,3) = toe_N_pos(:,3) + offset;
CoM_pos_P(:,3) = CoM_pos_P(:,3) + offset;
CoM_pos_A(:,3) = CoM_pos_A(:,3) + offset;

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
stance_P = false(n_total, 1);  % Unassisted paretic stance
stance_A = false(n_total, 1);  % Assisted paretic stance
stance_N = false(n_total, 1);  % Non-paretic stance

if has_gait_events
    % =====================================================================
    % METHOD 1: USE FORCE PLATE GAIT EVENTS (More Accurate)
    % =====================================================================
    % IC (Initial Contact) = heel strike = START of stance
    % TO (Toe Off) = end of stance = START of swing
    %
    % Stance phase: from IC to TO
    % Swing phase: from TO to next IC
    
    fprintf('Using force plate gait events for stance detection.\n');
    fprintf('  Paretic TO at %.1f%% of gait cycle\n', P_TO_pct);
    fprintf('  Non-paretic TO at %.1f%% of gait cycle\n', N_TO_pct);
    
    % Apply to each gait cycle in the simulation
    for stride = 1:n_strides
        cycle_start = (stride - 1) * n_cycle + 1;
        cycle_end = stride * n_cycle;
        
        for k = cycle_start:cycle_end
            % Phase within this cycle (0 to 100%)
            phase_in_cycle = (k - cycle_start) / n_cycle;
            phase_pct = phase_in_cycle * 100;
            
            % Paretic leg: stance from IC (0%) to TO
            if phase_pct <= P_TO_pct
                stance_P(k) = true;
                stance_A(k) = true;  % Assisted follows same timing
            end
            
            % Non-paretic is shifted by 50% (half a gait cycle)
            shifted_phase = mod(phase_pct + 50, 100);
            
            % Non-paretic: stance from IC (0%) to TO
            if shifted_phase <= N_TO_pct
                stance_N(k) = true;
            end
        end
    end
    
    fprintf('\nStance duration (from force plate events):\n');
    fprintf('  Paretic (unassisted): %.1f%%\n', 100 * sum(stance_P) / n_total);
    fprintf('  Paretic (assisted):   %.1f%%\n', 100 * sum(stance_A) / n_total);
    fprintf('  Non-paretic:          %.1f%%\n', 100 * sum(stance_N) / n_total);
    
    % Verify double support phases exist
    double_support_pct = 100 * sum(stance_P & stance_N) / n_total;
    fprintf('  Double support:       %.1f%%\n', double_support_pct);
    
else
    % =====================================================================
    % METHOD 2: CLEARANCE-BASED DETECTION (Fallback)
    % =====================================================================
    % Use foot height above ground to estimate stance/swing
    % Less accurate but works when force plate data is unavailable
    
    fprintf('Using clearance-based stance detection (no force plate data).\n');
    
    % Calculate foot clearance (minimum of ankle and toe height)
    clearance_P = min(ankle_P_pos(:,3), toe_P_pos(:,3));
    clearance_A = min(ankle_A_pos(:,3), toe_A_pos(:,3));
    clearance_N = min(ankle_N_pos(:,3), toe_N_pos(:,3));
    
    % Dynamic threshold based on minimum clearance in data
    stance_threshold = min([clearance_P; clearance_A; clearance_N]) + 0.02;
    
    fprintf('  Stance threshold (auto): %.3f m\n', stance_threshold);
    
    % Detect stance phases
    stance_P = clearance_P < stance_threshold;
    stance_A = clearance_A < stance_threshold;
    stance_N = clearance_N < stance_threshold;
    
    fprintf('\nStance duration (from clearance):\n');
    fprintf('  Paretic (unassisted): %.1f%%\n', 100*sum(stance_P)/n_total);
    fprintf('  Paretic (assisted):   %.1f%%\n', 100*sum(stance_A)/n_total);
    fprintf('  Non-paretic:          %.1f%%\n', 100*sum(stance_N)/n_total);
end

% =========================================================================
% GAIT PHASE LABELS (for analysis and visualization)
% =========================================================================
% Create detailed gait phase labels for each frame
%   0 = Flight (both feet off ground - rare in walking)
%   1 = Single support paretic (only paretic foot on ground)
%   2 = Single support non-paretic (only non-paretic foot on ground)
%   3 = Double support (both feet on ground)

gait_phase = zeros(n_total, 1);

for k = 1:n_total
    if stance_P(k) && stance_N(k)
        gait_phase(k) = 3;  % Double support
    elseif stance_P(k)
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
%% ========== GROUND REACTION FORCE ESTIMATION ==========
% Estimate GRF using inverse dynamics (F = ma).
% We calculate GRF for both unassisted and assisted scenarios to compare.

fprintf('Computing Ground Reaction Forces...\n');

% =========================================================================
% UNASSISTED GRF (what forces patient experiences without controller)
% =========================================================================

% Calculate CoM velocity and acceleration using central differences
CoM_vel_P = zeros(n_total, 3);
CoM_acc_P = zeros(n_total, 3);

for i = 2:n_total-1
    CoM_vel_P(i,:) = (CoM_pos_P(i+1,:) - CoM_pos_P(i-1,:)) / (2*dt);
end
CoM_vel_P(1,:) = CoM_vel_P(2,:);
CoM_vel_P(end,:) = CoM_vel_P(end-1,:);

for i = 2:n_total-1
    CoM_acc_P(i,:) = (CoM_pos_P(i+1,:) - 2*CoM_pos_P(i,:) + CoM_pos_P(i-1,:)) / (dt^2);
end
CoM_acc_P(1,:) = CoM_acc_P(2,:);
CoM_acc_P(end,:) = CoM_acc_P(end-1,:);

% Smooth acceleration to reduce numerical noise
window = max(3, round(n_cycle / 20));
CoM_acc_P = movmean(CoM_acc_P, window);

% Initialize GRF arrays
GRF_P = zeros(n_total, 3);          % Unassisted paretic GRF
GRF_N_unassist = zeros(n_total, 3); % Non-paretic GRF (with unassisted paretic)

% Total vertical force from Newton's second law: F = m(g + a)
GRF_total_z_P = patient_mass * (g + CoM_acc_P(:,3));

% Distribute force between legs based on stance phase
for i = 1:n_total
    if stance_P(i) && stance_N(i)
        % Double support - distribute weight between legs
        phase = mod(i-1, n_cycle) / n_cycle;
        
        % Weight transfer during double support phases
        if phase < 0.1
            % Loading response: weight transferring TO paretic leg
            ratio_P = 0.3 + 0.4 * (phase / 0.1);
        elseif phase > 0.5 && phase < 0.6
            % Pre-swing: weight transferring OFF paretic leg
            ratio_P = 0.7 - 0.4 * ((phase - 0.5) / 0.1);
        else
            ratio_P = 0.5;  % Mid-stance: roughly equal
        end
        
        % Adjust for asymmetry (paretic leg typically bears less weight)
        asym_factor = 1 - (asymmetry / 200);
        ratio_P = ratio_P * asym_factor;
        
        GRF_P(i,3) = GRF_total_z_P(i) * ratio_P;
        GRF_N_unassist(i,3) = GRF_total_z_P(i) * (1 - ratio_P);
        
    elseif stance_P(i)
        % Single support on paretic leg
        GRF_P(i,3) = GRF_total_z_P(i);
        
    elseif stance_N(i)
        % Single support on non-paretic leg
        GRF_N_unassist(i,3) = GRF_total_z_P(i);
    end
    
    % Horizontal force components (proportional to horizontal acceleration)
    if stance_P(i)
        GRF_P(i,1) = patient_mass * CoM_acc_P(i,1) * 0.5;  % A-P force
        GRF_P(i,2) = patient_mass * CoM_acc_P(i,2) * 0.3;  % M-L force
    end
    if stance_N(i)
        GRF_N_unassist(i,1) = patient_mass * CoM_acc_P(i,1) * 0.5;
        GRF_N_unassist(i,2) = patient_mass * CoM_acc_P(i,2) * 0.3;
    end
end

% Smooth and ensure non-negative vertical GRF
GRF_P = movmean(GRF_P, window);
GRF_N_unassist = movmean(GRF_N_unassist, window);
GRF_P(:,3) = max(0, GRF_P(:,3));
GRF_N_unassist(:,3) = max(0, GRF_N_unassist(:,3));

% =========================================================================
% ASSISTED GRF (what forces patient experiences WITH controller)
% =========================================================================
% The controller improves gait kinematics, which changes the GRF distribution

CoM_vel_A = zeros(n_total, 3);
CoM_acc_A = zeros(n_total, 3);

for i = 2:n_total-1
    CoM_vel_A(i,:) = (CoM_pos_A(i+1,:) - CoM_pos_A(i-1,:)) / (2*dt);
end
CoM_vel_A(1,:) = CoM_vel_A(2,:);
CoM_vel_A(end,:) = CoM_vel_A(end-1,:);

for i = 2:n_total-1
    CoM_acc_A(i,:) = (CoM_pos_A(i+1,:) - 2*CoM_pos_A(i,:) + CoM_pos_A(i-1,:)) / (dt^2);
end
CoM_acc_A(1,:) = CoM_acc_A(2,:);
CoM_acc_A(end,:) = CoM_acc_A(end-1,:);

CoM_acc_A = movmean(CoM_acc_A, window);

GRF_A = zeros(n_total, 3);         % Assisted paretic GRF
GRF_N_assist = zeros(n_total, 3);  % Non-paretic GRF (with assisted paretic)

GRF_total_z_A = patient_mass * (g + CoM_acc_A(:,3));

for i = 1:n_total
    if stance_A(i) && stance_N(i)
        phase = mod(i-1, n_cycle) / n_cycle;
        if phase < 0.1
            ratio_A = 0.3 + 0.4 * (phase / 0.1);
        elseif phase > 0.5 && phase < 0.6
            ratio_A = 0.7 - 0.4 * ((phase - 0.5) / 0.1);
        else
            ratio_A = 0.5;
        end
        
        % With assistance, asymmetry is reduced
        % The assisted leg can bear more weight because its kinematics
        % are closer to normal
        asym_factor_assist = 1 - (asymmetry * (1 - assist_level) / 200);
        ratio_A = ratio_A * asym_factor_assist;
        
        GRF_A(i,3) = GRF_total_z_A(i) * ratio_A;
        GRF_N_assist(i,3) = GRF_total_z_A(i) * (1 - ratio_A);
        
    elseif stance_A(i)
        GRF_A(i,3) = GRF_total_z_A(i);
        
    elseif stance_N(i)
        GRF_N_assist(i,3) = GRF_total_z_A(i);
    end
    
    if stance_A(i)
        GRF_A(i,1) = patient_mass * CoM_acc_A(i,1) * 0.5;
        GRF_A(i,2) = patient_mass * CoM_acc_A(i,2) * 0.3;
    end
    if stance_N(i)
        GRF_N_assist(i,1) = patient_mass * CoM_acc_A(i,1) * 0.5;
        GRF_N_assist(i,2) = patient_mass * CoM_acc_A(i,2) * 0.3;
    end
end

GRF_A = movmean(GRF_A, window);
GRF_N_assist = movmean(GRF_N_assist, window);
GRF_A(:,3) = max(0, GRF_A(:,3));
GRF_N_assist(:,3) = max(0, GRF_N_assist(:,3));

% =========================================================================
% GRF WAVEFORM MODULATION (M-shaped pattern)
% =========================================================================
% Apply the characteristic double-peak pattern seen in normal walking GRF:
%   - First peak (F1) at ~15% stance: heel strike loading
%   - Valley at ~35-50% stance: mid-stance unloading
%   - Second peak (F2) at ~75-95% stance: push-off

for i = 1:n_total
    phase_in_cycle = mod(i-1, n_cycle) / n_cycle;
    
    % Unassisted paretic leg modulation (typically has reduced peaks)
    if stance_P(i)
        stance_phase = phase_in_cycle / 0.6;  % Normalize to stance duration
        if stance_phase >= 0 && stance_phase < 0.15
            peak_mod = 1.0 + 0.15 * sin(pi * stance_phase / 0.15);
        elseif stance_phase > 0.75 && stance_phase < 0.95
            peak_mod = 1.0 + 0.12 * sin(pi * (stance_phase - 0.75) / 0.2);
        elseif stance_phase > 0.3 && stance_phase < 0.5
            peak_mod = 0.92;  % Mid-stance valley
        else
            peak_mod = 1.0;
        end
        GRF_P(i,3) = GRF_P(i,3) * peak_mod;
    end
    
    % Assisted paretic leg modulation (more normal peaks with assistance)
    if stance_A(i)
        stance_phase = phase_in_cycle / 0.6;
        if stance_phase >= 0 && stance_phase < 0.15
            peak_mod = 1.0 + 0.18 * sin(pi * stance_phase / 0.15);  % Higher peak
        elseif stance_phase > 0.75 && stance_phase < 0.95
            peak_mod = 1.0 + 0.15 * sin(pi * (stance_phase - 0.75) / 0.2);
        elseif stance_phase > 0.3 && stance_phase < 0.5
            peak_mod = 0.94;
        else
            peak_mod = 1.0;
        end
        GRF_A(i,3) = GRF_A(i,3) * peak_mod;
    end
    
    % Non-paretic leg modulation
    if stance_N(i)
        stance_phase = mod(phase_in_cycle + 0.5, 1) / 0.6;
        if stance_phase >= 0 && stance_phase < 0.15
            peak_mod = 1.0 + 0.18 * sin(pi * stance_phase / 0.15);
        elseif stance_phase > 0.75 && stance_phase < 0.95
            peak_mod = 1.0 + 0.15 * sin(pi * (stance_phase - 0.75) / 0.2);
        elseif stance_phase > 0.3 && stance_phase < 0.5
            peak_mod = 0.94;
        else
            peak_mod = 1.0;
        end
        GRF_N_unassist(i,3) = GRF_N_unassist(i,3) * peak_mod;
        GRF_N_assist(i,3) = GRF_N_assist(i,3) * peak_mod;
    end
end

% Final smoothing
GRF_P = movmean(GRF_P, max(1, round(window/2)));
GRF_A = movmean(GRF_A, max(1, round(window/2)));
GRF_N_unassist = movmean(GRF_N_unassist, max(1, round(window/2)));
GRF_N_assist = movmean(GRF_N_assist, max(1, round(window/2)));

% Normalize to body weight for clinical interpretation
GRF_P_BW = GRF_P / body_weight;
GRF_A_BW = GRF_A / body_weight;
GRF_N_unassist_BW = GRF_N_unassist / body_weight;
GRF_N_assist_BW = GRF_N_assist / body_weight;

fprintf('GRF computed.\n');

%% ========== FIGURE 0: ONLINE MATSUOKA CPG OUTPUT ==========
% Row 1: Live CPG reference vs non-paretic kinematics vs paretic
% Row 2: Adaptation variable v_i (transition signal) + dynamic safety clamp

if use_matsuoka
    fig0 = figure('Position', [30, 30, 1300, 700], 'Color', 'w', ...
                  'Name', sprintf('Subject %d - Online Matsuoka CPG', sID));

    gait_pct = linspace(0, 100, n_cycle);

    % Reconstruct a ground-contact indicator for plotting
    gc_signal = double(ankle_flex_P < ankle_stance_thresh);

    joint_names  = {'Hip', 'Knee', 'Ankle'};
    cpg_signals  = {cpg_hip_scaled,   cpg_knee_scaled,   cpg_ank_scaled};
    ref_signals  = {hip_ref_online,   knee_ref_online,   ankle_ref_online};
    nonp_signals = {hip_flex_N,       knee_flex_N,       ankle_flex_N};
    clamp_recs   = {clamp_hip_rec,    clamp_knee_rec,    clamp_ankle_rec};
    v_sums       = {sum(cpg_v_hip,2), sum(cpg_v_knee,2), sum(cpg_v_ank,2)};

    col_cpg  = [0.85, 0.33, 0.10];   % Orange  — raw CPG output (scaled)
    col_ref  = [0.47, 0.18, 0.56];   % Purple  — live reference to PID
    col_nonp = [0.20, 0.40, 0.85];   % Blue    — non-paretic (amplitude only)
    col_v    = [0.13, 0.55, 0.13];   % Green   — adaptation variable
    col_cl   = [0.80, 0.10, 0.10];   % Red     — dynamic clamp

    % --- Row 1: reference trajectories ---
    for jj = 1:3
        subplot(2, 3, jj);
        % Shade stance phase
        fill([gait_pct(gc_signal==1), fliplr(gait_pct(gc_signal==1))], ...
             [repmat(min(nonp_signals{jj})-5, 1, sum(gc_signal==1)), ...
              repmat(max(nonp_signals{jj})+5, 1, sum(gc_signal==1))], ...
             [0.9 0.95 1], 'EdgeColor', 'none', 'FaceAlpha', 0.4, ...
             'HandleVisibility', 'off'); hold on;
        plot(gait_pct, nonp_signals{jj},  '-',  'Color', col_nonp, 'LineWidth', 1.5);
        plot(gait_pct, cpg_signals{jj},   '--', 'Color', col_cpg,  'LineWidth', 1.5);
        plot(gait_pct, ref_signals{jj},   '-',  'Color', col_ref,  'LineWidth', 2.5);
        xlabel('Gait Cycle (%)');
        ylabel('Angle (°)');
        title(sprintf('%s — Online CPG Reference', joint_names{jj}));
        if jj == 1
            legend('Non-paretic (amplitude template)', ...
                   'CPG output (scaled)', ...
                   'Live PID reference', ...
                   'Location', 'best');
        end
        grid on;
    end

    % --- Row 2: adaptation variable + dynamic clamp ---
    for jj = 1:3
        subplot(2, 3, 3 + jj);

        yyaxis left;
        plot(gait_pct, v_sums{jj}, '-', 'Color', col_v, 'LineWidth', 2); hold on;
        % Mark sensory feedback events (stance transitions)
        trans_pts = find(diff(gc_signal) ~= 0);
        for tp = trans_pts'
            xline(gait_pct(tp), ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1, ...
                  'HandleVisibility', 'off');
        end
        ylabel('v_f + v_e  (adaptation)');

        yyaxis right;
        plot(gait_pct, clamp_recs{jj}, '-', 'Color', col_cl, 'LineWidth', 2);
        yline(corr_max, '--k', 'LineWidth', 1, 'Label', 'fixed max');
        yline(corr_min, ':k',  'LineWidth', 1, 'Label', 'min');
        ylim([0, corr_max * 1.15]);
        ylabel('Correction ceiling (°)');

        xlabel('Gait Cycle (%)');
        title(sprintf('%s — Phase-Aware Clamp (live v_i)', joint_names{jj}));
        if jj == 1
            legend({'Adapt. signal', 'Phase transitions', 'Dynamic clamp', ...
                    'Fixed ceiling', 'Min floor'}, 'Location', 'best');
        end
        grid on;
    end

    sgtitle(sprintf(['Subject %d — Online Matsuoka CPG with Sensory Feedback  ' ...
            '(\\tau_1=%.2f, \\tau_2=%.2f, w_{fe}=%.1f, \\beta=%.1f)  |  Phase-Aware Safety Clamp'], ...
            sID, tau1, tau2, w_fe, beta), 'FontSize', 11, 'FontWeight', 'bold');
end

%% ========== FIGURE 1: CONTROLLER PERFORMANCE ==========
% Visualize how well the PID controller tracks the healthy reference.
% This figure shows the "before and after" of the joint angles.

fig1 = figure('Position', [50, 50, 1200, 600], 'Color', 'w', ...
              'Name', sprintf('Subject %d - Controller Performance', sID));

gait_pct = linspace(0, 100, n_cycle);  % Gait cycle percentage (0-100%)

% -------------------------------------------------------------------------
% Row 1: Joint Angles vs Gait Cycle
% -------------------------------------------------------------------------
% These plots show:
%   BLUE       = Non-paretic kinematics (raw healthy reference)
%   MAGENTA -- = Matsuoka-blended reference (what PID actually tracks)
%   RED --     = Paretic without assistance
%   GREEN      = Paretic WITH CPG+PID assistance

subplot(2,3,1);
plot(gait_pct, hip_flex_N, 'b-', 'LineWidth', 1.5); hold on;
if use_matsuoka
    plot(gait_pct, hip_ref, 'm--', 'LineWidth', 1.5);
end
plot(gait_pct, hip_flex_P, 'r--', 'LineWidth', 1.5);
plot(gait_pct, hip_flex_A, 'g-', 'LineWidth', 2);
xlabel('Gait Cycle (%)'); ylabel('Angle (°)');
title('Hip Flexion');
if use_matsuoka
    legend('Non-paretic', 'CPG reference', 'Paretic', 'Assisted', 'Location', 'best');
else
    legend('Healthy', 'Paretic', 'Assisted', 'Location', 'best');
end
grid on;

subplot(2,3,2);
plot(gait_pct, knee_flex_N, 'b-', 'LineWidth', 1.5); hold on;
if use_matsuoka
    plot(gait_pct, knee_ref, 'm--', 'LineWidth', 1.5);
end
plot(gait_pct, knee_flex_P, 'r--', 'LineWidth', 1.5);
plot(gait_pct, knee_flex_A, 'g-', 'LineWidth', 2);
xlabel('Gait Cycle (%)'); ylabel('Angle (°)');
title('Knee Flexion');
if use_matsuoka
    legend('Non-paretic', 'CPG reference', 'Paretic', 'Assisted', 'Location', 'best');
else
    legend('Healthy', 'Paretic', 'Assisted', 'Location', 'best');
end
grid on;

subplot(2,3,3);
plot(gait_pct, ankle_flex_N, 'b-', 'LineWidth', 1.5); hold on;
if use_matsuoka
    plot(gait_pct, ankle_ref, 'm--', 'LineWidth', 1.5);
end
plot(gait_pct, ankle_flex_P, 'r--', 'LineWidth', 1.5);
plot(gait_pct, ankle_flex_A, 'g-', 'LineWidth', 2);
xlabel('Gait Cycle (%)'); ylabel('Angle (°)');
title('Ankle Angle');
if use_matsuoka
    legend('Non-paretic', 'CPG reference', 'Paretic', 'Assisted', 'Location', 'best');
else
    legend('Healthy', 'Paretic', 'Assisted', 'Location', 'best');
end
grid on;

% -------------------------------------------------------------------------
% Row 2: Controller Metrics
% -------------------------------------------------------------------------

% Bar chart: Gap vs Correction vs Residual
subplot(2,3,4);
bar([mean_gap_hip, mean_gap_knee, mean_gap_ankle; ...
     mean(abs(correction_hip)), mean(abs(correction_knee)), mean(abs(correction_ankle)); ...
     mean_res_hip, mean_res_knee, mean_res_ankle]');
set(gca, 'XTickLabel', {'Hip', 'Knee', 'Ankle'});
ylabel('Degrees');
title('Gap / Correction / Residual');
legend('Gap', 'Correction', 'Residual', 'Location', 'best');
grid on;

% Bar chart: Gap reduction percentage
subplot(2,3,5);
bar([reduction_hip, reduction_knee, reduction_ankle], 'FaceColor', [0.3 0.7 0.4]);
set(gca, 'XTickLabel', {'Hip', 'Knee', 'Ankle'});
ylabel('Reduction (%)');
title(sprintf('Gap Reduction (%.0f%% Assist)', assist_level * 100));
ylim([0 100]);
grid on;

% Line plot: Applied corrections over gait cycle
subplot(2,3,6);
plot(gait_pct, correction_hip, 'r-', 'LineWidth', 1.5); hold on;
plot(gait_pct, correction_knee, 'b-', 'LineWidth', 1.5);
plot(gait_pct, correction_ankle, 'g-', 'LineWidth', 1.5);
xlabel('Gait Cycle (%)'); ylabel('Correction (°)');
title('Applied Corrections');
legend('Hip', 'Knee', 'Ankle', 'Location', 'best');
grid on;

if use_matsuoka
    sgtitle(sprintf('Subject %d - Online Matsuoka CPG + PID Controller (%.0f%% Assist)', ...
            sID, assist_level*100), 'FontSize', 13, 'FontWeight', 'bold');
else
    sgtitle(sprintf('Subject %d - Assistive PID Controller (%.0f%% Assist)', sID, assist_level*100), ...
            'FontSize', 14, 'FontWeight', 'bold');
end

%% ========== FIGURE 2: POSITION vs TIME COMPARISON ==========
% Compare kinematics between unassisted, assisted, and non-paretic legs.

fig2 = figure('Position', [100, 50, 1400, 900], 'Color', 'w', ...
              'Name', sprintf('Subject %d - Kinematics Comparison', sID));

n_show = min(2 * n_cycle, n_total);  % Show 2 gait cycles
t_show = t(1:n_show);

% -------------------------------------------------------------------------
% Row 1: Toe Positions (X, Y, Z)
% -------------------------------------------------------------------------
subplot(3,3,1);
plot(t_show, toe_P_pos(1:n_show,1), 'r-', 'LineWidth', 1.5); hold on;
plot(t_show, toe_A_pos(1:n_show,1), 'g-', 'LineWidth', 2);
plot(t_show, toe_N_pos(1:n_show,1), 'b--', 'LineWidth', 1.5);
ylabel('X Position (m)');
title('Forward Position (X) vs Time');
legend('Paretic', 'Assisted', 'Non-Paretic', 'Location', 'best');
grid on;

subplot(3,3,2);
plot(t_show, toe_P_pos(1:n_show,2), 'r-', 'LineWidth', 1.5); hold on;
plot(t_show, toe_A_pos(1:n_show,2), 'g-', 'LineWidth', 2);
plot(t_show, toe_N_pos(1:n_show,2), 'b--', 'LineWidth', 1.5);
ylabel('Y Position (m)');
title('Lateral Position (Y) vs Time');
legend('Paretic', 'Assisted', 'Non-Paretic', 'Location', 'best');
grid on;

subplot(3,3,3);
plot(t_show, toe_P_pos(1:n_show,3), 'r-', 'LineWidth', 1.5); hold on;
plot(t_show, toe_A_pos(1:n_show,3), 'g-', 'LineWidth', 2);
plot(t_show, toe_N_pos(1:n_show,3), 'b--', 'LineWidth', 1.5);
ylabel('Z Position (m)');
title('Vertical Position (Z) - Toe Clearance');
legend('Paretic', 'Assisted', 'Non-Paretic', 'Location', 'best');
grid on;

% -------------------------------------------------------------------------
% Row 2: Joint Angles Over Time
% -------------------------------------------------------------------------
subplot(3,3,4);
plot(t_show, hip_P_full(1:n_show), 'r-', 'LineWidth', 1.5); hold on;
plot(t_show, hip_A_full(1:n_show), 'g-', 'LineWidth', 2);
plot(t_show, hip_N_full(1:n_show), 'b--', 'LineWidth', 1.5);
ylabel('Angle (°)');
title('Hip Flexion vs Time');
legend('Paretic', 'Assisted', 'Non-Paretic', 'Location', 'best');
grid on;

subplot(3,3,5);
plot(t_show, knee_P_full(1:n_show), 'r-', 'LineWidth', 1.5); hold on;
plot(t_show, knee_A_full(1:n_show), 'g-', 'LineWidth', 2);
plot(t_show, knee_N_full(1:n_show), 'b--', 'LineWidth', 1.5);
ylabel('Angle (°)');
title('Knee Flexion vs Time');
legend('Paretic', 'Assisted', 'Non-Paretic', 'Location', 'best');
grid on;

subplot(3,3,6);
plot(t_show, ankle_P_full(1:n_show), 'r-', 'LineWidth', 1.5); hold on;
plot(t_show, ankle_A_full(1:n_show), 'g-', 'LineWidth', 2);
plot(t_show, ankle_N_full(1:n_show), 'b--', 'LineWidth', 1.5);
ylabel('Angle (°)');
xlabel('Time (s)');
title('Ankle Angle vs Time');
legend('Paretic', 'Assisted', 'Non-Paretic', 'Location', 'best');
grid on;

% -------------------------------------------------------------------------
% Row 3: Center of Mass Comparison
% -------------------------------------------------------------------------
subplot(3,3,7);
plot(t_show, CoM_pos_P(1:n_show,1), 'r-', 'LineWidth', 1.5); hold on;
plot(t_show, CoM_pos_A(1:n_show,1), 'g-', 'LineWidth', 2);
ylabel('X Position (m)');
xlabel('Time (s)');
title('CoM Forward Position');
legend('Unassisted', 'Assisted', 'Location', 'best');
grid on;

subplot(3,3,8);
plot(t_show, CoM_pos_P(1:n_show,2), 'r-', 'LineWidth', 1.5); hold on;
plot(t_show, CoM_pos_A(1:n_show,2), 'g-', 'LineWidth', 2);
ylabel('Y Position (m)');
xlabel('Time (s)');
title('CoM Lateral Sway');
legend('Unassisted', 'Assisted', 'Location', 'best');
grid on;

subplot(3,3,9);
plot(t_show, CoM_pos_P(1:n_show,3), 'r-', 'LineWidth', 1.5); hold on;
plot(t_show, CoM_pos_A(1:n_show,3), 'g-', 'LineWidth', 2);
ylabel('Z Position (m)');
xlabel('Time (s)');
title('CoM Vertical Position');
legend('Unassisted', 'Assisted', 'Location', 'best');
grid on;

sgtitle(sprintf('Subject %d - Position & Joint Angles: Paretic vs Assisted vs Non-Paretic', sID), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ========== FIGURE 3: GROUND REACTION FORCES COMPARISON ==========
% Compare GRF between unassisted and assisted conditions.
% This shows how the controller affects weight distribution and loading.

fig3 = figure('Position', [150, 50, 1400, 800], 'Color', 'w', ...
              'Name', sprintf('Subject %d - GRF Comparison', sID));

% -------------------------------------------------------------------------
% Row 1: Vertical GRF
% -------------------------------------------------------------------------
subplot(2,3,1);
plot(t_show, GRF_P_BW(1:n_show,3), 'r-', 'LineWidth', 2); hold on;
plot(t_show, GRF_A_BW(1:n_show,3), 'g-', 'LineWidth', 2);
yline(1, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
ylabel('Force (BW)');
title('Vertical GRF (Fz) - Paretic Side');
legend('Unassisted', 'Assisted', '1 BW', 'Location', 'best');
ylim([0 1.5]);
grid on;

subplot(2,3,2);
plot(t_show, GRF_N_unassist_BW(1:n_show,3), 'b--', 'LineWidth', 1.5); hold on;
plot(t_show, GRF_N_assist_BW(1:n_show,3), 'b-', 'LineWidth', 2);
yline(1, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
ylabel('Force (BW)');
title('Vertical GRF (Fz) - Non-Paretic Side');
legend('w/ Unassisted', 'w/ Assisted', '1 BW', 'Location', 'best');
ylim([0 1.5]);
grid on;

subplot(2,3,3);
plot(t_show, GRF_P_BW(1:n_show,3) + GRF_N_unassist_BW(1:n_show,3), 'r--', 'LineWidth', 1.5); hold on;
plot(t_show, GRF_A_BW(1:n_show,3) + GRF_N_assist_BW(1:n_show,3), 'g-', 'LineWidth', 2);
yline(1, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
ylabel('Force (BW)');
title('Total Vertical GRF');
legend('Unassisted', 'Assisted', '1 BW', 'Location', 'best');
ylim([0 1.5]);
grid on;

% -------------------------------------------------------------------------
% Row 2: Horizontal GRF and Symmetry
% -------------------------------------------------------------------------
subplot(2,3,4);
plot(t_show, GRF_P_BW(1:n_show,1), 'r-', 'LineWidth', 2); hold on;
plot(t_show, GRF_A_BW(1:n_show,1), 'g-', 'LineWidth', 2);
yline(0, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
ylabel('Force (BW)');
xlabel('Time (s)');
title('Anterior-Posterior GRF (Fx) - Paretic');
legend('Unassisted', 'Assisted', 'Location', 'best');
ylim([-0.3 0.3]);
grid on;

subplot(2,3,5);
plot(t_show, GRF_P_BW(1:n_show,2), 'r-', 'LineWidth', 2); hold on;
plot(t_show, GRF_A_BW(1:n_show,2), 'g-', 'LineWidth', 2);
yline(0, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
ylabel('Force (BW)');
xlabel('Time (s)');
title('Medial-Lateral GRF (Fy) - Paretic');
legend('Unassisted', 'Assisted', 'Location', 'best');
ylim([-0.2 0.2]);
grid on;

% GRF Symmetry Index
% Measures how different the two legs' GRF are
% Lower = more symmetric = better
subplot(2,3,6);
sym_unassist = abs(GRF_P_BW(:,3) - GRF_N_unassist_BW(:,3)) ./ ...
               max(0.01, (GRF_P_BW(:,3) + GRF_N_unassist_BW(:,3))/2) * 100;
sym_assist = abs(GRF_A_BW(:,3) - GRF_N_assist_BW(:,3)) ./ ...
             max(0.01, (GRF_A_BW(:,3) + GRF_N_assist_BW(:,3))/2) * 100;

plot(t_show, sym_unassist(1:n_show), 'r-', 'LineWidth', 1.5); hold on;
plot(t_show, sym_assist(1:n_show), 'g-', 'LineWidth', 2);
ylabel('Asymmetry (%)');
xlabel('Time (s)');
title('GRF Symmetry Index');
legend(sprintf('Unassisted (mean=%.1f%%)', mean(sym_unassist(1:n_show))), ...
       sprintf('Assisted (mean=%.1f%%)', mean(sym_assist(1:n_show))), 'Location', 'best');
ylim([0 100]);
grid on;

sgtitle(sprintf('Subject %d - Ground Reaction Forces: Unassisted vs Assisted (%.0f%%)', ...
        sID, assist_level*100), 'FontSize', 14, 'FontWeight', 'bold');

%% ========== FIGURE 4: GRF vs GAIT CYCLE ==========
% Average GRF patterns over the normalized gait cycle.

fig4 = figure('Position', [200, 50, 1200, 500], 'Color', 'w', ...
              'Name', sprintf('Subject %d - GRF vs Gait Cycle', sID));

gait_pct_full = linspace(0, 100, n_cycle);

% Average over multiple cycles for smoother pattern
n_avg_cycles = min(n_strides, 5);
GRF_P_avg = zeros(n_cycle, 3);
GRF_A_avg = zeros(n_cycle, 3);
GRF_N_unassist_avg = zeros(n_cycle, 3);
GRF_N_assist_avg = zeros(n_cycle, 3);

for c = 1:n_avg_cycles
    start_idx = (c-1) * n_cycle + 1;
    end_idx = c * n_cycle;
    GRF_P_avg = GRF_P_avg + GRF_P_BW(start_idx:end_idx, :);
    GRF_A_avg = GRF_A_avg + GRF_A_BW(start_idx:end_idx, :);
    GRF_N_unassist_avg = GRF_N_unassist_avg + GRF_N_unassist_BW(start_idx:end_idx, :);
    GRF_N_assist_avg = GRF_N_assist_avg + GRF_N_assist_BW(start_idx:end_idx, :);
end
GRF_P_avg = GRF_P_avg / n_avg_cycles;
GRF_A_avg = GRF_A_avg / n_avg_cycles;
GRF_N_unassist_avg = GRF_N_unassist_avg / n_avg_cycles;
GRF_N_assist_avg = GRF_N_assist_avg / n_avg_cycles;

% Paretic leg comparison
subplot(1,3,1);
plot(gait_pct_full, GRF_P_avg(:,3), 'r--', 'LineWidth', 2); hold on;
plot(gait_pct_full, GRF_A_avg(:,3), 'g-', 'LineWidth', 2.5);
xline(60, '--k', 'LineWidth', 1.5);
xlabel('Gait Cycle (%)');
ylabel('Vertical GRF (BW)');
title('Paretic Leg');
legend('Unassisted', 'Assisted', 'Toe Off', 'Location', 'best');
ylim([0 1.4]);
xlim([0 100]);
grid on;

% Non-paretic leg comparison
subplot(1,3,2);
plot(gait_pct_full, GRF_N_unassist_avg(:,3), 'b--', 'LineWidth', 2); hold on;
plot(gait_pct_full, GRF_N_assist_avg(:,3), 'b-', 'LineWidth', 2.5);
xline(60, '--k', 'LineWidth', 1.5);
xlabel('Gait Cycle (%)');
ylabel('Vertical GRF (BW)');
title('Non-Paretic Leg');
legend('w/ Unassisted P', 'w/ Assisted P', 'Toe Off', 'Location', 'best');
ylim([0 1.4]);
xlim([0 100]);
grid on;

% Asymmetry comparison (area plot)
subplot(1,3,3);
sym_unassist_avg = abs(GRF_P_avg(:,3) - GRF_N_unassist_avg(:,3));
sym_assist_avg = abs(GRF_A_avg(:,3) - GRF_N_assist_avg(:,3));

area(gait_pct_full, sym_unassist_avg, 'FaceColor', [1 0.7 0.7], 'EdgeColor', 'r', 'LineWidth', 1.5, 'FaceAlpha', 0.5); hold on;
area(gait_pct_full, sym_assist_avg, 'FaceColor', [0.7 1 0.7], 'EdgeColor', 'g', 'LineWidth', 1.5, 'FaceAlpha', 0.5);
xlabel('Gait Cycle (%)');
ylabel('|Left - Right| (BW)');
title('GRF Asymmetry');
legend(sprintf('Unassisted (mean=%.2f)', mean(sym_unassist_avg)), ...
       sprintf('Assisted (mean=%.2f)', mean(sym_assist_avg)), 'Location', 'best');
xlim([0 100]);
grid on;

sgtitle(sprintf('Subject %d - Average Vertical GRF over Gait Cycle', sID), ...
        'FontSize', 14, 'FontWeight', 'bold');

%% ========== FIGURE 5: 3D WALKING ANIMATION ==========
% Interactive 3D visualization of the assisted walking pattern.
% Shows the GREEN (assisted) paretic leg and BLUE non-paretic leg.

fprintf('\nStarting 3D animation...\n');

fig5 = figure('Position', [100, 100, 1000, 700], 'Color', 'w', ...
              'Name', sprintf('Subject %d - Assisted Walking (%.0f%%)', sID, assist_level*100));

col_A = [0.2, 0.75, 0.3];   % Green - Assisted Paretic
col_N = [0.2, 0.4, 0.85];   % Blue - Non-Paretic

% Frame skipping for smooth animation (~400 frames total)
frame_skip = max(1, floor(n_total / 400));
frames = 1:frame_skip:n_total;

% Animation loop
for idx = 1:length(frames)
    i = frames(idx);
    
    % Check if figure was closed
    if ~ishandle(fig5), break; end
    
    clf;
    hold on;
    
    % Ground plane
    fill3([0, walkway_length, walkway_length, 0], ...
          [-0.6, -0.6, 0.6, 0.6], [0,0,0,0], ...
          [0.92, 0.92, 0.88], 'EdgeColor', 'none', 'FaceAlpha', 0.8, 'HandleVisibility', 'off');
    
    % Grid lines
    for gx = 0:1:walkway_length
        plot3([gx gx], [-0.6 0.6], [0 0], 'Color', [0.8 0.8 0.75], 'LineWidth', 0.5, 'HandleVisibility', 'off');
    end
    
    % Foot trajectory trails
    trail_start = max(1, i - trail_length * frame_skip);
    trail_idx = trail_start:frame_skip:i;
    
    if length(trail_idx) > 1
        plot3(toe_A_pos(trail_idx,1), toe_A_pos(trail_idx,2), toe_A_pos(trail_idx,3), ...
              '-', 'Color', [col_A, 0.5], 'LineWidth', 2, 'DisplayName', 'Trail Assisted');
        plot3(toe_N_pos(trail_idx,1), toe_N_pos(trail_idx,2), toe_N_pos(trail_idx,3), ...
              '-', 'Color', [col_N, 0.5], 'LineWidth', 2, 'DisplayName', 'Trail Non-Paretic');
    end
    
    % Pelvis (connects the two hips)
    plot3([hip_A_pos(i,1), hip_N_pos(i,1)], ...
          [hip_A_pos(i,2), hip_N_pos(i,2)], ...
          [hip_A_pos(i,3), hip_N_pos(i,3)], ...
          'Color', [0.3 0.3 0.3], 'LineWidth', 7, 'DisplayName', 'Pelvis');
    
    % Assisted paretic leg (Green)
    plot3([hip_A_pos(i,1), knee_A_pos(i,1), ankle_A_pos(i,1), toe_A_pos(i,1)], ...
          [hip_A_pos(i,2), knee_A_pos(i,2), ankle_A_pos(i,2), toe_A_pos(i,2)], ...
          [hip_A_pos(i,3), knee_A_pos(i,3), ankle_A_pos(i,3), toe_A_pos(i,3)], ...
          'Color', col_A, 'LineWidth', 5, 'DisplayName', sprintf('Assisted (%.0f%%)', assist_level*100));
    
    % Non-paretic leg (Blue)
    plot3([hip_N_pos(i,1), knee_N_pos(i,1), ankle_N_pos(i,1), toe_N_pos(i,1)], ...
          [hip_N_pos(i,2), knee_N_pos(i,2), ankle_N_pos(i,2), toe_N_pos(i,2)], ...
          [hip_N_pos(i,3), knee_N_pos(i,3), ankle_N_pos(i,3), toe_N_pos(i,3)], ...
          'Color', col_N, 'LineWidth', 5, 'DisplayName', 'Non-Paretic');
    
    % Joint markers
    plot3([hip_A_pos(i,1), knee_A_pos(i,1), ankle_A_pos(i,1)], ...
          [hip_A_pos(i,2), knee_A_pos(i,2), ankle_A_pos(i,2)], ...
          [hip_A_pos(i,3), knee_A_pos(i,3), ankle_A_pos(i,3)], ...
          'o', 'MarkerSize', 6, 'MarkerFaceColor', col_A, 'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
    plot3([hip_N_pos(i,1), knee_N_pos(i,1), ankle_N_pos(i,1)], ...
          [hip_N_pos(i,2), knee_N_pos(i,2), ankle_N_pos(i,2)], ...
          [hip_N_pos(i,3), knee_N_pos(i,3), ankle_N_pos(i,3)], ...
          'o', 'MarkerSize', 6, 'MarkerFaceColor', col_N, 'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
    
    % GRF vectors (arrows showing force magnitude)
    grf_scale = 0.001;  % 1000 N = 1 m arrow
    if GRF_A(i,3) > 10
        quiver3(ankle_A_pos(i,1), ankle_A_pos(i,2), 0, ...
                GRF_A(i,1)*grf_scale, GRF_A(i,2)*grf_scale, GRF_A(i,3)*grf_scale, ...
                0, 'Color', [0.3 0.8 0.3], 'LineWidth', 3, 'MaxHeadSize', 0.5, 'DisplayName', 'GRF Assisted');
    end
    if GRF_N_assist(i,3) > 10
        quiver3(ankle_N_pos(i,1), ankle_N_pos(i,2), 0, ...
                GRF_N_assist(i,1)*grf_scale, GRF_N_assist(i,2)*grf_scale, GRF_N_assist(i,3)*grf_scale, ...
                0, 'Color', [0.3 0.4 0.9], 'LineWidth', 3, 'MaxHeadSize', 0.5, 'DisplayName', 'GRF Non-Paretic');
    end
    
    % Center of mass marker
    plot3(CoM_pos_A(i,1), CoM_pos_A(i,2), CoM_pos_A(i,3), 'o', ...
          'MarkerSize', 12, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'DisplayName', 'CoM');
    
    % Axis settings
    axis equal;
    xlim([0 walkway_length]);
    ylim([-0.8 0.8]);
    zlim([0 1.4]);
    
    xlabel('X - Forward (m)');
    ylabel('Y - Lateral (m)');
    zlabel('Z - Height (m)');
    
    title(sprintf('Subject %d | ASSISTED (%.0f%%) | Time: %.1fs | Stride: %.2fm | Speed: %.2fm/s', ...
          sID, assist_level*100, t(i), stride_length, walking_speed), 'FontSize', 11, 'FontWeight', 'bold');
    
    grid on;
    box on;
    view([-30, 20]);  % Camera angle
    
    legend('Location', 'eastoutside', 'FontSize', 9);
    
    drawnow;
    pause(0.01);
end

rotate3d on;  % Enable mouse rotation after animation
fprintf('Animation complete. Drag to rotate the view.\n');

%% ========== SUMMARY ==========
% Print comprehensive summary of controller performance and gait improvement.

fprintf('\n================== SUMMARY ==================\n');
fprintf('Subject: %d\n', sID);
fprintf('Assistance level: %.0f%%\n', assist_level * 100);
fprintf('PID gains: Kp=%.2f, Ki=%.2f, Kd=%.2f\n', Kp, Ki, Kd);
if use_matsuoka
    fprintf('\nOnline Matsuoka CPG:\n');
    fprintf('  tau1=%.2f  tau2=%.2f  w_fe=%.1f  beta=%.1f\n', tau1, tau2, w_fe, beta);
    fprintf('  Sensory feedback gain: s_base=%.2f  s_pulse=%.2f\n', s_drive, s_drive+s_feedback);
    fprintf('  Reference: CPG rhythm + non-paretic amplitude template\n');
    if use_cpg_clamp
        fprintf('  Phase-aware clamp: %.1f°–%.1f° (transition→mid-phase)\n', corr_min, corr_max);
        fprintf('  Mean clamp (hip/knee/ankle): %.1f° / %.1f° / %.1f°\n', ...
                mean(clamp_hip_rec), mean(clamp_knee_rec), mean(clamp_ankle_rec));
    else
        fprintf('  Fixed safety clamp: %.1f°\n', corr_max);
    end
end
fprintf('\nGap reduction (closer to healthy pattern):\n');
fprintf('  Hip:   %.0f%%\n', reduction_hip);
fprintf('  Knee:  %.0f%%\n', reduction_knee);
fprintf('  Ankle: %.0f%%\n', reduction_ankle);
fprintf('  Mean:  %.0f%%\n', mean([reduction_hip, reduction_knee, reduction_ankle]));
fprintf('\nGRF Improvement (more symmetric loading):\n');
fprintf('  Asymmetry (unassisted): %.1f%%\n', mean(sym_unassist(1:n_show)));
fprintf('  Asymmetry (assisted):   %.1f%%\n', mean(sym_assist(1:n_show)));
fprintf('\nGait parameters (with assistance):\n');
fprintf('  Stride length: %.3f m\n', stride_length);
fprintf('  Walking speed: %.2f m/s\n', walking_speed);
fprintf('  Gait asymmetry: %.1f%%\n', asymmetry);
fprintf('=============================================\n');
