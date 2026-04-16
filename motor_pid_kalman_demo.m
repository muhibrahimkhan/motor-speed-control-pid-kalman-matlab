function results = motor_pid_kalman_demo()
% MOTOR_PID_KALMAN_DEMO
% Closed-loop DC motor speed control using:
%   1) PID control
%   2) noisy sensor measurements
%   3) Kalman filter state estimation
%
% This project is designed as a portfolio-quality MATLAB project that
% demonstrates control systems, filtering, algorithm design, simulation,
% and performance analysis.
%
% Output:
%   results - struct containing time series, controller performance metrics,
%             and tuned parameters.
%
% Recommended toolbox:
%   Control System Toolbox (helpful, but not required for this script)

    clc; close all;

    %% ---------------- Simulation Settings ----------------
    Ts = 0.01;              % sampling time [s]
    Tfinal = 8.0;           % total simulation time [s]
    t = 0:Ts:Tfinal;
    N = numel(t);

    %% ---------------- Motor Model ----------------
    % First-order DC motor speed model:
    %   dw/dt = -(1/tau)*w + (K/tau)*u
    %
    % State-space:
    %   x(k+1) = A*x(k) + B*u(k) + process_noise
    %   y(k)   = C*x(k) + measurement_noise

    K_motor = 1.15;         % motor gain
    tau = 0.18;             % motor time constant [s]

    A = exp(-Ts/tau);
    B = K_motor * (1 - exp(-Ts/tau));
    C = 1;
    D = 0;

    %% ---------------- PID Parameters ----------------
    % Hand-tuned values chosen for fast response with controlled overshoot
    Kp = 1.7;
    Ki = 7.5;
    Kd = 0.03;

    uMin = 0.0;
    uMax = 1.0;

    %% ---------------- Reference Profile ----------------
    ref = zeros(1, N);
    ref(t >= 0.5) = 60;     % target speed (e.g., RPM normalized example)
    ref(t >= 3.0) = 90;
    ref(t >= 5.5) = 70;

    %% ---------------- Noise Settings ----------------
    processVar = 0.10;      % process noise variance
    measVar = 10.0;         % measurement noise variance

    rng(7);                 % reproducible results

    %% ---------------- Kalman Filter Setup ----------------
    % Single-state Kalman filter for motor speed estimation
    Q = processVar;
    R = measVar;

    xhat = zeros(1, N);     % estimated state
    P = zeros(1, N);        % estimation covariance
    P(1) = 1.0;

    %% ---------------- Simulation Memory ----------------
    x_true = zeros(1, N);
    y_meas = zeros(1, N);
    y_filt = zeros(1, N);
    u = zeros(1, N);
    e = zeros(1, N);

    % PID state
    integralTerm = 0;
    prevError = 0;

    %% ---------------- Closed-Loop Simulation ----------------
    for k = 2:N
        % Sensor measurement from previous true state
        y_meas(k-1) = C * x_true(k-1) + sqrt(measVar) * randn;

        % -------- Kalman Predict --------
        x_pred = A * xhat(k-1) + B * u(k-1);
        P_pred = A * P(k-1) * A' + Q;

        % -------- Kalman Update --------
        Kk = P_pred * C' / (C * P_pred * C' + R);
        xhat(k) = x_pred + Kk * (y_meas(k-1) - C * x_pred);
        P(k) = (1 - Kk * C) * P_pred;

        y_filt(k) = xhat(k);

        % -------- PID Control on Estimated Speed --------
        e(k) = ref(k) - xhat(k);

        integralTerm = integralTerm + e(k) * Ts;
        derivativeTerm = (e(k) - prevError) / Ts;

        u_unsat = Kp * e(k) + Ki * integralTerm + Kd * derivativeTerm;
        u(k) = min(max(u_unsat, uMin), uMax);

        % Anti-windup (simple clamping rollback)
        if u(k) ~= u_unsat
            integralTerm = integralTerm - e(k) * Ts;
        end

        prevError = e(k);

        % -------- True Plant Update --------
        processNoise = sqrt(processVar) * randn;
        x_true(k) = A * x_true(k-1) + B * u(k) + processNoise;
    end

    % Final measurement point
    y_meas(end) = C * x_true(end) + sqrt(measVar) * randn;

    %% ---------------- Performance Metrics ----------------
    metrics = computePerformanceMetrics(t, ref, x_true);

    %% ---------------- Plotting ----------------
    makePlots(t, ref, x_true, y_meas, y_filt, u, e);

    %% ---------------- Package Results ----------------
    results = struct();
    results.time = t;
    results.reference = ref;
    results.trueSpeed = x_true;
    results.measuredSpeed = y_meas;
    results.filteredSpeed = y_filt;
    results.controlInput = u;
    results.error = e;
    results.metrics = metrics;
    results.parameters = struct( ...
        'Ts', Ts, ...
        'K_motor', K_motor, ...
        'tau', tau, ...
        'Kp', Kp, ...
        'Ki', Ki, ...
        'Kd', Kd, ...
        'Q', Q, ...
        'R', R);

    disp('Motor Control Project Results');
    disp(results.metrics);
end

function metrics = computePerformanceMetrics(t, ref, y)
% Compute simple control metrics using the final step segment

    % Analyze last major setpoint region
    idx = find(t >= 5.5);
    tSeg = t(idx) - t(idx(1));
    refSeg = ref(idx);
    ySeg = y(idx);

    target = refSeg(end);
    err = target - ySeg(end);
    steadyStateError = abs(err);

    % Overshoot
    peakVal = max(ySeg);
    overshoot = max(0, (peakVal - target) / max(target, eps) * 100);

    % Rise time (10% to 90% of final target)
    y10 = 0.1 * target;
    y90 = 0.9 * target;

    i10 = find(ySeg >= y10, 1, 'first');
    i90 = find(ySeg >= y90, 1, 'first');

    if isempty(i10) || isempty(i90)
        riseTime = NaN;
    else
        riseTime = tSeg(i90) - tSeg(i10);
    end

    % Settling time (within 2% band)
    band = 0.02 * max(target, eps);
    outOfBand = abs(ySeg - target) > band;
    lastOut = find(outOfBand, 1, 'last');

    if isempty(lastOut)
        settlingTime = 0;
    elseif lastOut == numel(tSeg)
        settlingTime = NaN;
    else
        settlingTime = tSeg(lastOut + 1);
    end

    metrics = struct( ...
        'steadyStateError', steadyStateError, ...
        'overshootPercent', overshoot, ...
        'riseTimeSeconds', riseTime, ...
        'settlingTimeSeconds', settlingTime);
end

function makePlots(t, ref, x_true, y_meas, y_filt, u, e)
% Generate portfolio-quality plots

    figure('Name', 'Motor Speed Control Response', 'NumberTitle', 'off');
    plot(t, ref, 'LineWidth', 2); hold on;
    plot(t, x_true, 'LineWidth', 2);
    plot(t, y_filt, '--', 'LineWidth', 1.8);
    xlabel('Time (s)');
    ylabel('Speed');
    title('Reference vs True Speed vs Kalman Estimate');
    legend('Reference', 'True Speed', 'Filtered Estimate', 'Location', 'best');
    grid on;

    figure('Name', 'Sensor Noise Filtering', 'NumberTitle', 'off');
    plot(t, y_meas, 'LineWidth', 1); hold on;
    plot(t, y_filt, 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Speed');
    title('Noisy Sensor Measurement vs Kalman Filter Estimate');
    legend('Measured Speed (Noisy)', 'Filtered Estimate', 'Location', 'best');
    grid on;

    figure('Name', 'Control Input', 'NumberTitle', 'off');
    plot(t, u, 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Control Input');
    title('PID Controller Output');
    grid on;

    figure('Name', 'Tracking Error', 'NumberTitle', 'off');
    plot(t, e, 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Error');
    title('Tracking Error Over Time');
    grid on;
end
