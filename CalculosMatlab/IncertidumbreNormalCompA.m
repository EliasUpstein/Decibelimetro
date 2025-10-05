%% ===============================================================
%   INCERTIDUMBRE DE LA CURVA A - EVALUACIÓN EN FRECUENCIA
%   (Ruido blanco vs ruido rosa sin diseñar el filtro)
%   Compatible con versiones antiguas de MATLAB
%% ===============================================================
clear; clc; close all;

%% ---------------------- DATOS DE LA CURVA A ----------------------
f = [10, 12.5, 16, 20, 25, 31.5, 40, 50, 63, 80, 100, 125, ...
       160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250, ...
       1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, ...
       10000, 12500, 16000, 20000];

A_ideal = [-70.4, -63.4, -56.7, -50.5, -44.7, -39.4, -34.6, -30.2, ...
           -26.2, -22.5, -19.1, -16.1, -13.4, -10.9, -8.6, -6.6, ...
           -4.8, -3.2, -1.9, -0.8, 0.0, 0.6, 1.0, 1.2, 1.3, ...
           1.2, 1.0, 0.5, -0.1, -1.1, -2.5, -4.3, -6.6, -9.3];

tol_min = [-4.5, -4.5, -4.5, -2.5, -2, -2, -1.5, -1.5, ...
           -1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.4, -1.4, ...
           -1.4, -1.4, -1.4, -1.4, -1.1, -1.4, -1.6, -1.6, -1.6, ...
           -1.6, -1.6, -2.1, -2.6, -3.1, -3.6, -6, -17, -17];

tol_max = [3.5, 3, 2.5, 2.5, 2.5, 2, 1.5, 1.5, ...
           1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.4, 1.4, ...
           1.4, 1.4, 1.4, 1.4, 1.1, 1.4, 1.6, 1.6, 1.6, ...
           1.6, 1.6, 2.1, 2.1, 2.1, 2.6, 3, 3.5, 4];
       
tol_min = tol_min/sqrt(3);
tol_max = tol_max/sqrt(3);

A_min = A_ideal + tol_min;
A_max = A_ideal + tol_max;

%% ---------------------- INTERPOLACIÓN Y PREPARACIÓN ----------------------
f_int = logspace(log10(10), log10(20000), 2000);

A_ideal_int = interp1(f, A_ideal, f_int, 'pchip');
A_min_int   = interp1(f, A_min, f_int, 'pchip');
A_max_int   = interp1(f, A_max, f_int, 'pchip');

H_ideal = 10.^(A_ideal_int/20);
H_min   = 10.^(A_min_int/20);
H_max   = 10.^(A_max_int/20);

%% ---------------------- CÁLCULO DE RMS TEÓRICO --------------------------
f_low  = 31.5;     % Hz
f_high = 8000;     % Hz
idx = (f_int >= f_low) & (f_int <= f_high);

% --- densidades espectrales de entrada ---
S_pink_raw = 1 ./ f_int(idx);                    % rosa ideal 1/f

% Normalizar para misma potencia total en el rango
S_pink  = S_pink_raw / trapz(f_int(idx), S_pink_raw);

% --- Potencias de salida ---
P_ideal_pink  = trapz(f_int(idx), (abs(H_ideal(idx)).^2).*S_pink);
P_min_pink    = trapz(f_int(idx), (abs(H_min(idx)).^2).*S_pink);
P_max_pink    = trapz(f_int(idx), (abs(H_max(idx)).^2).*S_pink);

% --- RMS ---
rms_ideal_pink  = sqrt(P_ideal_pink);
rms_min_pink    = sqrt(P_min_pink);
rms_max_pink    = sqrt(P_max_pink);

% --- Errores ---
err_min_pink = 100*(rms_min_pink/rms_ideal_pink - 1);
err_max_pink = 100*(rms_max_pink/rms_ideal_pink - 1);
err_min_dB_pink = 20*log10(rms_min_pink/rms_ideal_pink);
err_max_dB_pink = 20*log10(rms_max_pink/rms_ideal_pink);

%% ---------------------- RESULTADOS --------------------------
fprintf('\nRUIDO ROSA (1/f):\n');
fprintf('  RMS ideal = %.6e\n', rms_ideal_pink);
fprintf('  RMS min   = %.6e  (Error: %+5.2f %% | %+5.2f dB)\n', rms_min_pink, err_min_pink, err_min_dB_pink);
fprintf('  RMS max   = %.6e  (Error: %+5.2f %% | %+5.2f dB)\n', rms_max_pink, err_max_pink, err_max_dB_pink);

%% ---------------------- GRÁFICOS --------------------------
figure;
semilogx(f_int, A_ideal_int, 'k-', 'LineWidth', 1.6); hold on;
semilogx(f_int, A_min_int, '--r', 'LineWidth', 1.2);
semilogx(f_int, A_max_int, '--b', 'LineWidth', 1.2);
grid on;
xlabel('Frecuencia [Hz]');
ylabel('Atenuación [dB]');
title('Curvas de ponderación A (ideal y tolerancias IEC)');
legend('Ideal','Mínimo','Máximo','Location','best');

% Líneas verticales manuales
yl = ylim;
semilogx([f_low f_low], yl, 'g--');
semilogx([f_high f_high], yl, 'g--');
text(f_low*1.1, yl(1)+3, sprintf('%.1f Hz',f_low), 'Color','g');
text(f_high*0.9, yl(1)+3, sprintf('%.1f Hz',f_high), 'Color','g','HorizontalAlignment','right');
grid on;
