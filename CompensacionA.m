% ----------------------
% DATOS DE LA CURVA A
% ----------------------
% Frecuencias normalizadas según IEC (en Hz)
f_A = [10, 12.5, 16, 20, 25, 31.5, 40, 50, 63, 80, 100, 125, ...
       160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250, ...
       1600, 2000, 2500, 3150, 4000, 5000, 6300, 8000, ...
       10000, 12500, 16000];

% Atenuación en dB de la curva A
A_dB = [-70.4, -63.4, -56.7, -50.5, -44.7, -39.4, -34.6, -30.2, ...
        -26.2, -22.5, -19.1, -16.1, -13.4, -10.9, -8.6, -6.6, ...
        -4.8, -3.2, -1.9, -0.8, 0.0, 0.6, 1.0, 1.2, 1.3, ...
        1.2, 1.0, 0.5, -0.1, -1.1, -2.5, -4.3, -6.6];

% ----------------------
% COEFICIENTES IIR
% ----------------------
b = [0.234301792299513, -0.468603584599026, -0.234301792299513, ...
     0.937207169198054, -0.234301792299515, -0.468603584599025, ...
     0.234301792299513];

a = [1.000000000000000, -4.113043408775871, 6.553121752655047, ...
    -4.990849294163381, 1.785737302937573, -0.246190595319487, ...
     0.011224250033231];

% ----------------------
% RESPUESTA EN FRECUENCIA
% ----------------------
fs = 48000;
[H, f] = freqz(b, a, 2048, fs);
H_dB = 20*log10(abs(H));

% Interpolación de la curva A
A_interp = interp1(f_A, A_dB, f, 'pchip', 'extrap');

% Interpolación de la curva A
error = abs(H_dB - A_interp);
% ----------------------
% GRAFICAR
% ----------------------
figure;
semilogx(f, H_dB, 'b', 'LineWidth', 1.5); hold on;
semilogx(f, A_interp, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Frecuencia (Hz)');
ylabel('Magnitud (dB)');
title('Filtro IIR vs Curva A');
legend('Filtro IIR', 'Curva A ideal');
xlim([10 20000]);
ylim([-80 5]);

% === Gráfico: Errores ===
figure;
semilogx(f, error, 'g', 'LineWidth', 1.5);
grid on; xlim([10 22000]);
xlabel('Frecuencia (Hz)'); ylabel('Error absoluto (dB)');
title('Error absoluto del filtro IIR respecto a la curva de calibracion.');
legend('Error IIR');

% ----------------------
% MÉTRICA DE ERROR
% ----------------------
rmse = sqrt(mean((H_dB - A_interp).^2));
fprintf('Error cuadrático medio (RMSE): %.2f dB\n', rmse);
