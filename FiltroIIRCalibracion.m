clc; clear; close all;

% === Tabla de frecuencias y magnitudes de la curva A ===
f_A = [10 12.5 16 20 25 31.5 40 50 63 80 ...
       100 125 160 200 250 315 400 500 630 800 ...
       1000 1250 1600 2000 2500 3150 4000 ...
       5000 6300 8000 10000 12500 16000 20000];

A_dB = [14.1, 14.1, 13.2, 10, 10.2, 10, 10.7, 9.8, 10.25, 11.3, 11.2, 9.4, ...
     12.5, 9.8, 12.9, 7.9, 6.5, 12.8, 11.8, 10.6, 4.6, 4.7, 2.5, 7.4, ...
     1, 9.1, 16.5, 33, 28.2, 20.7, 38.8, 26.52, 12, 44.8];

% === Configuración general ===
fs = 48000;
f_nyq = fs / 2;
f_norm = f_A / f_nyq;
A_mag = 10.^(A_dB / 20);

% === Asegurarse que f_norm vaya de 0 a 1 ===
f_ext = [0, f_A, fs/2] / f_nyq;
A_ext = [A_mag(1), A_mag, A_mag(end)];  % Extensión de amplitud

% === Diseño IIR ===
N_iir = 10;
[b_iir, a_iir] = yulewalk(N_iir, f_ext, A_ext);

% === Respuesta en frecuencia ===
nfft = 4096;
[H_iir, f_resp] = freqz(b_iir, a_iir, nfft, fs);
H_dB_iir = 20 * log10(abs(H_iir));

% === Interpolación de la curva A para comparación ===
A_interp = interp1(f_A, A_dB, f_resp, 'linear', 'extrap');

% === Error absoluto ===
error_iir = abs(H_dB_iir - A_interp);
rmse = sqrt(mean((H_dB_iir - A_interp).^2));

% === Encontrar frecuencia de máximo error ===
[max_error, idx_max_error] = max(error_iir);
f_max_error = f_resp(idx_max_error);


% === Gráfico: Filtros vs. Curva A ===
figure;
semilogx(f_resp, H_dB_iir, 'g', 'LineWidth', 1.5); hold on;
semilogx(f_A, A_dB, 'ro', 'MarkerSize', 5, 'LineWidth', 1.5);  % Curva A en negativo (como filtro)
grid on; xlim([10 22000]);
xlabel('Frecuencia (Hz)'); ylabel('Magnitud (dB)');
title('Filtro IIR vs Curva Calibracion.');
legend('Filtro IIR', 'Curva Cal.');

% === Gráfico: Errores ===
figure;
semilogx(f_resp, error_iir, 'g', 'LineWidth', 1.5);
grid on; xlim([10 22000]);
xlabel('Frecuencia (Hz)'); ylabel('Error absoluto (dB)');
title('Error absoluto del filtro IIR respecto a la curva de calibracion.');
legend('Error IIR');

% === Mostrar errores máximos ===
fprintf('Máximo error IIR: %.3f dB a %.1f Hz\n', max_error, f_max_error);
fprintf('Error cuadrático medio (RMSE): %.2f dB\n', rmse);


% === Guardar coeficientes ===
dlmwrite('coef_iir_b.txt', b_iir(:)', 'delimiter', '\t', 'precision', '%.10f');
dlmwrite('coef_iir_a.txt', a_iir(:)', 'delimiter', '\t', 'precision', '%.10f');
