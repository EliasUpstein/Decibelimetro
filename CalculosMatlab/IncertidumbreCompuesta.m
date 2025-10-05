clear; clc; close all;

% =======================
% Parámetros del micrófono
% =======================
n = 24;             % Bits microfono
S = -26;            % Sensibilidad [dBFS]
Ref = 94;           % Nivel de referencia [dBSPL]
noise_floor = -87;  % Piso de ruido [dBFS]
psr = -75;          % Power-Supply Rejection [dBFS]
THD = 0.03;         % Distorsión armónica total (fracción)
k = 2;              % Coeficiente de expansión

% ---- Incertidumbre del patron----
uP = 1.4/2;  

% ---- Incertidumbre Filtro----
uA = 0.95/sqrt(3);

% ---- No linealidad de la sensibilidad ----
uS = 2/sqrt(3);     

% ---- Ruido propio del micrófono (EIN) ----
ENOB = floor((-noise_floor - 1.76)/6.02);  % Bits efectivos
muC = (1/sqrt(3)) * 2^(n - 2 - ENOB);      % Desvío estándar en cuentas

% ---- Ripple de fuente ----
Vripple = 20;  % [mVpp]
Lripple = psr + 20*log10(Vripple/100) - S + Ref;  % Nivel equivalente [dB SPL]

% ---- Rango de niveles ----
L = linspace(33, 120, 1000);  % dB SPL

% ===========================
% Cálculo de componentes
% ===========================

% (1) Incertidumbre por ruido
C = (2^(n-1)-1) * 10.^((L + S - Ref)/20);
uC_dB = (20 * muC) ./ (log(10) * C);

% (2) Incertidumbre por ripple (PSR)
Rmax = 10.^((Lripple - L)/10);
uPSR_dB = (10/log(10)) .* (Rmax/sqrt(3)) ./ (1 + Rmax);

% (3) Incertidumbre por THD
uTHD_dB = (10/sqrt(3)) * log10(1 + THD^2);

% (4) Incertidumbre total combinada
u_total = sqrt(uS.^2 + uC_dB.^2 + uPSR_dB.^2 + uTHD_dB.^2 + uP.^2 + uA.^2);

% (5) Incertidumbre expandida
U = k * u_total;

% ===========================
% GRÁFICO 1: Componentes relativas a la incertidumbre total
% ===========================
uC_rel  = (uC_dB ./ u_total) * 100;
uPSR_rel = (uPSR_dB ./ u_total) * 100;
uTHD_rel = (uTHD_dB ./ u_total) * 100;
uS_rel   = (uS ./ u_total) * 100;
uP_rel   = (uP ./ u_total) * 100;
uA_rel   = (uA ./ u_total) * 100;

figure('Name','Componentes relativas a la incertidumbre total');
plot(L, uC_rel,  'b', 'LineWidth', 1.5); hold on;
plot(L, uPSR_rel,'r', 'LineWidth', 1.5);
plot(L, uTHD_rel,'m--', 'LineWidth', 1.2);
plot(L, uS_rel,  'g--', 'LineWidth', 1.2);
plot(L, uP_rel,  'c--', 'LineWidth', 1.2);
plot(L, uA_rel,  'y--', 'LineWidth', 1.2);
grid on;
xlabel('Nivel medido [dB SPL]');
ylabel('Peso relativa [%]');
title('Peso relativa de cada componente en la incertidumbre total');
legend('Ruido', 'Ripple (PSR)', 'THD', 'Sensibilidad','Patron', 'Filtro A', 'Location', 'northeast');

% ===========================
% GRÁFICO 2: Incertidumbre expandida relativa con varios umbrales
% ===========================
U_rel = (U ./ L) * 100;  % En porcentaje
figure('Name','Incertidumbre expandida relativa');
plot(L, U_rel, 'k', 'LineWidth', 2); hold on;

% ---- Define los umbrales de interés ----
umbrales = [10]; % % de incertidumbre relativa que querés marcar

% Dibujar líneas horizontales y marcar puntos
for i = 1:length(umbrales)
    plot([min(L) max(L)], [umbrales(i) umbrales(i)], 'r--', 'LineWidth', 1);
    text(L(1)+3, umbrales(i)+0.5, sprintf('%.2f %%', umbrales(i)), 'Color', 'r');

    idx = find(U_rel < umbrales(i), 1, 'first');
    if ~isempty(idx)
        L_i = L(idx);
        U_i = U_rel(idx);
        plot(L_i, U_i, 'bo', 'MarkerFaceColor','b');
        text(L_i+2, U_i, sprintf('%.1f dB SPL', L_i));
        fprintf('>> Nivel donde la incertidumbre relativa cae por debajo del %d%%: %.2f dB SPL\n', umbrales(i), L_i);
    else
        fprintf('>> No se alcanza el umbral del %d%% en el rango analizado.\n', umbrales(i));
    end
end

grid on;
xlabel('Nivel medido [dB SPL]');
ylabel('Ur [%]');
title('Incertidumbre relativa (k = 2)');
% ===========================
% GRÁFICO: Incertidumbre expandida (absoluta)
% ===========================
figure('Name','Incertidumbre expandida absoluta');
plot(L, U, 'k', 'LineWidth', 2); grid on; hold on;
xlabel('Nivel medido [dB SPL]');
ylabel('U [dB]');
title('Incertidumbre expandida (k = 2)');

% ---- Puntos a marcar (puedes cambiar estos valores)
niveles_marcados = [41 45 50 60 100 120];  % dB SPL

for i = 1:length(niveles_marcados)
    [~, idx] = min(abs(L - niveles_marcados(i)));
    Lp = L(idx);
    Up = U(idx);
    plot(Lp, Up, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6);
    text(Lp + 1, Up, sprintf('%.0f dB U=%.2f dB', Lp, Up), ...
        'Color', 'b', 'FontSize', 9, 'VerticalAlignment', 'bottom');
    fprintf('Nivel %.0f dB SPL: Incertidumbre expandida = %.3f dB\n', Lp, Up);
end
