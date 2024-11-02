function dx = DerivacionNumerica(x,t)
    n = length(t);
    dx = zeros(n,6);
    tol = 1e-6;
    
    % Primera derivada
    dx(1,:) = (x(2,:) - x(1,:))./(t(2) - t(1));
    
    % Derivadas centrales
    for i=2:n-1
        dt_forward = t(i+1) - t(i);
        dt_backward = t(i) - t(i-1);
        
        if abs(dt_forward) < tol || abs(dt_backward) < tol
            % Buscar el último punto válido hacia atrás
            j = i-1;
            while j > 1 && abs(t(j) - t(j-1)) < tol
                j = j-1;
            end
            % Usar la derivada del último punto válido
            dx(i,:) = dx(j,:);
        else
            % Diferencia central ponderada normal
            w1 = dt_forward/(dt_forward + dt_backward);
            w2 = dt_backward/(dt_forward + dt_backward);
            dx(i,:) = w1*(x(i,:) - x(i-1,:))/dt_backward + ...
                      w2*(x(i+1,:) - x(i,:))/dt_forward;
        end
    end
    
    % Última derivada
    if abs(t(n) - t(n-1)) < tol
        % Si el último punto está repetido, usar la última derivada válida
        dx(n,:) = dx(n-1,:);
    else
        dx(n,:) = (x(n,:) - x(n-1,:))./(t(n) - t(n-1));
    end
end