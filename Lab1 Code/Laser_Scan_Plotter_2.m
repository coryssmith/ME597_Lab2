function Laser_Scan_Plotter
    % Filenames
    files = {
        'laser_content_circle.csv',...
        'laser_content_line.csv',...
        'laser_content_spiral.csv'
    };
    labels = {'circle','line','spiral'};
    
    for i = 1:numel(files)
        filename = files{i};
        
        fid = fopen(filename,'r');
        headerLine = fgetl(fid);   % skip header
        dataLine   = fgetl(fid);   % read next line
        fclose(fid);
        
        % --- 1) Parse the line using textscan ---
        % The format '"%[^"]",%f,%f' means:
        %   - Start with a double quote "
        %   - Read everything up to the next double quote into C{1}{1}
        %   - Then a comma
        %   - Then a float (C{2})
        %   - Then another comma
        %   - Then a float (C{3})
        formatSpec = '"%[^"]",%f,%f';
        C = textscan(dataLine, formatSpec);
        
        % If parsing fails, C might be empty or partial. So check:
        if isempty(C) || length(C) < 3 || isempty(C{1}) || isempty(C{2}) || isempty(C{3})
            error('Could not parse line properly. dataLine = %s', dataLine);
        end
        
        scan_str = C{1}{1};   % the full "array('f',[...])" minus surrounding quotes
        inc      = C{2};      % numeric angle increment
        t        = C{3};      % numeric timestamp
        
        % --- 2) Clean up the scan string ---
        % remove array('f', parentheses, etc. if needed
        scan_str = strrep(scan_str, "array('f',", "");
        scan_str = strrep(scan_str, "(", "");
        scan_str = strrep(scan_str, ")", "");
        scan_str = strrep(scan_str, "[", "");
        scan_str = strrep(scan_str, "]", "");
        
        % --- 3) Split by comma inside the array string & convert to double
        scan_parts = split(scan_str, ',');
        scan = str2double(strtrim(scan_parts));  % strtrim to remove leading spaces
        
        % --- 4) Build angle array & compute x,y
        n = numel(scan);
        theta = 0 : inc : (n-1)*inc;
        
        scan  = scan(:);      % make scan n×1
        theta = theta(:);     % make theta n×1
        
        x = scan .* cos(theta);
        y = scan .* sin(theta);
        
        % --- 5) Plot ---
        figure;
        plot(0,0,'ro','MarkerFaceColor','r','MarkerSize',5); % robot origin
        hold on;
        scatter(x,y,'.','MarkerEdgeColor','r');
        hold off;
        title(sprintf('X-Y scatter plot of robot laser scan at t=%.3fs for %s motion', ...
                      t, labels{i}));
        xlabel('x [meters]');  ylabel('y [meters]');
        legend('Robot','Laser Points');
        grid on;
    end
end
