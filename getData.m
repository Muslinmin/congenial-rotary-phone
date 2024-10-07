function data = getData(board)
    msgSize = 1;
    PACKETS = 1;  % Number of packets to store, 1 packet = 1 samples  
    datastring = '';
    dataIn = [];

    i = 1;
    for j = 1:PACKETS       
        % Wait for message Startheader '111'
        headerCount = 0;
        while (headerCount < 3)
            data = read(board, 1, 'char');
            if (data == '1')
                headerCount = headerCount + 1;
            else
                headerCount = 0;
            end
        end

        % Decode message till message endHeader 'FFF' detected
        headerCount = 0;
        while (headerCount < 3)
            data = read(board, 1, 'char');  % Correct usage of 'read'
            if (data == 'F')
                headerCount = headerCount + 1;
            else
                headerCount = 0;
                if (data ~= ',')
                    datastring = strcat(datastring, char(data));
                else
                    if (~isempty(datastring))
                        dataIn(i) = str2double(datastring);
                        i = i + 1;
                        datastring = '';
                    end
                end
            end
        end

        % Store the parsed data
        data = dataIn;
        %disp(data);
        %disp(datastring);

        % Flush the serial buffer
        flush(board);
    end
end
