% This function is used to read the data provided the file path and store
% it in a cell named 'data'



% Processing the data for computation purpose.

function data = getData(file_path)
% % Opening the file which contains the data for two fingers each made of a 
% % SCARA manipulator grasping a box at two contact locations.

    if class(file_path) == 'char'
        fileID = fopen(file_path);
        % N = 2;
        formatSpec = '%s';

        % Read data into a single cell array wherein one element contains the enter
        % line.
        data = textscan(fileID, formatSpec,'Delimiter', '\n');
        fclose(fileID);
    end
end