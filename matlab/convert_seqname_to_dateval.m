function val = convert_seqname_to_dateval(bagfullpath)
% Convert sequence folder name to a date value required by
% Body_T_Ars548(date) and Body_T_Zed2Imu(date).
% If the folder name contains 'aft' or 'eve', add 0.5 to the number.

    [parentFolder, ~, ~] = fileparts(bagfullpath); % Get parent folder
    [~, seqdate, ~] = fileparts(parentFolder);     % Get the name of the parent folder

    if contains(seqdate, 'aft') || contains(seqdate, 'eve')
        val = str2double(seqdate(1:end-4)) + 0.5;
    else
        val = str2double(seqdate);
    end
end
