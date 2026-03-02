function Xl = read_world()
    filename = "../data/world.dat";
    file_id = fopen(filename, 'r');
    if file_id == -1,
        printf('Error opening file: %s\n', filename);
        return;
    end

    Xl = [];
    while ~feof(file_id),
        line = fgetl(file_id);
        if isempty(line),
            continue;
        end

        data = sscanf(line, '%d %f %f %f');
        Xl = [Xl; data(2:4)'];
    end

end