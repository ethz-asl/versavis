function vector = readTopic(bag, topic)
    selected_bag = select(bag,'Topic',topic);
    selected_struct = readMessages(selected_bag);
    vector = zeros(length(selected_struct), 1);
    for i=1:length(selected_struct)
       vector(i) = selected_struct{i, 1}.Data; 
    end
end