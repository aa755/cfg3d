% filelist = {'blackDesktop.obj'};

%filelist = {'bed.obj','chair.obj','computerset.obj','fridge.obj','laptopOnTable.obj','shelf.obj','sofa_single2.obj','sofa_table.obj','tableChiars.obj','tv.obj'};
%bigfiles = {'office.obj','bathroom.obj','bedroom.obj','coach.obj','kitchen.obj','sofa_single.obj','sofa.obj'};

% num_positions = 3;
% height = 1.8;
% radius = 1.5;
% with_noise = 1;
% mesh2pc('computerset.obj', num_positions, height, radius, with_noise);

%for i=2:length(filelist)
    filename = 'blackDesktop.obj';
%     filename = 'dellNewest.obj';
    num_positions = 3;
    height = 1.8;
    radius = 1.5;
    with_noise = 1;
    
    mesh2pc(filename, num_positions, height, radius, with_noise);
%end