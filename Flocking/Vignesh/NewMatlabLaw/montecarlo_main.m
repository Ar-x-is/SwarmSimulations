clear
N_MC=1000
for i = 3120:4000+N_MC
    %tic
    i
    filename=strcat('MCdata',string(i));
    filename=strcat(filename','.csv');
    %filename
    montecarlo_params
    simparams
    %'running Simulation'
    main
    data=zeros(length(t),6*N+1);
    data(:,1)=t;
    data(:,2:end)=y;
    %disp('Writing to:')
    %filename
    writematrix(data,filename);
    toc
end