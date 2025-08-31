function receivedPower = ChanelRaytrace(pos_tx, pos_rx)
    % if ~exist('viewer', 'var')
    %     viewer = siteviewer("SceneModel","calle.stl");
    % end
    tx = txsite("cartesian", ...
        "AntennaPosition",pos_tx, ...
        "TransmitterPower",1, ...
        'TransmitterFrequency',700e6);
    rx = rxsite("cartesian", ...
        "AntennaPosition",pos_rx);
%     show(rx)
%     show(tx)
    rtpm = propagationModel("raytracing", ...
        "CoordinateSystem","cartesian", ...
        "Method","sbr", ...
        "MaxNumReflections",10, ...
        "BuildingsMaterial","concrete" , ...
        "TerrainMaterial","concrete");
%         raytrace(tx,rx,rtpm,"Type","pathloss");
    rays = raytrace(tx,rx,rtpm,"Type","pathloss");
    plPerfect = [rays{1}.PathLoss];
    receivedPower = 0;
    txPower = 10*log10(tx.TransmitterPower);
    for i = 1:length(plPerfect)
       receivedPower = receivedPower + 10^((txPower - plPerfect(i))/10);
       if i == 3
           break;
       end
    end
    receivedPower = 10*log10(receivedPower); %
    disp(['SNR recibida en el receptor: ', num2str(receivedPower+90), ' dB']);
end