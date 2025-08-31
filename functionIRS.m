function   SNR  = functionIRS(apx,apy,uex,uey,x,vehicleID)

fc = 7e8;     %  frecuencia de la portadora
c = physconst('lightspeed');
lambda = c/fc;
rng(2023);          % generador de numeros aleatorios

% Setup surface
Nr = 10;        % cantidad de elementos horizontales
Nc = 20;        % cantidad de elementos verticales
dr = 0.5*lambda;    % espaciado horizontal
dc = 0.5*lambda;    % espaciado vertical

% construct surface
ris = helperRISSurface('ElementSpacing',[dr dc],...
    'ReflectorElement',phased.IsotropicAntennaElement,'OperatingFrequency',fc);      

% scene
%apx=obj.payloadPosBuffer(1);
%apy=obj.payloadPosBuffer(2);
pos_ap = [apx;apy;1.5];    % se coloca a una altura de 1.5 metros del suelo
pos_ris = [107;0;1.5];    % annadir la posicion del IRS, ahora esta puesta en el origen de coordenadas, 
                      % se coloca a una altura 1.5 de metros del suelo
v_ap = zeros(3,1);       % annadir la velocidad de los carros, ahora esta en 0

%disp(ap)

 %dist_ap_irs=sqrt((apx-107)^2+(apy)^2);
% dist_ue_irs=sqrt((uex-107)^2+(uey)^2);

%uex=entity.data.pos(1);
%uey=entity.data.pos(2);
pos_ue = [uex;uey;1.5];   % se coloca a una altura de 1.5 metros del suelo
v_ue = zeros(3,1);     % annadir la velocidad de los carros, ahora esta en 0

% compute the range and angle of the RIS from the base station and the UE
[r_ap_ris,ang_ap_ris] = rangeangle(pos_ap,pos_ris);
[r_ue_ris,ang_ue_ris] = rangeangle(pos_ue,pos_ris);


% signal  
% xt = entity.data.Body
fs = 7e9;   %frecuencia de muestreo
N0dB = -89;
tx = phased.Transmitter('PeakPower',1,'Gain',1);
xt = tx(x);

% channel con maxima distancia de propagacion de 500 metros
chanAPToRIS = phased.FreeSpace('OperatingFrequency',fc,'SampleRate',fs,'MaximumDistanceSource','Property','MaximumDistance',500);
chanRISToUE = phased.FreeSpace('OperatingFrequency',fc,'SampleRate',fs,'MaximumDistanceSource','Property','MaximumDistance',500);
chanAPToUE = phased.FreeSpace('OperatingFrequency',fc,'SampleRate',fs,'MaximumDistanceSource','Property','MaximumDistance',500);

aaa = 1; % para escoger el caso en las pruebas

switch aaa

    case 1
        % SNR para linea de vision directa Tx-Rx
        chanOut1 = chanAPToUE(xt,pos_ap,pos_ue,v_ap,v_ue);
        SNR = pow2db(bandpower(chanOut1))-N0dB;
    
    case 2       
        % SNR con IRS con coeficientes de reflexion todos en 1
        %y annadido el enlace de linea de vision directa
        rcoeff_ris = ones(Nr*Nc,1);
        x_ris_in = chanAPToRIS(xt,pos_ap,pos_ris,v_ap,v_ue);
        x_ris_out = ris(x_ris_in,ang_ap_ris,ang_ue_ris,rcoeff_ris);
        chanOut1 = chanRISToUE(x_ris_out,pos_ris,pos_ue,v_ap,v_ue)+chanAPToUE(xt,pos_ap,pos_ue,v_ap,v_ue);
        SNR = pow2db(bandpower(chanOut1))-N0dB;

    case 3
        % channel estimation para configurar 
        % los coeficientes de refleccion del IRS
        stv = getSteeringVector(ris);
        r_ue_ap = norm(pos_ap-pos_ue);
        hd = db2mag(-fspl(r_ue_ap,lambda))*exp(1i*2*pi*r_ue_ap/c);
        g = db2mag(-fspl(r_ap_ris,lambda))*exp(1i*2*pi*r_ap_ris/c)*stv(fc,ang_ap_ris);
        hr = db2mag(-fspl(r_ue_ris,lambda))*exp(1i*2*pi*r_ue_ris/c)*stv(fc,ang_ue_ris);

        % compute optimal phase control
        rcoeff_ris = exp(1i*(angle(hd)-angle(hr)-angle(g)));
        
        % SNR con IRS con coeficientes de reflexion optimos
        %y annadido el enlace de linea de vision directa 
        x_ris_in = chanAPToRIS(xt,pos_ap,pos_ris,v_ap,v_ue);
        x_ris_out = ris(x_ris_in,ang_ap_ris,ang_ue_ris,rcoeff_ris);
        chanOut1 = chanRISToUE(x_ris_out,pos_ris,pos_ue,v_ap,v_ue)+chanAPToUE(xt,pos_ap,pos_ue,v_ap,v_ue);
        SNR = pow2db(bandpower(chanOut1))-N0dB;

    case 4
        % channel estimation para configurar 
        % los coeficientes de refleccion del IRS
        stv = getSteeringVector(ris);
        g = db2mag(-fspl(r_ap_ris,lambda))*exp(1i*2*pi*r_ap_ris/c)*stv(fc,ang_ap_ris);
        hr = db2mag(-fspl(r_ue_ris,lambda))*exp(1i*2*pi*r_ue_ris/c)*stv(fc,ang_ue_ris);
        
        % compute optimal phase control
        rcoeff_ris = exp(1i*(-angle(hr)-angle(g)));
        
        % SNR con IRS con coeficientes de reflexion optimos
        x_ris_in = chanAPToRIS(xt,pos_ap,pos_ris,v_ap,v_ue);
        x_ris_out = ris(x_ris_in,ang_ap_ris,ang_ue_ris,rcoeff_ris);
        chanOut1 = chanRISToUE(x_ris_out,pos_ris,pos_ue,v_ap,v_ue);
        SNR = pow2db(bandpower(chanOut1))-N0dB;

    case 5
       % SNR para linea de vision directa Tx-Rx con multitrayecto Rayleigh
%        if uex<107 && apx<107
%            h1 = 107-uex;
%            h = 107-apx;
%        else
%            h1 = uex-92;
%            h = apx-92;
%        end
%        d = abs(apy-uey);
%        cte = h/h1;
%        x = d/(1+h1/h);
%        y = sqrt(x^2 + h^2);
%        y1 = y*cte;
%        dist = y1 + y;

%        atenucion = 20*log10(4*pi*200*fc/c);
        
        rayleighchan = comm.RayleighChannel( ...
            'SampleRate', fs, ...
            'PathDelays', [0 1.5e-6 7e-6], ...
            'AveragePathGains', [20 28 33], ...
            'NormalizePathGains', true, ...
            'MaximumDopplerShift', fc/(fs/2)*10, ... % Ajuste para el ancho de banda de 10 MHz
            'DopplerSpectrum', doppler('Gaussian', 0.6), ...
            'RandomStream', 'Global stream', ...
            'PathGainsOutputPort', false);
        
        % x es la señal que quieres pasar a través del canal
        chanOut1 = rayleighchan(x);

        SNR = pow2db(bandpower(chanOut1))-N0dB;

    case 6
        % SNR para linea de vision directa Tx-Rx con multitrayecto
        numpaths = 3;
        propagator = phased.MultipathChannel('OperatingFrequency',fc,'SampleRate',fs);    
        chanOut = chanAPToUE(xt,pos_ap,pos_ue,v_ap,v_ue);
        pathmat = [0.000667 0.000833 0.001166; 0.04 0.053 0.067; 1.5376 0.3988 3.3213];
        dop = [1.0036 1.0034 1.0026];
        aloss = [fc 0 0 0];
        chanOutR = repmat(chanOut,numpaths);
        chanOutP = propagator(chanOutR, pathmat, dop, aloss);
        chanOut1 = sum(chanOutP, 2);
        SNR = pow2db(bandpower(chanOut1))-N0dB;

    case 7
        %SNR multitrayecto distancia
        pos_cen = [100;6;1.5];
        dist1 = norm(pos_cen-pos_ap);
        dist2 = norm(pos_cen-pos_ue);
        pos_ap2 = [apx+dist1+dist2;apy;1.5];
        chanOut1 = chanAPToUE(xt,pos_ap,pos_ap2,v_ap,v_ue);
        SNR = pow2db(bandpower(chanOut1))-N0dB;

    case 8
        %SNR multitrayecto coef1
        rcoeff_ris = ones(Nr*Nc,1);
        x_ris_in = chanAPToRIS(xt,pos_ap,pos_ris,v_ap,v_ue);
        x_ris_out = ris(x_ris_in,ang_ap_ris,ang_ue_ris,rcoeff_ris);
        chanOut1 = chanRISToUE(x_ris_out,pos_ris,pos_ue,v_ap,v_ue);
        SNR = pow2db(bandpower(chanOut1))-N0dB;      

    case 9
        %SNR 2 IRS
        ris_1 = helperRISSurface('ElementSpacing',[dr dc],...
               'ReflectorElement',phased.IsotropicAntennaElement,'OperatingFrequency',fc);  
        pos_ris_1 = [107;12;1.5]; 
        [r_ap_ris_1,ang_ap_ris_1] = rangeangle(pos_ap,pos_ris_1);
        [r_ue_ris_1,ang_ue_ris_1] = rangeangle(pos_ue,pos_ris_1);
        [r_ris_ris_1,ang_ris_ris_1] = rangeangle(pos_ris_1,pos_ris);
        
        % channel estimation para configurar 
        % los coeficientes de refleccion del IRS
        stv = getSteeringVector(ris);
        stv_1 = getSteeringVector(ris_1);
        g = db2mag(-fspl(r_ap_ris,lambda))*exp(1i*2*pi*r_ap_ris/c)*stv(fc,ang_ap_ris);
        hr = db2mag(-fspl(r_ue_ris,lambda))*exp(1i*2*pi*r_ue_ris/c)*stv(fc,ang_ue_ris);
        g_1 = db2mag(-fspl(r_ap_ris_1,lambda))*exp(1i*2*pi*r_ap_ris_1/c)*stv_1(fc,ang_ap_ris_1);
        hr_1 = db2mag(-fspl(r_ue_ris_1,lambda))*exp(1i*2*pi*r_ue_ris_1/c)*stv_1(fc,ang_ue_ris_1);
        k = db2mag(-fspl(r_ris_ris_1,lambda))*exp(1i*2*pi*r_ris_ris_1/c)*stv(fc,ang_ris_ris_1);
        k_1 = db2mag(-fspl(r_ris_ris_1,lambda))*exp(1i*2*pi*r_ris_ris_1/c)*stv_1(fc,ang_ris_ris_1);
        
        % compute optimal phase control
        rcoeff_ris = exp(1i*(-angle(k)-angle(g)));
        rcoeff_ris_1 = exp(1i*(-angle(hr_1)-angle(k_1)));
        rcoeff_risi = exp(1i*(-angle(k_1)-angle(g_1)));
        rcoeff_ris_1i = exp(1i*(-angle(hr)-angle(k)));       
        
        dist1 = norm(pos_ap-pos_ris);
        dist2 = norm(pos_ap-pos_ris_1);

        % SNR con IRS con coeficientes de reflexion optimos
        if dist1 < dist2 
        % if vehicleID == 1
            x_ris_in = chanAPToRIS(xt,pos_ap,pos_ris,v_ap,v_ue);
            x_ris_out = ris(x_ris_in,ang_ap_ris,ang_ris_ris_1,rcoeff_ris);
            x_ris_out_1 = ris_1(x_ris_out,ang_ris_ris_1,ang_ue_ris_1,rcoeff_ris_1);
            chanOut1 = chanRISToUE(x_ris_out_1,pos_ris_1,pos_ue,v_ap,v_ue);
        else
            x_ris_in_1 = chanAPToRIS(xt,pos_ap,pos_ris_1,v_ap,v_ue);
            x_ris_out_1 = ris_1(x_ris_in_1,ang_ap_ris_1,ang_ris_ris_1,rcoeff_risi);
            x_ris_out = ris(x_ris_out_1,ang_ris_ris_1,ang_ue_ris,rcoeff_ris_1i);
            chanOut1 = chanRISToUE(x_ris_out,pos_ris,pos_ue,v_ap,v_ue);
        end
        SNR = pow2db(bandpower(chanOut1))-N0dB;

    case 10
        receivedpower = ChanelRaytrace(pos_ap,pos_ue);
        SNR = receivedpower-N0dB;
end


%    disp(dist_ap_irs)
%     disp(dist_ue_irs)

% disp(bandpower(chanOut1));
% disp(chanOut1);
% disp('fin de sennal');
