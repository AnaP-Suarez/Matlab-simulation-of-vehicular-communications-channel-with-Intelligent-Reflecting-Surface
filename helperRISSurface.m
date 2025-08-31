classdef helperRISSurface < matlab.System
    % untitled2 Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    % Public, tunable properties
    properties(Nontunable)
        ReflectorElement
        Size = [10 20]
        ElementSpacing = [0.5 0.5]
        OperatingFrequency = 7e8
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
     properties (Access = private)
         pPropagationSpeed = physconst('lightspeed');
         cAntArray
         cTxArray
         cRxArray
     end


    methods
        function obj = helperRISSurface(varargin)
            setProperties(obj, nargin, varargin{:});
            if isempty(coder.target)
                if isempty(obj.ReflectorElement)
                    obj.ReflectorElement = phased.IsotropicAntennaElement;
                end
             else
%                  if ~coder.internal.is_defined(obj.ReflectorElement)
%                      obj.ReflectorElement = phased.IsotropicAntennaElement;
%                  end
            end
        end

        function stv = getSteeringVector(obj)
            antarray = getRISArray(obj);
            stv = phased.SteeringVector('SensorArray',antarray);
        end
        
         function risarray = getRISArray(obj)
%              if isLocked(obj)
%                  risarray2 = obj.cAntArray;
%              else
%                  risarray = phased.URA(obj.Size,obj.ElementSpacing,'Element',obj.ReflectorElement);
%              end
            risarray = phased.URA(obj.Size,obj.ElementSpacing,'Element',obj.ReflectorElement);
         end


        function antArray = getAntArray(obj)
            if isempty(obj.cTxArray) || isempty(obj.cRxArray)
                obj.cTxArray = phased.Radiator('Sensor', obj.getRISArray(), 'OperatingFrequency', obj.OperatingFrequency);
                obj.cRxArray = phased.Collector('Sensor', obj.getRISArray(), 'OperatingFrequency', obj.OperatingFrequency);
            end
            antArray = obj.getRISArray();
        end
    end

    methods (Access = protected)
         function setupImpl(obj)
             % Perform one-time calculations, such as computing constants
             obj.cAntArray = phased.URA(obj.Size,obj.ElementSpacing,'Element',obj.ReflectorElement);
             obj.cTxArray = phased.Radiator('Sensor',obj.cAntArray,'OperatingFrequency',obj.OperatingFrequency);
             obj.cRxArray = phased.Collector('Sensor',obj.cAntArray,'OperatingFrequency',obj.OperatingFrequency);
         end

        function y = stepImpl(obj,x,ang_in,ang_out,w)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            y = obj.cTxArray(obj.cRxArray(x,ang_in).*w.',ang_out);
        end

%         function resetImpl(obj)
%              % Initialize / reset discrete-state properties
%              reset(obj.cAntArray);
%              reset(obj.cTxArray);
%              reset(obj.cRxArray);
%         end

        function releaseImpl(obj)
            release(obj.cAntArray);
            release(obj.cTxArray);
            release(obj.cRxArray);
        end
    end
end