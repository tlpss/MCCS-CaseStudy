%==========================================================================
%   TP :            Case study: Exercse 1
%   Contact:        ezequiel.gonzalezdebada@epfl.ch
%==========================================================================
classdef ex1
    %Class gathering the solutions of exercise 1. 
    methods (Static)
        %
        function varargout = getSystemParameters
            % PARAMETERS = getSystemParameters() returns a 5-elements
            % column vector containing the value of the system parameters 
            % and the linearization point. Specifically it should contain 
            % (in the presented order):
            %   - k : value of curvature of the reference path.
            %   - car_length [m]: car's length.  
            %   - sigma_v : coefficient characterizing the dynamic of the
            %   actuator tracking the speed reference. 
            %   - sigma_S : coefficient characterizing the dynamic of the
            %   actuator tracking the steering wheel's reference position. 
            %   - spReg : value of the speed the vehicle should drive at. 

            varargout = {[1e-10;4;1;5;5]};               
        end
        %
        function varargout = getLinealModelArrays(parameters)
            % [A,B,C,D] = getLinealModelArrays(PARAMETERS) returns the
            % matrices A,B,C,D characterizing the continuous-time linear
            % model of the system. The input *parameters* corresponds to 
            % the output of the method *getSystemParameters*.
            
            %%- Calculate arrays A,B,C,D
            K = parameters(1);
            L = parameters(2);
            sigma_v = parameters(3);
            sigma_phi = parameters(4);
            v = parameters(5);
            A = [[0, K*v, 0, 1, 0];[0,0,v,0,0];[0,-K^2*v,0,K*(1-L^2)/L^2, v*(K^2+L^2)/16/L^3];[0,0,0,-sigma_v,0];[0,0,0,0,-sigma_phi]];
            B = zeros(5,2);
            B(4,1) = parameters(3);
            B(5,2) = parameters(4);
            C = eye(5);
            D = zeros(5,2);
            
            varargout = {A,B,C,D};
        end        
        %
        function varargout = getDiscreteLinearModel(A,B,C,D,sampling_time,method)
            % [PHI,GAM] =
            % getDiscreteLinearModel(A, B, C, D, SAMPLING_TIME, METHOD)
            % returns the PHI and GAMMA matrices characterizing the
            % discrete-time linear model of the system given the matrices
            % A,B,C,D of the continuous-time linear model and the desired 
            % SAMPLING_TIME.
            %
            % Additionally, the input METHOD is a string
            % indicating the method that should be used to calculate 
            % the matrices PHI and GAMMA. It can take values 
            % - Euler : Euler approximation as discretization method. 
            % - c2d : use the matlab command c2d. 
            Phi = [];
            Gamma = [];
            
            if strcmp(method,'Euler')
                % Calculate the discrete-time linear model using the 
                % Euler approximation.
                
                Phi = eye(5) + sampling_time*A;
                Gamma = sampling_time*B;               
                
            elseif strcmp(method,'c2d')
                %%- Build continuous representation of the system with 'ss'
                
                Mc = ss(A,B,C,D);
                
                %%- Calculate the discrete-time linear model of the system 
                % using the command 'c2d'
                
                Md = c2d(Mc,sampling_time ); 
                
                %%- Extract from Md, the Phi and Gamma matrices. 
                
                Phi = Md.A;
                Gamma = Md.B;
                
                %%-set up output of the function 
            end
            varargout = {Phi, Gamma};
        end                
        %
        function varargout = getWorkingTrajectory(sampling_time, simulation_time, parameters)
            % [NOMINAL_TRAJECTORY_X, NOMINAL_TRAJECTORY_U] =
            % getWorkingTrajectory(SAMPLING_TIME, SIMULTAION_TIME,
            % PARAMETERS)  
            % outputs the NOMINAL_TRAJECTORY_X and NOMINAL_TRAJECTORY_U
            % given the SAMPLING_TIME between data points, the
            % SIMULATION_TIME up to which the trajectory has to be created,
            % and the vector PARAMETERS with the value sof tha system's
            % parameters.
            %
            % The outputs NOMINAL_TRAJECTORY_X, and NOMINAL_TRAJECTORY_U must
            % be arrays [t | \bar{x}] and [t | \bar{u}] 
            % whose first column corresponds to the timespan of
            % the data point, and following columns store the information
            % of the states and inputs at the corresponding time.
            %
            % The defined output trajectories are meant to be imported in
            % Simulink with the "From Workspace" block. If any
            % additional doubt regarding how the data should be formated,
            % read the information provided in the mentioned simulink block.
            %
            % todos
            % - create time vector. 
            % - create the nominal states trajectory output
            % - create the control inputs nominal trajectory output
            
            %%- create time vector
            time_vector = (0:sampling_time:simulation_time);
            
            %%-create nominal state trajectory. 
            x1 = parameters(5)*time_vector;
            x2 = zeros(1,size(x1,2));
            x3 = zeros(1,size(x1,2));
            x4 = ones(1,size(x1,2))*parameters(5);
            x5 = ones(1,size(x1,2))*16*atan(parameters(1)/parameters(2));
            
            
            bar_x = [x1', x2', x3', x4', x5'];
            %disp(size(bar_x));
            %disp(size(time_vector'));
            nominal_trajectory_x = [time_vector', bar_x ];
            
            %%-create nominal control input trajectory. 
            bar_u = [x4', x5'];
            nominal_trajectory_u = [time_vector', bar_u];
            
            varargout = {nominal_trajectory_x, nominal_trajectory_u};
        end
        %
        function varargout = getInitialState(nominal_trajectory_x)
            %[X0, X0TILDE] = getInitialState(NOMINAL_TRAJECTORY_X)
            % returns the initial state X0 of the system and the
            % initial state X0TILDE of the linear models given the 
            % information on the exercise handout and the
            % NOMINAL_TRAJECTORY_X.
            %
            % The outputs should be column vectors. 
            %
            % Remember that by definition \tilde{x} = x - \overline{x}.
            
            
            %%- define the value of x0 for experiment 1
            x0_experiment_1 = nominal_trajectory_x(1,2:6);
            %x0_experiment_1
            %%- define the value of x0Tilde for experiment 1
            x0Tilde_experiment_1 = [0,0,0,0,0]; % x0 defined on trajectory!
            
            %including the different values for different experiments as a
            %cell
            x0 = {x0_experiment_1};
            x0Tilde = {x0Tilde_experiment_1};
            
            %set outputs of the function 
            varargout = {x0,x0Tilde};
        end
        %
        function varargout = getOpenLoopInputSignal(sampling_time, simulation_time)
            %[INPUT_CONTROL_ACTIONS_OPEN_LOOP] = getOpenLoopInputSignal(SAMPLING_TIME, SIMULATION_TIME)
            % outputs an input sequence to be applied in open loop to the 
            % models. The desired SAMPLING_TIME between data points as
            % well as the SIMULTION_TIME are provided. 
            %
            % As in the case of GETWORKINGTRAJECTORY function, the outputs
            % are meant to be used in Simulink with "From Workspace"
            % importing modules. If any additional doubt regarding how the
            % data should be structured, read the information provuded by
            % the mentioned simulink block. 
            %
            % todo:
            % - Declare an appropriate timespan vector. 
            % - Create the input_control_actions_open_loop array with the
            % sequence of control inputs to be applied in open loop. 
            %
            %
            % Notice: alternatively, this function can output a cell with
            % several arrays showing different control sequences to be
            % applied. This would make the provided script to run the
            % simulink mdel as many times as different control sequences
            % are gathered within the cell. Meaning that several
            % experiments can be set at once. 
            %
            
            %%- Create a time vector.
            %time_vector = ;
            
            %%- set the control sequence to be applied in open loop for the
            %%1st experiment. 
            time_vector = [];
            u1_open_loop = [];
            u2_open_loop = [];
            uOpenLoop_experiment_1 = [time_vector, u1_open_loop, u2_open_loop];
            
            %Include different values for the different experiments in a
            %cell.
            input_control_actions_open_loop = {uOpenLoop_experiment_1};
            
            %set output of the function
            varargout = input_control_actions_open_loop;
        end
        %
        
        function testNomTrajectory()
            [x,u]= ex1.getWorkingTrajectory(1,5,ex1.getSystemParameters());
            x,u
            
        
        end 
    end
    
    
    
end

