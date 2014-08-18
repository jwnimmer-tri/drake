classdef Concatenated < drakeFunction.DrakeFunction
  % DrakeFunction representing the concatenation of n functions
  %
  % Implements either
  % \f[
  % f(x) = 
  % \begin{bmatrix}
  %   f_1(x_1) \\
  %   f_2(x_2) \\
  %   \vdots \\
  %   f_n(x_n)
  % \end{bmatrix},\;
  % \frac{df}{dx} = 
  % \begin{bmatrix}
  %   \frac{d f_1}{d x_1} &                     &         & 0                  \\
  %                       & \frac{d f_2}{d x_2} &         &                    \\
  %                       &                     & \ddots  &                    \\
  %   0                   &                     &         & \frac{d f_n}{d x_n} 
  % \end{bmatrix}
  % \f]
  %
  % where \f$x = (x_1, x_2, \dots, x_n)^\prime\f$ or 
  %
  % \f[
  % f(x) = 
  % \begin{bmatrix}
  %   f_1(x) \\
  %   f_2(x) \\
  %   \vdots \\
  %   f_n(x)
  % \end{bmatrix},\;
  % \frac{df}{dx} = 
  % \begin{bmatrix}
  %   \frac{d f_1}{d x} \\
  %   \frac{d f_2}{d x} \\
  %   \vdots            \\
  %   \frac{d f_n}{d x} 
  % \end{bmatrix}
  % \f]
  properties (SetAccess = immutable)
    contained_functions     % Cell array of DrakeFunction objects
    n_contained_functions   % Number of elements in contained_functions
    same_input              % Logical scalar indicating whether all of
                            % the contained function share the same input
  end
  properties (Access = private)
    input_map
  end

  methods
    function obj = Concatenated(fcns,same_input)
      % obj = Concatenated(fcns, same_input) returns a DrakeFunction
      %   representing the concatenation of a given set of
      %   DrakeFunctions. If same_input = true, the input to the
      %   concatenated function is passed to all of the component
      %   functions. Otherwise, the input to the concatenated function
      %   is split and distributed to the component functions.
      %
      % obj = Concatenated(fcns) is the same as Concatenated(fcns,true).
      %
      % @param fcns         -- Cell array of DrakeFunction objects
      % @param same_input   -- Logical scalar
      if nargin < 2, same_input = false; end
      typecheck(fcns,'cell');
      assert(all(cellfun(@(arg)isa(arg,'drakeFunction.DrakeFunction'), fcns)));

      [input_frame,input_map] = drakeFunction.Concatenated.constructInputFrame(fcns,same_input);
      output_frame = drakeFunction.Concatenated.constructOutputFrame(fcns);

      obj = obj@drakeFunction.DrakeFunction(input_frame, output_frame);

      obj.contained_functions = fcns;
      obj.n_contained_functions = numel(fcns);
      obj.same_input = same_input;
      obj.input_map = input_map;
    end

    function [f,df] = eval(obj,x)
      [f_cell,df_cell] = evalContainedFunctions(obj,x);
      [f,df] = combineOutputs(obj,f_cell,df_cell);
    end

    function [iCfun, jCvar] = getSparsityPattern(obj)
      % [iCfun, jCvar] = getSparsityPattern(obj) returns the row and
      %   column indices of the potentially non-zero elements of this
      %   function's Jacobian.
      %
      % @param obj      -- drakeFunction.Concatenated object 
      % 
      % @retval iCfun   -- Vector of row indices of the non-zeros
      % @retval jCvar   -- Vector of column indices of the non-zeros
      f_cell = cell(obj.n_contained_functions,1);
      df_cell = cell(obj.n_contained_functions,1);
      for i = 1:obj.n_contained_functions
        f_cell{i} = NaN(obj.contained_functions{i}.getNumOutputs(),1);
        df_cell{i} = ones(obj.contained_functions{i}.getNumOutputs(), ...
                          obj.contained_functions{i}.getNumInputs());
      end
      [~,df] = combineOutputs(obj,f_cell,df_cell);
      [iCfun, jCvar] = find(df);
    end
  end

  methods (Access = private)
    function [f_cell,df_cell] = evalContainedFunctions(obj,x)
      % [f_cell,df_cell] = evalContainedFunctions(obj,x) returns the
      % function values and gradients for each of the component
      % functions
      % 
      % @param obj        -- drakeFunction.Concatenated object
      % @param x          -- Input vector
      %
      % @retval f_cell    -- Cell array of function values for each
      %                      component function
      % @retval df_cell   -- Cell array of Jacobians for each component
      %                      function
      x_cell = cell(1,obj.input_frame.getNumFrames);
      f_cell = cell(size(x_cell));
      df_cell = cell(size(x_cell));
      if obj.same_input
        x_cell(:) = {x};
      else
        x_cell = splitCoordinates(obj.input_frame, x);
      end
      contained_functions_local = obj.contained_functions;
      for i = 1:obj.n_contained_functions
        [f_cell{i},df_cell{i}] = contained_functions_local{i}.eval(reshape([x_cell{obj.input_map{i}}],[],1));
      end
    end

    function [f,df] = combineOutputs(obj,f_cell,df_cell)
      f = vertcat(f_cell{:});
      if obj.same_input
        df = vertcat(df_cell{:});
      else
        df = blkdiag(df_cell{:});
      end
    end
  end

  methods (Static, Access = private)
    function [input_frame, input_frame_to_fcn_map] = constructInputFrame(fcns, same_input)
      if nargin < 2, same_input = false; end
      fcn_input_frames = cellfun(@(fcn) fcn.getInputFrame(), ...
        fcns,'UniformOutput',false);
      if same_input
        % Check that all elements of fcns have the same input_frame
        input_frame = fcn_input_frames{1};
        assert(all(cellfun(@(frame) isequal_modulo_transforms(frame,input_frame),fcn_input_frames)), ...
          'Drake:DrakeFunction:InputFramesDoNotMatch', ...
          ['If ''same_input'' is set to true, all functions must ' ...
           'have the same input frame']);
        input_frame_to_fcn_map = repmat({1:input_frame.getNumFrames()},numel(fcns),1);
      else
        input_frame_to_fcn_map = cell(1,numel(fcns));
        input_frame_to_fcn_map{1} = 1:fcn_input_frames{1}.getNumFrames();
        for i = 2:numel(fcns)
          input_frame_to_fcn_map{i} = input_frame_to_fcn_map{i-1}(end)+(1:fcn_input_frames{i}.getNumFrames()); 
        end
        input_frame = MultiCoordinateFrame(fcn_input_frames);
      end
    end

    function output_frame = constructOutputFrame(fcns)
      fcn_output_frames = cellfun(@(fcn) fcn.getOutputFrame(), ...
        fcns,'UniformOutput',false);
      fcn_output_frames(cellfun(@(frame) frame.dim == 0, fcn_output_frames)) = [];
      output_frame = MultiCoordinateFrame.constructFrame(fcn_output_frames);
    end

  end
end
