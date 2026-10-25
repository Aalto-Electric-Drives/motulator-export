classdef Object < handle
    %OBJECT Base class of the classes of motulator.
    %   The arguments of the constructor of a class are its public properties, as
    %   in motulator: the leading arguments may be given by position, and the
    %   others as name-value arguments, e.g., SpeedController(J=0.015, alpha_s=25)
    %   or SpeedController('J', 0.015, 'alpha_s', 25). An empty value [] means None
    %   of motulator, i.e., the default.
    %
    %   PARAMS returns the arguments as a struct (nested for the arguments that
    %   are objects), which the masks of the blocks of the Simulink library take.

    methods
        function s = params(obj)
            %PARAMS Arguments of the object as a struct.
            s = struct();
            names = fieldnames(obj);
            for k = 1:numel(names)
                s.(names{k}) = value(obj.(names{k}));
            end
        end
    end

    methods (Access = protected)
        function set_args(obj, args, positional, required)
            %SET_ARGS Set the properties from the arguments of the constructor.
            %   SET_ARGS(OBJ, ARGS, POSITIONAL, REQUIRED) sets the properties named
            %   POSITIONAL from the leading arguments of the cell array ARGS and the
            %   others from the name-value pairs that follow, and checks that the
            %   properties REQUIRED are not empty.
            names = fieldnames(obj);
            k = 1;
            while k <= numel(args) && k <= numel(positional) ...
                    && ~is_name(args{k}, names)
                obj.(positional{k}) = args{k};
                k = k + 1;
            end
            if mod(numel(args) - k + 1, 2) ~= 0
                error('motulator:arguments', ['%s: the arguments after the ' ...
                    'positional ones must be name-value pairs.'], class(obj));
            end
            for j = k:2:numel(args)
                if ~is_name(args{j}, names)
                    error('motulator:arguments', '%s: unknown argument %s.', ...
                        class(obj), char(args{j}));
                end
                obj.(char(args{j})) = args{j + 1};
            end
            for j = 1:numel(required)
                if isempty(obj.(required{j}))
                    error('motulator:arguments', '%s: the argument %s is required.', ...
                        class(obj), required{j});
                end
            end
        end
    end

end

function v = value(v)
% Argument as a value of the struct: the arguments of an object, or a logical value
% as a number
if isa(v, 'motulator.internal.Object')
    v = v.params();
elseif islogical(v)
    v = double(v);
end
end

function tf = is_name(arg, names)
% Whether an argument is the name of a property
tf = (ischar(arg) && isrow(arg) || isa(arg, 'string') && isscalar(arg)) ...
    && any(strcmp(char(arg), names));
end
