function opts = options(args, opts)
%OPTIONS Name-value arguments of a method.
%   OPTS = OPTIONS(ARGS, OPTS) sets the fields of the struct OPTS (the defaults)
%   from the name-value pairs of the cell array ARGS.
if mod(numel(args), 2) ~= 0
    error('motulator:arguments', 'The arguments must be name-value pairs.');
end
for k = 1:2:numel(args)
    name = char(args{k});
    if ~isfield(opts, name)
        error('motulator:arguments', 'Unknown argument %s.', name);
    end
    opts.(name) = args{k + 1};
end
end
