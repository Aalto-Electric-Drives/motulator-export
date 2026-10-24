function check_class(value, classes)
%CHECK_CLASS Raise an error if VALUE is not of one of the classes CLASSES (a class
%   name or a cell array of them), i.e., not supported.
classes = cellstr(classes);
if ~any(cellfun(@(cls) isa(value, cls), classes))
    error('motulator:unsupported', 'Only %s supported, not %s.', ...
        strjoin(classes, ' or '), class(value));
end
end
