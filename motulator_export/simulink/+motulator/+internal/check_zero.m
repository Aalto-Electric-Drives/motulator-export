function check_zero(obj, name, what)
%CHECK_ZERO Raise an error if the property NAME of OBJ is nonzero (not modeled).
if any(obj.(name)(:) ~= 0)
    error('motulator:unsupported', '%s (%s ~= 0) not supported.', what, name);
end
end
