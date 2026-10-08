function str = num(x)
%NUM Number with full precision, or an expression given as a character vector.
if ischar(x)
    str = x;
    return
end
str = sprintf('%.17g', x);
end
