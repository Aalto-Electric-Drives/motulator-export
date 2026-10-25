function add_mask(blk, type, description, params, init, display)
%ADD_MASK Mask of a block of a class of motulator.
%   ADD_MASK(BLK, TYPE, DESCRIPTION, PARAMS, INIT, DISPLAY) adds to the block
%   BLK the mask of the type TYPE and the description DESCRIPTION with the
%   parameters PARAMS {variable, prompt, value}, the initialization commands
%   INIT, and the drawing commands DISPLAY of its icon (see icon).
mask = Simulink.Mask.create(blk);
mask.Type = type;
mask.Description = description;
mask.Initialization = init;
mask.Display = display;
for k = 1:size(params, 1)
    mask.addParameter('Type', 'edit', 'Name', params{k, 1}, ...
        'Prompt', params{k, 2}, 'Value', params{k, 3}, 'Tunable', 'off');
end
end
