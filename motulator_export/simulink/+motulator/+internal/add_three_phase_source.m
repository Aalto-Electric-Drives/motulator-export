function add_three_phase_source(blk, pos)
%ADD_THREE_PHASE_SOURCE Block of ThreePhaseSource with constant magnitude and
%   frequency.
%   The grid voltage e_g_ab = e_g*exp(1j*w_g*t) in stationary coordinates, from
%   Sine Wave blocks. The mask has the arguments of ThreePhaseSource.
import motulator.internal.*
add(blk, 'built-in/Subsystem', pos);
add([blk '/e_g_alpha'], 'built-in/Sin', [40 30 70 60], 'SineType', 'Time based', ...
    'Amplitude', 'e_g', 'Frequency', 'w_g', 'Phase', 'pi/2', 'SampleTime', '0');
add([blk '/e_g_beta'], 'built-in/Sin', [40 90 70 120], 'SineType', 'Time based', ...
    'Amplitude', 'e_g', 'Frequency', 'w_g', 'Phase', '0', 'SampleTime', '0');
add([blk '/Mux'], 'built-in/Mux', [130 0 135 80], 'Inputs', '2');
align(blk, 'Mux', 'Inport', 1, ...
    port_y(blk, 'e_g_alpha', 'Outport', 1));
add([blk '/e_g_ab'], 'built-in/Outport', [190 0 220 14]);
align(blk, 'e_g_ab', 'Inport', 1, ...
    port_y(blk, 'Mux', 'Outport', 1));
connect(blk, 'e_g_alpha/1', 'Mux/1');
route(blk, 'e_g_beta/1', 'Mux/2', 'x', 100);
connect(blk, 'Mux/1', 'e_g_ab/1');
add_mask(blk, 'ThreePhaseSource (motulator)', ...
    ['Balanced three-phase grid voltage with constant magnitude and frequency ' ...
    '(ThreePhaseSource): the output e_g_ab = e_g*exp(1j*w_g*t) (V) in stationary ' ...
    'coordinates.'], {
    'w_g', 'w_g: Angular frequency (rad/s)', 'ac_source.w_g'
    'e_g', 'e_g: Magnitude (V, peak, line-neutral)', 'ac_source.e_g'
    }, '', icon(blk, 'ThreePhaseSource'));
end
