function varargout = fp_checkpoint(action, run_dir, varargin)
% FP_CHECKPOINT
% Crash-safe checkpoint save and resume for long pendulum simulations.
%
% The simulation engine calls fp_checkpoint('save', ...) every N steps.
% On startup, fp_checkpoint('check', run_dir) looks for an incomplete run
% and returns the saved state so the simulation can resume mid-trajectory.
%
% Usage:
%   % Check for existing checkpoint before starting:
%   [found, state] = fp_checkpoint('check', run_dir);
%   if found
%       fprintf('Resuming from t=%.2f s\n', state.t_resume);
%   end
%
%   % Save checkpoint inside simulation loop (every N steps):
%   fp_checkpoint('save', run_dir, state);
%
%   % Clear checkpoint after successful completion:
%   fp_checkpoint('clear', run_dir);
%
% Checkpoint file: run_dir/checkpoint.mat
% Contains: current state vector, time, simulation config, partial results

chk_file = fullfile(run_dir, 'checkpoint.mat');

switch lower(action)

    % ---- CHECK: look for an existing checkpoint -------------------------
    case 'check'
        if ~isfile(chk_file)
            varargout{1} = false;
            varargout{2} = struct();
            return;
        end
        try
            chk = load(chk_file, 'checkpoint');
            c   = chk.checkpoint;
            % Validate: must have required fields
            required = {'t_resume','x_resume','k_resume','cfg','run_dir'};
            ok = all(cellfun(@(f) isfield(c,f), required));
            if ~ok
                warning('fp_checkpoint: checkpoint file corrupt, ignoring.');
                varargout{1} = false;
                varargout{2} = struct();
                return;
            end
            fprintf('[fp_checkpoint] Found checkpoint: t=%.3f s, step=%d\n', ...
                    c.t_resume, c.k_resume);
            varargout{1} = true;
            varargout{2} = c;
        catch ME
            warning('fp_checkpoint: could not load checkpoint: %s', ME.message);
            varargout{1} = false;
            varargout{2} = struct();
        end

    % ---- SAVE: write current simulation state ---------------------------
    case 'save'
        if isempty(varargin)
            error('fp_checkpoint save: must pass state struct');
        end
        state = varargin{1};
        state.saved_at  = char(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
        state.run_dir   = run_dir;
        checkpoint      = state;  %#ok<NASGU>
        save(chk_file, 'checkpoint', '-v7.3');

    % ---- CLEAR: remove checkpoint after successful completion -----------
    case 'clear'
        if isfile(chk_file)
            delete(chk_file);
            fprintf('[fp_checkpoint] Checkpoint cleared (run complete).\n');
        end

    otherwise
        error('fp_checkpoint: unknown action "%s"', action);
end

end
