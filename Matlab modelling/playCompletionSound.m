function playCompletionSound()
    % Define the Final Fantasy victory fanfare melody
    % The melody below approximates the notes of the Final Fantasy victory theme
    melodyFF = [659, 659, 659, 659, 523, 587, 659, 587, 659];  % E4, E4, E4, E4, C4, D4, E4, D4, E4
    durationsFF = [0.20, 0.20, 0.20, 0.60, 0.60, 0.60, 0.4, 0.2, 1];  % Corresponding durations in seconds
    
    % Choose a melody (in this case, the FF fanfare)
    selectedMelody = melodyFF;
    selectedDurations = durationsFF;
    
    % Define sampling frequency
    fs = 44100;  % Standard CD-quality sampling rate
    
    % Generate the sound waveform for the selected melody
    soundData = [];
    for i = 1:length(selectedMelody)
        t = linspace(0, selectedDurations(i), round(fs * selectedDurations(i)));
        note = sin(2 * pi * selectedMelody(i) * t);
        soundData = [soundData, note]; %#ok<AGROW>
    end
    
    % Normalize the sound data
    soundData = 0.5 * soundData / max(abs(soundData));  % Scale to medium volume
    
    % Play the sound and wait until playback is complete
    soundsc(soundData, fs);
end
