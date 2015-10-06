# mosquito
Gates Foundation Project to synthesize female mosquito wing tones so as to attract males for capture



Two Modes: Text Mode and Graphics Mode

-- set MODE to 0 for Text display, shows system time and current modulation frequency

-- set MODE to 1 for Graphics display, shows a graphical representation of modulation envelope in real time


Basic functionality:
Uses Amplitude Modulation to dynamically change the amplitude of the synthesized wing tones. Wing tones are comprised of 4 harmonics of 464 Hz, with varying amplitudes. Wing tones are generated as a continuous combination of 4 sine waves at the desired frequencies and phases. Amplitude modulation is done dynamically in the ISR, with the actual
amplitude being calculated in a thread. Amplitude is calculated as a bounded random walk between 0 and 0.8 Hz, with steps of size 0.05. This thread is yielded for 100 ms at the end of each amplitude change, giving a change of 10 times a second. This gives a smooth, random envelope to the output signal. 
