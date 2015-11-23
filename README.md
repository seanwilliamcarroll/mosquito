# mosquito
Gates Foundation Project to synthesize female mosquito wing tones so as to attract males for capture

This project makes use of the PIC32MX250F128B microcontroller. 

Basic functionality:
Uses Amplitude Modulation to dynamically change the amplitude of the synthesized wing tones. Wing tones are comprised of up to 6 harmonics of 464 Hz, with varying amplitudes. Wing tones are generated as a continuous combination of up to 6 sine waves at the desired frequencies and phases. Amplitude modulation is done dynamically in the ISR, with the actual amplitude being calculated in a thread. Amplitude is calculated as a bounded random walk between 0 and 0.8 Hz, with steps of size 0.05. This thread is yielded for 100 ms at the end of each amplitude change, giving a change of 10 times a second. This gives a smooth, random envelope to the output signal. 

A serial interface can be used to interface directly with the PIC32 to set different parameters, such as random walk bounds and step, number of harmonics of 464 Hz to play, the phase and amplitude weighting of each individual sine wave, etc. through a simple command line interface. 


Added User interface functionality through serial interface. 
