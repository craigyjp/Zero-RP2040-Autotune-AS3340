//
// Autotune
//            // Running total of the readings

#define A_NOTES_COUNT 8  // A0 (27.5 Hz), A1 (55 Hz), A2 (110 Hz), A3 (220 Hz), A4 (440 Hz), A5 (880 Hz), A6 (1760 Hz), A7 (3520 Hz)
const int A_NOTES[A_NOTES_COUNT] = {33, 45, 57, 69, 81, 93, 105, 117 };  // MIDI note numbers for A0 to A7

float midi_to_freqs[128][2] = {
  { 0, 4.09 },
  { 1, 4.33 },
  { 2, 4.59 },
  { 3, 4.86 },
  { 4, 5.15 },
  { 5, 5.45 },
  { 6, 5.78 },
  { 7, 6.12 },
  { 8, 6.49 },
  { 9, 6.87 },
  { 10, 7.28 },
  { 11, 7.71 },
  { 12, 8.17 },
  { 13, 8.66 },
  { 14, 9.17 },
  { 15, 9.72 },
  { 16, 10.30 },
  { 17, 10.91 },
  { 18, 11.56 },
  { 19, 12.25 },
  { 20, 12.98 },
  { 21, 13.75 },
  { 22, 14.57 },
  { 23, 15.43 },
  { 24, 16.35 },
  { 25, 17.32 },
  { 26, 18.35 },
  { 27, 19.45 },
  { 28, 20.60 },
  { 29, 21.83 },
  { 30, 23.12 },
  { 31, 24.50 },
  { 32, 25.96 },
  { 33, 27.50 },
  { 34, 29.14 },
  { 35, 30.87 },
  { 36, 32.7 },
  { 37, 34.65 },
  { 38, 36.71 },
  { 39, 38.89 },
  { 40, 41.20 },
  { 41, 43.65 },
  { 42, 46.25 },
  { 43, 49.00 },
  { 44, 51.91 },
  { 45, 55.00 },
  { 46, 58.27 },
  { 47, 61.74 },
  { 48, 65.41 },
  { 49, 69.3 },
  { 50, 73.42 },
  { 51, 77.78 },
  { 52, 82.41 },
  { 53, 87.31 },
  { 54, 92.50 },
  { 55, 98.00 },
  { 56, 103.83 },
  { 57, 110.00 },
  { 58, 116.54 },
  { 59, 123.47 },
  { 60, 130.81 },
  { 61, 138.59 },
  { 62, 146.83 },
  { 63, 155.56 },
  { 64, 164.81 },
  { 65, 174.61 },
  { 66, 185.00 },
  { 67, 196.00 },
  { 68, 207.65 },
  { 69, 220.00 },
  { 70, 233.08 },
  { 71, 246.94 },
  { 72, 261.63 },
  { 73, 277.18 },
  { 74, 293.66 },
  { 75, 311.13 },
  { 76, 329.63 },
  { 77, 349.23 },
  { 78, 369.99 },
  { 79, 392.00 },
  { 80, 415.30 },
  { 81, 440.00 },
  { 82, 466.16 },
  { 83, 493.88 },
  { 84, 523.25 },
  { 85, 554.37 },
  { 86, 587.33 },
  { 87, 622.25 },
  { 88, 659.25 },
  { 89, 698.46 },
  { 90, 739.99 },
  { 91, 783.99 },
  { 92, 830.61 },
  { 93, 880.00 },
  { 94, 932.33 },
  { 95, 987.77 },
  { 96, 1046.5 },
  { 97, 1108.73 },
  { 98, 1174.66 },
  { 99, 1244.51 },
  { 100, 1318.51 },
  { 101, 1396.91 },
  { 102, 1479.98 },
  { 103, 1567.98 },
  { 104, 1661.22 },
  { 105, 1760.00 },
  { 106, 1864.66 },
  { 107, 1975.53 },
  { 108, 2093.00 },
  { 109, 2217.46 },
  { 110, 2349.32 },
  { 111, 2489.02 },
  { 112, 2637.02 },
  { 113, 2793.83 },
  { 114, 2959.96 },
  { 115, 3135.96 },
  { 116, 3322.44 },
  { 117, 3520.00 },
  { 118, 3729.31 },
  { 119, 3951.07 },
  { 120, 4186.01 },
  { 121, 4434.92 },
  { 122, 4698.63 },
  { 123, 4978.03 },
  { 124, 5274.04 },
  { 125, 5587.65 },
  { 126, 5919.91 },
  { 127, 6271.93 },
};

// Glide
bool portamentoOn = false;        // Portamento state (on/off)
uint16_t portamentoTime = 0;      // Portamento time in milliseconds (0–10000 ms)
bool portamentoActive = false;  // Tracks if portamento is currently running
unsigned long glideStartTime = 0;  // Start time for portamento
float currentMV_a = 0.0;           // Current mV value for channel_a
float targetMV_a = 0.0;            // Target mV value for channel_a
float currentMV_b = 0.0;           // Current mV value for channel_b
float targetMV_b = 0.0;            // Target mV value for channel_b
float initialMV_a = 0.0;           // Initial mV value for channel_a
float initialMV_b = 0.0;           // Initial mV value for channel_b
float step_a = 0.0;       // Step size for channel_a
float step_b = 0.0;       // Step size for channel_b

// Note input
float AD_CH1, old_AD_CH1, AD_CH2, old_AD_CH2;
float AD_CH1_calb = 1.0;//reduce resistance error
float AD_CH2_calb = 1.0;//reduce resistance error
int sens1, sens2, oct1, oct2; //sens = AD input attn,amp.oct=octave shift
static int previousMidiNote = -1;  // Stores the last played note
static int old_ADC = 0;
int search_qnt;
int cmp1, cmp2;
uint16_t cv_qnt_thr_buf1[61] = {
    0, 68, 136, 204, 272, 340, 408, 476, 544, 612, 680, 748, 816,
    884, 952, 1020, 1088, 1156, 1224, 1292, 1360, 1428, 1496, 1564, 1632,
    1700, 1768, 1836, 1904, 1972, 2040, 2108, 2176, 2244, 2312, 2380,
    2448, 2516, 2584, 2652, 2720, 2788, 2856, 2924, 2992, 3060, 3128,
    3196, 3264, 3332, 3400, 3468, 3536, 3604, 3672, 3740, 3808, 3876,
    3944, 4012, 4080
};

#define NUM_SAMPLES 5  // Number of samples for smoothing
int adcBuffer[NUM_SAMPLES] = {0};  
int bufferIndex = 0;
#define ADC_NOISE_THRESHOLD 10  // Ignore ADC values below this
#define HYSTERESIS_THRESHOLD 5  // Minimum change to trigger new note
#define ADC_OFFSET 29   // Measured ADC offset at 0V

// Autotune
int16_t autotune_value[128][2];
int midiNote;
volatile long count = 0; // Declare the global variable as volatile
float frequency;        //storing frequency
boolean autotuneStart = false;
double measuredFrequency;
double measuredFreq;
int workingNote = 0;
const int numOscillators = 1;   // Number of oscillators - 1 as they start at 0
const int numNotes = 128;       // Number of MIDI notes
float targetFrequency = 0.00;
int tuneNote = 0;
byte note = 0;

int OCTAVE_A = 0;
int OCTAVE_B = 0;
int VOLTOFFSET = 3270;
int oscillator;

//
// Modulation
//

float FM_VALUE = 0.0f;
float FM_AT_VALUE = 0.0f;
float TM_VALUE = 0.0f;
float TM_AT_VALUE = 0.0f;
float MOD_VALUE = 0.0f;

int FM_RANGE_UPPER = 0;
int FM_RANGE_LOWER = 0;
int FM_AT_RANGE_UPPER = 0;
int FM_AT_RANGE_LOWER = 0;

int TM_RANGE_UPPER = 0;
int TM_RANGE_LOWER = 0;
int TM_AT_RANGE_UPPER = 0;
int TM_AT_RANGE_LOWER = 0;

float FM_MOD_WHEEL = 0.00f;
float FM_AT_WHEEL = 0.00f;
float TM_AT_WHEEL = 0.00f;
float TM_MOD_WHEEL = 0.00f;

int BEND_WHEEL = 0;
int TM_RANGE = 0;
int DETUNE = 0;
int INTERVAL_POT = 0;
int INTERVAL = 0;

int PW1 = 0;
int PW2 = 0;
int PWM1 = 0;
int PWM2 = 0;
float PWM_VALUE = 0.00f;
float PWM1_VALUE = 0.00f;
float PWM2_VALUE = 0.00f;

int masterChan;
int masterTran;
int previousMode;
int transpose = 0;
int8_t i;
int noteMsg;
int keyboardMode = 2;
int octave = 0;
int realoctave = -36;
int bend_data;
int note1;
int oscanote1;
int oscbnote1;
bool reset = false;
bool displayvalues = false;
bool osc1Through = false;
byte sendnote, sendvelocity;

unsigned int velmV;
unsigned int mV;
unsigned int additionalmV;
unsigned int finalmV;

float noteTrig[1];

uint32_t int_ref_on_flexible_mode = 0b00001001000010100000000000000000;  // { 0000 , 1001 , 0000 , 1010000000000000 , 0000 }

uint32_t sample_data = 0b00000000000000000000000000000000;

uint32_t sample_data1 = 0b00000000000000000000000000000000;

uint32_t vel_data1 = 0b00000000000000000000000000000000;

uint32_t channel_a = 0b00000010000000000000000000000000;
uint32_t channel_b = 0b00000010000100000000000000000000;
uint32_t channel_c = 0b00000010001000000000000000000000;
uint32_t channel_d = 0b00000010001100000000000000000000;
uint32_t channel_e = 0b00000010010000000000000000000000;
uint32_t channel_f = 0b00000010010100000000000000000000;
uint32_t channel_g = 0b00000010011000000000000000000000;
uint32_t channel_h = 0b00000010011100000000000000000000;
