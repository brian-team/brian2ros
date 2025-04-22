#include <portaudio.h>
#include <iostream>
#include <cstdlib>
#include <vector>

#define CHANNELS 2       // Stéréo
#define SAMPLE_RATE 48000
#define BUFFER_SIZE 128  // Taille du buffer d'entrée
PaStream* _init_input_stream()
{
    PaStream* stream;
    PaError err;

    err = Pa_Initialize();
    if (err != paNoError)
    {
        std::cerr << "Erreur d'initialisation PortAudio : " << Pa_GetErrorText(err) << std::endl;
        exit(err);
    }

    err = Pa_OpenDefaultStream(&stream,
                               CHANNELS,     // 2 canaux d'entrée (stéréo)
                               0,            // Pas de sortie
                               paFloat32,
                               SAMPLE_RATE,
                               BUFFER_SIZE,
                               NULL,
                               NULL);
    if (err != paNoError)
    {
        std::cerr << "Erreur d'ouverture du flux : " << Pa_GetErrorText(err) << std::endl;
        exit(err);
    }

    err = Pa_StartStream(stream);
    if (err != paNoError)
    {
        std::cerr << "Erreur au démarrage du flux : " << Pa_GetErrorText(err) << std::endl;
        exit(err);
    }

    return stream;
}

float get_sample(const double t, int side)  // side = 0 (gauche), 1 (droite)
{
    static PaStream* stream = _init_input_stream();
    static std::vector<float> stereo_buffer(BUFFER_SIZE * CHANNELS);  // 2 canaux interleaved
    static int next_sample = BUFFER_SIZE;  // position courante dans le buffer

    if (next_sample >= BUFFER_SIZE)
    {
        PaError err = Pa_ReadStream(stream, stereo_buffer.data(), BUFFER_SIZE);
        if (err != paNoError)
        {
            std::cerr << "Erreur de lecture : " << Pa_GetErrorText(err) << std::endl;
            return 0.0f;
        }
        next_sample = 0;
    }

    // Chaque frame a 2 échantillons : [L, R, L, R, ...]
    float sample = stereo_buffer[next_sample * 2 + side];
    next_sample++;

    return sample;
}

