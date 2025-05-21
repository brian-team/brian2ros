import sounddevice as sd
import numpy as np
import matplotlib.pyplot as plt
from scipy.io.wavfile import write

# --- Paramètres ---
duration = 5  # Durée de l'enregistrement (en secondes)
samplerate = 48000  # Taux d'échantillonnage (en Hz)

for i, device in enumerate(sd.query_devices()):
    print(f"{i}: {device['name']} (Entrées: {device['max_input_channels']}, Sorties: {device['max_output_channels']})")

# --- Enregistrement ---
print("🎙️ Enregistrement en cours...")
audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=2, dtype='float32', device=6)
sd.wait()  # Attendre que l'enregistrement soit terminé
print("✅ Enregistrement terminé.")

# --- Sauvegarde dans un fichier WAV ---
filename = "test_micro.wav"
write(filename, samplerate, audio)
print(f"💾 Audio sauvegardé sous : {filename}")

# --- Affichage du signal ---
audio_flat = audio[:,0]  # Convertir en 1D si nécessaire
time_axis = np.linspace(0, duration, len(audio_flat))

plt.figure(figsize=(10, 4))
plt.plot(time_axis, audio_flat, color='mediumblue')
plt.title("Signal audio enregistré")
plt.xlabel("Temps (s)")
plt.ylabel("Amplitude")
plt.grid(True)
plt.tight_layout()
plt.show()
