import pandas as pd
import matplotlib.pyplot as plt

# Lee los datos desde el archivo CSV
df = pd.read_csv('datos.csv')

# Visualiza las gráficas
fig, axs = plt.subplots(3, 2, figsize=(12, 8))
axs[0, 0].plot(df['Tiempo'], df['Posición X'], label='Posición X')
axs[0, 0].set_xlabel('Tiempo (s)')
axs[0, 0].set_ylabel('Posición X (m)')
axs[0, 0].legend()

axs[0, 1].plot(df['Tiempo'], df['Posición Y'], label='Posición Y')
axs[0, 1].set_xlabel('Tiempo (s)')
axs[0, 1].set_ylabel('Posición Y (m)')
axs[0, 1].legend()

axs[1, 0].plot(df['Tiempo'], df['Posición Z'], label='Posición Z')
axs[1, 0].set_xlabel('Tiempo (s)')
axs[1, 0].set_ylabel('Posición Z (m)')
axs[1, 0].legend()

axs[1, 1].plot(df['Tiempo'], df['Roll'], label='Roll')
axs[1, 1].set_xlabel('Tiempo (s)')
axs[1, 1].set_ylabel('Roll (rad)')
axs[1, 1].legend()

axs[2, 0].plot(df['Tiempo'], df['Pitch'], label='Pitch')
axs[2, 0].set_xlabel('Tiempo (s)')
axs[2, 0].set_ylabel('Pitch (rad)')
axs[2, 0].legend()

axs[2, 1].plot(df['Tiempo'], df['Yaw'], label='Yaw')
axs[2, 1].set_xlabel('Tiempo (s)')
axs[2, 1].set_ylabel('Yaw (rad)')
axs[2, 1].legend()

plt.tight_layout()
plt.show()