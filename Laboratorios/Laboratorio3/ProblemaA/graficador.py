import serial
import time
import csv
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

PUERTO_COM = 'COM8'
BAUD_RATE  = 9600
MAX_PUNTOS = 50

tiempos     = deque(maxlen=MAX_PUNTOS)
temperaturas= deque(maxlen=MAX_PUNTOS)
setpoints   = deque(maxlen=MAX_PUNTOS)
calefactor  = deque(maxlen=MAX_PUNTOS)
ventilador  = deque(maxlen=MAX_PUNTOS)

archivo_csv = "registro_temperatura.csv"
with open(archivo_csv, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["Timestamp", "Tiempo_seg", "Temperatura_C", "PuntoMedio_C", "Calefactor_ON", "PWM_Ventilador"])

try:
    ser = serial.Serial(PUERTO_COM, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"Conectado exitosamente a  {PUERTO_COM}")
    print(f"Registrando datos en '{archivo_csv}' ")
except Exception as e:
    print(f"Error al abrir {PUERTO_COM}: {e}")
    exit()

tiempo_inicial = time.time()

fig, (ax_temp, ax_act) = plt.subplots(2, 1, sharex=True, figsize=(10, 7))
fig.canvas.manager.set_window_title("¿Ahora sí está todo, no?")

def actualizar_grafica(frame):
    while ser.in_waiting:
        linea = ser.readline().decode('utf-8', errors='ignore').strip()
        
        if linea.startswith("DATA:"):
            try:
                datos = linea.replace("DATA:", "").split(",")
                if len(datos) == 4:
                    t_medida = float(datos[0])
                    sp_actual= float(datos[1])
                    cal_on   = int(datos[2])
                    fan_pwm  = int(datos[3])

                    t_segundos = round(time.time() - tiempo_inicial, 1)
                    tiempos.append(t_segundos)
                    temperaturas.append(t_medida)
                    setpoints.append(sp_actual)
                    calefactor.append(cal_on * 100)
                    ventilador.append((fan_pwm / 255.0) * 100)

                    with open(archivo_csv, mode='a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow([time.strftime("%H:%M:%S"), t_segundos, t_medida, sp_actual, cal_on, fan_pwm])

            except ValueError:
                pass

    if len(tiempos) > 0:
        ax_temp.clear()
    
        sp_ref = setpoints[-1]
        ax_temp.axhspan(sp_ref - 4.5, sp_ref + 4.5, color='green', alpha=0.15, label='Rango Ideal Confort')
        
        ax_temp.plot(list(tiempos), list(temperaturas), 'b-o', linewidth=2, markersize=5, label=f'Temp Actual: {temperaturas[-1]} °C')
        ax_temp.plot(list(tiempos), list(setpoints), 'g--', linewidth=1.5, label=f'Punto Medio (SP): {sp_ref} °C')

        ax_temp.set_title("Evolución de la Temperatura vs Rango Deseado", fontsize=12, fontweight='bold')
        ax_temp.set_ylabel("Temperatura (°C)", fontsize=10)
        ax_temp.legend(loc="upper left")
        ax_temp.grid(True, linestyle="--", alpha=0.6)

        ax_act.clear()
        ax_act.step(list(tiempos), list(calefactor), 'r', where='post', linewidth=2, label=f'Calefactor: {"ON" if calefactor[-1] > 0 else "OFF"}')
        ax_act.plot(list(tiempos), list(ventilador), 'c-s', linewidth=2, markersize=4, label=f'Ventilador: {ventilador[-1]:.0f}%')

        ax_act.set_title("Acciones del Sistema de Control", fontsize=12, fontweight='bold')
        ax_act.set_xlabel("Tiempo transcurrido (segundos)", fontsize=10)
        ax_act.set_ylabel("Potencia Actuador (%)", fontsize=10)
        ax_act.set_ylim(-10, 115)
        ax_act.legend(loc="upper left")
        ax_act.grid(True, linestyle="--", alpha=0.6)
ani = animation.FuncAnimation(fig, actualizar_grafica, interval=1000, cache_frame_data=False)

plt.tight_layout()
plt.show()

ser.close()
print("Conexión finalizada. Registro guardado con éxito.")
