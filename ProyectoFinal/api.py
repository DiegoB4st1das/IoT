from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import paho.mqtt.publish as publish

app = FastAPI()

# Evita bloqueos de seguridad (CORS) para que el HTML hable con Python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], 
    allow_methods=["*"],
    allow_headers=["*"],
)

# 1. Ruta para actualizar el Setpoint (RPM)
@app.post("/enviar_setpoint/{rpm}")
def update_setpoint(rpm: float):
    try:
        publish.single("iot/motor/setpoint", payload=str(rpm), hostname="localhost", port=1883)
        return {"status": "ok", "mensaje": f"Setpoint de {rpm} RPM enviado con éxito"}
    except Exception as e:
        return {"status": "error", "mensaje": str(e)}

# 2. Ruta para cambiar el sentido de giro
@app.post("/cambiar_direccion")
def toggle_direction():
    try:
        publish.single("iot/motor/direccion", payload="TOGGLE", hostname="localhost", port=1883)
        return {"status": "ok", "mensaje": "Comando de cambio de giro enviado con éxito"}
    except Exception as e:
        return {"status": "error", "mensaje": str(e)}
