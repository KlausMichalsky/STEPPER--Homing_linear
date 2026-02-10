# STEPPER--Homing_linear
### Rutina de homing para un "55mm Linear Mini Stepper" usando AccelStepper y un "hall effect sensor KY-035" (Base para rotot SCARA)
####  ALGORITMO:
  - Limita la búsqueda a ±10mm mecánicos durante el homing
  - Detecta flancos de entrada (movimiento hacia arriba) y salida (movimiento hacia abajo) del imán
  - Define ese flanco como posición 0 (referencia absoluta)
  - Usa velocidades rápidas y lentas para optimizar tiempo y precisión
  - Implementa un timeout y manejo de errores

####  HARDWARE:
  - MCU        : RP2040-Zero / (opcion para Arduino-Nano cambiando pins)
  - Motor      : 55mm Linear Mini Stepper (AliExpress)
  - Driver     : Step/Dir compatible con AccelStepper
  - Botón      : Inicio de homing (con debounce)
  - LED        : Estado del homing
  - Iman       : 5x5mm Iman
