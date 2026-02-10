// =======================================================================
//             🔸 H O M I N G  —  L I N E A R  ( S C A R A ) 🔸
// =======================================================================
//  Archivo    : STEPPER--Homing_linear.ino
//  Autor      : Klaus Michalsky
//  Fecha      : Feb-2026
//
//  ESTADO
//  -----------------------------------------------------------------------
//  ✅ Funcional (optimizable)
// =======================================================================

#include <Arduino.h>
#include <AccelStepper.h>
#include <Bounce2.h>

//  DEFINICION DE PINES
//  -----------------------------------------------------------------------
#define MOTOR_ENABLE 9
#define MOTOR_DIR 10
#define MOTOR_STEP 11

#define HALL_PIN 29
#define LED_PIN 2
#define BOTON_PIN 28 // A0 Nano -> cambiar para el Pico

// PARAMETROS
//  -----------------------------------------------------------------------
const int MICROSTEPPING = 8;
const int REDUCCION = 1;
const int PASOS_POR_VUELTA_MOTOR = 200;

const float HOMING_VEL_RAPIDA = 4000.0;
const float HOMING_VEL_LENTA = 500.0;
const float HOMING_ACCEL = 1000.0;

const unsigned long HOMING_TIMEOUT = 12000; // 12 segundos

const long LIMITE_BUSQUEDA_CORTA = -100; // pasos máximos si arranca fuera del imán

int dir = 1; // dirección inicial

// OBJETOS
//  -----------------------------------------------------------------------
AccelStepper motor(AccelStepper::DRIVER, MOTOR_STEP, MOTOR_DIR);
Bounce debouncer;

// ESTADO DE HOMING
//  -----------------------------------------------------------------------
// A partir de ahora, existe un tipo llamado EstadoHoming
// que solo puede tomar uno de estos valores
// enum EstadoHoming { ... }	Definición de un tipo
// EstadoHoming	                El tipo de dato
// HOMING_INACTIVO, ...         Valores posibles del tipo EstadoHoming
// estadoHoming	                Variable de ese tipo
enum EstadoHoming
{
    HOMING_INACTIVO,
    HOMING_BUSCAR_FLANCO_DE_SALIDA,
    HOMING_BUSCAR_FLANCO_DE_ENTRADA,
    HOMING_MOVER_HASTA_REFERENCIA,
    HOMING_OK,
    HOMING_ERROR
};

// CONTROL Y VARIABLES DE HOMING
//  -----------------------------------------------------------------------
EstadoHoming estadoHoming = HOMING_INACTIVO; // estado inicial
unsigned long homingStartTime = 0;           // tiempo de inicio de homing

long referencia = 0;       // posición de homing
long flanco = 0;           // posición de salida del imán
long posicionDeInicio = 0; // posición al iniciar homing

bool homingFallo = false; // marca si hubo falla en homing

// =======================================================================
// SETUP
// =======================================================================
void setup()
{
    Serial.begin(115200);

    // setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert);
    // true significa que la señal se invierte lógicamente
    motor.setPinsInverted(false, false, false);

    pinMode(HALL_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    pinMode(MOTOR_ENABLE, OUTPUT);
    pinMode(BOTON_PIN, INPUT_PULLUP);

    digitalWrite(MOTOR_ENABLE, HIGH);
    digitalWrite(LED_PIN, LOW);

    debouncer.attach(BOTON_PIN);
    debouncer.interval(25);

    motor.setMaxSpeed(HOMING_VEL_RAPIDA);
    motor.setAcceleration(HOMING_ACCEL);

    Serial.println("Sistema listo. Presiona el botón para homing.");
}

// =======================================================================
// LOOP
// =======================================================================
void loop()
{
    debouncer.update();

    // 🔄 Reset de homingFallo al soltar el botón
    // fell() → el botón pasó de HIGH → LOW
    // rose() → el botón pasó de LOW → HIGH
    // ⚠️ Importante: por qué no resetear en fell()
    // Si lo hicieras al presionar:
    // podrías borrar el error sin intención
    // incluso mientras el botón sigue apretado
    // creando estados raros
    // 👉 Resetear al soltar es lo correcto.
    if (debouncer.rose() && homingFallo)
    {
        homingFallo = false;
        Serial.println("🔄 Homing reset");
    }
    // 🔒 si hay error latched, no hacemos nada
    if (homingFallo)
    {
        return;
    }
    // 🏠 Inicia homing al apretar el botón
    if (!homingFallo && debouncer.fell() && estadoHoming == HOMING_INACTIVO)
    {
        digitalWrite(MOTOR_ENABLE, LOW);
        digitalWrite(LED_PIN, LOW);
        Serial.println("🔹 Iniciando homing...");
        motor.setCurrentPosition(0);
        posicionDeInicio = motor.currentPosition();
        homingStartTime = millis(); // Guarda tiempo de inicio de homing al momento de presionar el botón
        bool imanPresente = !digitalRead(HALL_PIN);
        if (!imanPresente)
        {
            Serial.println("⚠️ Arranque fuera del imán (búsqueda limitada)");
        }
        estadoHoming = HOMING_BUSCAR_FLANCO_DE_SALIDA;
    }
    if (estadoHoming != HOMING_INACTIVO)
    {
        homingStep();
    }
}

// =======================================================================
// MAQUINA DE ESTADOS
// =======================================================================
void homingStep()
{
    // Invierte la logica del HAll (imán presente = LOW)
    bool imanPresente = !digitalRead(HALL_PIN);

    // ⏱️ Timeout de homing (si tiempo de homing excede el límite)
    if (millis() - homingStartTime > HOMING_TIMEOUT) //
    {
        estadoHoming = HOMING_ERROR;
    }

    switch (estadoHoming)
    {
    case HOMING_BUSCAR_FLANCO_DE_SALIDA:

        motor.setSpeed(HOMING_VEL_RAPIDA);
        motor.runSpeed();
        if (!imanPresente)
        {
            flanco = motor.currentPosition();
            Serial.print("Flanco de salida en: ");
            Serial.println(flanco);
            motor.moveTo(flanco + 500); // avanza 500 pasos para alejarse un poquito del imán
            estadoHoming = HOMING_BUSCAR_FLANCO_DE_ENTRADA;
        }
        break;

    case HOMING_BUSCAR_FLANCO_DE_ENTRADA:
        motor.setSpeed(-HOMING_VEL_LENTA);
        motor.runSpeed();
        if (imanPresente)
        {
            flanco = motor.currentPosition();
            Serial.print("Flanco de entrada en: ");
            Serial.println(flanco);
            estadoHoming = HOMING_MOVER_HASTA_REFERENCIA;
        }
        // Probar esto:
        else if (motor.currentPosition() <= LIMITE_BUSQUEDA_CORTA)
        {
            Serial.println("⚠️ Homing error: límite de búsqueda corta alcanzado");
            estadoHoming = HOMING_ERROR;
        }
        break;

    case HOMING_MOVER_HASTA_REFERENCIA:
        motor.setSpeed(-HOMING_VEL_LENTA);
        motor.runSpeed();

        if (motor.currentPosition() <= flanco - 500)
        {
            motor.stop();
            motor.setCurrentPosition(0); // ESTE es el cero (referencia)
            estadoHoming = HOMING_OK;
            digitalWrite(MOTOR_ENABLE, HIGH);
        }
        break;

    case HOMING_OK:
        Serial.println("✅ Homing OK. Referencia = 0");
        digitalWrite(LED_PIN, HIGH);
        estadoHoming = HOMING_INACTIVO;
        digitalWrite(MOTOR_ENABLE, HIGH);
        break;

    case HOMING_ERROR:
        Serial.println("❌ ERROR de homing");
        digitalWrite(MOTOR_ENABLE, HIGH);
        homingFallo = true;               // marca la falla
        estadoHoming = HOMING_INACTIVO;   // vuelve a IDLE
        break;
      
    default:
        break;
    }
}
