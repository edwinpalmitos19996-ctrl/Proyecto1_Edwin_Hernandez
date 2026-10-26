#include <Arduino.h>
#include <xtensa/core-macros.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// Mapeo de pines
const int PIN_PWM_A = 12; // Canal LEDC para motor (L9110S)
const int PIN_PWM_B = 14; // Canal LEDC para motor (L9110S)
const int PIN_EMERGENCY_BTN = 33; // Microinterruptor de paro de emergencia

//motor 

// motor config PWM
const uint32_t PWM_FREQ = 1000;       // 1000 Hz
const uint8_t  PWM_RESOLUTION = 10;   // 10 bits (0 - 1023)
const uint8_t  LEDC_CHANNEL_A = 0;
const uint8_t  LEDC_CHANNEL_B = 1;
// init motor

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile bool g_btnPressedFlag = false;
volatile uint32_t g_pressCounter = 0;
volatile uint32_t g_lastIsrCycle = 0;

// Umbral de filtrado: 4,000,000 ciclos a 160 MHz equivalen a 25 ms
const uint32_t DEBOUNCE_CYCLES = 4000000;

// ==========================================
// DEFINICIÓN DE FSM (ESTADOS Y EVENTOS)
// ==========================================
enum class State {
    IDLE,
    RUNNING,
    FAULT
};

enum class Event {
    NONE,
    BTN_PRESS,
    TIMEOUT
};

// Variables de estado del sistema
State currentState = State::IDLE;

// ==========================================
// FASE 2: CAPA DE ABSTRACCIÓN DE HARDWARE (HAL)
// ==========================================
void initHardware() {
    // Configuración de canales PWM LEDC
    ledcSetup(LEDC_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(LEDC_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
    
    ledcAttachPin(PIN_PWM_A, LEDC_CHANNEL_A);
    ledcAttachPin(PIN_PWM_B, LEDC_CHANNEL_B);

    // Configuración del botón de emergencia con resistencia pull-up interna
    pinMode(PIN_EMERGENCY_BTN, INPUT_PULLUP);
}

// Funciones de envoltura (wrapper functions) seguras para el motor
void motorForward(uint32_t dutyCycle) {
    ledcWrite(LEDC_CHANNEL_A, dutyCycle);
    ledcWrite(LEDC_CHANNEL_B, 0);
}

void motorDynamicBraking() {
    ledcWrite(LEDC_CHANNEL_A, 1023);
    ledcWrite(LEDC_CHANNEL_B, 1023);
}

void motorCoast() {
    ledcWrite(LEDC_CHANNEL_A, 0);
    ledcWrite(LEDC_CHANNEL_B, 0);
}

// ==========================================
// FASE 3: ISR Y DEPURACIÓN POR CICLOS (DEBOUNCING)
// ==========================================
void IRAM_ATTR emergencyISR() {
    uint32_t currentCycle = XTHAL_GET_CCOUNT();
    
    // Aritmética modular segura de 32 bits para el desbordamiento (rollover) de CCOUNT
    if ((currentCycle - g_lastIsrCycle) > DEBOUNCE_CYCLES) {
        portENTER_CRITICAL_ISR(&mux);
        g_lastIsrCycle = currentCycle;
        g_btnPressedFlag = true;
        g_pressCounter++;
        portEXIT_CRITICAL_ISR(&mux);
    }
}

// ==========================================
// FASE 4: HANDLERS DE LA FSM ORIENTADA A EVENTOS
// ==========================================
void handleIdleEvent(Event ev) {
    if (ev == Event::BTN_PRESS) {
        motorForward(768); // ~75% de velocidad PWM
        currentState = State::RUNNING;
    }
}

void handleRunningEvent(Event ev) {
    if (ev == Event::BTN_PRESS) {
        motorDynamicBraking();
        currentState = State::FAULT;
    }
}

void handleFaultEvent(Event ev) {
    if (ev == Event::BTN_PRESS) {
        motorCoast();
        currentState = State::IDLE;
    }
}

// Tipo de definición para los punteros a funciones de la matriz FSM
typedef void (*StateHandler_t)(Event);

// Matriz bidimensional constexpr de punteros a funciones O(1) [Estado x Evento]
// Filas: Estados (IDLE, RUNNING, FAULT)
// Columnas: Eventos (NONE, BTN_PRESS, TIMEOUT)
constexpr StateHandler_t fsmMatrix[3][3] = {
    // NONE                   BTN_PRESS           TIMEOUT
    {  [](Event e){},         handleIdleEvent,    [](Event e){} },   // IDLE
    {  [](Event e){},         handleRunningEvent, [](Event e){} },   // RUNNING
    {  [](Event e){},         handleFaultEvent,   [](Event e){} }    // FAULT
};

// ==========================================
// SETUP Y BUCLE PRINCIPAL
// ==========================================
void setup() {
    Serial.begin(115200);
    initHardware();

    // Asignación de interrupción de hardware al flanco de bajada (FALLING)
    attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCY_BTN), emergencyISR, FALLING);
}

void loop() {
    Event currentEvent = Event::NONE;

    // Lectura segura de banderas volatile protegidas por Spinlock
    portENTER_CRITICAL(&mux);
    if (g_btnPressedFlag) {
        g_btnPressedFlag = false;
        currentEvent = Event::BTN_PRESS;
    }
    portEXIT_CRITICAL(&mux);

    // Despacho del evento a través de la matriz determinista O(1) sin bloquear el CPU
    if (currentEvent != Event::NONE) {
        int stateIndex = static_cast<int>(currentState);
        int eventIndex = static_cast<int>(currentEvent);
        
        fsmMatrix[stateIndex][eventIndex](currentEvent);
    }
}