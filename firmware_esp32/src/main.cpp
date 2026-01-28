#include <Arduino.h>
#include "config.h" 

#define PINO_MOTOR_IN1      M3_IN1  
#define PINO_MOTOR_IN2      M3_IN2  
#define PINO_ENABLE_PWM     M3_PWM 

// Configurações do Encoder
#define PULSOS_POR_VOLTA    1440  
#define INTERVALO_LEITURA   100   // em milissegundos

// Configurações do PWM (LEDC)
#define FREQ_PWM            5000 
#define CANAL_PWM           0    
#define RESOLUCAO_PWM       8  

// Variáveis para o Encoder (Volatile é obrigatório para interrupções)
volatile long ticks_contador = 0; 

// Variáveis para cálculo de RPM
unsigned long tempo_anterior = 0;
long ticks_anterior = 0;

bool sentido_horario = false; 

// --- FUNÇÃO DE INTERRUPÇÃO (ISR) ---
// Roda automaticamente toda vez que o encoder A mudar de LOW para HIGH
void IRAM_ATTR lerEncoder() {
  // Lê o estado do Canal B para saber a direção real
  int estado_b = digitalRead(M3_ENC_B);
  
  // Se B for diferente de A (ou HIGH na borda de subida), incrementa ou decrementa
  if (estado_b > 0) {
    ticks_contador++;
  } else {
    ticks_contador--;
  }
}

void setup() {
  Serial.begin(115200);

  // Configuração Motor
  pinMode(PINO_MOTOR_IN1, OUTPUT);
  pinMode(PINO_MOTOR_IN2, OUTPUT);
  
  // Configuração PWM
  ledcSetup(CANAL_PWM, FREQ_PWM, RESOLUCAO_PWM);
  ledcAttachPin(PINO_ENABLE_PWM, CANAL_PWM);

  // Configuração Encoder
  pinMode(M3_ENC_A, INPUT); 
  pinMode(M3_ENC_B, INPUT);

  // Ativa a interrupção no pino A (chama a função 'lerEncoder' na subida do sinal)
  attachInterrupt(digitalPinToInterrupt(M3_ENC_A), lerEncoder, RISING);
}

void loop() {
  // --- CONTROLE DO MOTOR (Mantido do seu código) ---
  if (sentido_horario) {
    digitalWrite(PINO_MOTOR_IN1, HIGH);
    digitalWrite(PINO_MOTOR_IN2, LOW);
  } else {
    digitalWrite(PINO_MOTOR_IN1, LOW);
    digitalWrite(PINO_MOTOR_IN2, HIGH);
  }

  // Velocidade fixa para teste
  ledcWrite(CANAL_PWM, 240);
  
  // --- CÁLCULO DE RPM E IMPRESSÃO (A cada 100ms) ---
  unsigned long tempo_atual = millis();
  
  if (tempo_atual - tempo_anterior >= INTERVALO_LEITURA) {
    
    // 1. Captura segura dos ticks (pausa interrupção rapidinho para ler variável volátil)
    noInterrupts();
    long ticks_agora = ticks_contador;
    interrupts();

    // 2. Calcula quantos ticks passaram desde a última leitura
    long delta_ticks = ticks_agora - ticks_anterior;

    // 3. Fórmula do RPM:
    // (Ticks_no_Intervalo / Ticks_por_Volta) * (Ms_em_um_Minuto / Intervalo_Leitura)
    float rpm = ((float)delta_ticks / PULSOS_POR_VOLTA) * (60000.0 / INTERVALO_LEITURA);

    // 4. Mostra no Serial Plotter/Monitor
    // Dica: Use o Serial Plotter do VSCode para ver o gráfico!
    Serial.print(">Ticks: ");
    Serial.print(ticks_agora);
    Serial.print(" | ");
    Serial.print(">RPM: ");
    Serial.println(rpm);

    // 5. Atualiza as variáveis de controle
    ticks_anterior = ticks_agora;
    tempo_anterior = tempo_atual;
  }
}