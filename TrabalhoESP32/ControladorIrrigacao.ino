/**
 * @file    ControladorIrrigacao.ino
 * @brief   Controlador de irrigação por temperatura para ESP32.
 * @version 1.0
 * @date    2026-11-05
 *
 * @details
 * Lê um sensor Grove ASAIR AM2302 (equivalente ao DHT22, temperatura e
 * umidade relativa do ar) e controla um relé que aciona o irrigador.
 *
 * Regra de operação:
 *  1. Se a temperatura ultrapassar TEMP_LIMITE_C, o irrigador é ligado
 *     por TEMPO_IRRIGACAO_MS (2 min) e depois desligado automaticamente.
 *  2. Após o desligamento, inicia-se um período de espera (TEMPO_ESPERA_MS,
 *     5 min) durante o qual o irrigador NÃO pode religar.
 *  3. Terminada a espera, se a temperatura ainda estiver acima do limite,
 *     um novo ciclo é iniciado. Caso contrário, o irrigador permanece
 *     desligado até que a temperatura volte a ultrapassar o limite.
 *
 * A umidade do ar é lida e exibida no monitor serial para supervisão;
 * não participa da decisão de acionamento.
 *
 * Conectividade (v1.0):
 *  - Wi-Fi com duas redes cadastradas (WLL-Inatel e Jupter); o ESP32 conecta
 *    na de melhor sinal disponível e reconecta sozinho se cair.
 *  - Telemetria via MQTT (porta 1883) para o ThingsBoard, tópico
 *    v1/devices/me/telemetry, autenticando com o Access Token do dispositivo.
 *    Campos enviados: temperatura (°C), umidade (%), irrigador (true/false) e
 *    localização (latitude/longitude, definidas manualmente no código).
 *  - A telemetria é enviada a cada INTERVALO_TELEMETRIA_MS e também
 *    imediatamente quando o irrigador liga ou desliga.
 *  - A rede NUNCA bloqueia o controle: sem Wi-Fi/MQTT o irrigador continua
 *    funcionando normalmente, e a telemetria é retomada quando a rede volta.
 *
 * Segurança (fail-safe): em falha de leitura do sensor a saída é desligada
 * e nenhum novo ciclo é iniciado.
 *
 * Toda a temporização usa millis() (sem delay()), portanto o laço principal
 * nunca bloqueia. millis() é seguro contra overflow com subtração unsigned.
 *
 * Bibliotecas (Gerenciador de Bibliotecas da Arduino IDE):
 *  - DHT sensor library       (Adafruit)
 *  - Adafruit Unified Sensor  (Adafruit, dependência da anterior)
 *  - PubSubClient             (Nick O'Leary)
 *
 * Placa: "ESP32 Dev Module" (pacote esp32 da Espressif).
 * Serial: 115200 baud.
 */

#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include <DHT.h>

/* ========================================================================== */
/*  CONFIGURAÇÃO DO USUÁRIO                                                   */
/* ========================================================================== */

// ---- Pinagem (GPIO do ESP32) ----
constexpr uint8_t PINO_SENSOR = 4;    ///< Sinal do AM2302 (fio amarelo do cabo Grove)
constexpr uint8_t PINO_RELE   = 26;   ///< Saída para o módulo relé do irrigador

// ---- Sensor ----
#define TIPO_SENSOR DHT22             ///< AM2302 é a versão cabeada do DHT22

// ---- Lógica do relé ----
constexpr bool RELE_ATIVO_EM_NIVEL_ALTO = true;  ///< false para módulos relé ativos em LOW

// ---- Parâmetros de controle ----
constexpr float    TEMP_LIMITE_C      = 30.0f;                 ///< Limite de temperatura (°C)
constexpr uint32_t TEMPO_IRRIGACAO_MS = 2UL * 60UL * 1000UL;   ///< Duração da irrigação: 2 min
constexpr uint32_t TEMPO_ESPERA_MS    = 5UL * 60UL * 1000UL;   ///< Espera mínima entre ciclos: 5 min

// ---- Amostragem ----
// O DHT22 não deve ser lido mais de uma vez a cada ~2 s.
constexpr uint32_t INTERVALO_LEITURA_MS = 2000;  ///< Período entre leituras do sensor
constexpr uint32_t INTERVALO_LOG_MS     = 5000;  ///< Período entre mensagens de status

// ---- Tolerância a falhas ----
constexpr uint8_t MAX_FALHAS_CONSECUTIVAS = 3;   ///< Leituras inválidas seguidas até declarar falha

// ---- Wi-Fi (tenta as redes cadastradas e usa a de melhor sinal) ----
constexpr char WIFI_SSID_1[]  = "WLL-Inatel";
constexpr char WIFI_SENHA_1[] = "inatelsemfio";
constexpr char WIFI_SSID_2[]  = "Jupter";
constexpr char WIFI_SENHA_2[] = "************";
constexpr uint32_t INTERVALO_WIFI_MS = 10000;    ///< Período entre tentativas de (re)conexão Wi-Fi

// ---- Localização do irrigador (informada manualmente) ----
constexpr double LATITUDE_GRAUS  = -22.256814;   ///< Latitude em graus
constexpr double LONGITUDE_GRAUS = -45.695789;   ///< Longitude em graus

// ---- MQTT / ThingsBoard ----
constexpr char     MQTT_SERVIDOR[]  = "demo.thingsboard.io";  ///< dashboard
constexpr uint16_t MQTT_PORTA       = 1883;
constexpr char     MQTT_TOKEN[]     = "RVd1q5qvqS50s04fFT9k"; ///< Access Token do dispositivo
constexpr char     MQTT_TOPICO[]    = "v1/devices/me/telemetry";
constexpr uint32_t INTERVALO_MQTT_MS       = 5000;   ///< Período entre tentativas de reconexão MQTT
constexpr uint32_t INTERVALO_TELEMETRIA_MS = 10000;  ///< Período de envio periódico da telemetria

/* ========================================================================== */
/*  TIPOS E VARIÁVEIS GLOBAIS                                                 */
/* ========================================================================== */

/** Estados da máquina de estados do controlador. */
enum class Estado : uint8_t {
  OCIOSO,       ///< Irrigador desligado, pronto para iniciar novo ciclo
  IRRIGANDO,    ///< Irrigador ligado, contando TEMPO_IRRIGACAO_MS
  ESPERA        ///< Irrigador desligado, contando TEMPO_ESPERA_MS
};

DHT sensor(PINO_SENSOR, TIPO_SENSOR);

WiFiMulti    wifiMulti;
WiFiClient   clienteWifi;
PubSubClient mqtt(clienteWifi);

uint32_t ultimaTentativaWifi  = 0;   ///< millis() da última tentativa de Wi-Fi
uint32_t ultimaTentativaMqtt  = 0;   ///< millis() da última tentativa de MQTT
uint32_t ultimaTelemetria     = 0;   ///< millis() do último envio de telemetria
bool     enviarTelemetriaJa   = true;///< Força envio imediato (mudança de estado / reconexão)
bool     irrigadorLigado      = false; ///< Estado lógico do irrigador

Estado   estadoAtual    = Estado::OCIOSO;
uint32_t instanteEstado = 0;      ///< millis() da última transição de estado
uint32_t ultimaLeitura  = 0;      ///< millis() da última leitura do sensor
uint32_t ultimoLog      = 0;      ///< millis() do último log

float   temperaturaC    = NAN;    ///< Última temperatura válida (°C)
float   umidadePercent  = NAN;    ///< Última umidade relativa do ar válida (%)
uint8_t falhasSeguidas  = 0;      ///< Contador de leituras inválidas consecutivas
bool    sensorOk        = false;  ///< Estado de saúde do sensor

/* ========================================================================== */
/*  FUNÇÕES AUXILIARES                                                        */
/* ========================================================================== */

/**
 * @brief Aciona ou libera o relé respeitando a polaridade configurada.
 * @param ligar true para ligar o irrigador, false para desligar.
 */
void definirIrrigador(bool ligar) {
  const bool nivel = ligar ? RELE_ATIVO_EM_NIVEL_ALTO : !RELE_ATIVO_EM_NIVEL_ALTO;
  digitalWrite(PINO_RELE, nivel ? HIGH : LOW);
  if (irrigadorLigado != ligar) enviarTelemetriaJa = true;   // reporta mudança imediatamente
  irrigadorLigado = ligar;
}

/**
 * @brief Realiza a transição para um novo estado e registra o instante.
 * @param novo Estado de destino.
 */
void mudarEstado(Estado novo) {
  estadoAtual    = novo;
  instanteEstado = millis();

  switch (novo) {
    case Estado::IRRIGANDO:
      definirIrrigador(true);
      Serial.println(F("[EVENTO] Irrigador LIGADO"));
      break;
    case Estado::ESPERA:
      definirIrrigador(false);
      Serial.println(F("[EVENTO] Irrigador DESLIGADO - aguardando intervalo minimo"));
      break;
    case Estado::OCIOSO:
      definirIrrigador(false);
      Serial.println(F("[EVENTO] Controlador em repouso"));
      break;
  }
}

/**
 * @brief Lê o AM2302 e atualiza as variáveis globais.
 *
 * Uma leitura inválida (NAN) isolada é ignorada e o último valor válido é
 * mantido. Após MAX_FALHAS_CONSECUTIVAS leituras inválidas seguidas o
 * sensor é considerado em falha (sensorOk = false).
 */
void lerSensor() {
  const float t = sensor.readTemperature();
  const float h = sensor.readHumidity();

  if (isnan(t) || isnan(h)) {
    if (falhasSeguidas < MAX_FALHAS_CONSECUTIVAS) falhasSeguidas++;
    if (falhasSeguidas >= MAX_FALHAS_CONSECUTIVAS) {
      sensorOk       = false;
      temperaturaC   = NAN;
      umidadePercent = NAN;
    }
    return;
  }

  falhasSeguidas = 0;
  sensorOk       = true;
  temperaturaC   = t;
  umidadePercent = h;
}

/**
 * @brief Indica se as condições atuais permitem iniciar a irrigação.
 * @return true se o sensor está saudável e a temperatura excede o limite.
 */
bool condicaoParaIrrigar() {
  if (!sensorOk) return false;                  // fail-safe
  return temperaturaC > TEMP_LIMITE_C;
}

/**
 * @brief Mantém o Wi-Fi conectado (não bloqueia enquanto estiver conectado).
 *
 * Quando desconectado, tenta reconectar a cada INTERVALO_WIFI_MS. A chamada
 * a wifiMulti.run() pode levar alguns segundos (varredura + conexão); o
 * controle do irrigador usa millis(), portanto apenas é atrasado, nunca perdido.
 */
void manterWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  const uint32_t agora = millis();
  if (agora - ultimaTentativaWifi < INTERVALO_WIFI_MS) return;
  ultimaTentativaWifi = agora;

  Serial.println(F("[WIFI] Conectando..."));
  if (wifiMulti.run(8000) == WL_CONNECTED) {
    Serial.print(F("[WIFI] Conectado a "));
    Serial.print(WiFi.SSID());
    Serial.print(F(" | IP "));
    Serial.println(WiFi.localIP());
    enviarTelemetriaJa = true;
  } else {
    Serial.println(F("[WIFI] Sem conexao"));
  }
}

/**
 * @brief Mantém a conexão MQTT com o ThingsBoard (exige Wi-Fi conectado).
 */
void manterMqtt() {
  if (WiFi.status() != WL_CONNECTED || mqtt.connected()) return;
  const uint32_t agora = millis();
  if (agora - ultimaTentativaMqtt < INTERVALO_MQTT_MS) return;
  ultimaTentativaMqtt = agora;

  Serial.println(F("[MQTT] Conectando ao ThingsBoard..."));
  // ThingsBoard: usuário = Access Token, senha vazia.
  if (mqtt.connect("ESP32-Irrigador", MQTT_TOKEN, nullptr)) {
    Serial.println(F("[MQTT] Conectado"));
    enviarTelemetriaJa = true;
  } else {
    Serial.print(F("[MQTT] Falha, rc="));
    Serial.println(mqtt.state());
  }
}

/**
 * @brief Envia temperatura, umidade e estado do irrigador ao ThingsBoard.
 *
 * Com falha do sensor, temperatura e umidade são omitidas (não se envia
 * valor inventado); o estado do irrigador é sempre enviado.
 */
void enviarTelemetria() {
  if (!mqtt.connected()) return;

  char json[160];
  if (sensorOk) {
    snprintf(json, sizeof(json),
             "{\"temperatura\":%.1f,\"umidade\":%.0f,\"irrigador\":%s,"
             "\"latitude\":%.6f,\"longitude\":%.6f}",
             temperaturaC, umidadePercent, irrigadorLigado ? "true" : "false",
             LATITUDE_GRAUS, LONGITUDE_GRAUS);
  } else {
    snprintf(json, sizeof(json),
             "{\"irrigador\":%s,\"latitude\":%.6f,\"longitude\":%.6f}",
             irrigadorLigado ? "true" : "false", LATITUDE_GRAUS, LONGITUDE_GRAUS);
  }

  if (mqtt.publish(MQTT_TOPICO, json)) {
    Serial.print(F("[MQTT] Enviado: "));
    Serial.println(json);
    ultimaTelemetria   = millis();
    enviarTelemetriaJa = false;
  } else {
    Serial.println(F("[MQTT] Falha ao publicar"));
  }
}

/**
 * @brief Imprime o status atual no monitor serial.
 */
void imprimirStatus() {
  static const char* const nomes[] = {"OCIOSO", "IRRIGANDO", "ESPERA"};
  Serial.print(F("[STATUS] Estado="));
  Serial.print(nomes[static_cast<uint8_t>(estadoAtual)]);
  if (sensorOk) {
    Serial.print(F(" | Temp="));   Serial.print(temperaturaC, 1);   Serial.print(F(" C"));
    Serial.print(F(" | Umidade=")); Serial.print(umidadePercent, 0); Serial.println(F(" %"));
  } else {
    Serial.println(F(" | Sensor: FALHA"));
  }
}

/* ========================================================================== */
/*  SETUP E LOOP                                                              */
/* ========================================================================== */

/**
 * @brief Inicialização: serial, GPIOs (saída em estado seguro) e sensor.
 */
void setup() {
  Serial.begin(115200);

  pinMode(PINO_RELE, OUTPUT);
  definirIrrigador(false);            // garante irrigador desligado ao ligar

  sensor.begin();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  wifiMulti.addAP(WIFI_SSID_1, WIFI_SENHA_1);
  wifiMulti.addAP(WIFI_SSID_2, WIFI_SENHA_2);
  mqtt.setServer(MQTT_SERVIDOR, MQTT_PORTA);
  mqtt.setSocketTimeout(3);           // limita o bloqueio em conexões MQTT falhas

  Serial.println(F("=== Controlador de Irrigacao ESP32 v1.2.0 ==="));
  Serial.print(F("Limite: "));  Serial.print(TEMP_LIMITE_C, 1);
  Serial.println(F(" C"));

  delay(2000);                        // tempo de estabilização do DHT22 (única espera bloqueante)
  lerSensor();
  mudarEstado(Estado::OCIOSO);
}

/**
 * @brief Laço principal: amostra o sensor e executa a máquina de estados.
 */
void loop() {
  const uint32_t agora = millis();

  // --- Leitura periódica do sensor ---
  if (agora - ultimaLeitura >= INTERVALO_LEITURA_MS) {
    ultimaLeitura = agora;
    lerSensor();
  }

  // --- Máquina de estados ---
  switch (estadoAtual) {

    case Estado::OCIOSO:
      if (condicaoParaIrrigar()) {
        mudarEstado(Estado::IRRIGANDO);
      }
      break;

    case Estado::IRRIGANDO:
      // Desliga ao fim do tempo, independentemente da temperatura.
      // Em falha do sensor, desliga imediatamente por segurança.
      if (agora - instanteEstado >= TEMPO_IRRIGACAO_MS || !sensorOk) {
        mudarEstado(Estado::ESPERA);
      }
      break;

    case Estado::ESPERA:
      // Após 5 min volta a OCIOSO; se ainda estiver quente, OCIOSO
      // iniciará novo ciclo na próxima iteração.
      if (agora - instanteEstado >= TEMPO_ESPERA_MS) {
        mudarEstado(Estado::OCIOSO);
      }
      break;
  }

  // --- Rede e telemetria (nunca interfere no controle) ---
  manterWifi();
  manterMqtt();
  mqtt.loop();
  if (enviarTelemetriaJa || agora - ultimaTelemetria >= INTERVALO_TELEMETRIA_MS) {
    enviarTelemetria();
  }

  // --- Log periódico ---
  if (agora - ultimoLog >= INTERVALO_LOG_MS) {
    ultimoLog = agora;
    imprimirStatus();
  }
}
