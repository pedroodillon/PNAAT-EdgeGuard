# EdgeGuard

## Sistema IoT de Monitoramento Operacional e da Condição de Máquinas Rotativas

Este repositório reúne o firmware e os arquivos utilizados na prova de conceito desenvolvida pela equipe EdgeGuard no PNAAT 2026.

Máquinas rotativas antigas podem continuar em uso sem oferecer conectividade ou histórico de operação. Isso dificulta acompanhar por quanto tempo permaneceram em funcionamento e perceber mudanças no seu comportamento. Nossa proposta é acompanhar uma máquina rotativa de bancada por corrente elétrica, áudio e vibração, sem substituir o equipamento nem alterar seu controle original.

Na bancada, usamos um motor DC de 12 V reaproveitado de uma furadeira e um ESP32-S3 como controlador principal. O firmware lê a corrente pelo ACS712, calcula o nível de áudio captado pelo INMP441, coleta a aceleração linear do BNO085 e executa no próprio microcontrolador um modelo exportado pelo Edge Impulse. Os dados são publicados por Wi-Fi e MQTT/TLS no AWS IoT Core.

## Funcionamento

O sistema trata separadamente o estado de funcionamento do motor e sua condição mecânica:

- **Estado operacional:** a corrente medida pelo ACS712 determina se o motor está `OFF` ou `RUNNING`. Durante a transição, o firmware também mantém internamente os estados `STARTUP` e `STEADY`.
- **Condição mecânica:** o BNO085 fornece aceleração linear nos eixos X, Y e Z. A cada janela de dois segundos, o modelo classifica a vibração entre `NORMAL`, `L1` e `L2`.
- **Áudio:** o INMP441 é lido em paralelo a 16 kHz. O firmware calcula o RMS do sinal e inclui esse valor na telemetria. O áudio não é entrada do modelo embarcado disponível neste repositório.
- **Comunicação:** corrente, áudio, vibração e resultados da inferência são reunidos em uma mensagem JSON e enviados por MQTT com QoS 1.

### Fluxo de dados

```text
Motor DC de 12 V
   |
   +--> ACS712 --------------------> corrente / estado operacional ----+
   |                                                                  |
   +--> INMP441 -------------------> áudio RMS ------------------------+--> ESP32-S3
   |                                                                  |       |
   +--> BNO085 --------------------> vibração X/Y/Z -------------------+       |
                                                                              +--> Edge Impulse
                                                                              |    inferência local
                                                                              |
                                                                              +--> telemetria JSON
                                                                                   |
                                                                                   +--> Wi-Fi
                                                                                   +--> MQTT/TLS
                                                                                   +--> AWS IoT Core
```

## Hardware

| Componente | Ligação utilizada | Função |
|---|---|---|
| ESP32-S3 LoRa V3 | ESP-IDF | Aquisição, processamento, inferência e comunicação |
| ACS712 20 A | `ADC1_CHANNEL_0` — GPIO 1 | Medição da corrente do motor |
| INMP441 | BCLK GPIO 39, WS GPIO 40, DATA GPIO 41 e L/R em 3,3 V | Captura de áudio pelo canal direito |
| BNO085 | SDA GPIO 6, SCL GPIO 7, INT GPIO 5, RST GPIO 4 e endereço `0x4A` | Aceleração linear nos eixos X, Y e Z |
| Motor DC 12 V | Alimentação externa | Máquina utilizada nos ensaios |
| Fonte de bancada | Saída DC regulada | Alimentação e ajuste do motor |

Todos os módulos ligados ao ESP32-S3 precisam compartilhar a mesma referência de GND. O motor utiliza alimentação externa, e o ACS712 deve ser colocado em série com um dos condutores de alimentação do motor.

A saída do ACS712 chega ao ADC por meio do condicionamento usado na bancada. A tensão aplicada ao GPIO deve permanecer dentro da faixa aceita pelo ESP32-S3. O firmware considera uma sensibilidade efetiva de `31,25 mV/A`; a troca do módulo ou do circuito de condicionamento exige nova calibração.

Durante a inicialização, o motor deve permanecer desligado. O firmware utiliza as primeiras 500 leituras para calcular o zero do ACS712.

### Parâmetros da leitura de corrente

| Parâmetro | Valor no firmware |
|---|---:|
| Amostras para calibração de zero | 500 |
| Amostras por leitura média | 100 |
| Intervalo entre amostras | 2 ms |
| Sensibilidade efetiva no ADC | 31,25 mV/A |
| Banda morta | 0,08 A |
| Entrada no estado de funcionamento | 0,25 A |
| Retorno ao estado desligado | 0,15 A |
| Duração do estado de partida | 3 s |

Os limiares de ligar e desligar são diferentes para evitar que pequenas oscilações façam o estado alternar continuamente.

## Modelo embarcado

O modelo exportado pelo Edge Impulse está em `components/edge_impulse/`, junto com o SDK utilizado pelo firmware. A inferência recebe somente os três eixos de aceleração do BNO085.

| Parâmetro | Valor |
|---|---:|
| Frequência configurada | 200 Hz |
| Intervalo entre amostras | 5 ms |
| Amostras por janela | 400 |
| Duração da janela | 2 s |
| Eixos por amostra | 3 |
| Valores de entrada | 1.200 |
| Classes | `L1`, `L2`, `NORMAL` |
| Motor de inferência | TensorFlow Lite / Edge Impulse |
| Quantização | int8 |

Antes de iniciar os sensores, o programa confere se o número de amostras, a quantidade de eixos e o tamanho da entrada correspondem aos parâmetros do modelo. Ao terminar uma janela, calcula o RMS da vibração, executa `run_classifier()` e mantém como estado mecânico a classe de maior probabilidade.

O conjunto de dados usado no treinamento não está publicado em `data/`. O repositório contém o modelo já exportado para execução no ESP32-S3, mas não inclui os arquivos originais de coleta nem métricas detalhadas do treinamento.

## Telemetria

A publicação ocorre aproximadamente a cada dois segundos. O tópico, o identificador do dispositivo e o endpoint ficam em `main/aws_iot_config.h`.

Exemplo do payload gerado pelo firmware:

```json
{
  "device_id": "P101-EdgeGuard-01",
  "status": "online",
  "current_a": 0.000,
  "audio_rms": 0.000000,
  "vibration_rms": 0.0000,
  "operational_state": "OFF",
  "mechanical_state": "NORMAL",
  "ai_confidence": 0.0000,
  "startup_peak_a": 0.000
}
```

Até a primeira inferência, o campo `mechanical_state` pode aparecer como `UNKNOWN`.

## Organização do repositório

O projeto ESP-IDF utilizado na compilação está na raiz. O ponto de entrada ativo é `main/main.cpp`.

```text
PNAAT-EdgeGuard/
├── CMakeLists.txt
├── sdkconfig
├── dependencies.lock
├── partitions.csv
├── main/
│   ├── main.cpp
│   ├── CMakeLists.txt
│   ├── idf_component.yml
│   ├── aws_iot_config.h
│   └── certs/
│       └── AmazonRootCA1.pem
├── components/
│   ├── bno085/
│   ├── edge_impulse/
│   ├── i2c_config/
│   ├── inmp441/
│   └── sh2/
├── firmware/
├── model/
├── data/
├── server/
├── tests/
└── THIRD_PARTY.md
```

- `main/`: aplicação integrada e configurações do projeto principal.
- `components/bno085/`: integração do sensor de movimento.
- `components/inmp441/`: aquisição do microfone por I2S.
- `components/i2c_config/`: configuração do barramento I2C.
- `components/sh2/`: biblioteca utilizada na comunicação com o BNO085.
- `components/edge_impulse/`: SDK, parâmetros e modelo exportado.
- `firmware/`: código mantido de etapas anteriores do projeto; não é o ponto de entrada do `CMakeLists.txt` da raiz.
- `model/`, `data/`, `server/` e `tests/`: diretórios de apoio. Seus READMEs registram somente a função das pastas e não substituem arquivos de treinamento, serviços ou testes executáveis.

Somente `main/main.cpp` é registrado como fonte da aplicação em `main/CMakeLists.txt`. Os arquivos de teste, backup ou validação mantidos em `main/` não são compilados pelo projeto principal.

## Pré-requisitos

- Git;
- ESP-IDF 6.1.x com suporte ao ESP32-S3;
- ESP-IDF Component Manager;
- cabo USB compatível com a placa;
- rede Wi-Fi com acesso ao endpoint configurado;
- dispositivo, certificado e política configurados no AWS IoT Core;
- bancada montada com o motor desligado durante a inicialização.

O arquivo `dependencies.lock` foi gerado com ESP-IDF 6.1.0. A dependência MQTT declarada em `main/idf_component.yml` é `espressif/mqtt` 1.1.0.

## Configuração

### 1. Obter o código

```bash
git clone https://github.com/pedroodillon/PNAAT-EdgeGuard.git
cd PNAAT-EdgeGuard
```

Os comandos do ESP-IDF devem ser executados na raiz, onde está o `CMakeLists.txt` principal.

### 2. Configurar o Wi-Fi

Crie o arquivo `main/wifi_config.h`:

```cpp
#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#define WIFI_SSID "NOME_DA_REDE"
#define WIFI_PASSWORD "SENHA_DA_REDE"

#endif
```

Esse arquivo está no `.gitignore` e não deve ser publicado.

### 3. Configurar o AWS IoT Core

No AWS IoT Core, prepare um dispositivo com certificado ativo e uma política que permita a conexão do client ID e a publicação no tópico utilizado pelo projeto. Depois, ajuste `main/aws_iot_config.h`:

```cpp
#define AWS_IOT_HOST "SEU_ENDPOINT-ats.iot.SUA_REGIAO.amazonaws.com"
#define AWS_IOT_PORT 8883
#define AWS_IOT_CLIENT_ID "P101-EdgeGuard-01"
#define AWS_IOT_TOPIC "edgeguard/P101-EdgeGuard-01/telemetry"
```

A CA da Amazon já está em:

```text
main/certs/AmazonRootCA1.pem
```

Adicione localmente o certificado e a chave privada do dispositivo com estes nomes:

```text
main/certs/device-cert.pem.crt
main/certs/device-private.pem.key
```

Os três arquivos são incorporados ao firmware durante a compilação. O certificado do dispositivo e a chave privada estão ignorados pelo Git e não devem ser enviados ao repositório.

### 4. Compilar

Com o ambiente do ESP-IDF ativo:

```bash
idf.py set-target esp32s3
idf.py build
```

O projeto utiliza a tabela de partições definida em `partitions.csv`.

### 5. Gravar e abrir o monitor serial

Substitua `<PORTA>` pela porta da placa:

```bash
idf.py -p <PORTA> flash monitor
```

Exemplos:

```text
Windows: COM5
Linux:   /dev/ttyUSB0 ou /dev/ttyACM0
```

## Confirmação do funcionamento

Mantenha o motor desligado ao reiniciar o ESP32-S3. A sequência esperada é:

1. calibração do zero do ACS712;
2. inicialização do INMP441;
3. inicialização do barramento I2C e do BNO085;
4. conexão à rede Wi-Fi;
5. conexão MQTT/TLS ao AWS IoT Core;
6. coleta das janelas de vibração e execução da inferência;
7. publicação periódica da telemetria.

Após cada janela de inferência, o monitor serial apresenta um painel com esta estrutura:

```text
P-101 / EDGE GUARD
CURRENT_A          : ... A
AUDIO_RMS          : ...
OPERATIONAL_STATE  : OFF | RUNNING
VIBRATION_RMS      : ... m/s^2
MECHANICAL_STATE   : NORMAL | L1 | L2
AI_CONFIDENCE      : ... %
L1                 : ... %
L2                 : ... %
NORMAL             : ... %
```

A comunicação é confirmada quando o monitor registra a conexão com a AWS, retorna um identificador de publicação e a mensagem aparece no tópico configurado.

## Problemas comuns

| Sintoma | Verificação |
|---|---|
| Erro indicando ausência de `wifi_config.h` | Crie o arquivo conforme a seção de configuração do Wi-Fi |
| Erro ao localizar certificado ou chave | Confira os nomes e o diretório `main/certs/` |
| O sistema permanece aguardando a rede | Verifique SSID, senha, sinal e acesso da rede ao endpoint AWS |
| O AWS IoT não aceita a conexão | Confira endpoint, client ID, certificado ativo e política associada |
| `AUDIO_RMS` permanece zerado | Confira BCLK, WS, DATA e o pino L/R em 3,3 V |
| BNO085 não fornece amostras | Confira SDA, SCL, INT, RST, GND e o endereço `0x4A` |
| Corrente incorreta após o boot | Reinicie com o motor desligado e confira o condicionamento do sinal do ACS712 |
| Estado do motor oscila | Verifique ruído no ADC, GND comum e os limiares de corrente |

## Limites do protótipo

- O modelo embarcado utiliza somente a vibração triaxial; o áudio é enviado como telemetria.
- O repositório não contém o dataset original nem as métricas completas de treinamento.
- A telemetria não é armazenada localmente quando a conexão MQTT está indisponível.
- O firmware não utiliza microSD, RTC, LED de sinalização ou dashboard local.
- As classes `L1` e `L2` representam as condições usadas no treinamento da bancada e não constituem diagnóstico certificado de uma falha industrial.
- A montagem não substitui instrumentos de proteção, supervisão ou segurança de máquinas.

## Código de terceiros

O projeto inclui o SDK do Edge Impulse, a biblioteca SH2 e o driver do BNO085. As licenças e atribuições disponíveis estão nos próprios componentes e em `THIRD_PARTY.md`.

## Equipe

**EdgeGuard**

