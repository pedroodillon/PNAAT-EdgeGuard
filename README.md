# EdgeGuard

## Sistema IoT de Monitoramento Operacional e da Condição de Máquinas Rotativas

Este repositório reúne o firmware, os arquivos do modelo e o software de apoio da prova de conceito desenvolvida pela equipe EdgeGuard.

Máquinas rotativas antigas podem continuar em uso sem oferecer conectividade ou histórico de operação. Isso dificulta saber por quanto tempo funcionaram e perceber mudanças no seu comportamento. Nossa proposta é acompanhar essas máquinas por meio de áudio, vibração e corrente elétrica, sem substituir o equipamento nem alterar seu controle original.

Estamos usando uma bancada com motor DC de 12 V reaproveitado de uma furadeira. O ESP32-S3 será responsável pela aquisição e pelo processamento local. Os resultados serão enviados por Wi-Fi e MQTT a um computador de apoio, onde ficarão o broker, o histórico e a visualização.

**O protótipo está em desenvolvimento.** A captura de áudio, o RTC e o funcionamento do motor já foram testados individualmente. A integração dos sensores, do modelo e da comunicação ainda está em andamento ou pendente.

## Situação da bancada

| Item | O que já foi feito | O que falta |
|---|---|---|
| DS3231 | Leitura e retenção de data e hora testadas; funcionamento independente. | Integrar a referência temporal aos registros do sistema. |
| INMP441 | Captura de áudio funcionando no ESP32-S3. | Integrar a captura ao restante do fluxo. |
| Motor DC | Motor de 12 V da furadeira instalado na bancada e em funcionamento. | Utilizá-lo nos ensaios de coleta e validação. |
| BNO085 | Sensor disponível e em processo de integração. | Concluir a leitura de vibração e a integração ao fluxo principal. |
| ACS712 20 A | Duas unidades compradas para medição de corrente. | Integrar ao ESP32-S3, validar a adequação ao ADC, calibrar e definir os limiares. |
| MQTT | Comunicação prevista na arquitetura. | Integrar a publicação dos resultados e testar a reconexão. |
| Modelo | Edge Impulse definido para treinamento e avaliação. | Treinar, avaliar, exportar e integrar o modelo ao ESP32-S3. |

As seções abaixo descrevem o funcionamento previsto para a versão integrada, não resultados já obtidos.

## Como o sistema vai funcionar

Separamos o monitoramento em dois caminhos:

- **Condição da máquina:** áudio e vibração serão analisados por um modelo treinado para reconhecer mudanças em relação ao comportamento de referência.
- **Estado operacional:** a corrente do motor será comparada com limiares calibrados nos ensaios, sem usar o modelo.

### Arquitetura e fluxo de dados

```text
Motor DC de 12 V
   |
   +--> BNO085 -------------+
   |    vibração            |
   |                        |
   +--> INMP441 ------------+--> ESP32-S3 --> Wi-Fi / MQTT --> Computador de apoio
   |    áudio               |        |
   |                        |        +--> pré-processamento
   +--> ACS712 -------------+        +--> inferência do modelo
        corrente            |        +--> regras de estado
                            |        +--> registros e fila temporária
DS3231 ---------------------+        +--> LED local
data e hora
```

O DS3231 fornece a referência de data e hora ao ESP32-S3; ele não mede uma grandeza do motor. O computador de apoio mantém o broker MQTT, o histórico e a visualização. O treinamento e a exportação do modelo ocorrem no computador, antes de sua implantação no ESP32-S3.

### Áudio e vibração

O BNO085 fornecerá a aceleração triaxial, e o INMP441 fornecerá as amostras de áudio. O firmware organizará os sinais em janelas associadas à data e à hora do DS3231. Essas janelas passarão pela filtragem e pela extração de características antes da inferência.

O modelo será preparado no computador de apoio e executado no ESP32-S3. Sua saída será um escore de anomalia, usado para indicar mudanças em relação ao padrão de referência.

Vamos comparar vibração isolada, áudio isolado e a combinação dos dois sinais. A comparação usará a mesma divisão de sessões, para que os resultados das três configurações possam ser avaliados nas mesmas condições.

### Corrente e estado operacional

O ACS712 20 A fornecerá um sinal analógico ao ADC do ESP32-S3. Com a leitura calibrada, o firmware aplicará limiares para a classificação prevista no projeto: **desligada, energizada e operando sob carga**.

A separação desses estados ainda precisa ser validada na bancada. A integração do sensor, a adequação da leitura ao ADC e a definição dos limiares permanecem pendentes.

A corrente será usada somente nesse caminho. **O dataset e as entradas do modelo serão formados exclusivamente por áudio e vibração.**

### Registro e comunicação

O ESP32-S3 reunirá horário, estado operacional, corrente, escore de anomalia e alertas. O LED fará a indicação local, e os registros serão publicados por Wi-Fi e MQTT para o computador de apoio.

Durante interrupções temporárias da comunicação, o processamento local deverá continuar ativo. Os registros não enviados ficarão em uma fila temporária em RAM, com reconexão automática ao broker. O uso de microSD é apenas uma possibilidade caso a fila em RAM seja insuficiente; não é uma dependência da estrutura inicial.

## Hardware

| Componente | Interface | Uso no projeto |
|---|---|---|
| ESP32-S3 LoRa V3 | I2C, I2S, ADC, GPIO e Wi-Fi | Aquisição, processamento local, inferência e comunicação. |
| BNO085 | I2C | Aceleração triaxial para análise de vibração. |
| INMP441 | I2S | Captura digital de áudio. |
| ACS712 20 A | Saída analógica / ADC | Medição da corrente consumida pelo motor. |
| DS3231 | I2C | Referência de data e hora para janelas e eventos. |
| Motor DC de 12 V e driver | Alimentação e controle | Máquina de teste para os ensaios. |
| Fonte do motor | DC | Alimentação independente da eletrônica de controle. |
| LED local | GPIO | Indicação de estado ou alerta; GPIO a definir na integração. |
| Computador de apoio | Rede local | Treinamento, broker MQTT, histórico e visualização. |

A comunicação prevista é Wi-Fi com MQTT em rede local.

## Software e dependências

| Recurso | Finalidade | Definição atual |
|---|---|---|
| Git e GitHub | Versionamento e hospedagem do código e da documentação. | Ferramentas adotadas para o repositório. |
| ESP-IDF | Desenvolvimento do firmware do ESP32-S3. | Framework principal previsto. |
| FreeRTOS | Organização e sincronização das tarefas. | Integrado ao ESP-IDF. |
| APIs de I2C, I2S e ADC do ESP-IDF | Comunicação com sensores e aquisição dos sinais. | Recursos previstos para o firmware. |
| ESP-MQTT | Publicação dos resultados no broker. | Integração pendente. |
| Edge Impulse | Preparação dos dados, treinamento, avaliação e exportação. | Plataforma definida; modelo ainda não treinado. |
| Broker MQTT | Recepção das mensagens do ESP32-S3. | Software específico a definir durante a integração. |
| Runtime de inferência | Execução do modelo quantizado no ESP32-S3. | A definir após a escolha e a validação do modelo. |

As versões das ferramentas e as bibliotecas específicas dos sensores serão registradas conforme forem validadas. Ainda não há uma configuração completa de dependências confirmada para a execução integrada.

## Organização do repositório

A estrutura inicial separa o firmware, o desenvolvimento do modelo, o software do computador e os testes. Os diretórios representam a organização prevista; sua presença não significa que todos os módulos estejam implementados.

```text
PNAAT-EdgeGuard/
├── README.md
├── .gitignore
├── firmware/
│   └── components/
│       ├── acquisition/
│       ├── preprocessing/
│       ├── inference/
│       ├── operational_state/
│       ├── mqtt/
│       └── indicator/
├── model/
│   ├── training/
│   ├── configs/
│   └── artifacts/
├── server/
│   ├── broker/
│   ├── storage/
│   └── visualization/
├── data/
└── tests/
    ├── firmware/
    └── integration/
```

- `firmware/components/`: módulos de aquisição, pré-processamento, inferência, estado operacional, MQTT e indicação local.
- `model/`: arquivos de treinamento, configurações, exportações e artefatos do modelo.
- `server/`: configuração do broker, armazenamento e visualização no computador de apoio.
- `data/`: organização das sessões e dos dados de áudio e vibração.
- `tests/`: testes do firmware e da integração entre os módulos.

## Preparação do ambiente

Para trabalhar no firmware, é necessário ter o Git instalado, o ESP-IDF configurado para o ESP32-S3 e a placa conectada e reconhecida pelo computador.

Nos testes de comunicação, o ESP32-S3 e o computador de apoio deverão estar na mesma rede local, com um broker MQTT disponível.

### Obter o código

```bash
git clone https://github.com/pedroodillon/PNAAT-EdgeGuard.git
cd PNAAT-EdgeGuard/firmware
```

### Compilar e gravar

Com o ambiente ESP-IDF ativo e o projeto de firmware disponível nessa pasta, o fluxo previsto é:

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

A árvore de diretórios, sozinha, não constitui um firmware compilável. Esses comandos dependem da presença dos arquivos do projeto ESP-IDF e de suas dependências.

A configuração do broker e os detalhes da publicação MQTT serão documentados durante a integração. Por enquanto, não há um procedimento completo de execução do sistema ponta a ponta.

## Requisitos

Estes são os compromissos da prova de conceito. As metas abaixo ainda serão verificadas nos ensaios.

### Funcionais

| ID | Requisito |
|---|---|
| RF-01 | Capturar vibração e áudio em janelas associadas à data e hora. |
| RF-02 | Identificar os estados desligada, energizada e sob carga pela corrente medida no ACS712 20 A, com calibração e limiares definidos nos testes. |
| RF-03 | Registrar sessões em condição normal e em pelo menos duas condições anômalas seguras. |
| RF-04 | Filtrar os sinais e extrair características compatíveis com a memória e a capacidade de processamento do ESP32-S3. |
| RF-05 | Executar o modelo com áudio e vibração e gerar o escore de anomalia. |
| RF-06 | Indicar o estado localmente e publicar por MQTT horário, estado operacional, corrente, escore de anomalia e alertas. |
| RF-07 | Armazenar o histórico e calcular métricas experimentais no computador de apoio. |

### Não funcionais

| ID | Requisito |
|---|---|
| RNF-01 | Usar Wi-Fi e MQTT, mantendo o processamento local mesmo sem conexão com o broker. |
| RNF-02 | Publicar o resultado em até 2 segundos após o encerramento de cada janela. |
| RNF-03 | Operar continuamente por pelo menos 30 minutos e entregar no mínimo 90% das mensagens geradas no teste. |
| RNF-04 | Reconectar automaticamente ao broker e manter os registros pendentes em fila durante interrupções temporárias. |
| RNF-05 | Operar sem reinicializações inesperadas e permitir o acompanhamento de memória e processamento. |
| RNF-06 | Separar a alimentação do motor da eletrônica de controle, proteger as partes móveis e permitir parada manual. |
| RNF-07 | Organizar o firmware em módulos com ESP-IDF e FreeRTOS, registrando as dependências utilizadas. |

## Dataset e validação

O dataset será formado pelas sessões de áudio e vibração do motor em condição normal e em pelo menos duas condições anômalas seguras. A corrente não será incluída como entrada do modelo.

A separação entre treino, validação e teste será feita **por sessão**, evitando que trechos de uma mesma coleta sejam distribuídos entre esses conjuntos. Vamos manter a posição, a orientação e a fixação dos sensores durante as comparações e usar a mesma divisão de sessões nas três configurações: vibração, áudio e fusão.

| Etapa | Verificação | Evidência prevista |
|---|---|---|
| 1. Sensores | Testar individualmente BNO085, INMP441, ACS712 e DS3231. | Logs, capturas e parâmetros de teste. |
| 2. Coleta | Registrar sessões normais e anômalas seguras. | Arquivos e identificação das sessões. |
| 3. Treinamento | Comparar vibração, áudio e fusão no Edge Impulse. | Métricas das três configurações. |
| 4. Implantação | Exportar o modelo para o ESP32-S3 e medir memória e tempo de inferência. | Modelo exportado e medições de recursos. |
| 5. Integração | Reunir estado operacional, modelo, LED, MQTT e histórico. | Logs, mensagens e capturas. |
| 6. Teste contínuo | Executar 30 minutos de operação e testar a reconexão MQTT. | Registros de execução e resultados dos testes. |

## Cuidados de configuração e bancada

- Não versionar SSID, senhas, tokens, chaves ou certificados privados.
- Manter arquivos locais com credenciais fora do controle de versão; publicar apenas configurações reproduzíveis e seguras.
- Conferir a montagem antes de energizar a bancada.
- Usar uma fonte independente para o motor.
- Validar a adequação da leitura do ACS712 ao ADC durante a integração.
- Proteger as partes móveis e manter uma forma de parada manual durante os ensaios.

## Limites da primeira versão

A primeira versão se concentra na bancada com ESP32-S3, sensores de áudio, vibração e corrente, referência de horário, LED e comunicação MQTT. O resultado esperado é registrar o estado da máquina, reconhecer mudanças de condição e formar um histórico experimental.

Não fazem parte desta versão:

- Raspberry Pi 5, câmera ou visão computacional;
- integração com CLP, ERP ou sistemas de controle industrial;
- tensão, corrente ou RPM como entradas do modelo;
- diagnóstico certificado de falhas;
- prognóstico;
- cálculo completo de OEE.

## Equipe

**EdgeGuard — PNAAT 2026 | Cariri TCC**
