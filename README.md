# Mangue Baja TCU - 24: Documentação Técnica da Unidade de Controle de Temperatura

Este repositório contém o firmware para a Unidade de Controle Traseira (TCU - Temperature Control Unit) do projeto Mangue Baja. O software é desenvolvido sobre a plataforma Mbed OS e se destina a rodar em um microcontrolador compatível, como a ST NUCLEO-F103RB.

A TCU é uma ECU especializada, focada no monitoramento de sensores críticos do powertrain e na aquisição de dado da velocidade, além de controlar atuadores com base em comandos recebidos pela rede CAN.

## Funcionalidades Técnicas

-   **Monitoramento de Temperatura Dual**: O sistema monitora continuamente as temperaturas de componentes críticos do powertrain:
    -   **Temperatura do Motor**: Utiliza um sensor analógico (termistor) e uma equação de conversão exponencial para obter a temperatura do motor com precisão.
    -   **Temperatura da CVT**: Emprega um sensor de temperatura infravermelho sem contato, o **MLX90614**, que se comunica via I2C para medir a temperatura da CVT sem a necessidade de contato físico, aumentando a robustez da medição.

-   **Aquisição de Dados Dinâmicos**:
    -   **Velocidade do Veículo**: Calcula a velocidade em tempo real a partir de um sensor de frequência acoplado ao disco de freio traseiro. A leitura é feita por interrupção para máxima precisão. As constantes físicas, como diâmetro da roda e número de furos do disco sensor, são parametrizadas.
    -   **Filtragem de Sinal**: Aplica um filtro digital **FIR (Finite Impulse Response)** customizado sobre o sinal de velocidade calculado para atenuar ruídos e fornecer uma leitura mais estável.

-   **Monitoramento do Sistema Elétrico**:
    -   **Tensão e SoC**: Mede a tensão da bateria através de um divisor de tensão e calcula o Estado de Carga (SoC - State of Charge) a partir desta medição.
    -   **Corrente do Sistema**: Monitora a corrente total consumida pelo sistema.
    -   Para garantir a estabilidade das leituras analógicas, que são inerentemente ruidosas, o firmware implementa uma técnica de **média móvel** em software.

-   **Controle de Atuador via CAN**: A TCU recebe comandos da rede CAN (especificamente, mensagens com `THROTTLE_ID`) para controlar um servomotor. O servo é posicionado via PWM para estados pré-definidos (`SERVO_MID`, `SERVO_RUN`, `SERVO_CHOKE`), cujos valores de pulso são parametrizados.

-   **Arquitetura Orientada a Eventos em Tempo Real**: Assim como outras ECUs do projeto, a TCU é construída sobre o Mbed OS e utiliza uma arquitetura de máquina de estados orientada a eventos para garantir um comportamento robusto e determinístico. `Tickers` e interrupções de hardware enfileiram tarefas que são processadas sequencialmente pelo loop principal.

## Hardware e Tech Stack

-   **Placa Alvo**: Microcontrolador compatível com Mbed OS (e.g., ST NUCLEO-F103RB).
-   **Sistema Operacional**: Mbed OS.
-   **Protocolos de Comunicação**:
    -   **CAN**: 1000 Kbps.
    -   **I2C**: Para o sensor de temperatura MLX90614.
    -   **Serial (UART)**: 115200 baud para depuração ou comunicação com outros dispositivos.
-   **Sensores e Atuadores**:
    -   Sensor de Temperatura IR: MLX90614.
    -   Sensor de Frequência (efeito Hall ou indutivo) para velocidade.
    -   Sensores Analógicos para temperatura do motor, tensão e corrente.
    -   Servomotor (controlado por PWM).
-   **Bibliotecas Chave**:
    -   **Mbed OS**: Fornece a camada de abstração de hardware (HAL) e o kernel do RTOS.
    -   **MLX90614**: Driver customizado para o sensor de temperatura.
    -   **CANMsg**: Wrapper C++ para simplificar a manipulação de mensagens CAN.
    -   **FIR**: Biblioteca de filtro digital FIR.

## Arquitetura do Firmware

O firmware adota um padrão de **máquina de estados orientada a eventos**, que é altamente eficaz para sistemas embarcados em tempo real. Esta abordagem desacopla a detecção de eventos da sua execução.

1.  **Geração de Eventos (Assíncrona)**:
    -   **`InterruptIn`**: Interrupções de hardware, como a recepção de uma mensagem CAN (`canISR`) ou um pulso do sensor de velocidade (`frequencyCounterISR`), executam o mínimo de código possível no contexto da interrupção. Elas geralmente agendam uma função de tratamento mais longa para ser executada na `EventQueue` da thread principal, garantindo a segurança e a responsividade do sistema.
    -   **`Ticker`**: Vários `Tickers` são configurados para disparar em frequências diferentes (1Hz, 5Hz, etc.). A cada disparo, eles enfileiram um ou mais "estados" (`state_t`) em um `CircularBuffer`. Cada estado corresponde a uma tarefa a ser executada, como `TEMP_MOTOR_ST` ou `SPEED_ST`.

2.  **Enfileiramento de Tarefas**:
    -   O `CircularBuffer<state_t, BUFFER_SIZE>` funciona como uma fila de tarefas (First-In, First-Out). É o principal mecanismo de comunicação entre os geradores de eventos periódicos e o loop de processamento.

3.  **Loop de Processamento (Síncrono)**:
    -   A thread principal (`main`) executa um loop infinito que constantemente verifica se há um novo estado no `CircularBuffer`.
    -   Ao retirar um estado da fila, um `switch-case` direciona o fluxo de execução para a rotina apropriada, que realiza a leitura do sensor, os cálculos necessários e a transmissão dos dados via CAN.
    -   Este modelo garante que as tarefas sejam executadas de forma ordenada e sem concorrência por recursos compartilhados.

## Estrutura e Descrição dos Arquivos

```
MangueBaja_TCU/
├── main.cpp                  # Lógica principal, máquina de estados e inicializações
├── Other_things/             # Arquivos de exemplo e configuração do Mbed
├── BAJA_DEFS/                # Definições específicas do projeto
│   ├── defs.h
│   ├── FIR.h
│   └── rear_defs.h
├── CANMsg/                   # Wrapper para a classe CANMessage
│   └── CANMsg.h
└── MLX/                      # Driver para o sensor de temperatura IR
├── MLX90614.cpp
└── MLX90614.h
```
### Descrição Detalhada dos Arquivos

-   **`main.cpp`**: Arquivo central do firmware. Contém a lógica de inicialização de hardware, a definição dos objetos do Mbed OS (`Thread`, `EventQueue`, `Ticker`), as rotinas de serviço de interrupção (ISRs) e o loop infinito com a máquina de estados `switch-case` que impulsiona toda a funcionalidade do sistema. Ele também contém as funções de conversão de dados brutos dos sensores (e.g., `Voltage_moving_average`, `CVT_Temperature`) em unidades físicas.

-   **`MLX/MLX90614.h` & `MLX90614.cpp`**: Driver para o sensor de temperatura infravermelho MLX90614. A classe encapsula a comunicação I2C, o acesso aos registradores de temperatura do objeto e ambiente, e a conversão dos dados brutos para graus Celsius.

-   **`CANMsg/CANMsg.h`**: Fornece uma classe C++ que herda de `CANMessage`. A principal contribuição é a sobrecarga dos operadores `<<` e `>>`, permitindo uma sintaxe limpa e segura para empacotar e desempacotar diferentes tipos de dados no payload de 8 bytes da mensagem CAN.

-   **`BAJA_DEFS/defs.h`**: Arquivo de definições da rede CAN. Centraliza todos os IDs das mensagens CAN, garantindo que todas as ECUs do projeto utilizem os mesmos identificadores para interoperabilidade.

-   **`BAJA_DEFS/rear_defs.h`**: Contém definições e constantes físicas específicas para os cálculos realizados na TCU. Isso inclui o diâmetro da roda, o número de furos do disco sensor, valores de resistores do divisor de tensão e as larguras de pulso do servomotor para cada modo de operação.

-   **`BAJA_DEFS/FIR.h`**: Contém a implementação da classe `FIR` para filtragem de sinais, com métodos para realizar a convolução e atualizar o histórico de amostras do filtro.
