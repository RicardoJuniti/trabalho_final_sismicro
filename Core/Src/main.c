/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "funcoes_SPI_display.h"
#include "stm32f1xx_hal_conf.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DT_ADC 199           // dt = 199 ms, sample rate 5 samples/seg
#define DT_VARRE_DISP 6      // dt = ~7ms p/ varrer display (142 vz/s)
#define DT_REFRESH 89        // DT = ~90ms p/ nova requisição de dados
#define DT_DISP_MODO1 3999   // modo 1 muda display a cada 4000 ms
#define DT_DISP_MODO2 1999   // modo 2 muda display a cada 2000 ms
#define DT_LEDS 199          // intervalo tempo para piscar leds
#define DT_CRONO 99          // dt = 99 ms, entra no 100 e ajusta crono
#define DT_DEB_LOW 200       // tempo debouncing fall
#define DT_DEB_HIGH 100      // tempo debouncing rising
#define NDGDISP 4            // tem NDGDISP displays no painel

#define MD_CRONO 0           // cronômetro =0 incrementa, =1 decrementa
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/*  Algumas STRINGS de MENSAGENS desse projeto (Prof. J Ranhel):
    "rqcrn" requisita o envio do valor do CRONOMETRO do outro kit para o seu
    "rqadc" req o valor de tensao em Volts, do ADC do outro kit para o seu
    "rqbzz" req que o colega toque o buzzer (5x 200ms por 200 ms OFF)
    "XXXXa" seu kit recebe/envia os digitos do ADC em ASCII + 'a'
    "XXXXc" seu kit recebe/envia os digitos do cronometro em ASCII + 'c'
    A mensagem é montada de BufOUT[0] até MSD BufOUT[3], c/ 5 caracteres.
    Para receber um valor é necessário requisitá-lo.
    Interv entre reqs(DT_REFRESH) deve ser <100ms p/ captar déc seg do crono*/
#define REQCRN "rqcrn"       // define a string para pedir dado crono ext
#define REQADC "rqadc"       // define a string para pedir dado ADC ext
#define REQBZZ "rqbzz"       // define a string para solicitar servico
/*  macro = função que copia uma STRING(n chars) p/ o BufOUT[](n items) */
#define STR_BUFF(str) do { \
    const char *src = str; \
    strncpy((char *)(BufOUT), src, sizeBuffs); \
} while (0)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// buffers para entrada e saida de dados via USART
uint8_t BufOUT[5];                     // def buffer OUT
uint8_t BufIN[5];                      // def buffer IN
int8_t DspHex[] = {16,16,16,16};       // vetor val display (se=16 display off)
size_t sizeBuffs = sizeof(BufOUT);     // tamanho dos buffers - usa geral
// os vetores seguintes tem idx[0] = digito menos significativo no display
int8_t Crono[] = {0,0,0,0};            // vetor com vals dec do cronometro
int8_t ValAdc[] = {0,0,0,0};           // vetor com vals decimais do ADC
int8_t ExCrono[] = {0,0,0,0};          // vetor externo vals dec do crono
int8_t ExValAdc[] = {0,0,0,0};         // vetor externo vals dec do ADC
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int qualMSG = 0,                       // var que controla qual MSG enviar
    modo = 0 ;                         // 'modo' do display (o que está mostrando)
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // definir os estados da FSM que controla 'debouncing' dos botões
  static enum {DB_NORMAL, DB_FALL, DB_LOW, DB_RISING}
  sttBTA1 = DB_NORMAL,                 // var estado FSM debouncing A1
  sttBTA2 = DB_NORMAL,                 // var estado FSM debouncing A2
  sttBTA3 = DB_NORMAL;                 // var estado FSM debouncing A3
  uint32_t tNow = 0,                   // tempo que representa agora
         tIN_A1 = 0,                   // salva tempo debouncing A1
         tIN_A2 = 0,                   // salva tempo debouncing A2
         tIN_A3 = 0,                   // salva tempo debouncing A3
      tIN_varre = 0;                   // salva tempo última varredura
  int flgPending = 0;                  // flag se tem uma msg pendente p/ envio
  uint8_t ptoDec = 0;  // qual PTO ligar? (ex:0xA=>1010 => 1000=MSD + 0010=DG2)
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_ADC1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */

  reset_pinos_emula_SPI ();            // começa SPI emulada com pinos = 'high'
  // para a UART começar a receber os dados pelo RX
  HAL_UART_Receive_IT(&huart1, BufIN, sizeBuffs);// reativa RECEPÇÃO por interrupção

  // COMECAR O DISPLAY TODO LIGADO
  for(int i=0; i<4; i++)
  {
	  mostrar_no_display("8888", 0x0F);
  }

  // COMECAR O CRONOMETRO CRESCENTE
  // LIGA O ADC PINO PA0 5S/s

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  

  // esse while consome: ~6.6uS, ~20uS c/ callbacks UART, e ~50uS em IRQs ticks
  // para enviar um buffer 5 bytes na UART ~400 uS
  while (1)
  {
    /* USER CODE END WHILE */
	// VERIFICACAO DOS BOTÕES

	// TRATAMENTO DOS BOTÕES
	// Definindo os estados
	typedef enum {
	    ESTADO_INICIAL,
	    ESTADO_NORMAL,
	    ESTADO_ALTERNANCIA_A1,
	    ESTADO_ALTERNANCIA_A2,
	    ESTADO_ACIONAR_BUZZER
	} estado_t;
	
	estado_t estadoAtual = ESTADO_INICIAL;
	
	unsigned long tempoAnterior = 0;
	unsigned long intervalo = 4000; // Intervalo para alternância de A1 (4s)
	unsigned long intervaloA2 = 2000; // Intervalo para alternância de A2 (2s)
	
	bool exibirAmigo = false; // Controla se exibe os valores do amigo
	bool exibirCronometro = true; // Controla se exibe o cronômetro ou ADC
	
	int contadorAlternanciaA2 = 0; // Controla a alternância de 4 valores no A2
	switch(estadoAtual) {
        case ESTADO_INICIAL:
            // Display todo ligado e cronômetro disparado
            iniciarCronometro();
            estadoAtual = ESTADO_NORMAL;
            break;
		
	case ESTADO_NORMAL:
            break;

        case ESTADO_ALTERNANCIA_A1:
            if (millis() - tempoAnterior >= intervalo) {
                // Alternar entre cronômetro e ADC
                exibirCronometro = !exibirCronometro;
                tempoAnterior = millis();
            }
            
            // Exibir o valor alternado
            if (exibirCronometro) {
                exibirCronometroLocal();
            } else {
                exibirADC();
            }
            break;

        case ESTADO_ALTERNANCIA_A2:
            if (millis() - tempoAnterior >= intervaloA2) {
                // Alterar para o próximo valor a ser exibido no ciclo A2
                contadorAlternanciaA2 = (contadorAlternanciaA2 + 1) % 4; // Ciclar entre 0, 1, 2, 3
                tempoAnterior = millis();
            }

            // Exibir os valores de acordo com o contador
            switch (contadorAlternanciaA2) {
                case 0:
                    exibirCronometroLocal();
                    break;
                case 1:
                    exibirADC();
                    break;
                case 2:
                    exibirCronometroAmigo();
                    break;
                case 3:
                    exibirADCAmigo();
                    break;
            }
            break;

        case ESTADO_ACIONAR_BUZZER:
            if (digitalRead(A3) == HIGH) {
                acionarBuzzer();
            }

            // Voltar para o estado normal
            estadoAtual = ESTADO_NORMAL;
            break;

        default:
            estadoAtual = ESTADO_INICIAL;
            break;
    }
}

	  
	// - A1 (PA1): MOSTRA CRONOMETRO E ADC A CADA 2v/4s
	// - A2 (PA2): REQUISITA DADOS DA PLACA DO AMIGO
	//			   COMEÇA A MOSTRAR OS NOSSOS VALORES E OS VALORES DO AMIGO 4v/2s
	//			   SE PRESSIONAR A1 VOLTA A MOSTRAR SO OS NOSSOS
	// - A3 (PA3): SOLICITAR ACIONAMENTO DO BUZZER DA PLACA DO AMIGO


	// MAQUINA EXIBICACAO DISPLAY
	  
	  
	// MAQUINA PISCA LEDS

	// MAQUINA TOCA BUZZER

    tNow = HAL_GetTick();          // obter o tempo atual dessa iteracao

    // TRANSFORMAR ISSO TUDO PRA FUNÇÕES
		// tarefa #1: polling PA1 e fazer debouncing fall e rise
		switch(sttBTA1) {
		case DB_NORMAL:                    // se bt sem acionar muito tempo - normal
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 0) {
			tIN_A1 = tNow;                 // salva tempo do fall
			estadoAtual = ESTADO_ALTERNANCIA_A1;
	// aqui entra o que acontece quando A1 é ativado
			sttBTA1 = DB_FALL;             // prox estado 'FALL'
		  }
		  break;
		case DB_FALL:
		  if ((tNow-tIN_A1)>DT_DEB_LOW) {  // tempo do debouncing L?
			sttBTA1 = DB_LOW;              // prox estado 'LOW'
		  }
		  break;
		case DB_LOW:
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 1) {
			tIN_A1 = tNow;                 // salva tempo da subida
			sttBTA1 = DB_RISING;           // prox estado 'DB_RISING'
		  }
		  break;
		case DB_RISING:
		  if ((tNow-tIN_A1)>DT_DEB_HIGH) { // tempo debouncing H?
			sttBTA1 = DB_NORMAL;           // prox estado 'DB_NORMAL'
		  }
		  break;
		}

		// tarefa #2: polling PA2, debouncing fall e rise
		switch(sttBTA2) {
		 case DB_NORMAL:                    // se bt sem acionar muito tempo - normal
		   if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0) {
			 tIN_A2 = tNow;                 // salva tempo do fall
			 estadoAtual = ESTADO_ALTERNANCIA_A2;
	// aqui entra o que acontece quando A2 é ativado
			 sttBTA2 = DB_FALL;             // prox estado 'FALL'
		   }
		   break;
		 case DB_FALL:
		   if ((tNow-tIN_A2)>DT_DEB_LOW) { // tempo do debouncing L?
			 sttBTA2 = DB_LOW;             // prox estado 'LOW'
		   }
		   break;
		 case DB_LOW:
		   if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1) {
			 tIN_A2 = tNow;                 // salva tempo da subida
			 sttBTA2 = DB_RISING;           // prox estado 'DB_RISING'
		   }
		   break;
		 case DB_RISING:
		   if ((tNow-tIN_A2)>DT_DEB_HIGH) { // tempo debouncing H?
			 sttBTA2 = DB_NORMAL;           // prox estado 'DB_NORMAL'
		   }
		   break;
		 }

		// tarefa #3: polling PA3, debouncing fall e rise
		switch(sttBTA3) {
		 case DB_NORMAL:                    // se bt sem acionar muito tempo - normal
		   if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0) {
			 tIN_A3 = tNow;                 // salva tempo do fall
			 estadoAtual = ESTADO_ACIONAR_BUZZER;
	// aqui entra o que acontece quando A3 é ativado
			 sttBTA3 = DB_FALL;             // prox estado 'FALL'
		   }
		   break;
		 case DB_FALL:
		   if ((tNow-tIN_A3)>DT_DEB_LOW) { // tempo do debouncing L?
			 sttBTA3 = DB_LOW;             // prox estado 'LOW'
		   }
		   break;
		 case DB_LOW:
		   if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1) {
			 tIN_A3 = tNow;                 // salva tempo da subida
			 sttBTA3 = DB_RISING;           // prox estado 'DB_RISING'
		   }
		   break;
		 case DB_RISING:
		   if ((tNow-tIN_A3)>DT_DEB_HIGH) { // tempo debouncing H?
			 sttBTA3 = DB_NORMAL;           // prox estado 'DB_NORMAL'
		   }
		   break;
		 }

    // tarefa obrigatória #1: tem que mandar dados? Montar a mensagem
    // ( não precisa ser aqui! ) exemplo de como montar uma msg:
    //   qualMSG = 1;                  // mudar a var qualMSG
    //   STR_BUFF(REQCRN);             // preenche bufOUT c/ "rqcrn"
    //   ou montar o BufOUT[] manualmente com val do Crono local
    //   BufOUT[0] = conv_num_ASC(Crono[0]); // converter num para ASCII
    //   BufOUT[1] = conv_num_ASC(Crono[1]);
    //   BufOUT[2] = conv_num_ASC(Crono[2]);
    //   BufOUT[3] = conv_num_ASC(Crono[3]);
    //   BufOUT[4] = 'c';              // ASCII caractere 'c'

    // tarefa obrigatória #2: tem MSG pra enviar? enviar via UART
    if (qualMSG != 0 || flgPending != 0) {
      // se UART nao estiver com uma transissao em andamento:
      if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX) {
        HAL_UART_Transmit_IT(&huart1, BufOUT, sizeBuffs); // transmite a MSG
        flgPending = 0;                // deslg flag pendencia
        qualMSG = 0;                   // volta qualMSG p/ zero
      } else {
        flgPending = 1;                // liga flag pendencia transmissao
      }
    }

    // tarefa obrigatória #3: fazer a varredura nos displays
    if ((tNow-tIN_varre) > DT_VARRE_DISP) { // tempo de mudar display?
      tIN_varre = tNow;                // registra tempo última varredura
// transferir Crono[],ValAdc[],ExCrono[] ou ExValAdc[] p/ vetor DspHex[]
// exemplo para crono:
      for (int i=0; i<NDGDISP; i++) DspHex[i] = Crono[i]; // DspHex = Crono

// ajustar ponto decimal

// chama fn varredura passando o ponteiro de DspHex (tam fixo)
      mostrar_no_display(DspHex, ptoDec); // chama fn mostrar no display
    }
  }  // -- fim do loop infinito while(1)
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB12 PB13 PB14
                           PB15 PB5 PB6 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// fn de callback do Systick, ajusta crono e ADC
void Ajusta_Crono_ADC()
 {
	static uint16_t conta = 0;           // conta para ajustar qdo alterar o crono
	static uint16_t conta_ADC = 0;       // conta p/ ajustar qdo disparar ADC
	// CRONOMETRO UP/DOWN:
	if (conta >= DT_CRONO)               // quando conta == DT_CRONO (ms)
	{
		conta = 0;                         // retorna conta para zero
		if (MD_CRONO == 0) {               // MD_CRONO = 0 incrementa o cronômetro
			++Crono[0];                      // inc decimo de segundos
			if (Crono[0] > 9) {              // se > 9
				Crono[0] = 0;                  // volta p/ zero
				++Crono[1];                    // inc unidade de segundo
				if (Crono[1] > 9) {            // se > 9
					Crono[1] = 0;                // volta p/ zero
					++Crono[2];                  // inc dezena de segundos
					if (Crono[2] > 5) {          // se > 5
						Crono[2] = 0;              // volta p/ zero
						++Crono[3];                // inc minutos
						if (Crono[3] > 9) {        // se > 9
							Crono[3] = 0;            // volta p/ zero
		} } } } } else {                   // MD_CRONO = 1 decrementa o cronômetro
			--Crono[0];                      // dec decimo de segundos
			if (Crono[0] < 0) {              // se < 0
				Crono[0] = 9;                  // volta p/ 9
				--Crono[1];                    // dec unidade de segundo
				if (Crono[1] < 0) {            // se < 0
					Crono[1] = 9;                // volta p/ 9
					--Crono[2];                  // dec dezena de segundos
					if (Crono[2] < 0) {          // se < 0
						Crono[2] = 5;              // volta p/ 5
						--Crono[3];                // dec minutos
						if (Crono[3] < 0) {        // se < 0
							Crono[3] = 9;            // volta p/ 9
		} } } } } } else {
		++conta;
	}                 // inc contador de vezes

	// dispara o ADC ?
	if (conta_ADC >= DT_ADC) {           // se atingiu a qtde ms p/ disparar ADC
		conta_ADC = 0;                     // volta contador ADC p/ zero
		HAL_ADC_Start_IT(&hadc1);          // dispara ADC p/ conversao por IRQ
	} else {
		++conta_ADC;
	}              // se nao, apenas inc o conta

}

// fn que atende ao callback da ISR do conversor ADC1
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  uint16_t val_adc = 0;                // define var para ler ADC
  if(hadc->Instance == ADC1) {         // se veio ADC1
    val_adc = HAL_ADC_GetValue(&hadc1);// capta valor adc
    // converter o valor lido em valores hexa p/ display
    int miliVolt = val_adc*3300/4095;
    int uniADC = miliVolt/1000;
    int decADC = (miliVolt-(uniADC*1000))/100;
    int cnsADC = (miliVolt-(uniADC*1000)-(decADC*100))/10;
    int mlsADC = miliVolt-(uniADC*1000)-(decADC*100)-(cnsADC*10);
    ValAdc[3] = uniADC;         // dig mais significativo
    ValAdc[2] = decADC;
    ValAdc[1] = cnsADC;
    ValAdc[0] = mlsADC;
  }
}

// fn que atende ao callback da ISR quando RECEBE dado pela UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // o que veio na UART?
  // se veio "rxxxx", o cliente solicitou o dado ou buzzer
  if (BufIN[0]=='r') {                 // então veio um request...

// vocês vão fazer o tratamento do que veio pela UART

  }

  // OBRIGATÓRIO: é preciso voltar a escutar a UART continuamente...
  HAL_UART_Receive_IT(&huart1, BufIN, sizeBuffs);// reativa RECEPÇÃO por interrupção
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
