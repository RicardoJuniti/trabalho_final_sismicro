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

//------------------------------------------
// INCLUDES
//------------------------------------------
#include "main.h"
#include "funcoes_SPI_display.h"
#include "stm32f1xx_hal_conf.h"
#include <stdint.h>
#include <stdbool.h>


//------------------------------------------
// DEFINES
//------------------------------------------
#define DT_ADC 				199           // dt = 199 ms, sample rate 5 samples/seg
#define DT_VARRE_DISP		6      // dt = ~7ms p/ varrer display (142 vz/s)
#define DT_REFRESH			89        // DT = ~90ms p/ nova requisição de dados
#define DT_DISP_MODO1 		3999   // modo 1 muda display a cada 4000 ms
#define DT_DISP_MODO2 		1999   // modo 2 muda display a cada 2000 ms
#define DT_LEDS 			199          // intervalo tempo para piscar leds
#define DT_CRONO 			99          // dt = 99 ms, entra no 100 e ajusta crono
#define DT_DEB_LOW			200       // tempo debouncing fall
#define DT_DEB_HIGH 		100      // tempo debouncing rising
#define NDGDISP				4            // tem NDGDISP displays no painel

#define MD_CRONO 			0           // cronômetro =0 incrementa, =1 decrementa

#define MS_INTERVALO_ALTER_A1	4000
#define MS_INTERVALO_ALTER_A2	2000

#define QTDE_TOCA_BUZZER	 		4
#define MS_INTERVALO_TOCA_BUZZER	200


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
#define REQBZZ "rqbzz"       // define a string para solicitar buzzer

/*  macro = função que copia uma STRING(n chars) p/ o BufOUT[](n items) */
#define STR_BUFF(str) do { \
    const char *src = str; \
    strncpy((char *)(BufOUT), src, sizeBuffs); \
} while (0)


//------------------------------------------
// TYPEDEFS
//------------------------------------------

typedef enum
{
	DB_NORMAL,
	DB_FALL,
	DB_LOW,
	DB_RISING

} enum_estado_botoes;

// Estados gerais
typedef enum {
	ESTADO_INICIAL,
	ESTADO_NORMAL,
	ESTADO_ALTERNANCIA_A1,
	ESTADO_ALTERNANCIA_A2,
	ESTADO_ACIONAR_BUZZER

} estado_t;


//------------------------------------------
// LOCAL VARIABLES
//------------------------------------------

// Handles gerais
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart1;

// buffers para entrada e saida de dados via USART
uint8_t BufOUT[5];                     // def buffer OUT
uint8_t BufIN[5];                      // def buffer IN

int8_t DspHex[] = {16,16,16,16};       // vetor val display (se=16 display off)
size_t sizeBuffs = sizeof(BufOUT);     // tamanho dos buffers - usa geral

// os vetores seguintes tem idx[0] = digito menos significativo no display
int8_t Crono[] = {0,0,0,0};            // vetor com vals dec do cronometro
int8_t ValAdc[] = {0,0,0,0};           // vetor com vals decimais do ADC

int8_t ExCrono[] = {4,3,2,1};          // vetor externo vals dec do crono
int8_t ExValAdc[] = {1,2,3,4};         // vetor externo vals dec do ADC

int32_t qualMSG = 0;                   // var que controla qual MSG enviar
int32_t modo = 0;                      // 'modo' do display (o que está mostrando)

estado_t estadoAtual = ESTADO_INICIAL;

const int8_t array_teste_display[] = {8,8,8,8};

uint32_t tempoAnterior = 0;

bool exibirCronometro = true; // Controla se exibe o cronômetro ou ADC

int32_t contadorAlternanciaA2 = 0; // Controla a alternância de 4 valores no A2


//------------------------------------------
// LOCAL FUNCTIONS PROTOTYPES
//------------------------------------------
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);


//------------------------------------------
// MAIN
//------------------------------------------
int main(void)
{
	enum_estado_botoes sttBTA1 = DB_NORMAL;                 // var estado FSM debouncing A1
	enum_estado_botoes sttBTA2 = DB_NORMAL;                 // var estado FSM debouncing A2
	enum_estado_botoes sttBTA3 = DB_NORMAL;                 // var estado FSM debouncing A3
	uint32_t tNow = 0;                   // tempo que representa agora
	uint32_t tIN_A1 = 0;                   // salva tempo debouncing A1
	uint32_t tIN_A2 = 0;                   // salva tempo debouncing A2
	uint32_t tIN_A3 = 0;                   // salva tempo debouncing A3
	uint32_t tIN_varre = 0;                   // salva tempo última varredura
	uint32_t flgPending = 0;                  // flag se tem uma msg pendente p/ envio

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  // começa SPI emulada com pinos = 'high'
  reset_pinos_emula_SPI ();

  // para a UART começar a receber os dados pelo RX
  HAL_UART_Receive_IT(&huart1, BufIN, sizeBuffs);

  // esse while consome: ~6.6uS, ~20uS c/ callbacks UART, e ~50uS em IRQs ticks
  // para enviar um buffer 5 bytes na UART ~400 uS
  while (1)
  {
	switch(estadoAtual)
	{
        case ESTADO_INICIAL:
        break;

        case ESTADO_ALTERNANCIA_A1:
            if (HAL_GetTick() - tempoAnterior >= MS_INTERVALO_ALTER_A1)
            {
                // Alternar entre cronômetro e ADC
                exibirCronometro = !exibirCronometro;
                tempoAnterior = HAL_GetTick();
            }
        break;

        case ESTADO_ALTERNANCIA_A2:
            if (HAL_GetTick() - tempoAnterior >= MS_INTERVALO_ALTER_A2)
            {
            	tempoAnterior = HAL_GetTick();

                // Alterar para o próximo valor a ser exibido no ciclo A2
                contadorAlternanciaA2++;

                if(contadorAlternanciaA2 > 3)
                {
                	contadorAlternanciaA2 = 0;
                }
            }
        break;

        case ESTADO_ACIONAR_BUZZER:
//            if (digitalRead(A3) == HIGH) {
//                acionarBuzzer();
//            }

            // Voltar para o estado normal
            estadoAtual = ESTADO_NORMAL;
        break;

        default:
            estadoAtual = ESTADO_INICIAL;
        break;
    }


	// MAQUINA EXIBICACAO DISPLAY
      if ((tNow-tIN_varre) > DT_VARRE_DISP)
      {
    	  tIN_varre = tNow;

    	 switch(estadoAtual)
    	 {
    	 	 case ESTADO_INICIAL:
    	 		 // Mostra "8888"
    	 		for (int i=0; i<NDGDISP; i++) DspHex[i] = array_teste_display[i];

    	 		mostrar_no_display(DspHex, 0x0F);
    	     break;

    	 	 case ESTADO_ALTERNANCIA_A1:
    	 		if(exibirCronometro)
    	 		{
    	 			// Mostra nosso crono
					for (int i=0; i<NDGDISP; i++) DspHex[i] = Crono[i];

					mostrar_no_display(DspHex, 0x0A);
    	 		}
    	 		else
    	 		{
    	 			// Mostra ADC
					for (int i=0; i<NDGDISP; i++) DspHex[i] = ValAdc[i];

					mostrar_no_display(DspHex, 0x08);
    	 		}
			 break;

    	 	 case ESTADO_ALTERNANCIA_A2:
    	 		 switch(contadorAlternanciaA2)
    	 		 {
    	 		 	 case 0:
    	 		 		// Mostra nosso crono
						for (int i=0; i<NDGDISP; i++) DspHex[i] = Crono[i];

						mostrar_no_display(DspHex, 0x0A);
    	 			 break;

    	 		 	 case 1:
    	 		 		// Mostra ADC
						for (int i=0; i<NDGDISP; i++) DspHex[i] = ValAdc[i];

						mostrar_no_display(DspHex, 0x08);
    	 		 	 break;

    	 		 	 case 2:
    	 		 		// Mostra crono do amigo
						for (int i=0; i<NDGDISP; i++) DspHex[i] = ExCrono[i];

						mostrar_no_display(DspHex, 0x0A);
    	 		 	 break;

    	 		 	 case 3:
    	 		 		// Mostra ADC do amigo
						for (int i=0; i<NDGDISP; i++) DspHex[i] = ExValAdc[i];

						mostrar_no_display(DspHex, 0x08);
    	 		 	 break;
    	 		 }
    	 	 break;

    	 	 default:
    	     break;
    	 }
      }


	// - A1 (PA1): MOSTRA CRONOMETRO E ADC A CADA 2v/4s
	// - A2 (PA2): REQUISITA DADOS DA PLACA DO AMIGO
	//			   COMEÇA A MOSTRAR OS NOSSOS VALORES E OS VALORES DO AMIGO 4v/2s
	//			   SE PRESSIONAR A1 VOLTA A MOSTRAR SO OS NOSSOS
	// - A3 (PA3): SOLICITAR ACIONAMENTO DO BUZZER DA PLACA DO AMIGO

	// MAQUINA PISCA LEDS

	// MAQUINA TOCA BUZZER

    tNow = HAL_GetTick();

    // TRANSFORMAR ISSO TUDO PRA FUNÇÕES
    // Primeiro botão
		switch(sttBTA1) {
		case DB_NORMAL:
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 0)
		  {
			tIN_A1 = tNow;
			estadoAtual = ESTADO_ALTERNANCIA_A1;

			sttBTA1 = DB_FALL;
		  }
		  break;
		case DB_FALL:
		  if ((tNow-tIN_A1)>DT_DEB_LOW)
		  {
			sttBTA1 = DB_LOW;
		  }
		  break;
		case DB_LOW:
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 1)
		  {
			tIN_A1 = tNow;
			sttBTA1 = DB_RISING;
		  }
		  break;
		case DB_RISING:
		  if ((tNow-tIN_A1)>DT_DEB_HIGH)
		  {
			sttBTA1 = DB_NORMAL;
		  }
		  break;
		}

	// Primeiro botão
		switch(sttBTA2) {
		 case DB_NORMAL:
		   if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0)
		   {
			 tIN_A2 = tNow;
			 estadoAtual = ESTADO_ALTERNANCIA_A2;

			 sttBTA2 = DB_FALL;
		   }
		   break;
		 case DB_FALL:
		   if ((tNow-tIN_A2)>DT_DEB_LOW)
		   {
			 sttBTA2 = DB_LOW;
		   }
		   break;
		 case DB_LOW:
		   if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1)
		   {
			 tIN_A2 = tNow;
			 sttBTA2 = DB_RISING;
		   }
		   break;
		 case DB_RISING:
		   if ((tNow-tIN_A2)>DT_DEB_HIGH)
		   {
			 sttBTA2 = DB_NORMAL;
		   }
		   break;
		 }

	// Terceiro botão
		switch(sttBTA3) {
		 case DB_NORMAL:
		   if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0)
		   {
			 tIN_A3 = tNow;
			 estadoAtual = ESTADO_ACIONAR_BUZZER;

			 sttBTA3 = DB_FALL;
		   }
		   break;
		 case DB_FALL:
		   if ((tNow-tIN_A3)>DT_DEB_LOW)
		   {
			 sttBTA3 = DB_LOW;
		   }
		   break;
		 case DB_LOW:
		   if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1)
		   {
			 tIN_A3 = tNow;
			 sttBTA3 = DB_RISING;
		   }
		   break;
		 case DB_RISING:
		   if ((tNow-tIN_A3)>DT_DEB_HIGH)
		   {
			 sttBTA3 = DB_NORMAL;
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
    if (qualMSG != 0 || flgPending != 0)
    {
      // se UART nao estiver com uma transissao em andamento
      if (HAL_UART_GetState(&huart1) != HAL_UART_STATE_BUSY_TX)
      {
    	  // transmite a MSG
    	  HAL_UART_Transmit_IT(&huart1, BufOUT, sizeBuffs);

    	  flgPending = 0;
		  qualMSG = 0;
      }
      else
      {

    	  flgPending = 1;
      }
    }
  }
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

  ADC_ChannelConfTypeDef sConfig = {0};

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
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

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
