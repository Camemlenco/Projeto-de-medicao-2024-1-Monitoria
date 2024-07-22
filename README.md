# Projeto de laboratório da disciplina Sistemas de Medição - UFMG

Este projeto tem o objetivo de fazer a medição de 4 sinais no funcionamento de uma lâmpada LED: tensão, corrente, iluminância, e temperatura. O circuito de condicionamento de sinais possui as saídas com tensão de 0 a 5V, conectadas respectivamente nos pino A0 à A3 do Arduino UNO.

O Arduino processa os dados para exibir na saída do terminal serial as seguintes informações: 

 - Forma de onda de tensão, corrente, iluminância e temperatura
 - Tensão e corrente RMS
 - Potência ativa em W
 - Energia em Wh

Os dados são transmitidos ao se enviar o caracter "a" pelo terminal serial

# Funcionamento do código

Os trechos do código serão descritos por partes

## Variáveis Globais

    #define  N  4 //Number of channels

    #define  tam  180 //Size of data buffers
    
    #define  fAmostragem  2283.65 //Frequência de amostragem que deve ser calculada manualmente para a configuração realizada
    
    #define  sampleFactor  19 //Fator de redução da frequencia de amostragem da luminância e temperatura(deve ser calculado x2-1)
    
      
    //Define valores de ganho e offset dos sinais de tensão e corrente
    #define  vOffset  2.5
    
    #define  vGain  100
    
    #define  IOffset  2.5
    
    #define  IGain  0.1
    
      
    
    bool sendStatus = false; //Flag to start data processing
    
    int  dataVector[N][tam]; //Data vectors for each channel
    
    float energy = 0.0; //Stores the total energy consumed
    
    float activePower = 0.0; //Stores the calculated active power
    
    float voltageRMS = 0.0; //Stores the calculated voltage RMS
    
    float currentRMS = 0.0; //Stores the calculated current RMS

> A frequência de amostragem deve ser calculada manualmente e preenchida, de acordo com a configuração posterior do timer

> O sampleFactor corresponde ao fator de redução de frequência de amostragem da temperatura e iluminância. Para obter um fator de 10x, ou seja, ter uma uma amostragem de corrente e tensão 10 vezes maior, sampleFactor será igual a (10*2)-1 = 19

## Void setup

    //Initialization
    
    void  setup()
    
    {
    
	    //Set serial port configuration and establish communication
	    
	    Serial.begin(115200);
	    
	      
	    
	    //Configure the ADC pins
	    
	    pinMode(A0, INPUT);
	    
	    pinMode(A1, INPUT);
	    
	    pinMode(A2, INPUT);
	    
	    pinMode(A3, INPUT);
	    
	    pinMode(7, OUTPUT); // configura o pino como saída
	    
	    pinMode(LED_BUILTIN, OUTPUT);
	    
	    //---------- Configure the ADC --------------------------------
	    
	    //Configure the ADC mux
	    
	    ADMUX = 0x40; //MUX[3:0]=0000 -> select analog channel 0,
	    
	    //ADLAR=0 -> the ADC results are right adjusted
	    
	    //REFS[1:0]=01, set voltage reference do AVcc
	    
	      
	    
	    //Configure the ADC prescaler and enable interrupts
	    
	    ADCSRA = 0x08 | 0x20 | 0x05; //ADIE=1, enable ADC interrupts *****************************************************
	    
	    //ADATE=1, enable auto-trigger
	    
	    //ADPS[2:0]=101, set prescaler to 32 -> ADC_clk = 16e6/32 = 500kHz
	    
	      
	    
	    //Configure the ADC trigger source
	    
	    ADCSRB = 0x03; //ACME=0, disable analog comparator multiplexer
	    
	    //ADTS[2:0]=011 -> trigger source = Timer/Counter0 compare match A
	    
	    //Disable the ADC digital input buffers
	    
	    DIDR0 = 0xFF;
	    
	      
	    
	    //-------- Configure the timer/counter 0 --------------------------
	    
	    //ATENÇÃO: a ordem dos comandos altera o comportamento do sistema,
	    
	    //portanto procure manter a definida abaixo.
	    
	    TCCR0A = 0x02; //COM0A[1:0] = 00, COM0B[1:0] = 00 -> normal port operation, OC0A, OC0B disconnected.
	    
	    //WGM0[2:0] = 010 -> CTC mode (clear timer/counter on compare)
	    
	    TCCR0B = 0x00; //FOC0A, FOC0B = 0 -> force output compare A, B = 0
	    
	    //CS0[2:0] = 000 -> no clock source (timer/counter stopped)
	    
	    TCNT0 = 0; //Reset the counter 0
	    
	    //////////////////AJUSTE DE FREQÛENCIA DE AMOSTRAGEM AQUI/////////////////////////
	    
	    OCR0A = 51; //Set the compare register A -> Divide o clock por OCR0A+1, para obter frequência de amostragem -> f=250K/(OCR0A+1)
	    
	    /////////////////////////////////////////////////////////////////////////////
	    
	    OCR0B = 0; //Reset the compare register B
	    
	    TIMSK0 = 0x02; //OCIE0A = 1 -> timer/counter 0 output compare A match interrupt enable.
	    
	      
	    
	    //--------- Start acquisition ------------------------------------
	    
	    //Enable global interrupts
	    
	    SREG |= 0x80; //Enable global interrupts
	    
	    //Enable the AD converter
	    
	    ADCSRA |= 0x80; //ADEN=1, enable AD converter
	    
	    //Start the timer
	    
	    TCCR0B = 0x03; //CS0[2:0] = 011 -> clkIO/64 = 250kHz *****************************************************
    
    }
