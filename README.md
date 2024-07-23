# Projeto de laboratório da disciplina Sistemas de Medição - UFMG

Este projeto tem o objetivo de fazer a medição de 4 sinais relacionados ao funcionamento de uma lâmpada LED: tensão, corrente, iluminância, e temperatura. O circuito de condicionamento de sinais possui as saídas com tensão de 0 a 5V, conectadas respectivamente nos pino A0 à A3 do Arduino UNO.

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

> Os valores de offset e ganho devem ser estimados de acordo com os valores reais do circuito de condicionamento

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

> Neste trecho do código, é feita toda a configuração dos registradores

> A frequência de amostragem geral do ADC é dada pelo seguinte cálculo: Frequência oscilador principal Arduino / Divisor de clock definido pelo resgitrador TCCR0B / (Contador definido pelo resgitrador OCR0A +1)
>
> Neste caso, o resultado é: fADC = 16MHz/64/52 = 4807,7Hz

>Caso não houvesse o fator de ampliação para tensão e corrente, a frequência de amostragem de cada canal seria fADC/4. Porém, com a estratégia de amplificação, ela se torna: fADC / 4 * [1+ 1- (fator de amostragem/100)]
>
>Neste caso, o resultado é: fTensão = fCorrente = 4807,7 / 4 * 1,9 = 2283.65Hz

>Para a iluminância e temperatura, o valor será 4807,7/4/10 = 120,19Hz

>**Para o ajuste da freqûencia, o único registrador que deve ser alterado é o OCR0A**

## Rotina de interrupção do ADC

    //ADC interrupt service routine
    
    ISR(ADC_vect)
    
    {
    
	    int sample, CH; //Resultado da conversão e Canal selecionado
	    
	    static  int counter = 0; //Controls the number of samples
	    
	    static  float tensao, corrente, power; //Varíaveis para cálculo da energia em tempo real, não são utilizadas na montagem do vetor de dados
	    
	    static  int samplingCounter = 0; //Contador de amostras
	    
	    static  int lastLux, lastTemp; //Memória de últimas leituras de temperatura e luminância
	    
	      
	    
	    //Read the latest sample
	    
	    sample = ADCL; //Read the lower byte
	    
	    sample += ADCH<<8; //Read the upper byte
	    
	      
	    
	    ///Real time routine
	    
	    CH = ADMUX & 0x0F; //Salva na varíavel CH qual é o canal vigente do ADC
	    
	      
	    
	    if(CH == 0){ //Salva valor de tensão caso o canal seja 0
	    
		    tensao = getVoltage(sample);
	    
	    }
	    
	    if(CH == 1){ //Salva valor de corrente caso o canal seja 1, calcula o valor de potência intantânea e incrementa o contador de energia
	    
		    corrente = getCurrent(sample);
		    
		    power = sq(tensao * corrente);
		    
		    power = sqrt(power);
		    
		      
		    
		    energy += power / fAmostragem / 3600;
	    
	    }
	    
	    //Verify if it is time to transmit
	    
	    if  (sendStatus == false){ //Controla o salvamento das leituras no vetor de dados
	    
		    dataVector[CH][counter] = sample; //Store the data
		    
		    //Controla a correta atualização do contador do vetor, e do salvamento dos dados
		    
		    if(samplingCounter < sampleFactor){ //Atualização do contador do vetor caso esteja medindo apenas o canal 0 e 1
		    
			    if  (CH+1 >= N-2){
			    
			    counter++; //update the number of samples
			    
			    }
		    
			    dataVector[2][counter] = lastLux; //Repete os valores da última medição para luminância
			    
			    dataVector[3][counter] = lastTemp; //Repete os valores da última medição para temperatura
		    
		    }
		    
		    if(samplingCounter == sampleFactor+1){ //Salvamento da última medição de luminância
		    
			    lastLux = sample;
	    
		    }
		    
		    if(samplingCounter == sampleFactor+2){ //Salvamento da última medição de temperatura, e atualização do contador do vetor caso esteja medindo o canal 3
		    
			    lastTemp = sample;
			    
			    counter++;
		    
		    }
		    
		      
		    
		    // Reset do contador do vetor após preenchimento completo, e ativação da flag de envio
		    
		    if  (counter == tam){
		    
			    counter = 0;
			    
			    sendStatus = true;
		    
		    }
	    
	    }
	    
	      
	    
	    //Organiza quais os próximos canais a serem lidos de acordo com o número de medições realizadas
	    
	    if(samplingCounter < sampleFactor){ //Caso o número de amostras seja menor que sampleFactor, alterna-se a leitura entre os canais 0 e 1
	    
		    if  (++CH < N-2)
		    
		    ADMUX += 1; //If not, go to the next channel
		    
		    else{
		    
		    ADMUX &= 0xF0; //If so, turn to channel 0
		    
		    }
	    
	    }
	    
	    if(samplingCounter == sampleFactor){ //Caso o número de amostras seja igual a sampleFactor, troca para canal 2
	    
		    ADMUX &= 0xF0; //If so, turn to channel 2
		    
		    ADMUX += 2;
	    
	    }
	    
	    if(samplingCounter == sampleFactor+1){ //Caso o número de amostras seja igual a sampleFactor+1, troca para canal 3
	    
		    ADMUX &= 0xF0; //If so, turn to channel 3
		    
		    ADMUX += 3;
	    
	    }
	    
	    if(samplingCounter >= sampleFactor+2){ //Caso o número de amostras seja igual a sampleFactor+2, troca para canal 0 novamente, e zera o contador de amostras
		    
		    ADMUX &= 0xF0; //If so, turn to channel 0 and
		    
		    samplingCounter = 0;
	    
	    }
	    
	      
	    
	    samplingCounter++; //Atualiza o contador de medições
    
    }

>É feita a medição constante dos sinais nesta rotina, independente da situação do flag sendStatus. A medição da tensão e corrente é alteranada, e após um número de X de aquisições, é feita uma medição de iluminância e temperatura. 

>O cálculo de energia em tempo real é feito ao se coletar uma amostra de tensão e uma de corrente

>Apenas quando o flag sendStatus é falso, o vetor de dados é preenchido. Entretanto, as aquisições continuam sendo feitas independente disto. Os valores de iluminância e temperatura são repetidos no vetor de dados com o último valor medido, até que uma nova aquisição dessas grandezas seja feita.

## Rotina de interrupção do timer

    //TIMER interrupt service routine - USADO PARA CRIAR UM SINAL DE TESTE NO PINO LED_BUILTIN
    
    ISR(TIMER0_COMPA_vect)  {
    
	    //just clears the flag, as needed, but can be used for secondary ADC actions
	    
	    //In this example just toggles the digital output 7 at timer_clk/2 Hz
	    
	    
	    static bool toogle = true;
	    
	    static int counter = 0;
	    
	      
	    
	    if (counter == 60){
	    
		    counter = 0;
	    
		    if (toogle) {
		    
			    digitalWrite(LED_BUILTIN, HIGH);
			    
			    digitalWrite(7, HIGH);
			    
			    toogle = false;
		    
		    } else {
		    
			    digitalWrite(LED_BUILTIN, LOW);
			    
			    digitalWrite(7, LOW);
			    
			    toogle = true;
		    
		    }
	    
	    }
	    
	    else
	    
		    counter++;
    }
    
>Rotina necessária para limpar o flag do timer, porém pode ser utilizada também para gerar um sinal de teste no pino 7 

## Funções de processamento

    float  getVoltage(int  data){ // Converte valor do ADC usando modelo de medição para tensão
    
	    float voltage = (float)data;
	    
	    voltage = (voltage * (5.0/1023) - vOffset) * vGain;
	    
	    return voltage;
    
    }
    
    float  getCurrent(int  data2){ // Converte valor do ADC usando modelo de medição para corrente
    
	    float current = (float)data2;
	    
	    current = ((current * (5.0/1023.0)) - IOffset) * IGain;
	    
	    return current;
    
    }
    
    float  getLux(int  lux){ // Converte valor do ADC usando modelo de medição para luminância
    
	    return  float(lux);
    
    }
    
    float  getTemp(int  data){ // Converte valor do ADC usando modelo de medição para temperatura
    
	    float temperature = (float)data;
	    
	    temperature = (temperature * (5.0/1023.0)) * 100.0;
	    
	    return temperature;
    
    }
    
      
    
    void  processData(){ //Calcula a tensão RMS, corrente RMS, potência ativa, e energia
    
	    activePower = 0.0;
	    
	    voltageRMS = 0.0;
	    
	    currentRMS = 0.0;
	    
	    for(int k=0;k<tam;k++){
	    
		    activePower += sq(getVoltage(dataVector[0][k]) * getCurrent(dataVector[1][k]));
		    
		    voltageRMS += sq(getVoltage(dataVector[0][k]));
		    
		    currentRMS += sq(getCurrent(dataVector[1][k]));
	    
	    }
	    
	    voltageRMS = sqrt(voltageRMS/tam);
	    
	    currentRMS = sqrt(currentRMS/tam);
	    
	    activePower = sqrt(activePower/tam);
    
    
    }
>As funções getVoltage, getCurrent, getLux e getTemp recebem um valor de leitura do ADC entre 0 a 1023, e retornam com as valores nas respectivas unidades: volts, amperes, lux e graus celsius. Os valores de ganho e offset podem ser ajustados no início do código

>A função processData calcula a tensão e corrente RMS, e a potência ativa, utilizando o vetor de dados. Os valores são gravados nas variáveis globais respectivas.

## Void loop

    //Loop to send data to the main computer
    
    void  loop()
    
    {
    
	    int i,j;
	    
	    char cmd;
	    
	    //Verify if it is time to transmit data
	    
	    if  (sendStatus == true){
	    
		    //Wait for the command from the host
		    
		    cmd = 'x';
		    
		    while  (cmd != 'a'){ //Comando de letra "a" deve ser enviado pelo terminal
		    
			    if  (Serial.available() > 0)
			    
			    cmd = Serial.read();
		    
		    }
		    
		    processData(); //Chama a função que calcula os valores RMS, potência e energia
		    
		    //Transmite dados de forma de onda, convertendo o valor para as respectivas unidades
		    
		    for(i=0; i<tam; i++){
		    
			    for(j=0; j<(N); j++){
			    
				    if(j==0){
				    
					    Serial.print(getVoltage(dataVector[j][i]));
				    
				    }
				    
				    if(j==1){
				    
					    Serial.print(getCurrent(dataVector[j][i]));
				    
				    }
				    
				    if(j==2){
				    
					    Serial.print(getLux(dataVector[j][i]));
				    
				    }
				    
				    if(j==3){
				    
					    Serial.print(getTemp(dataVector[j][i]));
				    
				    }
				    
				    Serial.print("\t");
				    
				    }
			    
			    Serial.println();
		    
		    }
		    
		    //Transmite dados calculados
		    
		    Serial.print("Voltage RMS=");
		    
		    Serial.println(voltageRMS);
		    
		    Serial.print("Current RMS=");
		    
		    Serial.println(currentRMS);
		    
		    Serial.print("Active Power=");
		    
		    Serial.println(activePower);
		    
		    Serial.print("Energy=");
		    
		    Serial.println(energy);
		    
		    //Restart acquisition
		    
		    noInterrupts();
		    
		    sendStatus = false;
		    
		    interrupts();
		    
		}
    
    }

>No loop, espera-se o comando da letra "a" no terminal para que os dados sejam processados e enviados.
>Após um envio, o flag sendStatus fica falso e um novo vetor de dados é adquirido.

## Resultados

