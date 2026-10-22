#include "project.h"
#include "fft_application.h"

#define STATE_IDLE          0
#define STATE_SAMPLING      1
#define STATE_UART_TRANSFER 2

/* --- Global Variables --- */
uint16_t adc_array[1024];           // Array to store 1024 16-bit samples
uint16_t sample_index = 0;         
uint8_t  current_state = STATE_IDLE; 
uint8_t  transfer_count = 0;        
uint8_t  data_sent_flag = 0;  // Prevents spamming the UART in a non-blocking state

int32    fft_array[2*1024];     // 2*N, interleaved re/im


int main(void)
{
    CyGlobalIntEnable; /* Enable global interrupts. */

    // 1. Start all hardware blocks
    UART_1_Start();
   // WaveDAC8_1_Start(); 
    ADC_DelSig_1_Start();
    
    // 2. Turn on the ADC continuous conversion engine
    ADC_DelSig_1_StartConvert();

    for(;;)
    {
        switch(current_state)
        {
            case STATE_IDLE:
  
                LED_RED_Write(1);
                LED_GREEN_Write(0);
                LED_ORANGE_Write(0);
                
 
                if (BUTTON_Read() == 1) 
                {
                    transfer_count = 0;
                    sample_index = 0;
                    data_sent_flag = 0; // Reset the UART flag
                    current_state = STATE_SAMPLING;
                }
                break;

            case STATE_SAMPLING:
                LED_ORANGE_Write(1);
                LED_GREEN_Write(0);
                LED_RED_Write(0);


                if (sample_index < 1024)
                {
                    if(ADC_DelSig_1_IsEndConversion(ADC_DelSig_1_RETURN_STATUS))
                    {
                        adc_array[sample_index] = ADC_DelSig_1_GetResult16();
                        sample_index++;
                    }
                } 
     
   
   
                else 
                {
                    // 1. Check if ANY data is in the buffer
                    if (UART_1_GetRxBufferSize() > 0)
                    {
   
                        if (UART_1_ReadRxData() == 's') 
                        {
         
                            UART_1_ClearRxBuffer(); 

                            fft_app(adc_array, fft_array, 1024); 
                            
                            current_state = STATE_UART_TRANSFER;
                        }
                        else
                        {
  
                            UART_1_ClearRxBuffer(); 
                        }
                    }
                }
                break;

            case STATE_UART_TRANSFER:
 
                if (data_sent_flag == 0)
                {
                    UART_1_PutArray((uint8_t *)adc_array, 2048);
                    UART_1_PutArray((uint8_t *)fft_array, 2*1024*4);   // 8192 bytes
                    data_sent_flag = 1; // Mark as sent
                }

                // Non-blocking wait for handshake
                if (UART_1_GetChar() == 'o') 
                {
  
                    transfer_count++; 
                    data_sent_flag = 0; // Reset flag for the next cycle

  
                    if (transfer_count >= 10) 
                    {
                         
                        LED_ORANGE_Write(0);
                        LED_GREEN_Write(1);
                        LED_RED_Write(0);
                        CyDelay(1000);
                        
                        current_state = STATE_IDLE;
                        
                    } 

                    else 
                    {
                        CyDelay(400);
                        sample_index = 0;               
                        current_state = STATE_SAMPLING; 
                    }
                }
                break;
                
            default:
                current_state = STATE_IDLE;
                break;
        }
    }
}