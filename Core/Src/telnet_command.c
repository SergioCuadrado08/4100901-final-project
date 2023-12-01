
#include "telnet_command.h"
#include "main.h"



uint8_t telnet_receive(USART_TypeDef *USARRx){
	uint8_t key = LL_USART_ReceiveData8(USARRx);
	return key;
}
void telnet_transmit(USART_TypeDef *USARTx, const char *str){

    while (*str != '\0') {
        while (!LL_USART_IsActiveFlag_TXE(USARTx)) {
            // Esperar a que el registro de datos de transmisión esté vacío
        }
        LL_USART_TransmitData8(USARTx, *str++);
    }

    while (!LL_USART_IsActiveFlag_TC(USARTx)) {
        // Esperar a que se complete la transmisión
    }
}
