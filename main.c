#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "UART/UART.h"  // Asegúrate de que la ruta sea correcta

// Array para marcar eventos en los botones PD2 a PD7
volatile uint8_t buttonEvent[8] = {0};
// Variable para almacenar el último estado conocido del puerto D
volatile uint8_t lastPortD;

// ISR para el cambio de pines en el puerto D (PCINT2_vect)
// Detecta flanco descendente (de HIGH a LOW) en PD2 a PD7
ISR(PCINT2_vect) {
	uint8_t current = PIND;
	for (uint8_t i = 2; i <= 7; i++) {
		uint8_t mask = (1 << i);
		// Si el pin estaba en HIGH y ahora es LOW, marca el evento
		if ((lastPortD & mask) && !(current & mask)) {
			buttonEvent[i] = 1;
		}
	}
	lastPortD = current;
}

int main(void) {
	// Inicializa el UART a 9600 baudios
	initUART9600();

	// Configura PD2 a PD7 como entradas con resistencias internas pull?up
	DDRD &= ~((1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7));
	PORTD |= ((1 << PD2) | (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7));

	// Habilita la interrupción de cambio de pines para el puerto D (PCIE2)
	PCICR |= (1 << PCIE2);
	// Habilita los PCINT para PD2 a PD7 (PD2 = PCINT18, ..., PD7 = PCINT23)
	PCMSK2 |= (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20) |
	(1 << PCINT21) | (1 << PCINT22) | (1 << PCINT23);

	// Inicializa lastPortD con el estado actual del puerto D
	lastPortD = PIND;

	// Habilita las interrupciones globales
	sei();

	while (1) {
		// Verifica si se detectó algún evento en los botones
		uint8_t anyEvent = 0;
		for (uint8_t i = 2; i <= 7; i++) {
			if (buttonEvent[i]) {
				anyEvent = 1;
				break;
			}
		}

		// Si se detectó algún evento, construye y envía el mensaje
		if (anyEvent) {
			WriteTextUART("");
			uint8_t first = 1;
			for (uint8_t i = 2; i <= 7; i++) {
				if (buttonEvent[i]) {
					if (!first) {
						WriteTextUART(", ");
					}
					// Limpia la bandera para ese botón
					buttonEvent[i] = 0;
					// Envía el identificador del botón, por ejemplo "D2"
					WriteTextUART("");
					char digit = '0' + i;
					writeUART(digit);
					first = 0;
				}
			}
			WriteTextUART("\r\n");
		}
		_delay_ms(50);  // Pequeño retardo para evitar saturar la salida
	}

	return 0;
}
