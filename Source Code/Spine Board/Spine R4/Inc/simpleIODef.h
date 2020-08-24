/*
 * simpleIODef.h
 *
 */

#ifndef SIMPLEIODEF_H_
#define SIMPLEIODEF_H_

// ************************************************************************************************** Simple IO
typedef struct {
    volatile uint32_t INP;			// 16 bit input from port (read)
    volatile uint32_t OUT;			// 16 bit output for port (set)
    volatile uint16_t SET;			// 16 bit signals to set pins (to high)
    volatile uint16_t CLR;			// 16 bit signals to clr pins (to low)
} GPIO_SimpleTypeDef;

#define PORTA	((GPIO_SimpleTypeDef *) (GPIOA_BASE + 0x10))
#define PORTB	((GPIO_SimpleTypeDef *) (GPIOB_BASE + 0x10))
#define PORTC	((GPIO_SimpleTypeDef *) (GPIOC_BASE + 0x10))
#define PORTD	((GPIO_SimpleTypeDef *) (GPIOD_BASE + 0x10))
//#define PORTE	((GPIO_SimpleTypeDef *) (GPIOE_BASE + 0x10))
//#define PORTF	((GPIO_SimpleTypeDef *) (GPIOF_BASE + 0x10))
//#define PORTG	((GPIO_SimpleTypeDef *) (GPIOG_BASE + 0x10))
#define PORTH	((GPIO_SimpleTypeDef *) (GPIOH_BASE + 0x10))
//#define PORTI	((GPIO_SimpleTypeDef *) (GPIOI_BASE + 0x10))
//#define PORTJ	((GPIO_SimpleTypeDef *) (GPIOJ_BASE + 0x10))
//#define PORTK	((GPIO_SimpleTypeDef *) (GPIOK_BASE + 0x10))

#endif /* SIMPLEIODEF_H_ */
