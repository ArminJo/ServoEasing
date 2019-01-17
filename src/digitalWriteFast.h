#define BIT_READ(value, bit)            ((value) &   (1UL << (bit)))
#define BIT_SET(value, bit)             ((value) |=  (1UL << (bit)))
#define BIT_CLEAR(value, bit)           ((value) &= ~(1UL << (bit)))
#define BIT_WRITE(value, bit, bitvalue) (bitvalue ? BIT_SET(value, bit) : BIT_CLEAR(value, bit))

#if !defined(digitalPinToPortReg)
#if (defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || \
     defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__))
// Arduino Mega Pins
#define __digitalPinToPortReg(P) \
(((P) >= 22 && (P) <= 29) ? &PORTA : \
((((P) >= 10 && (P) <= 13) || ((P) >= 50 && (P) <= 53)) ? &PORTB : \
(((P) >= 30 && (P) <= 37) ? &PORTC : \
((((P) >= 18 && (P) <= 21) || (P) == 38) ? &PORTD : \
((((P) >= 0 && (P) <= 3) || (P) == 5) ? &PORTE : \
(((P) >= 54 && (P) <= 61) ? &PORTF : \
((((P) >= 39 && (P) <= 41) || (P) == 4) ? &PORTG : \
((((P) >= 6 && (P) <= 9) || (P) == 16 || (P) == 17) ? &PORTH : \
(((P) == 14 || (P) == 15) ? &PORTJ : \
(((P) >= 62 && (P) <= 69) ? &PORTK : &PORTL))))))))))

#define __digitalPinToDDRReg(P) \
(((P) >= 22 && (P) <= 29) ? &DDRA : \
((((P) >= 10 && (P) <= 13) || ((P) >= 50 && (P) <= 53)) ? &DDRB : \
(((P) >= 30 && (P) <= 37) ? &DDRC : \
((((P) >= 18 && (P) <= 21) || (P) == 38) ? &DDRD : \
((((P) >= 0 && (P) <= 3) || (P) == 5) ? &DDRE : \
(((P) >= 54 && (P) <= 61) ? &DDRF : \
((((P) >= 39 && (P) <= 41) || (P) == 4) ? &DDRG : \
((((P) >= 6 && (P) <= 9) || (P) == 16 || (P) == 17) ? &DDRH : \
(((P) == 14 || (P) == 15) ? &DDRJ : \
(((P) >= 62 && (P) <= 69) ? &DDRK : &DDRL))))))))))

#define __digitalPinToPINReg(P) \
(((P) >= 22 && (P) <= 29) ? &PINA : \
((((P) >= 10 && (P) <= 13) || ((P) >= 50 && (P) <= 53)) ? &PINB : \
(((P) >= 30 && (P) <= 37) ? &PINC : \
((((P) >= 18 && (P) <= 21) || (P) == 38) ? &PIND : \
((((P) >= 0 && (P) <= 3) || (P) == 5) ? &PINE : \
(((P) >= 54 && (P) <= 61) ? &PINF : \
((((P) >= 39 && (P) <= 41) || (P) == 4) ? &PING : \
((((P) >= 6 && (P) <= 9) || (P) == 16 || (P) == 17) ? &PINH : \
(((P) == 14 || (P) == 15) ? &PINJ : \
(((P) >= 62 && (P) <= 69) ? &PINK : &PINL))))))))))

#define __digitalPinToBit(P) \
(((P) >=  7 && (P) <=  9) ? (P) - 3 : \
(((P) >= 10 && (P) <= 13) ? (P) - 6 : \
(((P) >= 22 && (P) <= 29) ? (P) - 22 : \
(((P) >= 30 && (P) <= 37) ? 37 - (P) : \
(((P) >= 39 && (P) <= 41) ? 41 - (P) : \
(((P) >= 42 && (P) <= 49) ? 49 - (P) : \
(((P) >= 50 && (P) <= 53) ? 53 - (P) : \
(((P) >= 54 && (P) <= 61) ? (P) - 54 : \
(((P) >= 62 && (P) <= 69) ? (P) - 62 : \
(((P) == 0 || (P) == 15 || (P) == 17 || (P) == 21) ? 0 : \
(((P) == 1 || (P) == 14 || (P) == 16 || (P) == 20) ? 1 : \
(((P) == 19) ? 2 : \
(((P) == 5 || (P) == 6 || (P) == 18) ? 3 : \
(((P) == 2) ? 4 : \
(((P) == 3 || (P) == 4) ? 5 : 7)))))))))))))))

// 15 PWM
#define __digitalPinToTimer(P) \
(((P) == 13 || (P) ==  4) ? &TCCR0A : \
(((P) == 11 || (P) == 12) ? &TCCR1A : \
(((P) == 10 || (P) ==  9) ? &TCCR2A : \
(((P) ==  5 || (P) ==  2 || (P) ==  3) ? &TCCR3A : \
(((P) ==  6 || (P) ==  7 || (P) ==  8) ? &TCCR4A : \
(((P) == 46 || (P) == 45 || (P) == 44) ? &TCCR5A : 0))))))
#define __digitalPinToTimerBit(P) \
(((P) == 13) ? COM0A1 : (((P) ==  4) ? COM0B1 : \
(((P) == 11) ? COM1A1 : (((P) == 12) ? COM1B1 : \
(((P) == 10) ? COM2A1 : (((P) ==  9) ? COM2B1 : \
(((P) ==  5) ? COM3A1 : (((P) ==  2) ? COM3B1 : (((P) ==  3) ? COM3C1 : \
(((P) ==  6) ? COM4A1 : (((P) ==  7) ? COM4B1 : (((P) ==  8) ? COM4C1 : \
(((P) == 46) ? COM5A1 : (((P) == 45) ? COM5B1 : COM5C1))))))))))))))

#elif (defined(__AVR_ATmega644__) || \
       defined(__AVR_ATmega644P__))
// Arduino 644 Pins
#define __digitalPinToPortReg(P) \
(((P) >= 0 && (P) <= 7) ? &PORTB : (((P) >= 8 && (P) <= 15) ? &PORTD : (((P) >= 16 && (P) <= 23) ? &PORTC : &PORTA)))
#define __digitalPinToDDRReg(P) \
(((P) >= 0 && (P) <= 7) ? &DDRB : (((P) >= 8 && (P) <= 15) ? &DDRD : (((P) >= 8 && (P) <= 15) ? &DDRC : &DDRA)))
#define __digitalPinToPINReg(P) \
(((P) >= 0 && (P) <= 7) ? &PINB : (((P) >= 8 && (P) <= 15) ? &PIND : (((P) >= 8 && (P) <= 15) ? &PINC : &PINA)))
#define __digitalPinToBit(P) \
(((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 15) ? (P) - 8 : (((P) >= 16 && (P) <= 23) ? (P) - 16 : (P) - 24)))

// 6 PWM
#define __digitalPinToTimer(P) \
(((P) ==  3 || (P) ==  4) ? &TCCR0A : \
(((P) == 12 || (P) == 13) ? &TCCR1A : \
(((P) == 14 || (P) == 15) ? &TCCR2A : 0)))
#define __digitalPinToTimerBit(P) \
(((P) ==  3) ? COM0A1 : (((P) ==  4) ? COM0B1 : \
(((P) == 13) ? COM1A1 : (((P) == 12) ? COM1B1 : \
(((P) == 15) ? COM2A1 : COM2B1)))))

// --- Arduino Leonardo ---
#elif (defined(ARDUINO_AVR_LEONARDO) || \
       defined(__AVR_ATmega16U4__) || \
       defined(__AVR_ATmega32U4__))

#define UART_RX_PIN     (0) //PD2
#define UART_TX_PIN     (1) //PD3

#define I2C_SDA_PIN     (2) //PD1
#define I2C_SCL_PIN     (3) //PD0

#define SPI_HW_SS_PIN   (17) //PB0
#define SPI_HW_MOSI_PIN (16) //PB2
#define SPI_HW_MISO_PIN (14) //PB3
#define SPI_HW_SCK_PIN  (15) //PB1

#define __digitalPinToPortReg(P) \
((((P) >= 0 && (P) <= 4) || (P) == 6 || (P) == 12 || (P) == 24 || (P) == 25 || (P) == 29) ? &PORTD : (((P) == 5 || (P) == 13) ? &PORTC : (((P) >= 18 && (P) <= 23)) ? &PORTF : (((P) == 7) ? &PORTE : &PORTB)))
#define __digitalPinToDDRReg(P) \
((((P) >= 0 && (P) <= 4) || (P) == 6 || (P) == 12 || (P) == 24 || (P) == 25 || (P) == 29) ? &DDRD : (((P) == 5 || (P) == 13) ? &DDRC : (((P) >= 18 && (P) <= 23)) ? &DDRF : (((P) == 7) ? &DDRE : &DDRB)))
#define __digitalPinToPINReg(P) \
((((P) >= 0 && (P) <= 4) || (P) == 6 || (P) == 12 || (P) == 24 || (P) == 25 || (P) == 29) ? &PIND : (((P) == 5 || (P) == 13) ? &PINC : (((P) >= 18 && (P) <= 23)) ? &PINF : (((P) == 7) ? &PINE : &PINB)))
#define __digitalPinToBit(P) \
(((P) >= 8 && (P) <= 11) ? (P) - 4 : (((P) >= 18 && (P) <= 21) ? 25 - (P) : (((P) == 0) ? 2 : (((P) == 1) ? 3 : (((P) == 2) ? 1 : (((P) == 3) ? 0 : (((P) == 4) ? 4 : (((P) == 6) ? 7 : (((P) == 13) ? 7 : (((P) == 14) ? 3 : (((P) == 15) ? 1 : (((P) == 16) ? 2 : (((P) == 17) ? 0 : (((P) == 22) ? 1 : (((P) == 23) ? 0 : (((P) == 24) ? 4 : (((P) == 25) ? 7 : (((P) == 26) ? 4 : (((P) == 27) ? 5 : 6 )))))))))))))))))))

#else
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
// we have only PORTB - Extension 01.03.2018
#define __digitalPinToPortReg(P) (&PORTB)
#define __digitalPinToDDRReg(P)  (&DDRB)
#define __digitalPinToPINReg(P)  (&PINB)
#define __digitalPinToBit(P) \
(((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P) - 8 : (P) - 14))
#else
// Standard Arduino Pins
#define __digitalPinToPortReg(P) \
(((P) >= 0 && (P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define __digitalPinToDDRReg(P) \
(((P) >= 0 && (P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define __digitalPinToPINReg(P) \
(((P) >= 0 && (P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define __digitalPinToBit(P) \
(((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P) - 8 : (P) - 14))
#endif

#if defined(__AVR_ATmega8__)
// 3 PWM
#define __digitalPinToTimer(P) \
(((P) ==  9 || (P) == 10) ? &TCCR1A : (((P) == 11) ? &TCCR2 : 0))
#define __digitalPinToTimerBit(P) \
(((P) ==  9) ? COM1A1 : (((P) == 10) ? COM1B1 : COM21))
#else  //168,328

// 6 PWM
#define __digitalPinToTimer(P) \
(((P) ==  6 || (P) ==  5) ? &TCCR0A : \
(((P) ==  9 || (P) == 10) ? &TCCR1A : \
(((P) == 11 || (P) ==  3) ? &TCCR2A : 0)))
#define __digitalPinToTimerBit(P) \
(((P) ==  6) ? COM0A1 : (((P) ==  5) ? COM0B1 : \
(((P) ==  9) ? COM1A1 : (((P) == 10) ? COM1B1 : \
(((P) == 11) ? COM2A1 : COM2B1)))))
#endif  //defined(__AVR_ATmega8__)

#endif
#endif  //#if !defined(digitalPinToPortReg)

#if !defined(digitalWriteFast)
#define digitalWriteFast(P, V) \
if (__builtin_constant_p(P) && __builtin_constant_p(V)) { \
  BIT_WRITE(*__digitalPinToPortReg(P), __digitalPinToBit(P), (V)); \
} else { \
  digitalWrite((P), (V)); \
}
#endif

// INPUT_PULLUP extension by AJ 01.03.2018
#if !defined(pinModeFast)
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
// we have only PORTB - Extension 01.03.2018
#define pinModeFast(P, V) \
if (__builtin_constant_p(P) && __builtin_constant_p(V)) { \
  if (V == INPUT_PULLUP) {\
    BIT_WRITE(*__digitalPinToDDRReg(P), __digitalPinToBit(P), (INPUT)); \
    BIT_WRITE(*__digitalPinToPortReg(P), __digitalPinToBit(P), (HIGH)); \
  } else { \
    BIT_WRITE(*__digitalPinToDDRReg(P), __digitalPinToBit(P), (V)); \
  } \
} else { \
 pinMode((P), (V)); \
}
#else
#define pinModeFast(P, V) \
if (__builtin_constant_p(P) && __builtin_constant_p(V)) { \
/* does not compile neatly. if (digitalPinToTimer(P)) { \
    BIT_CLEAR(*__digitalPinToTimer(P), __digitalPinToTimerBit(P)); \
  } */\
  if (V == INPUT_PULLUP) {\
    BIT_WRITE(*__digitalPinToDDRReg(P), __digitalPinToBit(P), (INPUT)); \
    BIT_WRITE(*__digitalPinToPortReg(P), __digitalPinToBit(P), (HIGH)); \
  } else { \
    BIT_WRITE(*__digitalPinToDDRReg(P), __digitalPinToBit(P), (V)); \
  } \
} else {  \
  pinMode((P), (V)); \
}
#endif
#endif

#if !defined(digitalReadFast)
#define digitalReadFast(P) ( (int) __digitalReadFast((P)) )
#define __digitalReadFast(P) \
(__builtin_constant_p(P)) \
  ? BIT_READ(*__digitalPinToPINReg(P), __digitalPinToBit(P)) \
  : digitalRead((P))
#endif

// Extension by AJ 01.03.2018
#if !defined(digitalToggleFast)
#define digitalToggleFast(P) BIT_SET(*__digitalPinToPINReg(P), __digitalPinToBit(P))
#endif

