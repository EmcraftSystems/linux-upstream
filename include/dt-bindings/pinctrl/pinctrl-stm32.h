#ifndef _DT_BINDINGS_PINCTRL_STM32_H
#define _DT_BINDINGS_PINCTRL_STM32_H

/* Modes */
#define IN		0
#define OUT		1
#define ALT		2
#define ANALOG		3

/* Alternate functions */
#define ALT0		((0 << 2) | ALT)
#define ALT1		((1 << 2) | ALT)
#define ALT2		((2 << 2) | ALT)
#define ALT3		((3 << 2) | ALT)
#define ALT4		((4 << 2) | ALT)
#define ALT5		((5 << 2) | ALT)
#define ALT6		((6 << 2) | ALT)
#define ALT7		((7 << 2) | ALT)
#define ALT8		((8 << 2) | ALT)
#define ALT9		((9 << 2) | ALT)
#define ALT10		((10 << 2) | ALT)
#define ALT11		((11 << 2) | ALT)
#define ALT12		((12 << 2) | ALT)
#define ALT13		((13 << 2) | ALT)
#define ALT14		((14 << 2) | ALT)
#define ALT15		((15 << 2) | ALT)

/* Pull-Up/Down */
#define NO_PULL		0
#define PULL_UP		1
#define PULL_DOWN	2

/* Type */
#define PUSH_PULL	(0 << 2)
#define OPEN_DRAIN	(1 << 2)

/* Speed */
#define LOW_SPEED	(0 << 3)
#define MEDIUM_SPEED	(1 << 3)
#define FAST_SPEED	(2 << 3)
#define HIGH_SPEED	(3 << 3)

#endif /* _DT_BINDINGS_PINCTRL_STM32_H */
